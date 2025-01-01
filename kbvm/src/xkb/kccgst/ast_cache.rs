mod error;

use {
    crate::xkb::{
        clone_with_delta::CloneWithDelta,
        code::Code,
        code_loader::{CodeLoader, CodeType},
        code_map::CodeMap,
        diagnostic::{DiagnosticKind, DiagnosticSink},
        interner::{Interned, Interner},
        kccgst::{
            ast::Item,
            ast_cache::error::{not_found, AstCacheError},
            lexer::Lexer,
            parser::{parse_item, snoop_ty_and_name},
            token::Token,
        },
        meaning::MeaningCache,
        span::{Span, SpanExt, Spanned},
    },
    hashbrown::{hash_map::Entry, HashMap},
    kbvm_proc::ad_hoc_display,
    std::{collections::VecDeque, path::PathBuf, sync::Arc},
};

#[derive(Default)]
pub(crate) struct AstCache {
    files: HashMap<FileKey, Value>,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
struct FileKey {
    ty: CodeType,
    file: Interned,
}

#[derive(Debug)]
struct SeenMap {
    file: Arc<PathBuf>,
    code: Code,
    span: Span,
    tokens_or_item: TokensOrItem,
}

#[derive(Debug)]
enum TokensOrItem {
    Invalid,
    Tokens(Vec<Spanned<Token>>),
    Item(Spanned<Item>),
}

#[derive(Debug)]
struct Value {
    lexers: VecDeque<Lexer>,
    maps: Vec<SeenMap>,
    map_names: HashMap<MapName, usize>,
}

#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
enum MapName {
    Unnamed,
    Default,
    Named(Interned),
}

impl AstCache {
    #[expect(clippy::too_many_arguments)]
    pub(crate) fn get(
        &mut self,
        diagnostics: &mut DiagnosticSink,
        map: &mut CodeMap,
        loader: &mut CodeLoader,
        interner: &mut Interner,
        meaning_cache: &mut MeaningCache,
        ty: CodeType,
        file: Interned,
        file_map: Option<Interned>,
        include_span: Span,
    ) -> Result<Spanned<Item>, Spanned<AstCacheError>> {
        let key = FileKey { ty, file };
        let value = match self.files.entry(key) {
            Entry::Occupied(v) => v.into_mut(),
            Entry::Vacant(v) => {
                let lexers = loader
                    .load(interner, ty, file)
                    .flat_map(|res| match res {
                        Ok((path, code)) => {
                            let span = map.add(Some(&path), Some(include_span), &code);
                            Some(Lexer::new(Some(&path), &code, span.lo))
                        }
                        Err(e) => {
                            diagnostics.push(
                                map,
                                DiagnosticKind::FileReadFailed,
                                e.spanned2(include_span),
                            );
                            None
                        }
                    })
                    .collect();
                v.insert(Value {
                    lexers,
                    maps: Default::default(),
                    map_names: Default::default(),
                })
            }
        };
        let mut first_emitted = false;
        let desired_map = file_map.map(MapName::Named).unwrap_or(MapName::Default);
        loop {
            let idx = get_idx(
                diagnostics,
                map,
                interner,
                meaning_cache,
                file,
                file_map,
                include_span,
                key,
                value,
                &mut first_emitted,
                desired_map,
            )?;
            let entry = &mut value.maps[idx];
            let new_span = map.add(Some(&entry.file), Some(include_span), &entry.code);
            let delta = new_span.lo - entry.span.lo;
            match &entry.tokens_or_item {
                TokensOrItem::Invalid => {}
                TokensOrItem::Item(item) => return Ok(item.clone_with_delta(delta)),
                TokensOrItem::Tokens(t) => {
                    match parse_item(map, diagnostics, interner, meaning_cache, t, delta) {
                        Ok(item) => {
                            let ret = item.clone_with_delta(delta);
                            entry.tokens_or_item = TokensOrItem::Item(item);
                            return Ok(ret);
                        }
                        Err(e) => {
                            diagnostics.push(map, e.val.diagnostic_kind(), e);
                            entry.tokens_or_item = TokensOrItem::Invalid;
                        }
                    }
                }
            }
        }
    }
}

#[expect(clippy::too_many_arguments)]
fn get_idx(
    diagnostics: &mut DiagnosticSink,
    map: &mut CodeMap,
    interner: &mut Interner,
    meaning_cache: &mut MeaningCache,
    file: Interned,
    file_map: Option<Interned>,
    include_span: Span,
    key: FileKey,
    value: &mut Value,
    first_emitted: &mut bool,
    desired_map: MapName,
) -> Result<usize, Spanned<AstCacheError>> {
    if let Entry::Occupied(o) = value.map_names.entry(desired_map) {
        if !matches!(value.maps[*o.get()].tokens_or_item, TokensOrItem::Invalid) {
            return Ok(*o.get());
        }
        o.remove();
    }
    while let Some(lexer) = value.lexers.front_mut() {
        let mut tokens = vec![];
        match lexer.lex_item(interner, &mut tokens) {
            Ok(_) if tokens.is_empty() => {
                value.lexers.pop_front();
                continue;
            }
            Err(e) => {
                value.lexers.pop_front();
                diagnostics.push(map, e.val.diagnostic_kind(), e);
                continue;
            }
            Ok(_) => {}
        }
        let (default, ty, name) = match snoop_ty_and_name(&tokens, interner, meaning_cache) {
            Ok(res) => res,
            Err(e) => {
                diagnostics.push(map, e.val.diagnostic_kind(), e);
                continue;
            }
        };
        let name_ns = name.map(|n| n.val);
        if ty.val != key.ty {
            diagnostics.push(
                map,
                DiagnosticKind::UnexpectedItemType,
                ad_hoc_display!("unexpected item type").spanned2(ty.span),
            );
            continue;
        }
        let seen_map = SeenMap {
            file: lexer.path().unwrap().clone(),
            code: lexer.code().clone(),
            span: lexer.span(),
            tokens_or_item: TokensOrItem::Tokens(tokens),
        };
        let idx = value.maps.len();
        value.maps.push(seen_map);
        if let Some(default) = default {
            if value.map_names.try_insert(MapName::Default, idx).is_err() {
                diagnostics.push(
                    map,
                    DiagnosticKind::MultipleDefaultItems,
                    ad_hoc_display!("file contains multiple default items").spanned2(default),
                );
            }
        }
        let map_name = name_ns.map(MapName::Named).unwrap_or(MapName::Unnamed);
        if value.map_names.try_insert(map_name, idx).is_err() {
            diagnostics.push(
                map,
                DiagnosticKind::DuplicateItemName,
                ad_hoc_display!("duplicate item name in file")
                    .spanned2(name.map(|n| n.span).unwrap_or(ty.span)),
            );
        }
        if desired_map == map_name || (default.is_some() && desired_map == MapName::Default) {
            return Ok(idx);
        }
    }
    if desired_map == MapName::Default {
        let first = value
            .maps
            .iter()
            .position(|m| !matches!(m.tokens_or_item, TokensOrItem::Invalid));
        if let Some(first) = first {
            if !*first_emitted {
                *first_emitted = true;
                diagnostics.push(
                    map,
                    DiagnosticKind::UsingFirstInsteadOfDefault,
                    ad_hoc_display!("default map not found, using first instead")
                        .spanned2(include_span),
                );
            }
            return Ok(first);
        }
    }
    Err(not_found(interner, file, file_map, include_span))
}
