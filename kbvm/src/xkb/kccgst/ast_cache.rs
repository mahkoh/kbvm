mod error;

use {
    crate::xkb::{
        clone_with_delta::CloneWithDelta,
        code::Code,
        code_loader::{CodeLoader, CodeType},
        code_map::CodeMap,
        diagnostic::{Diagnostic, Severity},
        interner::{Interned, Interner},
        kccgst::{
            ast::Item,
            ast_cache::error::{not_found, AstCacheError},
            lexer::Lexer,
            meaning::MeaningCache,
            parser::{parse_item, snoop_ty_and_name},
            token::Token,
        },
        span::{Span, Spanned},
    },
    hashbrown::{hash_map::Entry, HashMap},
    std::{collections::VecDeque, path::PathBuf, sync::Arc},
};

#[derive(Default)]
pub struct AstCache {
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
    Tokens(Vec<Spanned<Token>>),
    Item(Spanned<Item>),
}

#[derive(Debug)]
struct Value {
    lexers: VecDeque<Lexer>,
    default_name: Option<Option<Interned>>,
    maps: HashMap<Option<Interned>, SeenMap>,
}

impl AstCache {
    pub fn get(
        &mut self,
        diagnostics: &mut Vec<Diagnostic>,
        map: &mut CodeMap,
        loader: &mut CodeLoader,
        interner: &mut Interner,
        meaning_cache: &mut MeaningCache,
        ty: CodeType,
        file: Interned,
        mut file_map: Option<Interned>,
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
                            diagnostics.push(Diagnostic::new(
                                map,
                                Severity::Warning,
                                e,
                                include_span,
                            ));
                            None
                        }
                    })
                    .collect();
                v.insert(Value {
                    lexers,
                    default_name: None,
                    maps: Default::default(),
                })
            }
        };
        let real_file_map = file_map;
        if file_map.is_none() {
            if let Some(default_name) = value.default_name {
                file_map = default_name;
            }
        }
        loop {
            let mut entry = match value.maps.entry(file_map) {
                Entry::Occupied(o) => o,
                _ => loop {
                    let Some(lexer) = value.lexers.front_mut() else {
                        return Err(not_found(interner, file, real_file_map, include_span));
                    };
                    let mut tokens = vec![];
                    match lexer.lex_item(interner, &mut tokens) {
                        Ok(_) if tokens.is_empty() => {
                            value.lexers.pop_front();
                            continue;
                        }
                        Err(e) => {
                            diagnostics.push(Diagnostic::new(
                                map,
                                Severity::Warning,
                                e.val,
                                e.span,
                            ));
                            continue;
                        }
                        Ok(_) => {}
                    }
                    let (default, ty, name) =
                        match snoop_ty_and_name(&tokens, interner, meaning_cache) {
                            Ok(res) => res,
                            Err(e) => {
                                diagnostics.push(e.into_diagnostic(map, Severity::Warning));
                                continue;
                            }
                        };
                    let name_ns = name.map(|n| n.val);
                    if ty.val != key.ty {
                        diagnostics.push(Diagnostic::new(
                            map,
                            Severity::Warning,
                            literal_display!("Unexpected item type"),
                            ty.span,
                        ));
                        continue;
                    }
                    if let Some(default) = default {
                        if value.default_name.is_none() {
                            value.default_name = Some(name_ns);
                            if file_map.is_none() {
                                file_map = name_ns;
                            }
                        } else {
                            diagnostics.push(Diagnostic::new(
                                map,
                                Severity::Warning,
                                literal_display!("File contains multiple default items"),
                                default,
                            ));
                        }
                    }
                    let seen_map = SeenMap {
                        file: lexer.path().unwrap().clone(),
                        code: lexer.code().clone(),
                        span: lexer.span(),
                        tokens_or_item: TokensOrItem::Tokens(tokens),
                    };
                    let map = match value.maps.entry(name_ns) {
                        Entry::Occupied(_) => {
                            diagnostics.push(Diagnostic::new(
                                map,
                                Severity::Warning,
                                literal_display!("Duplicate item name in file"),
                                name.map(|n| n.span).or(default).unwrap_or(ty.span),
                            ));
                            continue;
                        }
                        Entry::Vacant(v) => v.insert_entry(seen_map),
                    };
                    if file_map == name_ns {
                        break map;
                    }
                },
            };
            let new_span = map.add(
                Some(&entry.get().file),
                Some(include_span),
                &entry.get().code,
            );
            let delta = new_span.lo - entry.get().span.lo;
            match &entry.get().tokens_or_item {
                TokensOrItem::Tokens(t) => match parse_item(interner, meaning_cache, &t, delta) {
                    Ok(item) => {
                        let ret = item.clone_with_delta(delta);
                        entry.get_mut().tokens_or_item = TokensOrItem::Item(item);
                        return Ok(ret);
                    }
                    Err(e) => {
                        diagnostics.push(e.into_diagnostic(map, Severity::Warning));
                        entry.remove();
                    }
                },
                TokensOrItem::Item(item) => return Ok(item.clone_with_delta(delta)),
            }
        }
    }
}
