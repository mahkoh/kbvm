#[cfg(test)]
mod tests;

use {
    crate::{
        from_bytes::FromBytes,
        xkb::{
            code::Code,
            code_loader::{CodeLoader, CodeType},
            code_map::CodeMap,
            diagnostic::{DiagnosticKind, DiagnosticSink},
            group::GroupIdx,
            interner::{Interned, Interner},
            kccgst::{
                ast::{
                    Compat, CompatmapDecl, CompositeMap, ConfigItem, ConfigItemType, Decl, Decls,
                    DirectOrIncluded, Geometry, GeometryDecl, Include, Item, ItemType, KeycodeDecl,
                    Keycodes, NestedConfigItem, Symbols, SymbolsDecl, Types, TypesDecl,
                },
                MergeMode,
            },
            meaning::MeaningCache,
            rmlvo::{
                lexer::Lexer,
                parser::{
                    parse_line, Line, MappingKey, MappingKeyIndex, MappingValue, ParserCache,
                    RuleKey,
                },
            },
            span::{Span, SpanExt, Spanned},
        },
    },
    hashbrown::{hash_set::Entry, HashMap, HashSet},
    kbvm_proc::ad_hoc_display,
    linearize::{static_map, StaticMap},
    std::{io::Write, path::PathBuf, sync::Arc},
};

pub struct Group {
    pub layout: Option<Interned>,
    pub variant: Option<Interned>,
}

pub(crate) fn create_item(
    map: &mut CodeMap,
    interner: &mut Interner,
    loader: &mut CodeLoader,
    diagnostics: &mut DiagnosticSink,
    meaning_cache: &mut MeaningCache,
    rules: Spanned<Interned>,
    model: Option<Interned>,
    options: &[Interned],
    groups: &[Group],
) -> Item {
    let includes = create_includes(
        map,
        interner,
        loader,
        diagnostics,
        meaning_cache,
        rules,
        model,
        options,
        groups,
    );
    fn to_decls<T>(includes: Vec<Spanned<Include>>, map: impl Fn(Include) -> T) -> Decls<T> {
        Decls {
            decls: includes
                .into_iter()
                .map(|i| {
                    i.map(|i| Decl {
                        merge_mode: Some(i.mm),
                        ty: DirectOrIncluded::Direct(map(i)),
                    })
                })
                .collect(),
        }
    }
    let items = includes.map(|k, v| {
        macro_rules! map {
            ($($ident:ident, $decl:ty;)*) => {
                match k {
                    $(
                        MappingValue::$ident => ConfigItemType::$ident($ident {
                            name: None,
                            decls: to_decls(v, |i| <$decl>::Include(i)),
                        }),
                    )*
                }
            };
        }
        NestedConfigItem {
            flags: Default::default(),
            item: ConfigItem {
                specific: map! {
                    Keycodes, KeycodeDecl;
                    Symbols, SymbolsDecl;
                    Types, TypesDecl;
                    Compat, CompatmapDecl;
                    Geometry, GeometryDecl;
                },
            },
        }
    });
    let map = CompositeMap {
        name: None,
        config_items: items.into_iter().map(|(_, v)| v.spanned(0, 0)).collect(),
    };
    let item = Item {
        flags: Default::default(),
        ty: ItemType::Composite(map),
    };
    item
}

pub(crate) fn create_includes(
    map: &mut CodeMap,
    interner: &mut Interner,
    loader: &mut CodeLoader,
    diagnostics: &mut DiagnosticSink,
    meaning_cache: &mut MeaningCache,
    rules: Spanned<Interned>,
    model: Option<Interned>,
    options: &[Interned],
    groups: &[Group],
) -> StaticMap<MappingValue, Vec<Spanned<Include>>> {
    let options: HashSet<_> = options.iter().copied().collect();

    let mut active_includes = HashSet::new();
    let mut macros = HashMap::new();
    let mut cache = ParserCache::default();
    let mut tokens = vec![];
    let mut have_mapping = false;
    let mut mapping_has_options = false;
    let mut skip_to_next_mapping = false;
    let mut mapping_keys = vec![];
    let mut mapping_values = vec![];
    let mut matched_groups = vec![(false, false); groups.len()];
    let mut stash = vec![];
    let mut includes = static_map! {
        _ => vec!(),
    };
    if groups.is_empty() {
        return includes;
    }

    let mut lexers: Vec<(Option<Interned>, Lexer)> = vec![];
    if let Some(lexer) = create_lexer(map, interner, loader, diagnostics, rules) {
        lexers.push((None, lexer));
    }
    macro_rules! pop {
        () => {
            if let Some((Some(inc), _)) = lexers.pop() {
                active_includes.remove(&inc);
            }
        };
    }

    while let Some((_, lexer)) = lexers.last_mut() {
        tokens.clear();
        if let Err(e) = lexer.lex_line(interner, &mut tokens) {
            diagnostics.push(map, DiagnosticKind::SyntaxError, e);
            pop!();
            continue;
        }
        if tokens.is_empty() {
            pop!();
            continue;
        }
        let line = match parse_line(
            map,
            diagnostics,
            interner,
            &mut cache,
            meaning_cache,
            &tokens,
        ) {
            Ok(l) => l,
            Err(e) => {
                diagnostics.push(map, e.val.diagnostic_kind(), e);
                continue;
            }
        };
        match line.val {
            Line::Macro(n, v) => {
                macros.insert(n, v.to_vec());
            }
            Line::Include(i) => {
                let active = active_includes.entry(i);
                if matches!(active, Entry::Occupied(_)) {
                    diagnostics.push(
                        map,
                        DiagnosticKind::RecursiveInclude,
                        ad_hoc_display!("ignoring recursive include").spanned2(line.span),
                    );
                    continue;
                }
                let lexer = create_lexer(map, interner, loader, diagnostics, i.spanned2(line.span));
                if let Some(lexer) = lexer {
                    active.insert();
                    lexers.push((Some(i), lexer));
                }
            }
            Line::Mapping(k, v) => {
                have_mapping = true;
                skip_to_next_mapping = false;
                mapping_has_options = k.iter().any(|m| m == &MappingKey::Option);
                mapping_keys.clear();
                mapping_keys.extend_from_slice(k);
                mapping_values.clear();
                mapping_values.extend_from_slice(v);
            }
            Line::Rule(k, v) => {
                if !have_mapping || skip_to_next_mapping {
                    continue;
                }
                if k.len() != mapping_keys.len() {
                    diagnostics.push(
                        map,
                        DiagnosticKind::InvalidNumberOfRuleKeys,
                        ad_hoc_display!("expected {} rule keys but got {}", mapping_keys.len() => usize, k.len() => usize)
                            .spanned2(line.span),

                    );
                    continue;
                }
                if v.len() != mapping_values.len() {
                    diagnostics.push(
                        map,
                        DiagnosticKind::InvalidNumberOfRuleValues,
                        ad_hoc_display!("expected {} rule values but got {}", mapping_values.len() => usize, v.len() => usize)
                            .spanned2(line.span),
                    );
                    continue;
                }
                let mut matches = true;
                for (b, _) in &mut matched_groups {
                    *b = true;
                }
                let mut matched_any_group = false;
                for (mk, rk) in mapping_keys.iter().zip(k.iter()) {
                    match mk {
                        MappingKey::Model => match rk {
                            RuleKey::Star => continue,
                            RuleKey::Macro(g) => {
                                matches = macros
                                    .get(g)
                                    .iter()
                                    .flat_map(|g| g.iter())
                                    .any(|i| model == Some(*i));
                            }
                            RuleKey::Ident(i) => {
                                matches = model == Some(*i);
                            }
                        },
                        MappingKey::Option => match rk {
                            RuleKey::Star => continue,
                            RuleKey::Macro(g) => {
                                matches = macros
                                    .get(g)
                                    .iter()
                                    .flat_map(|g| g.iter())
                                    .any(|i| options.contains(i));
                            }
                            RuleKey::Ident(i) => {
                                matches = options.contains(i);
                            }
                        },
                        MappingKey::Layout(idx) | MappingKey::Variant(idx) => {
                            let idx = idx.unwrap_or(MappingKeyIndex::Single);
                            let groups_range = match idx {
                                MappingKeyIndex::Single if groups.len() == 1 => 0..1,
                                MappingKeyIndex::First => 0..1,
                                MappingKeyIndex::Later => 1..groups.len(),
                                MappingKeyIndex::Any => 0..groups.len(),
                                MappingKeyIndex::Value(idx) if groups.len() != 1 => {
                                    let offset = idx.to_offset();
                                    offset..offset + 1
                                }
                                _ => {
                                    matches = false;
                                    break;
                                }
                            };
                            for (_, b) in &mut matched_groups {
                                *b = false;
                            }
                            let mut matched_any = false;
                            for group_offset in groups_range {
                                let Some(group) = groups.get(group_offset) else {
                                    break;
                                };
                                let key = match mk {
                                    MappingKey::Layout(_) => group.layout,
                                    _ => group.variant,
                                };
                                let matches = match rk {
                                    RuleKey::Star => key.is_some(),
                                    RuleKey::Macro(m) => macros
                                        .get(m)
                                        .iter()
                                        .flat_map(|g| g.iter())
                                        .any(|i| key == Some(*i)),
                                    RuleKey::Ident(i) => key == Some(*i),
                                };
                                if matches {
                                    matched_any = true;
                                    matched_groups[group_offset].1 = true;
                                }
                            }
                            if !matched_any {
                                matches = false;
                                break;
                            }
                            matched_any_group = true;
                            let mut any_viable = false;
                            for (g, l) in &mut matched_groups {
                                *g &= *l;
                                any_viable |= *g;
                            }
                            if !any_viable {
                                matches = false;
                                break;
                            }
                        }
                    };
                    if !matches {
                        break;
                    }
                }
                if matches {
                    skip_to_next_mapping = !mapping_has_options;
                    for (mv, rv) in mapping_values.iter().zip(v.iter()) {
                        let mut expand = |idx: Option<usize>| {
                            expand(
                                map,
                                lexer.path(),
                                line.span,
                                interner,
                                diagnostics,
                                rv.ident,
                                idx,
                                model,
                                groups,
                                &mut stash,
                                &mut includes[*mv],
                            );
                        };
                        if matched_any_group {
                            for (idx, (matched, _)) in matched_groups.iter().copied().enumerate() {
                                if !matched {
                                    continue;
                                }
                                expand(Some(idx))
                            }
                        } else {
                            expand(None)
                        }
                    }
                }
            }
        }
    }

    includes
}

fn expand(
    map: &mut CodeMap,
    path: &Arc<PathBuf>,
    include_span: Span,
    interner: &mut Interner,
    diagnostics: &mut DiagnosticSink,
    value: Spanned<Interned>,
    idx: Option<usize>,
    model: Option<Interned>,
    groups: &[Group],
    stash: &mut Vec<u8>,
    includes: &mut Vec<Spanned<Include>>,
) {
    stash.clear();
    let input = interner.get(value.val).to_owned();
    let mut flush = |map: &mut CodeMap,
                     interner: &mut Interner,
                     mut mm: Option<Spanned<MergeMode>>,
                     stash: Spanned<&mut Vec<u8>>| {
        let mut flush_once = |map: &mut CodeMap,
                              interner: &mut Interner,
                              mm: Option<Spanned<MergeMode>>,
                              stash: Spanned<&[u8]>| {
            if stash.val.is_empty() {
                return;
            }
            let mm = match mm {
                Some(mm) => mm,
                _ => {
                    includes.clear();
                    MergeMode::Include.spanned2(value.span)
                }
            };
            let code = Arc::new(stash.val.to_vec());
            let code = Code::new(&code);
            let code_span = map.add(Some(path), Some(include_span), &code);
            let interned = interner.intern(&code.to_slice());
            includes.push(
                Include {
                    mm,
                    path: interned.spanned2(code_span),
                    resolved: None,
                }
                .spanned2(stash.span),
            );
        };
        if stash.val.ends_with(b":all") {
            let len = stash.val.len() - 3;
            for idx in 0..groups.len() {
                stash.val.truncate(len);
                write!(stash.val, "{}", idx + 1).unwrap();
                flush_once(map, interner, mm, stash.as_ref().map(|s| &***s));
                if mm.is_none() {
                    let lo = stash.span.lo + len as u64;
                    mm = Some(MergeMode::Override.spanned(lo, lo + 3));
                }
            }
        } else {
            flush_once(map, interner, mm, stash.as_ref().map(|s| &***s));
        }
        stash.val.clear();
    };
    let bytes = input.as_bytes();
    let mut mm = None;
    let mut start = 0;
    let mut offset = 0;
    let mut last_was_colon;
    let mut last_was_colon_next = false;
    while offset < bytes.len() {
        let b = bytes[offset];
        last_was_colon = last_was_colon_next;
        last_was_colon_next = b == b':';
        if matches!(b, b'+' | b'|') {
            let lo = value.span.lo + start as u64;
            let hi = value.span.lo + offset as u64;
            flush(map, interner, mm, stash.spanned(lo, hi));
            let mode = match b == b'+' {
                true => MergeMode::Override,
                false => MergeMode::Augment,
            };
            let lo = hi;
            mm = Some(mode.spanned(lo, lo + 1));
            offset += 1;
            start = offset;
            continue;
        }
        offset += 1;
        if b != b'%' {
            stash.push(b);
            continue;
        }
        let encoding_start = offset - 1;
        let mut encoding =
            parse_percent_encoding(bytes, &mut offset, last_was_colon, value.span.lo);
        if let Some(e) = &encoding {
            if let Some(Some(idx)) = e.index {
                if idx.to_offset() >= groups.len() {
                    encoding = None;
                }
            }
        }
        if let Some(e) = &encoding {
            if e.component == Component::Index && idx.is_none() {
                encoding = None;
            }
        }
        let encoding_end = offset;
        let encoding = match encoding {
            Some(e) => e,
            _ => {
                let lo = value.span.lo + encoding_start as u64;
                let hi = value.span.lo + encoding_end as u64;
                diagnostics.push(
                    map,
                    DiagnosticKind::InvalidPercentEncoding,
                    ad_hoc_display!("invalid percent encoding").spanned(lo, hi),
                );
                continue;
            }
        };
        if let Some(new_mm) = encoding.merge_mode {
            let lo = value.span.lo + start as u64;
            let hi = value.span.lo + encoding_start as u64;
            flush(map, interner, mm, stash.spanned(lo, hi));
            mm = Some(new_mm);
            start = encoding_start;
        }
        if encoding.parenthesize {
            stash.push(b'(');
        }
        if let Some(prefix) = encoding.prefix {
            stash.push(prefix);
        }
        let mut group = idx.map(|i| &groups[i]);
        if let Some(Some(idx)) = encoding.index {
            group = Some(&groups[idx.to_offset()]);
        }
        'write_component: {
            let component = match encoding.component {
                Component::Model => model,
                Component::Layout => group.and_then(|g| g.layout),
                Component::Variant => group.and_then(|g| g.variant),
                Component::Index => {
                    write!(stash, ":{}", idx.unwrap() + 1).unwrap();
                    break 'write_component;
                }
            };
            if let Some(c) = component {
                stash.extend_from_slice(interner.get(c));
            }
        }
        if encoding.parenthesize {
            stash.push(b')');
        }
    }
    let lo = value.span.lo + start as u64;
    let hi = value.span.lo + offset as u64;
    flush(map, interner, mm, stash.spanned(lo, hi));
}

struct PercentEncoding {
    parenthesize: bool,
    merge_mode: Option<Spanned<MergeMode>>,
    prefix: Option<u8>,
    component: Component,
    index: Option<Option<GroupIdx>>,
}

#[derive(Copy, Clone, Eq, PartialEq)]
enum Component {
    Model,
    Layout,
    Variant,
    Index,
}

fn find_percent_encoding_range(bytes: &[u8], offset: &mut usize) -> Option<()> {
    let mut b = *bytes.get(*offset)?;
    *offset += 1;
    if b == b'(' {
        while *offset < bytes.len() {
            let b = bytes[*offset];
            *offset += 1;
            if b == b')' {
                return Some(());
            }
        }
        return None;
    }
    if matches!(b, b'+' | b'|' | b'-' | b'_') {
        b = *bytes.get(*offset)?;
        *offset += 1;
    }
    if b == b'i' {
        return Some(());
    }
    if !matches!(b, b'm' | b'l' | b'v') {
        return None;
    }
    if let Some(&b) = bytes.get(*offset) {
        if b == b'[' {
            *offset += 1;
            while *offset < bytes.len() {
                let b = bytes[*offset];
                *offset += 1;
                if b == b']' {
                    return Some(());
                }
            }
            return None;
        }
    }
    Some(())
}

fn parse_percent_encoding(
    bytes: &[u8],
    offset: &mut usize,
    last_was_colon: bool,
    span_lo: u64,
) -> Option<PercentEncoding> {
    let start = *offset;
    find_percent_encoding_range(bytes, offset)?;
    let mut bytes = &bytes[start..*offset];
    let mut parenthesize = false;
    if bytes[0] == b'(' {
        parenthesize = true;
        bytes = &bytes[1..bytes.len() - 1];
    }
    let mut merge_mode = None;
    let mut prefix = None;
    let b = *bytes.get(0)?;
    if matches!(b, b'|' | b'+') {
        let mm = match b == b'|' {
            true => MergeMode::Augment,
            false => MergeMode::Override,
        };
        let hi = span_lo + *offset as u64;
        merge_mode = Some(mm.spanned(hi - 1, hi));
        bytes = &bytes[1..];
    } else if matches!(b, b'-' | b'_') {
        prefix = Some(b);
        bytes = &bytes[1..];
    }
    let b = *bytes.get(0)?;
    let component = match b {
        b'm' => Component::Model,
        b'l' => Component::Layout,
        b'v' => Component::Variant,
        b'i' if last_was_colon => Component::Index,
        _ => return None,
    };
    bytes = &bytes[1..];
    let mut pe = PercentEncoding {
        parenthesize,
        merge_mode,
        prefix,
        component,
        index: None,
    };
    if bytes.is_empty() {
        return Some(pe);
    }
    if component == Component::Index {
        return None;
    }
    if bytes[0] != b'[' || *bytes.last().unwrap() != b']' {
        return None;
    }
    bytes = &bytes[1..bytes.len() - 1];
    let index = if bytes == b"%i" {
        None
    } else {
        let idx = u32::from_bytes_dec(bytes)?;
        Some(GroupIdx::new(idx)?)
    };
    pe.index = Some(index);
    Some(pe)
}

fn create_lexer(
    map: &mut CodeMap,
    interner: &mut Interner,
    loader: &mut CodeLoader,
    diagnostics: &mut DiagnosticSink<'_>,
    path: Spanned<Interned>,
) -> Option<Lexer> {
    let (real_path, code) = 'find_code: {
        for f in loader.load(interner, CodeType::Rules, path.val) {
            match f {
                Ok(c) => break 'find_code c,
                Err(e) => {
                    diagnostics.push(map, DiagnosticKind::FileReadFailed, e.spanned2(path.span));
                    continue;
                }
            }
        }
        diagnostics.push(
            map,
            DiagnosticKind::FileNotFound,
            ad_hoc_display!("could not find file to include").spanned2(path.span),
        );
        return None;
    };
    let span = map.add(Some(&real_path), Some(path.span), &code);
    let lexer = Lexer::new(&real_path, &code, span.lo);
    Some(lexer)
}
