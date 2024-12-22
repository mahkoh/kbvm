pub(crate) mod error;
#[cfg(test)]
mod tests;

use {
    crate::{
        from_bytes::FromBytes,
        xkb::{
            code::Code,
            code_map::CodeMap,
            diagnostic::{DiagnosticKind, DiagnosticSink},
            group::GroupIdx,
            interner::{Interned, Interner},
            meaning::{Meaning, MeaningCache},
            rmlvo::{
                parser::error::{
                    Expected, ParserError, AFTER_EXCLAM, MAPPING_KEY, MAPPING_VALUE, MLVO,
                    RULE_KEY, START_OF_LINE,
                },
                token::{Punctuation, Token},
            },
            span::{Span, SpanExt, Spanned},
        },
    },
    kbvm_proc::ad_hoc_display,
    linearize::Linearize,
    std::sync::{Arc, LazyLock},
};

#[derive(Default)]
pub struct ParserCache {
    group_assignments: Vec<Interned>,
    mapping_keys: Vec<MappingKey>,
    mapping_values: Vec<MappingValue>,
    rule_keys: Vec<RuleKey>,
    rule_values: Vec<RuleValue>,
}

struct Parser<'a, 'b> {
    map: &'a mut CodeMap,
    diagnostics: &'a mut DiagnosticSink<'b>,
    tokens: &'a [Spanned<Token>],
    interner: &'a mut Interner,
    meaning_cache: &'a mut MeaningCache,
    pos: usize,
}

#[derive(Debug)]
pub(crate) enum Line<'a> {
    Macro(Interned, &'a [Interned]),
    Include(Interned),
    Mapping(&'a [MappingKey], &'a [MappingValue]),
    Rule(&'a [RuleKey], &'a [RuleValue]),
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) enum MappingKey {
    Model,
    Option,
    Layout(Option<MappingKeyIndex>),
    Variant(Option<MappingKeyIndex>),
}

#[derive(Copy, Clone, Debug, Linearize)]
pub(crate) enum MappingValue {
    Keycodes,
    Types,
    Compat,
    Symbols,
    Geometry,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) enum MappingKeyIndex {
    Single,
    First,
    Later,
    Any,
    Value(GroupIdx),
}

#[derive(Debug)]
pub(crate) enum RuleKey {
    Star,
    Macro(Interned),
    Ident(Interned),
}

#[derive(Debug)]
pub(crate) struct RuleValue {
    pub(crate) ident: Spanned<Interned>,
}

pub(crate) fn parse_line<'a>(
    map: &mut CodeMap,
    diagnostics: &mut DiagnosticSink,
    interner: &mut Interner,
    cache: &'a mut ParserCache,
    meaning_cache: &mut MeaningCache,
    tokens: &[Spanned<Token>],
) -> Result<Spanned<Line<'a>>, Spanned<ParserError>> {
    Parser {
        map,
        diagnostics,
        tokens,
        interner,
        meaning_cache,
        pos: 0,
    }
    .parse_line(cache)
}

impl Parser<'_, '_> {
    fn next(
        &mut self,
        expected: &'static [Expected],
    ) -> Result<Spanned<Token>, Spanned<ParserError>> {
        if self.pos >= self.tokens.len() {
            return Err(self.expected_but_eof(self.tokens.last().unwrap().span, expected));
        }
        let token = self.tokens[self.pos];
        self.pos += 1;
        Ok(token)
    }

    fn try_peek(&mut self) -> Option<Spanned<Token>> {
        self.tokens.get(self.pos).copied()
    }

    fn peek(
        &mut self,
        expected: &'static [Expected],
    ) -> Result<Spanned<Token>, Spanned<ParserError>> {
        if self.pos >= self.tokens.len() {
            return Err(self.expected_but_eof(self.tokens.last().unwrap().span, expected));
        }
        Ok(self.tokens[self.pos])
    }

    fn parse_any_ident(&mut self) -> Result<Spanned<Interned>, Spanned<ParserError>> {
        self.parse_ident(&[Expected::AnyIdent])
    }

    fn parse_ident(
        &mut self,
        expected: &'static [Expected],
    ) -> Result<Spanned<Interned>, Spanned<ParserError>> {
        let t = self.next(expected)?;
        match t.val {
            Token::Ident(i) => Ok(i.spanned2(t.span)),
            _ => Err(self.unexpected_token(expected, t)),
        }
    }

    fn consume_token(&mut self, token: Punctuation) -> Result<Span, Spanned<ParserError>> {
        let t = self.next(token.expected())?;
        if t.val != Token::Punctuation(token) {
            return Err(self.unexpected_token(token.expected(), t));
        }
        Ok(t.span)
    }

    fn parse_line<'a>(
        &mut self,
        cache: &'a mut ParserCache,
    ) -> Result<Spanned<Line<'a>>, Spanned<ParserError>> {
        let t = self.peek(START_OF_LINE)?;
        if t.val == token![!] {
            return self.parse_exclam_line(cache);
        }
        self.parse_rule(cache)
    }

    fn parse_exclam_line<'a>(
        &mut self,
        cache: &'a mut ParserCache,
    ) -> Result<Spanned<Line<'a>>, Spanned<ParserError>> {
        self.pos += 1;
        let t = self.peek(AFTER_EXCLAM)?;
        match t.val {
            Token::Ident(i) => {
                let meaning = self.meaning_cache.get_case_sensitive(self.interner, i);
                if meaning == Meaning::Include {
                    self.parse_include()
                } else {
                    self.parse_mapping(cache)
                }
            }
            Token::GroupName(g) => self.parse_group_assignment(cache, g.spanned2(t.span)),
            Token::Punctuation(_) => Err(self.unexpected_token(AFTER_EXCLAM, t)),
        }
    }

    fn parse_group_assignment<'a>(
        &mut self,
        cache: &'a mut ParserCache,
        g: Spanned<Interned>,
    ) -> Result<Spanned<Line<'a>>, Spanned<ParserError>> {
        let lo = g.span.lo;
        self.pos += 1;
        let mut hi = self.consume_token(punctuation![=])?.hi;
        cache.group_assignments.clear();
        while self.pos < self.tokens.len() {
            let ident = self.parse_any_ident()?;
            hi = ident.span.hi;
            cache.group_assignments.push(ident.val);
        }
        let line: Line<'a> = Line::Macro(g.val, &cache.group_assignments);
        Ok(line.spanned(lo, hi))
    }

    fn parse_include<'a>(&mut self) -> Result<Spanned<Line<'a>>, Spanned<ParserError>> {
        self.pos += 1;
        let ident = self.parse_any_ident()?;
        if let Some(t) = self.try_peek() {
            return Err(self.expected_eof(t));
        }
        let val = self.interner.get(ident.val);
        if !val.contains(&b'%') {
            return Ok(ident.map(Line::Include));
        }
        let mut res = vec![];
        let mut last_was_percent = false;
        for (offset, c) in val.as_bytes().iter().copied().enumerate() {
            if last_was_percent {
                last_was_percent = false;
                let lo = ident.span.lo + offset as u64 - 1;
                let hi = lo + 2;
                match c {
                    b'H' => {
                        static HOME: LazyLock<Option<Vec<u8>>> = LazyLock::new(|| {
                            std::env::var_os("HOME").map(|v| v.into_encoded_bytes())
                        });
                        match &*HOME {
                            None => {
                                self.diagnostics.push(
                                    self.map,
                                    DiagnosticKind::UnknownRulesEscapeSequence,
                                    ad_hoc_display!("$HOME is not set").spanned(lo, hi),
                                );
                                res.push(b'%');
                                res.push(c);
                            }
                            Some(h) => {
                                res.extend_from_slice(h);
                            }
                        }
                    }
                    b'S' => {
                        res.extend_from_slice(b"/usr/share/X11/xkb/rules");
                        res.push(c);
                    }
                    b'E' => {
                        res.extend_from_slice(b"/etc/xkb/rules");
                        res.push(c);
                    }
                    _ => {
                        self.diagnostics.push(
                            self.map,
                            DiagnosticKind::UnknownRulesEscapeSequence,
                            ad_hoc_display!("unknown escape sequence %{}", c as char => char)
                                .spanned(lo, hi),
                        );
                        res.push(b'%');
                        res.push(c);
                    }
                }
            } else if c == b'%' {
                last_was_percent = true;
            } else {
                res.push(c);
            }
        }
        let code = Code::new(&Arc::new(res));
        let interned = self.interner.intern(&code.to_slice());
        Ok(Line::Include(interned).spanned2(ident.span))
    }

    fn parse_mapping<'a>(
        &mut self,
        cache: &'a mut ParserCache,
    ) -> Result<Spanned<Line<'a>>, Spanned<ParserError>> {
        let lo = self.peek(RULE_KEY)?.span.lo;
        let mut hi;
        cache.mapping_keys.clear();
        cache.mapping_values.clear();
        loop {
            let t = self.next(MAPPING_KEY)?;
            hi = t.span.hi;
            match t.val {
                token![=] => break,
                Token::Ident(i) => {
                    let key = self.interner.get(i);
                    let key = match key.as_bytes() {
                        b"model" => MappingKey::Model,
                        b"option" => MappingKey::Option,
                        _ => {
                            let parse_suffix = |prefix: &[u8], suffix: &[u8]| {
                                let offset = prefix.len();
                                if suffix.is_empty() {
                                    return Ok(None);
                                }
                                let Some(value) = suffix.strip_prefix(b"[") else {
                                    return Err(self.index_start(i.spanned2(t.span), offset));
                                };
                                let Some(value) = value.strip_suffix(b"]") else {
                                    return Err(self.index_end(i.spanned2(t.span)));
                                };
                                let idx = match value {
                                    b"single" => MappingKeyIndex::Single,
                                    b"first" => MappingKeyIndex::First,
                                    b"later" => MappingKeyIndex::Later,
                                    b"any" => MappingKeyIndex::Any,
                                    _ => {
                                        let Some(value) = u32::from_bytes_dec(value) else {
                                            return Err(self.index(i.spanned2(t.span), offset));
                                        };
                                        let Some(idx) = GroupIdx::new(value) else {
                                            return Err(self.index(i.spanned2(t.span), offset));
                                        };
                                        MappingKeyIndex::Value(idx)
                                    }
                                };
                                Ok(Some(idx))
                            };
                            let layout = b"layout";
                            let variant = b"variant";
                            if let Some(suffix) = key.strip_prefix(layout) {
                                MappingKey::Layout(parse_suffix(layout, suffix)?)
                            } else if let Some(suffix) = key.strip_prefix(variant) {
                                MappingKey::Variant(parse_suffix(variant, suffix)?)
                            } else {
                                return Err(self.unexpected_token(MAPPING_KEY, t));
                            }
                        }
                    };
                    cache.mapping_keys.push(key);
                }
                _ => return Err(self.unexpected_token(MLVO, t)),
            }
        }
        while self.pos < self.tokens.len() {
            let ident = self.parse_ident(MAPPING_VALUE)?;
            hi = ident.span.hi;
            let meaning = self
                .meaning_cache
                .get_case_sensitive(self.interner, ident.val);
            macro_rules! map {
                ($($ident:ident,)* => $($tt:tt)*) => {
                    match meaning {
                        $(Meaning::$ident => MappingValue::$ident,)*
                        _ => $($tt)*
                    }
                };
            }
            let val = map! {
                Keycodes,
                Symbols,
                Types,
                Compat,
                Geometry,
                => return Err(self.unexpected_token(MAPPING_VALUE, ident.map(Token::Ident))),
            };
            cache.mapping_values.push(val);
        }
        Ok(Line::Mapping(&cache.mapping_keys, &cache.mapping_values).spanned(lo, hi))
    }

    fn parse_rule<'a>(
        &mut self,
        cache: &'a mut ParserCache,
    ) -> Result<Spanned<Line<'a>>, Spanned<ParserError>> {
        cache.rule_keys.clear();
        cache.rule_values.clear();
        let lo = self.peek(RULE_KEY)?.span.lo;
        let mut hi;
        loop {
            let key = self.next(RULE_KEY)?;
            hi = key.span.hi;
            let key = match key.val {
                token![=] => break,
                token![*] => RuleKey::Star,
                Token::Ident(i) => RuleKey::Ident(i),
                Token::GroupName(i) => RuleKey::Macro(i),
                _ => return Err(self.unexpected_token(RULE_KEY, key)),
            };
            cache.rule_keys.push(key);
        }
        while self.pos < self.tokens.len() {
            let ident = self.parse_any_ident()?;
            hi = ident.span.hi;
            cache.rule_values.push(RuleValue { ident });
        }
        Ok(Line::Rule(&cache.rule_keys, &cache.rule_values).spanned(lo, hi))
    }
}
