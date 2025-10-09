pub(crate) mod error;
#[cfg(test)]
mod tests;

use {
    crate::{
        Keysym, ModifierMask,
        xkb::{
            code::Code,
            code_map::CodeMap,
            code_slice::CodeSlice,
            compose::{
                parser::error::{Expected, LHS, ParserError},
                token::Token,
            },
            context::Environment,
            diagnostic::{DiagnosticKind, DiagnosticSink},
            interner::Interner,
            meaning::{Meaning, MeaningCache},
            span::{SpanExt, SpanUnit, Spanned},
        },
    },
    isnt::std_1::primitive::IsntSliceExt,
    kbvm_proc::ad_hoc_display,
    std::sync::Arc,
};

struct Parser<'a, 'b, 'c> {
    map: &'a mut CodeMap,
    diagnostics: &'a mut DiagnosticSink<'b, 'c>,
    tokens: &'a [Spanned<Token>],
    interner: &'a mut Interner,
    meaning_cache: &'a mut MeaningCache,
    pos: usize,
    env: &'a Environment,
    locale_file: Option<&'a str>,
}

#[derive(Debug)]
pub(crate) enum Line {
    Include(CodeSlice<'static>),
    Production(Production),
}

#[derive(Debug)]
pub(crate) struct Production {
    pub(crate) steps: Vec<Step>,
    pub(crate) string: Option<CodeSlice<'static>>,
    pub(crate) keysym: Option<Keysym>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct Step {
    pub(crate) keysym: Keysym,
}

pub(crate) fn parse_line(
    map: &mut CodeMap,
    diagnostics: &mut DiagnosticSink,
    interner: &mut Interner,
    meaning_cache: &mut MeaningCache,
    tokens: &[Spanned<Token>],
    env: &Environment,
    locale_file: Option<&str>,
) -> Result<Option<Spanned<Line>>, Spanned<ParserError>> {
    let line = Parser {
        map,
        diagnostics,
        tokens,
        interner,
        meaning_cache,
        pos: 0,
        env,
        locale_file,
    }
    .parse_line()?;
    let lo = tokens.first().map(|t| t.span.lo).unwrap_or_default();
    let hi = tokens.last().map(|t| t.span.hi).unwrap_or_default();
    Ok(line.map(|l| l.spanned(lo, hi)))
}

impl Parser<'_, '_, '_> {
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

    fn parse_line(&mut self) -> Result<Option<Line>, Spanned<ParserError>> {
        let t = self.peek(LHS)?;
        if let Token::Ident(i) = t.val {
            let meaning = self.meaning_cache.get_case_insensitive(self.interner, i);
            if meaning == Meaning::Include {
                self.pos += 1;
                return self.parse_include();
            }
        }
        self.parse_production()
    }

    fn parse_include(&mut self) -> Result<Option<Line>, Spanned<ParserError>> {
        let t = self.next(&[Expected::AnyString])?;
        let Token::String(s) = t.val else {
            return Err(self.unexpected_token(&[Expected::AnyString], t));
        };
        let mut bytes = self.interner.get(s).to_owned();
        if bytes.contains(&b'%') {
            let mut out = vec![];
            let mut last_was_percent = false;
            for (offset, &b) in bytes.as_bytes().iter().enumerate() {
                if last_was_percent {
                    last_was_percent = false;
                    let lo = t.span.lo + offset as SpanUnit;
                    let hi = lo + 2;
                    match b {
                        b'%' => out.push(b'%'),
                        b'H' => {
                            if let Some(home) = &self.env.home {
                                out.extend_from_slice(home.as_bytes());
                            } else {
                                self.diagnostics.push(
                                    self.map,
                                    DiagnosticKind::HomeNotSet,
                                    ad_hoc_display!("$HOME is not set").spanned(lo, hi),
                                );
                                return Ok(None);
                            }
                        }
                        b'L' => {
                            if let Some(dir) = &self.locale_file {
                                out.extend_from_slice(dir.as_bytes());
                            } else {
                                self.diagnostics.push(
                                    self.map,
                                    DiagnosticKind::LocaleComposeFileNotResolved,
                                    ad_hoc_display!("%L could not be resolved").spanned(lo, hi),
                                );
                                return Ok(None);
                            }
                        }
                        b'S' => {
                            out.extend_from_slice(self.env.xlocaledir.as_bytes());
                        }
                        _ => {
                            self.diagnostics.push(
                                self.map,
                                DiagnosticKind::UnknownComposeIncludeEscape,
                                ad_hoc_display!("unknown escape sequence %{}", b as char => char)
                                    .spanned(lo, hi),
                            );
                            return Ok(None);
                        }
                    }
                } else if b == b'%' {
                    last_was_percent = true;
                } else {
                    out.push(b);
                }
            }
            bytes = Code::new(&Arc::new(out)).to_slice().to_owned();
        }
        if let Some(next) = self.try_peek() {
            return Err(self.expected_eol(next));
        }
        Ok(Some(Line::Include(bytes)))
    }

    fn parse_production(&mut self) -> Result<Option<Line>, Spanned<ParserError>> {
        let mut steps = vec![];
        loop {
            let (mask, _mods) = self.parse_modifiers()?;
            let token = self.next(LHS)?;
            if token.val == token![:] && mask.0 == 0 && steps.is_not_empty() {
                break;
            }
            let Token::Keysym(ks) = token.val else {
                return Err(self.unexpected_token(&[Expected::AnyKeysym], token));
            };
            let ks = self.interner.get(ks);
            let Some(ks) = Keysym::from_str(ks) else {
                return Err(self.unknown_keysym(ks, token.span));
            };
            steps.push(Step { keysym: ks });
        }
        let mut string = None;
        let mut keysym = None;
        let mut t = self.next(&[Expected::AnyString, Expected::AnyIdent])?;
        'rhs: {
            if let Token::String(s) = t.val {
                string = Some(self.interner.get(s).to_owned());
                match self.try_peek() {
                    Some(n) => {
                        self.pos += 1;
                        t = n
                    }
                    _ => break 'rhs,
                }
            }
            let Token::Ident(ks) = t.val else {
                return Err(self.unexpected_token(&[Expected::AnyIdent], t));
            };
            let ks = self.interner.get(ks);
            let Some(ks) = Keysym::from_str(ks) else {
                return Err(self.unknown_keysym(ks, t.span));
            };
            keysym = Some(ks);
        }
        if let Some(next) = self.try_peek() {
            return Err(self.expected_eol(next));
        }
        Ok(Some(Line::Production(Production {
            steps,
            string,
            keysym,
        })))
    }

    fn parse_modifiers(&mut self) -> Result<(ModifierMask, ModifierMask), Spanned<ParserError>> {
        let Some(t) = self.try_peek() else {
            return Ok((ModifierMask(0), ModifierMask(0)));
        };
        let mut mask = ModifierMask(0);
        if let Token::Ident(i) = t.val {
            let meaning = self.meaning_cache.get_case_insensitive(self.interner, i);
            if meaning == Meaning::None {
                self.pos += 1;
                return Ok((ModifierMask(!0), ModifierMask(0)));
            }
        }
        if t.val == token![!] {
            mask.0 = !0;
            self.pos += 1;
        }
        let mut mods = ModifierMask(0);
        while let Some(mut t) = self.try_peek() {
            let mut tilde = false;
            if t.val == token![~] {
                self.pos += 1;
                tilde = true;
                t = self.peek(&[Expected::AnyModifier])?;
            }
            if let Token::Ident(i) = t.val {
                let meaning = self.meaning_cache.get_case_insensitive(self.interner, i);
                let mm = match meaning {
                    Meaning::Ctrl => ModifierMask::CONTROL,
                    Meaning::Caps | Meaning::Lock => ModifierMask::LOCK,
                    Meaning::Shift => ModifierMask::SHIFT,
                    Meaning::Alt | Meaning::Meta => ModifierMask::MOD1,
                    _ => return Err(self.unexpected_token(&[Expected::AnyModifier], t)),
                };
                mask |= mm;
                if tilde {
                    mods &= !mm;
                } else {
                    mods |= mm;
                }
                self.pos += 1;
            } else if tilde {
                return Err(self.unexpected_token(&[Expected::AnyModifier], t));
            } else {
                break;
            }
        }
        Ok((mask, mods))
    }
}
