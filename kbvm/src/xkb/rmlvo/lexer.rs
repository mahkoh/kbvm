#[cfg(test)]
mod tests;

use {
    crate::xkb::{
        code::Code,
        code_slice::CodeSlice,
        diagnostic::DiagnosticKind,
        interner::Interner,
        rmlvo::token::Token,
        span::{SpanExt, SpanUnit, Spanned},
    },
    std::{path::PathBuf, sync::Arc},
    thiserror::Error,
};

#[derive(Debug)]
pub(crate) struct Lexer {
    path: Arc<PathBuf>,
    code: Code,
    span_lo: SpanUnit,
    pos: usize,
}

struct LineLexer<'a> {
    code: CodeSlice<'a>,
    interner: &'a mut Interner,
    span_lo: SpanUnit,
    pos: usize,
}

#[derive(Debug, Error, Eq, PartialEq)]
pub(crate) enum LexerError {
    #[error("empty macro name")]
    EmptyMacroName,
    #[error("Unexpected byte {:?}", *.0 as char)]
    UnexpectedByte(u8),
}

impl LexerError {
    pub(crate) fn diagnostic_kind(&self) -> DiagnosticKind {
        match self {
            LexerError::EmptyMacroName => DiagnosticKind::EmptyMacroName,
            LexerError::UnexpectedByte(_) => DiagnosticKind::UnlexableByte,
        }
    }
}

enum One {
    Eof,
    Eol,
    Token(Spanned<Token>),
}

impl Lexer {
    pub(crate) fn new(path: &Arc<PathBuf>, code: &Code, span_lo: SpanUnit) -> Self {
        Self {
            path: path.clone(),
            code: code.clone(),
            span_lo,
            pos: 0,
        }
    }

    pub(crate) fn lex_line(
        &mut self,
        interner: &mut Interner,
        output: &mut Vec<Spanned<Token>>,
    ) -> Result<(), Spanned<LexerError>> {
        let mut lexer = LineLexer {
            code: self.code.to_slice(),
            interner,
            span_lo: self.span_lo,
            pos: self.pos,
        };
        lexer.lex_line(output)?;
        self.pos = lexer.pos;
        Ok(())
    }

    pub(crate) fn path(&self) -> &Arc<PathBuf> {
        &self.path
    }
}

impl LineLexer<'_> {
    fn lex_line(&mut self, output: &mut Vec<Spanned<Token>>) -> Result<(), Spanned<LexerError>> {
        let mut lexed_any = false;
        loop {
            match self.lex_one()? {
                One::Eof => break,
                One::Eol if lexed_any => break,
                One::Eol => {}
                One::Token(t) => {
                    lexed_any = true;
                    output.push(t);
                }
            }
        }
        Ok(())
    }

    fn lex_one(&mut self) -> Result<One, Spanned<LexerError>> {
        use LexerError::*;
        let mut b;
        loop {
            while self.pos < self.code.len() {
                if matches!(self.code[self.pos], b' ' | b'\t' | b'\r') {
                    self.pos += 1;
                } else {
                    break;
                }
            }
            b = match self.code.get(self.pos) {
                Some(c) => *c,
                _ => return Ok(One::Eof),
            };
            match b {
                b'/' if self.code.get(self.pos + 1) == Some(&b'/') => {
                    self.pos += 2;
                    while self.pos < self.code.len() {
                        b = self.code[self.pos];
                        self.pos += 1;
                        if b == b'\n' {
                            break;
                        }
                    }
                    return Ok(One::Eol);
                }
                b'\\' => {
                    self.pos += 1;
                    if self.code.get(self.pos) == Some(&b'\r') {
                        self.pos += 1;
                    }
                    if let Some(&b) = self.code.get(self.pos) {
                        if b != b'\n' {
                            let lo = self.span_lo + self.pos as SpanUnit;
                            return Err(UnexpectedByte(b).spanned(lo, lo + 1));
                        }
                        self.pos += 1;
                    }
                }
                _ => break,
            }
        }
        let mut start = self.pos;
        let lo = self.span_lo + start as SpanUnit;
        self.pos += 1;
        'punctuation: {
            let t = match b {
                b'\n' => return Ok(One::Eol),
                b'=' => token![=],
                b'*' => token![*],
                b'!' => token![!],
                _ => break 'punctuation,
            };
            return Ok(One::Token(t.spanned(lo, lo + 1)));
        }
        let mut is_macro = false;
        if b == b'$' {
            b = match self.code.get(self.pos) {
                Some(b) => *b,
                _ => return Err(EmptyMacroName.spanned(lo, lo + 1)),
            };
            is_macro = true;
            self.pos += 1;
            start += 1;
        }
        self.pos -= 1;
        while b >= b'!' && b <= b'~' && b != b'\\' {
            self.pos += 1;
            b = match self.code.get(self.pos) {
                None => break,
                Some(b) => *b,
            };
        }
        if start == self.pos {
            let lo = self.span_lo.saturating_add(self.pos as SpanUnit);
            return Err(UnexpectedByte(b).spanned(lo, lo.saturating_add(1)));
        }
        let end = self.pos;
        let value = self.interner.intern(&self.code.slice(start..end));
        let token = match is_macro {
            true => Token::GroupName(value),
            false => Token::Ident(value),
        };
        let hi = self.span_lo.saturating_add(self.pos as SpanUnit);
        Ok(One::Token(token.spanned(lo, hi)))
    }
}
