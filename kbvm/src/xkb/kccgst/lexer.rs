#[cfg(test)]
mod tests;

use {
    crate::{
        from_bytes::FromBytes,
        xkb::{
            code::Code,
            code_slice::CodeSlice,
            interner::Interner,
            kccgst::token::{
                Punctuation,
                Token::{self, Float, Ident, Integer, KeyName, String},
            },
            span::{Span, SpanExt, Spanned},
        },
    },
    std::{num::ParseFloatError, path::PathBuf, str::FromStr, sync::Arc},
    thiserror::Error,
};

#[derive(Debug)]
pub struct Lexer {
    path: Option<Arc<PathBuf>>,
    code: Code,
    span_lo: u64,
    pos: usize,
}

struct ItemLexer<'a> {
    code: CodeSlice<'a>,
    interner: &'a mut Interner,
    span_lo: u64,
    pos: usize,
}

#[derive(Debug, Error, Eq, PartialEq)]
pub enum LexerError {
    #[error("Unterminated key name")]
    UnterminatedKeyName,
    #[error("Unterminated String")]
    UnterminatedString,
    #[error("Invalid float literal")]
    InvalidFloatLiteral(#[source] ParseFloatError),
    #[error("Invalid integer literal")]
    InvalidIntegerLiteral,
    #[error("Unexpected by {0:?}")]
    UnexpectedByte(u8),
}

impl Lexer {
    pub fn new(path: Option<&Arc<PathBuf>>, code: &Code, span_lo: u64) -> Self {
        Self {
            path: path.cloned(),
            code: code.clone(),
            span_lo,
            pos: 0,
        }
    }

    pub fn lex_item(
        &mut self,
        interner: &mut Interner,
        output: &mut Vec<Spanned<Token>>,
    ) -> Result<(), Spanned<LexerError>> {
        let mut lexer = ItemLexer {
            code: self.code.to_slice(),
            interner,
            span_lo: self.span_lo,
            pos: self.pos,
        };
        lexer.lex_item(output)?;
        self.pos = lexer.pos;
        Ok(())
    }

    pub fn path(&self) -> Option<&Arc<PathBuf>> {
        self.path.as_ref()
    }

    pub fn code(&self) -> &Code {
        &self.code
    }

    pub fn span(&self) -> Span {
        Span {
            lo: self.span_lo,
            hi: self.span_lo + self.code.len() as u64,
        }
    }
}

impl<'a> ItemLexer<'a> {
    fn lex_item(&mut self, output: &mut Vec<Spanned<Token>>) -> Result<(), Spanned<LexerError>> {
        let mut depth = 0u64;
        while let Some(t) = self.lex_one()? {
            output.push(t);
            if let Token::Punctuation(p) = t.val {
                match p {
                    Punctuation::Obrace => depth += 1,
                    Punctuation::Cbrace if depth == 0 => {}
                    Punctuation::Cbrace => depth -= 1,
                    punctuation![;] if depth == 0 => break,
                    _ => {}
                }
            }
        }
        Ok(())
    }

    fn lex_one(&mut self) -> Result<Option<Spanned<Token>>, Spanned<LexerError>> {
        use LexerError::*;
        let mut b;
        loop {
            while self.pos < self.code.len() {
                if matches!(self.code[self.pos], b' ' | b'\t'..=b'\r') {
                    self.pos += 1;
                } else {
                    break;
                }
            }
            match self.code.get(self.pos) {
                Some(c) => b = *c,
                _ => return Ok(None),
            };
            let is_comment = match b {
                b'#' => true,
                b'/' if self.code.get(self.pos + 1) == Some(&b'/') => true,
                _ => false,
            };
            if is_comment {
                while self.pos < self.code.len() {
                    if self.code[self.pos] == b'\n' {
                        break;
                    }
                    self.pos += 1;
                }
            } else {
                break;
            }
        }
        let start = self.pos;
        let lo = self.span_lo + start as u64;
        self.pos += 1;
        'single_character: {
            let t = match b {
                b';' => token![;],
                b'{' => Punctuation::Obrace.into(),
                b'}' => Punctuation::Cbrace.into(),
                b'=' => token![=],
                b'[' => Punctuation::Obracket.into(),
                b']' => Punctuation::Cbracket.into(),
                b'(' => Punctuation::Oparen.into(),
                b')' => Punctuation::Cparen.into(),
                b'.' => token![.],
                b',' => token![,],
                b'+' => token![+],
                b'-' => token![-],
                b'*' => token![*],
                b'/' => token![/],
                b'!' => token![!],
                b'~' => token![~],
                _ => break 'single_character,
            };
            return Ok(Some(t.spanned(lo, lo + 1)));
        }
        let next = |err: LexerError, pos: usize| match self.code.get(pos) {
            Some(c) => Ok(*c),
            _ => {
                let hi = self.span_lo + pos as u64;
                Err(err.spanned(lo, hi))
            }
        };
        if b == b'<' {
            loop {
                b = next(UnterminatedKeyName, self.pos)?;
                self.pos += 1;
                match b {
                    b'>' => break,
                    b'!'..=b'~' => {}
                    _ => {
                        let hi = self.span_lo + self.pos as u64 - 1;
                        return Err(UnterminatedKeyName.spanned(lo, hi));
                    }
                }
            }
            let start = start + 1;
            let end = self.pos - 1;
            let slice = self.code.slice(start..end);
            let interned = self.interner.intern(&slice);
            let hi = self.span_lo + self.pos as u64;
            return Ok(Some(KeyName(interned).spanned(lo, hi)));
        }
        if b == b'"' {
            let mut esc = false;
            loop {
                b = next(UnterminatedString, self.pos)?;
                self.pos += 1;
                match b {
                    b'"' if !esc => break,
                    b'\\' if !esc => esc = true,
                    _ => esc = false,
                }
            }
            let start = start + 1;
            let end = self.pos - 1;
            let slice = self.code.slice(start..end);
            let interned = self.interner.intern(&slice);
            let hi = self.span_lo + self.pos as u64;
            return Ok(Some(String(interned).spanned(lo, hi)));
        }
        if matches!(b, b'_' | b'a'..=b'z' | b'A'..=b'Z') {
            while self.pos < self.code.len() {
                b = self.code[self.pos];
                if matches!(b, b'_' | b'a'..=b'z' | b'A'..=b'Z' | b'0'..=b'9') {
                    self.pos += 1;
                } else {
                    break;
                }
            }
            let end = self.pos;
            let slice = self.code.slice(start..end);
            let interned = self.interner.intern(&slice);
            let hi = self.span_lo + end as u64;
            return Ok(Some(Ident(interned).spanned(lo, hi)));
        };
        if matches!(b, b'0'..=b'9') {
            let mut hex = false;
            let mut digits_start = start;
            if b == b'0' && self.code.get(self.pos) == Some(&b'x') {
                hex = true;
                self.pos += 1;
                digits_start += 2;
            }
            let mut have_dot = false;
            while self.pos < self.code.len() {
                match self.code[self.pos] {
                    b'0'..=b'9' => {}
                    b'a'..=b'f' | b'A'..=b'F' if hex => {}
                    b'.' if !have_dot && !hex => have_dot = true,
                    _ => break,
                }
                self.pos += 1;
            }
            let end = self.pos;
            let slice = self.code.slice(start..end);
            let interned = self.interner.intern(&slice);
            let hi = self.span_lo + end as u64;
            let token = match have_dot {
                true => {
                    let digits = std::str::from_utf8(&self.code[digits_start..end]).unwrap();
                    match f64::from_str(digits) {
                        Ok(f) => Float(interned, f),
                        Err(e) => return Err(InvalidFloatLiteral(e).spanned(lo, hi)),
                    }
                }
                false => match i64::from_bytes(&self.code[digits_start..end], hex) {
                    Some(f) => Integer(interned, f),
                    _ => return Err(InvalidIntegerLiteral.spanned(lo, hi)),
                },
            };
            return Ok(Some(token.spanned(lo, hi)));
        }
        let hi = self.span_lo + self.pos as u64;
        Err(UnexpectedByte(b).spanned(lo, hi))
    }
}
