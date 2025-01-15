#[cfg(test)]
mod tests;

use {
    crate::xkb::{
        code::Code,
        code_map::CodeMap,
        code_slice::CodeSlice,
        compose::token::Token,
        diagnostic::{DiagnosticKind, DiagnosticSink},
        interner::Interner,
        span::{SpanExt, Spanned},
    },
    kbvm_proc::ad_hoc_display,
    std::sync::Arc,
    thiserror::Error,
};

#[derive(Debug)]
pub(crate) struct Lexer {
    code: Code,
    span_lo: u64,
    pos: usize,
    str_buf: Vec<u8>,
}

struct LineLexer<'a, 'b, 'c> {
    code: CodeSlice<'a>,
    map: &'a mut CodeMap,
    diagnostics: &'a mut DiagnosticSink<'b, 'c>,
    interner: &'a mut Interner,
    span_lo: u64,
    pos: usize,
    str_buf: &'a mut Vec<u8>,
}

#[derive(Debug, Error, Eq, PartialEq)]
pub(crate) enum LexerError {
    #[error("unterminated keysym")]
    UnterminatedKeysym,
    #[error("unterminated string")]
    UnterminatedString,
    #[error("Unexpected byte {:?}", *.0 as char)]
    UnexpectedByte(u8),
}

impl LexerError {
    pub(crate) fn diagnostic_kind(&self) -> DiagnosticKind {
        match self {
            LexerError::UnexpectedByte(_) => DiagnosticKind::UnlexableByte,
            LexerError::UnterminatedKeysym => DiagnosticKind::UnterminatedKeysym,
            LexerError::UnterminatedString => DiagnosticKind::UnterminatedString,
        }
    }
}

enum One {
    Eof,
    Eol,
    Token(Spanned<Token>),
}

impl Lexer {
    pub(crate) fn new(code: &Code, span_lo: u64) -> Self {
        Self {
            code: code.clone(),
            span_lo,
            pos: 0,
            str_buf: Default::default(),
        }
    }

    pub(crate) fn lex_line(
        &mut self,
        map: &mut CodeMap,
        diagnostics: &mut DiagnosticSink<'_, '_>,
        interner: &mut Interner,
        output: &mut Vec<Spanned<Token>>,
    ) -> Result<(), Spanned<LexerError>> {
        let mut lexer = LineLexer {
            code: self.code.to_slice(),
            map,
            diagnostics,
            interner,
            span_lo: self.span_lo,
            pos: self.pos,
            str_buf: &mut self.str_buf,
        };
        lexer.lex_line(output)?;
        self.pos = lexer.pos;
        Ok(())
    }
}

impl LineLexer<'_, '_, '_> {
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
            if b == b'#' {
                self.pos += 1;
                while self.pos < self.code.len() {
                    b = self.code[self.pos];
                    self.pos += 1;
                    if b == b'\n' {
                        break;
                    }
                }
                return Ok(One::Eol);
            } else {
                break;
            }
        }
        let mut start = self.pos;
        self.pos += 1;
        let lo = self.span_lo + start as u64;
        'punctuation: {
            let t = match b {
                b'\n' => return Ok(One::Eol),
                b'~' => token![~],
                b':' => token![:],
                b'!' => token![!],
                _ => break 'punctuation,
            };
            return Ok(One::Token(t.spanned(lo, lo + 1)));
        }
        if b == b'<' {
            start += 1;
            while self.pos < self.code.len() {
                b = self.code[self.pos];
                self.pos += 1;
                match b {
                    b'\n' => {
                        let hi = self.span_lo + self.pos as u64 - 1;
                        return Err(UnterminatedKeysym.spanned(lo, hi));
                    }
                    b'>' => {
                        let hi = self.span_lo + self.pos as u64;
                        let end = self.pos - 1;
                        let value = self.interner.intern(&self.code.slice(start..end));
                        return Ok(One::Token(Token::Keysym(value).spanned(lo, hi)));
                    }
                    _ => {}
                }
            }
            let hi = self.span_lo + self.pos as u64;
            return Err(UnterminatedKeysym.spanned(lo, hi));
        }
        if b == b'"' {
            start += 1;
            self.str_buf.clear();
            let mut has_escape = false;
            while self.pos < self.code.len() {
                b = self.code[self.pos];
                self.pos += 1;
                match b {
                    b'\n' => {
                        let hi = self.span_lo + self.pos as u64;
                        return Err(UnterminatedString.spanned(lo, hi));
                    }
                    b'\\' => {
                        has_escape = true;
                        if self.pos >= self.code.len() {
                            continue;
                        }
                        b = self.code[self.pos];
                        self.pos += 1;
                        match b {
                            b'\\' => self.str_buf.push(b'\\'),
                            b'"' => self.str_buf.push(b'"'),
                            b'x' | b'X' => {
                                let mut c = 0;
                                for _ in 0..2 {
                                    if self.pos >= self.code.len() {
                                        break;
                                    }
                                    b = self.code[self.pos];
                                    match b {
                                        b'0'..=b'9' => c = c << 4 | (b - b'0'),
                                        b'a'..=b'f' => c = c << 4 | (b - b'a' + 10),
                                        b'A'..=b'F' => c = c << 4 | (b - b'A' + 10),
                                        _ => break,
                                    }
                                    self.pos += 1;
                                }
                                self.str_buf.push(c);
                            }
                            b'0'..=b'7' => {
                                let lo = self.span_lo + self.pos as u64 - 1;
                                let mut c = (b - b'0') as u16;
                                for _ in 0..2 {
                                    if self.pos >= self.code.len() {
                                        break;
                                    }
                                    b = self.code[self.pos];
                                    match b {
                                        b'0'..=b'7' => c = (c << 3) | (b - b'0') as u16,
                                        _ => break,
                                    }
                                    self.pos += 1;
                                }
                                if c > u8::MAX as u16 {
                                    let hi = self.span_lo + self.pos as u64;
                                    self.diagnostics.push(
                                        &mut self.map,
                                        DiagnosticKind::OctalStringEscapeOverflow,
                                        ad_hoc_display!("octal string escape overflow")
                                            .spanned(lo, hi),
                                    );
                                } else {
                                    self.str_buf.push(c as u8);
                                }
                            }
                            _ => {
                                let lo = self.span_lo + self.pos as u64 - 2;
                                let hi = self.span_lo + self.pos as u64;
                                self.diagnostics.push(
                                    &mut self.map,
                                    DiagnosticKind::UnknownEscapeSequence,
                                    ad_hoc_display!("unknown escape sequence").spanned(lo, hi),
                                );
                                self.str_buf.push(b'\\');
                                self.str_buf.push(b);
                            }
                        }
                    }
                    b'"' => {
                        let slice = if has_escape {
                            let code = Code::new(&Arc::new(self.str_buf.to_vec()));
                            code.to_slice().to_owned()
                        } else {
                            let end = self.pos - 1;
                            self.code.slice(start..end).to_owned()
                        };
                        let value = self.interner.intern(&slice);
                        let hi = self.span_lo + self.pos as u64;
                        return Ok(One::Token(Token::String(value).spanned(lo, hi)));
                    }
                    _ => {
                        self.str_buf.push(b);
                    }
                }
            }
            let hi = self.span_lo + self.pos as u64;
            return Err(UnterminatedString.spanned(lo, hi));
        }
        if let b'a'..=b'z' | b'A'..=b'Z' | b'_' = b {
            while self.pos < self.code.len() {
                b = self.code[self.pos];
                if let b'a'..=b'z' | b'A'..=b'Z' | b'_' | b'0'..=b'9' = b {
                    self.pos += 1;
                } else {
                    break;
                }
            }
            let end = self.pos;
            let value = self.interner.intern(&self.code.slice(start..end));
            let hi = self.span_lo + self.pos as u64;
            return Ok(One::Token(Token::Ident(value).spanned(lo, hi)));
        }
        Err(UnexpectedByte(b).spanned(lo, lo + 1))
    }
}
