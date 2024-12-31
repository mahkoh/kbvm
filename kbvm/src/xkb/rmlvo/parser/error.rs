use {
    crate::xkb::{
        code_slice::CodeSlice,
        diagnostic::DiagnosticKind,
        interner::Interned,
        rmlvo::{
            parser::Parser,
            token::{Punctuation, Token},
        },
        span::{Span, SpanExt, Spanned},
    },
    bstr::ByteSlice,
    debug_fn::debug_fn,
    std::fmt::{self, Formatter},
    thiserror::Error,
};

#[derive(Debug, Clone, Error)]
pub(crate) enum ParserError {
    #[error(
        "expected {}, but encountered EOF",
        debug_fn(|f| write_expected(f, .0.expected)),
    )]
    ExpectedButEof(ExpectedButEof),
    #[error(
        "expected {}, but found {}",
        debug_fn(|f| write_expected(f, .0.expected)),
        debug_fn(|f| write_actual(f, &.0.actual)),
    )]
    UnexpectedToken(UnexpectedToken),
    #[error(
        "expected end of line, but found {}",
        debug_fn(|f| write_actual(f, .0)),
    )]
    ExpectedEol(ActualToken),
    #[error(
        "expected `[` but found `{}`",
        *.0 as char,
    )]
    IndexStart(u8),
    #[error(
        "expected `]` but found `{}`",
        *.0 as char,
    )]
    IndexEnd(u8),
    #[error(
        "expected `single`, `first`, `later`, `any`, or a group index, but found `{}`",
        .0.as_bytes().as_bstr()
    )]
    Index(CodeSlice<'static>),
}

impl ParserError {
    pub(crate) fn diagnostic_kind(&self) -> DiagnosticKind {
        match self {
            ParserError::ExpectedButEof(_) => DiagnosticKind::UnexpectedEof,
            ParserError::UnexpectedToken(_) => DiagnosticKind::UnexpectedToken,
            ParserError::ExpectedEol(_) => DiagnosticKind::ExpectedEol,
            ParserError::IndexStart(_) => DiagnosticKind::ExpectedIndexStart,
            ParserError::IndexEnd(_) => DiagnosticKind::ExpectedIndexEnd,
            ParserError::Index(_) => DiagnosticKind::InvalidMatcherIndex,
        }
    }
}

fn write_actual(f: &mut Formatter<'_>, actual: &ActualToken) -> fmt::Result {
    match actual {
        ActualToken::Ident(i) => write!(f, "`{}`", i.as_bytes().as_bstr()),
        ActualToken::GroupName(i) => write!(f, "`${}`", i.as_bytes().as_bstr()),
        ActualToken::Token(t) => match t {
            Token::Ident(_) => f.write_str("an identifier"),
            Token::GroupName(_) => f.write_str("a group name"),
            Token::Punctuation(p) => write!(f, "`{}`", punctuation_string(*p)),
        },
    }
}

fn write_expected(f: &mut Formatter<'_>, expected: &[Expected]) -> fmt::Result {
    if let Some(e) = get_unique_expected(expected) {
        return write_single_expected(f, e, false);
    }
    write_expected_(f, expected, true)
}

fn write_expected_(f: &mut Formatter<'_>, expected: &[Expected], or_prefix: bool) -> fmt::Result {
    let last = expected.len() - 1;
    for (idx, e) in expected.iter().enumerate() {
        if idx > 0 {
            f.write_str(", ")?;
        }
        if let Expected::Nested(n) = e {
            if idx == last {
                write_expected_(f, n, or_prefix)?;
                continue;
            }
        } else {
            if or_prefix && idx == last {
                f.write_str("or ")?;
            }
        }
        write_single_expected(f, e, false)?;
    }
    Ok(())
}

fn get_unique_expected(expected: &[Expected]) -> Option<&Expected> {
    if expected.len() != 1 {
        return None;
    }
    if let Expected::Nested(e) = &expected[0] {
        return get_unique_expected(e);
    }
    Some(&expected[0])
}

fn write_single_expected(
    f: &mut Formatter<'_>,
    expected: &Expected,
    or_prefix: bool,
) -> fmt::Result {
    match expected {
        Expected::Nested(n) => write_expected_(f, n, or_prefix),
        Expected::Ident(i) => write!(f, "`{}`", i),
        Expected::AnyIdent => f.write_str("an identifier"),
        Expected::AnyGroupName => f.write_str("a group name"),
        Expected::Punctuation(p) => {
            write!(f, "`{}`", punctuation_string(*p))
        }
    }
}

fn punctuation_string(p: Punctuation) -> &'static str {
    match p {
        Punctuation::Equals => "=",
        Punctuation::Times => "*",
        Punctuation::Exclam => "!",
    }
}

#[derive(Debug, Clone)]
pub(crate) struct ExpectedButEof {
    expected: &'static [Expected],
}

#[derive(Debug, Clone)]
pub(crate) struct UnexpectedToken {
    expected: &'static [Expected],
    actual: ActualToken,
}

#[derive(Debug)]
pub(crate) enum Expected {
    Nested(&'static [Expected]),
    Ident(&'static str),
    AnyIdent,
    AnyGroupName,
    Punctuation(Punctuation),
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum ActualToken {
    Ident(CodeSlice<'static>),
    GroupName(CodeSlice<'static>),
    Token(Token),
}

impl Parser<'_, '_> {
    pub(super) fn expected_but_eof(
        &self,
        span: Span,
        expected: &'static [Expected],
    ) -> Spanned<ParserError> {
        ParserError::ExpectedButEof(ExpectedButEof { expected }).spanned2(span)
    }

    fn token_to_actual(&self, token: Spanned<Token>) -> ActualToken {
        match token.val {
            Token::Ident(i) => ActualToken::Ident(self.interner.get(i).to_owned()),
            Token::GroupName(i) => ActualToken::GroupName(self.interner.get(i).to_owned()),
            _ => ActualToken::Token(token.val),
        }
    }

    pub(super) fn expected_eol(&self, token: Spanned<Token>) -> Spanned<ParserError> {
        let actual = self.token_to_actual(token);
        ParserError::ExpectedEol(actual).spanned2(token.span)
    }

    pub(super) fn unexpected_token(
        &self,
        expected: &'static [Expected],
        token: Spanned<Token>,
    ) -> Spanned<ParserError> {
        let actual = self.token_to_actual(token);
        ParserError::UnexpectedToken(UnexpectedToken { expected, actual }).spanned2(token.span)
    }

    pub(super) fn index_start(
        &self,
        ident: Spanned<Interned>,
        offset: usize,
    ) -> Spanned<ParserError> {
        let slice = self.interner.get(ident.val).to_owned();
        let lo = ident.span.lo + offset as u64;
        ParserError::IndexStart(slice[offset]).spanned(lo, lo + 1)
    }

    pub(super) fn index_end(&self, ident: Spanned<Interned>) -> Spanned<ParserError> {
        let slice = self.interner.get(ident.val).to_owned();
        let hi = ident.span.hi;
        ParserError::IndexEnd(*slice.last().unwrap()).spanned(hi - 1, hi)
    }

    pub(super) fn index(&self, ident: Spanned<Interned>, offset: usize) -> Spanned<ParserError> {
        let actual = self.interner.get(ident.val);
        let actual = actual.slice(offset + 1..actual.len() - 1);
        let lo = ident.span.lo + offset as u64 + 1;
        let hi = ident.span.hi - 1;
        ParserError::Index(actual.to_owned()).spanned(lo, hi)
    }
}

pub(super) const START_OF_LINE: &[Expected] = &[
    Expected::Punctuation(punctuation![!]),
    Expected::Punctuation(punctuation![*]),
    Expected::AnyIdent,
];

pub(super) const AFTER_EXCLAM: &[Expected] = &[
    Expected::Ident("include"),
    Expected::Punctuation(punctuation![*]),
    Expected::Nested(MLVO),
];

pub(super) const RULE_KEY: &[Expected] = &[
    Expected::Punctuation(punctuation![*]),
    Expected::Punctuation(punctuation![=]),
    Expected::AnyIdent,
    Expected::AnyGroupName,
];

pub(super) const MLVO: &[Expected] = &[
    Expected::Ident("model"),
    Expected::Ident("layout"),
    Expected::Ident("variant"),
    Expected::Ident("option"),
];

pub(super) const MAPPING_KEY: &[Expected] = &[
    Expected::Nested(MLVO),
    Expected::Punctuation(punctuation![=]),
];

pub(super) const MAPPING_VALUE: &[Expected] = &[
    Expected::Ident("keycodes"),
    Expected::Ident("symbols"),
    Expected::Ident("types"),
    Expected::Ident("compat"),
    Expected::Ident("geometry"),
];
