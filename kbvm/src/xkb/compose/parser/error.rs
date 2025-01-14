use {
    crate::xkb::{
        code_slice::CodeSlice,
        compose::{
            parser::Parser,
            token::{Punctuation, Token},
        },
        diagnostic::DiagnosticKind,
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
        "unknown keysym {}",
        .0.as_bstr(),
    )]
    UnknownKeysym(CodeSlice<'static>),
    #[error("rule has no conditions")]
    NoSteps,
}

impl ParserError {
    pub(crate) fn diagnostic_kind(&self) -> DiagnosticKind {
        match self {
            ParserError::ExpectedButEof(_) => DiagnosticKind::UnexpectedEof,
            ParserError::UnexpectedToken(_) => DiagnosticKind::UnexpectedToken,
            ParserError::ExpectedEol(_) => DiagnosticKind::ExpectedEol,
            ParserError::UnknownKeysym(_) => DiagnosticKind::UnknownKeysym,
            ParserError::NoSteps => DiagnosticKind::ComposeRuleWithoutConditions,
        }
    }
}

fn write_actual(f: &mut Formatter<'_>, actual: &ActualToken) -> fmt::Result {
    match actual {
        ActualToken::Ident(i) => write!(f, "`{}`", i.as_bytes().as_bstr()),
        ActualToken::String(i) => write!(f, "{:?}", i.as_bytes().as_bstr()),
        ActualToken::Keysym(i) => write!(f, "<{}>", i.as_bytes().as_bstr()),
        ActualToken::Token(t) => match t {
            Token::Ident(_) => f.write_str("an identifier"),
            Token::Punctuation(p) => write!(f, "`{}`", punctuation_string(*p)),
            Token::String(_) => f.write_str("a string"),
            Token::Keysym(_) => f.write_str("a keysym"),
        },
    }
}

fn write_expected(f: &mut Formatter<'_>, expected: &[Expected]) -> fmt::Result {
    if let Some(e) = get_unique_expected(expected) {
        return write_single_expected(f, e);
    }
    write_expected_(f, expected, true)
}

fn write_expected_(f: &mut Formatter<'_>, expected: &[Expected], or_prefix: bool) -> fmt::Result {
    let last = expected.len() - 1;
    for (idx, e) in expected.iter().enumerate() {
        if idx > 0 {
            f.write_str(", ")?;
        }
        if or_prefix && idx == last {
            f.write_str("or ")?;
        }
        write_single_expected(f, e)?;
    }
    Ok(())
}

fn get_unique_expected(expected: &[Expected]) -> Option<&Expected> {
    if expected.len() != 1 {
        return None;
    }
    Some(&expected[0])
}

fn write_single_expected(f: &mut Formatter<'_>, expected: &Expected) -> fmt::Result {
    match expected {
        Expected::AnyIdent => f.write_str("an identifier"),
        Expected::AnyString => f.write_str("a string"),
        Expected::AnyModifier => f.write_str("a modifier"),
        Expected::AnyKeysym => f.write_str("a keysym"),
        Expected::Punctuation(p) => {
            write!(f, "`{}`", punctuation_string(*p))
        }
    }
}

fn punctuation_string(p: Punctuation) -> &'static str {
    match p {
        Punctuation::Exclam => "!",
        Punctuation::Tilde => "~",
        Punctuation::Colon => ":",
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
    AnyIdent,
    AnyString,
    AnyModifier,
    AnyKeysym,
    Punctuation(Punctuation),
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum ActualToken {
    Ident(CodeSlice<'static>),
    String(CodeSlice<'static>),
    Keysym(CodeSlice<'static>),
    Token(Token),
}

impl Parser<'_, '_, '_> {
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
            Token::Keysym(i) => ActualToken::Keysym(self.interner.get(i).to_owned()),
            Token::String(i) => ActualToken::String(self.interner.get(i).to_owned()),
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

    pub(super) fn unknown_keysym(&self, code: &CodeSlice<'_>, span: Span) -> Spanned<ParserError> {
        ParserError::UnknownKeysym(code.to_owned()).spanned2(span)
    }

    pub(super) fn no_steps(&self, span: Span) -> Spanned<ParserError> {
        ParserError::NoSteps.spanned2(span)
    }
}

pub(super) const LHS: &[Expected] = &[
    Expected::Punctuation(punctuation![!]),
    Expected::Punctuation(punctuation![~]),
    Expected::Punctuation(punctuation![:]),
    Expected::AnyKeysym,
    Expected::AnyModifier,
];
