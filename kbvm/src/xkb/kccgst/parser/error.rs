use {
    crate::xkb::{
        code_slice::CodeSlice,
        diagnostic::DiagnosticKind,
        interner::Interned,
        kccgst::{
            parser::{DeclCandidate, Parser},
            token::{
                Punctuation::{self, Cbrace, Obrace, Obracket},
                Token,
            },
        },
        meaning::Meaning,
        span::{Span, SpanExt, Spanned},
    },
    bstr::ByteSlice,
    debug_fn::debug_fn,
    std::fmt::{self, Formatter},
    thiserror::Error,
    Punctuation::Oparen,
};

#[derive(Debug, Clone, Error)]
pub(crate) enum ParserError {
    #[error("expression is too deeply nested")]
    TooDeeplyNested,
    #[error(
        "expected {}, but encountered EOF",
        debug_fn(|f| write_expected(f, .0.expected)),
    )]
    ExpectedButEof(ExpectedButEof),
    #[error(
        "expected {}, but found {}",
        debug_fn(|f| write_expected(f, .0.expected)),
        debug_fn(|f| write_decl_candidate(f, .0.actual)),
    )]
    UnexpectedDeclCandidate(UnexpectedDeclCandidate),
    #[error(
        "expected {}, but found {}",
        debug_fn(|f| write_expected(f, .0.expected)),
        debug_fn(|f| write_actual(f, &.0.actual)),
    )]
    UnexpectedToken(UnexpectedToken),
    #[error(
        "could not parse integer as u32: `{}`",
        .0.slice.as_bytes().as_bstr(),
    )]
    InvalidU32(InvalidU32),
}

impl ParserError {
    pub(crate) fn diagnostic_kind(&self) -> DiagnosticKind {
        match self {
            ParserError::TooDeeplyNested => DiagnosticKind::TooDeeplyNested,
            ParserError::ExpectedButEof(_) => DiagnosticKind::UnexpectedEof,
            ParserError::UnexpectedDeclCandidate(_) => DiagnosticKind::UnexpectedDeclaration,
            ParserError::UnexpectedToken(_) => DiagnosticKind::UnexpectedToken,
            ParserError::InvalidU32(_) => DiagnosticKind::U32Overflow,
        }
    }
}

fn write_decl_candidate(f: &mut Formatter<'_>, actual: DeclCandidate) -> fmt::Result {
    let name = match actual {
        DeclCandidate::None => "an unrecognized statement",
        DeclCandidate::Doodad(_) => "a doodad declaration",
        DeclCandidate::GroupCompat => "a group compat statement",
        DeclCandidate::Include(..) => "an include statement",
        DeclCandidate::Interpret => "an interpret declaration",
        DeclCandidate::KeyAlias => "a key alias declaration",
        DeclCandidate::KeyName(_) => "a key name",
        DeclCandidate::Keys => "a keys declaration",
        DeclCandidate::KeyType => "a key type declaration",
        DeclCandidate::IndicatorMap => "an indicator mapping",
        DeclCandidate::IndicatorName(_) => "an indicator declaration",
        DeclCandidate::ModMap => "a mod-map declaration",
        DeclCandidate::Overlay => "an overlay declaration",
        DeclCandidate::Row => "a row declaration",
        DeclCandidate::Section => "a section declaration",
        DeclCandidate::Shape => "a shape declaration",
        DeclCandidate::Symbols => "a key-symbol mapping",
        DeclCandidate::Var(_) => "an assignment",
        DeclCandidate::Vmod => "a virtual modifier declaration",
    };
    f.write_str(name)
}

fn write_actual(f: &mut Formatter<'_>, actual: &ActualToken) -> fmt::Result {
    match actual {
        ActualToken::Ident(i) => write!(f, "`{}`", i.as_bytes().as_bstr()),
        ActualToken::Token(t) => match t {
            Token::Ident(_) => f.write_str("an identifier"),
            Token::String(_) => f.write_str("a string"),
            Token::KeyName(_) => f.write_str("a key name"),
            Token::Integer(_, _) => f.write_str("an integer"),
            Token::Float(_, _) => f.write_str("a float"),
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
        Expected::Ident(i) => write!(f, "`{}`", i.name()),
        Expected::VarDecl => f.write_str("an assignment"),
        Expected::AnyIdent => f.write_str("an identifier"),
        Expected::String => f.write_str("a string"),
        Expected::KeyName => f.write_str("a key name"),
        Expected::Integer => f.write_str("an integer"),
        Expected::Float => f.write_str("a float"),
        Expected::Punctuation(p) => {
            write!(f, "`{}`", punctuation_string(*p))
        }
    }
}

fn punctuation_string(p: Punctuation) -> &'static str {
    match p {
        Punctuation::Semicolon => ";",
        Obrace => "{",
        Cbrace => "}",
        Punctuation::Equals => "=",
        Obracket => "[",
        Punctuation::Cbracket => "]",
        Oparen => "(",
        Punctuation::Cparen => ")",
        Punctuation::Dot => ".",
        Punctuation::Comma => ",",
        Punctuation::Plus => "+",
        Punctuation::Minus => "-",
        Punctuation::Times => "*",
        Punctuation::Divide => "/",
        Punctuation::Exclam => "!",
        Punctuation::Invert => "~",
    }
}

#[derive(Debug, Clone)]
pub(crate) struct InvalidU32 {
    slice: CodeSlice<'static>,
}

#[derive(Debug, Clone)]
pub(crate) struct ExpectedButEof {
    expected: &'static [Expected],
}

#[derive(Debug, Clone)]
pub(crate) struct UnexpectedDeclCandidate {
    expected: &'static [Expected],
    actual: DeclCandidate,
}

#[derive(Debug, Clone)]
pub(crate) struct UnexpectedToken {
    expected: &'static [Expected],
    actual: ActualToken,
}

#[derive(Debug)]
pub(crate) enum Expected {
    Nested(&'static [Expected]),
    Ident(Meaning),
    VarDecl,
    AnyIdent,
    String,
    KeyName,
    Integer,
    Float,
    Punctuation(Punctuation),
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum ActualToken {
    Ident(CodeSlice<'static>),
    Token(Token),
}

impl Parser<'_, '_, '_> {
    pub(super) fn too_deeply_nested(&self, span: Span) -> Spanned<ParserError> {
        ParserError::TooDeeplyNested.spanned2(span + self.diagnostic_delta)
    }

    pub(super) fn expected_but_eof(&self, expected: &'static [Expected]) -> Spanned<ParserError> {
        let span = match self.tokens.last() {
            Some(t) => t.span,
            _ => Span { lo: 0, hi: 0 },
        };
        ParserError::ExpectedButEof(ExpectedButEof { expected })
            .spanned2(span + self.diagnostic_delta)
    }

    pub(super) fn unexpected_decl_candidate(
        &self,
        expected: &'static [Expected],
        actual: DeclCandidate,
        span: Span,
    ) -> Spanned<ParserError> {
        ParserError::UnexpectedDeclCandidate(UnexpectedDeclCandidate { expected, actual })
            .spanned2(span + self.diagnostic_delta)
    }

    pub(super) fn unexpected_token(
        &self,
        expected: &'static [Expected],
        token: Spanned<Token>,
    ) -> Spanned<ParserError> {
        let actual = match token.val {
            Token::Ident(i) => ActualToken::Ident(self.interner.get(i).to_owned()),
            _ => ActualToken::Token(token.val),
        };
        ParserError::UnexpectedToken(UnexpectedToken { expected, actual })
            .spanned2(token.span + self.diagnostic_delta)
    }

    pub(super) fn invalid_u32(&self, name: Spanned<Interned>) -> Spanned<ParserError> {
        let slice = self.interner.get(name.val).to_owned();
        ParserError::InvalidU32(InvalidU32 { slice }).spanned2(name.span + self.diagnostic_delta)
    }
}

pub(super) const EXPR_TOKENS: &[Expected] = &[
    Expected::AnyIdent,
    Expected::String,
    Expected::KeyName,
    Expected::Integer,
    Expected::Float,
    Expected::Punctuation(punctuation![+]),
    Expected::Punctuation(punctuation![-]),
    Expected::Punctuation(punctuation![!]),
    Expected::Punctuation(punctuation![~]),
    Expected::Punctuation(Oparen),
    Expected::Punctuation(Obracket),
    Expected::Punctuation(Obrace),
];

pub(super) const OUTLINE_TOKENS: &[Expected] = &[Expected::Punctuation(Obrace), Expected::VarDecl];

pub(super) struct KeycodeDeclExpectation;

impl ParseDeclExpectationBase for KeycodeDeclExpectation {
    const EXPECTED: &'static [Expected] = &[
        Expected::KeyName,
        Expected::Ident(Meaning::Alias),
        Expected::VarDecl,
        Expected::Ident(Meaning::Indicator),
        Expected::Ident(Meaning::Virtual),
    ];
}

pub(super) struct TypesDeclExpectation;

impl ParseDeclExpectationBase for TypesDeclExpectation {
    const EXPECTED: &'static [Expected] = &[
        Expected::Ident(Meaning::Type),
        Expected::VarDecl,
        Expected::Ident(Meaning::VirtualModifiers),
    ];
}

pub(super) struct CompatmapDeclExpectation;

impl ParseDeclExpectationBase for CompatmapDeclExpectation {
    const EXPECTED: &'static [Expected] = &[
        Expected::Ident(Meaning::Interpret),
        Expected::Ident(Meaning::Group),
        Expected::Ident(Meaning::Indicator),
        Expected::VarDecl,
        Expected::Ident(Meaning::VirtualModifiers),
    ];
}

pub(super) struct SymbolsDeclExpectation;

impl ParseDeclExpectationBase for SymbolsDeclExpectation {
    const EXPECTED: &'static [Expected] = &[
        Expected::Ident(Meaning::Key),
        Expected::VarDecl,
        Expected::Ident(Meaning::VirtualModifiers),
        Expected::Ident(Meaning::ModifierMap),
        Expected::Ident(Meaning::ModMap),
        Expected::Ident(Meaning::Modmap),
    ];
}

pub(super) struct GeometryDeclExpectation;

impl ParseDeclExpectationBase for GeometryDeclExpectation {
    const EXPECTED: &'static [Expected] = &[
        Expected::Ident(Meaning::Alias),
        Expected::VarDecl,
        Expected::Ident(Meaning::Shape),
        Expected::Ident(Meaning::Section),
        Expected::Ident(Meaning::Indicator),
        Expected::Ident(Meaning::Modmap),
        Expected::Nested(DOODAD_EXPECTED),
    ];
}

pub(super) trait ParseDeclExpectationBase {
    const EXPECTED: &'static [Expected];
}

pub(super) trait ParseDeclExpectation {
    fn expected_plus_merge_mode(&self) -> &'static [Expected];

    fn expected_plus_merge_mode_plus_cbrace(&self) -> &'static [Expected];

    fn expected_plus_include(&self) -> &'static [Expected];
}

impl<T> ParseDeclExpectation for T
where
    T: ParseDeclExpectationBase,
{
    fn expected_plus_merge_mode(&self) -> &'static [Expected] {
        &[
            Expected::Nested(MERGE_MODE_EXPECTED),
            Expected::Nested(Self::EXPECTED),
        ]
    }

    fn expected_plus_merge_mode_plus_cbrace(&self) -> &'static [Expected] {
        &[
            Expected::Punctuation(Cbrace),
            Expected::Nested(MERGE_MODE_EXPECTED),
            Expected::Nested(Self::EXPECTED),
        ]
    }

    fn expected_plus_include(&self) -> &'static [Expected] {
        &[Expected::String, Expected::Nested(Self::EXPECTED)]
    }
}

pub(super) const MERGE_MODE_EXPECTED: &[Expected] = &[
    Expected::Ident(Meaning::Include),
    Expected::Ident(Meaning::Augment),
    Expected::Ident(Meaning::Override),
    Expected::Ident(Meaning::Replace),
    Expected::Ident(Meaning::Alternate),
];

pub(super) const DOODAD_EXPECTED: &[Expected] = &[
    Expected::Ident(Meaning::Text),
    Expected::Ident(Meaning::Outline),
    Expected::Ident(Meaning::Solid),
    Expected::Ident(Meaning::Logo),
];

pub(super) const SECTION_ITEM_EXPECTED: &[Expected] = &[
    Expected::Ident(Meaning::Row),
    Expected::Ident(Meaning::Overlay),
    Expected::Ident(Meaning::Indicator),
    Expected::Nested(DOODAD_EXPECTED),
    Expected::VarDecl,
];

pub(super) const KEY_EXPECTED: &[Expected] = &[Expected::KeyName, Expected::Punctuation(Obrace)];

pub(super) const FLAGS_EXPECTED: &[Expected] = &[
    Expected::Ident(Meaning::Partial),
    Expected::Ident(Meaning::Default),
    Expected::Ident(Meaning::Hidden),
    Expected::Ident(Meaning::AlphanumericKeys),
    Expected::Ident(Meaning::ModifierKeys),
    Expected::Ident(Meaning::KeypadKeys),
    Expected::Ident(Meaning::FunctionKeys),
    Expected::Ident(Meaning::AlternateGroup),
];

pub(super) const CONFIG_ITEM_EXPECTED: &[Expected] = &[
    Expected::Ident(Meaning::XkbKeycodes),
    Expected::Ident(Meaning::XkbTypes),
    Expected::Ident(Meaning::XkbCompatibilityMap),
    Expected::Ident(Meaning::XkbCompatibility),
    Expected::Ident(Meaning::XkbCompatMap),
    Expected::Ident(Meaning::XkbCompat),
    Expected::Ident(Meaning::XkbSymbols),
    Expected::Ident(Meaning::XkbGeometry),
];
