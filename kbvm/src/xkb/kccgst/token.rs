use {
    crate::xkb::{interner::Interned, kccgst::parser::error::Expected},
    kbvm_proc::CloneWithDelta,
    std::slice,
};

#[derive(Copy, Clone, Debug, PartialEq, CloneWithDelta)]
pub(crate) enum Token {
    Ident(Interned),
    String(Interned),
    KeyName(Interned),
    Integer(Interned, i64),
    Float(Interned, f64),
    Punctuation(Punctuation),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, CloneWithDelta)]
pub(crate) enum Punctuation {
    Semicolon,
    Obrace,
    Cbrace,
    Equals,
    Obracket,
    Cbracket,
    Oparen,
    Cparen,
    Dot,
    Comma,
    Plus,
    Minus,
    Times,
    Divide,
    Exclam,
    Invert,
}

impl PartialEq<Punctuation> for Token {
    fn eq(&self, other: &Punctuation) -> bool {
        *self == Token::Punctuation(*other)
    }
}

impl Punctuation {
    pub(crate) fn expected(self) -> &'static [Expected] {
        macro_rules! c {
            ($($ident:ident,)*) => {
                match self {
                    $(Self::$ident => &Expected::Punctuation(Self::$ident),)*
                }
            };
        }
        let e = c![
            Semicolon, Obrace, Cbrace, Equals, Obracket, Cbracket, Oparen, Cparen, Dot, Comma,
            Plus, Minus, Times, Divide, Exclam, Invert,
        ];
        slice::from_ref(e)
    }
}

impl From<Punctuation> for Token {
    fn from(value: Punctuation) -> Self {
        Self::Punctuation(value)
    }
}
