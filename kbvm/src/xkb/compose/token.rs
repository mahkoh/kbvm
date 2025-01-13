use {crate::xkb::interner::Interned, kbvm_proc::CloneWithDelta};

#[derive(Copy, Clone, Debug, PartialEq, CloneWithDelta)]
pub(crate) enum Token {
    String(Interned),
    Ident(Interned),
    Keysym(Interned),
    Punctuation(Punctuation),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, CloneWithDelta)]
pub(crate) enum Punctuation {
    Exclam,
    Tilde,
    Colon,
}

impl PartialEq<Punctuation> for Token {
    fn eq(&self, other: &Punctuation) -> bool {
        *self == Token::Punctuation(*other)
    }
}

// impl Punctuation {
//     pub(crate) fn expected(self) -> &'static [Expected] {
//         macro_rules! c {
//             ($($ident:ident),*) => {
//                 match self {
//                     $(Self::$ident => &Expected::Punctuation(Self::$ident),)*
//                 }
//             };
//         }
//         let e = c![Exclam, Tilde, Colon];
//         slice::from_ref(e)
//     }
// }

impl From<Punctuation> for Token {
    fn from(value: Punctuation) -> Self {
        Self::Punctuation(value)
    }
}
