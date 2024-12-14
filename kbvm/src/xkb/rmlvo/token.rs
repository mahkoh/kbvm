use {
    crate::xkb::{interner::Interned, rmlvo::parser::error::Expected},
    kbvm_proc::CloneWithDelta,
    std::slice,
};

#[derive(Copy, Clone, Debug, PartialEq, CloneWithDelta)]
pub(crate) enum Token {
    Ident(Interned),
    GroupName(Interned),
    Punctuation(Punctuation),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, CloneWithDelta)]
pub(crate) enum Punctuation {
    Equals,
    Times,
    Exclam,
}

impl Punctuation {
    pub(crate) fn expected(self) -> &'static [Expected] {
        macro_rules! c {
            ($($ident:ident),*) => {
                match self {
                    $(Self::$ident => &Expected::Punctuation(Self::$ident),)*
                }
            };
        }
        let e = c![Equals, Times, Exclam];
        slice::from_ref(e)
    }
}
