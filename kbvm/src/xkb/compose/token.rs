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
