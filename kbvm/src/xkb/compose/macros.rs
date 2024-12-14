macro_rules! token {
    ($($tt:tt)*) => {
        $crate::xkb::compose::token::Token::Punctuation(punctuation!($($tt)*))
    };
}

macro_rules! punctuation {
    (:) => {
        $crate::xkb::compose::token::Punctuation::Colon
    };
    (~) => {
        $crate::xkb::compose::token::Punctuation::Tilde
    };
    (!) => {
        $crate::xkb::compose::token::Punctuation::Exclam
    };
}
