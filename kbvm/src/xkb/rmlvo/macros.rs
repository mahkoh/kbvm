macro_rules! token {
    ($($tt:tt)*) => {
        $crate::xkb::rmlvo::token::Token::Punctuation(punctuation!($($tt)*))
    };
}

macro_rules! punctuation {
    (=) => {
        $crate::xkb::rmlvo::token::Punctuation::Equals
    };
    (*) => {
        $crate::xkb::rmlvo::token::Punctuation::Times
    };
    (!) => {
        $crate::xkb::rmlvo::token::Punctuation::Exclam
    };
}
