macro_rules! token {
    ($($tt:tt)*) => {
        Token::Punctuation(punctuation!($($tt)*))
    };
}

macro_rules! punctuation {
    (;) => {
        Punctuation::Semicolon
    };
    (=) => {
        Punctuation::Equals
    };
    (.) => {
        Punctuation::Dot
    };
    (,) => {
        Punctuation::Comma
    };
    (+) => {
        Punctuation::Plus
    };
    (-) => {
        Punctuation::Minus
    };
    (*) => {
        Punctuation::Times
    };
    (/) => {
        Punctuation::Divide
    };
    (!) => {
        Punctuation::Exclam
    };
    (~) => {
        Punctuation::Invert
    };
}
