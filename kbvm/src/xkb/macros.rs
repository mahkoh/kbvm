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

macro_rules! literal_display {
    ($msg:expr) => {{
        struct Tmp;
        impl std::fmt::Display for Tmp {
            fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                f.write_str($msg)
            }
        }
        Tmp
    }};
}
