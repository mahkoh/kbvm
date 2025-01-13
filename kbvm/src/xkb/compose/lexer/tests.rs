use {
    crate::xkb::{
        code::Code,
        code_map::CodeMap,
        compose::{
            lexer::{Lexer, LexerError},
            token::Token,
        },
        diagnostic::{DiagnosticSink, WriteToStderr},
        interner::{Interned, Interner},
        span::{SpanExt, Spanned},
    },
    std::{path::PathBuf, sync::Arc},
};

fn empty_path() -> Arc<PathBuf> {
    Arc::new(PathBuf::from(""))
}

fn l_(interner: &mut Interner, input: &str) -> Result<Vec<Spanned<Token>>, Spanned<LexerError>> {
    let mut output = vec![];
    let code = Code::new(&input.as_bytes().to_vec().into());
    let mut map = CodeMap::default();
    let span = map.add(None, None, &code);
    let mut handler = WriteToStderr;
    let mut sink = DiagnosticSink::new(&mut handler);
    Lexer::new(&empty_path(), &code, span.lo).lex_line(
        &mut map,
        &mut sink,
        interner,
        &mut output,
    )?;
    Ok(output)
}

fn l(interner: &mut Interner, input: &str) -> Vec<Spanned<Token>> {
    l_(interner, input).unwrap()
}

fn q(input: &str) -> Vec<Spanned<Token>> {
    let mut interner = Interner::default();
    l(&mut interner, input)
}

fn e(input: &str) -> Spanned<LexerError> {
    let mut interner = Interner::default();
    l_(&mut interner, input).unwrap_err()
}

fn i(interner: &mut Interner, s: &str) -> Interned {
    let code = Code::new(&s.as_bytes().to_vec().into());
    let code = code.to_slice();
    interner.intern(&code)
}

#[test]
fn two_single_char() {
    let chars = [(':', token![:]), ('!', token![!]), ('~', token![~])];
    for (c1, t1) in chars {
        for (c2, t2) in chars {
            let input = format!("{c1}{c2}");
            let expected = [t1.spanned(0, 1), t2.spanned(1, 2)];
            assert_eq!(q(&input), expected);
        }
    }
}

#[test]
fn empty_lines() {
    let mut interner = Interner::default();
    let multi_key = i(&mut interner, "Multi_key");
    let space = i(&mut interner, "space");
    let nbsp = i(&mut interner, " ");
    let nobreakspace = i(&mut interner, "nobreakspace");
    let input = r#"
        # comment
        <Multi_key> <space> <space>		: " "	nobreakspace # NO-BREAK SPACE
    "#;
    assert_eq!(
        l(&mut interner, input),
        [
            Token::Keysym(multi_key),
            Token::Keysym(space),
            Token::Keysym(space),
            token![:],
            Token::String(nbsp),
            Token::Ident(nobreakspace),
        ]
    );
}

#[test]
fn unterminated_keysym() {
    let input = r#"
        <Multi_key> <space		: nobreakspace
    "#;
    assert_eq!(e(input), LexerError::UnterminatedKeysym);
    let input = r#"
        <Multi_key> <space		: nobreakspace"#;
    assert_eq!(e(input), LexerError::UnterminatedKeysym);
}

#[test]
fn unterminated_string() {
    let input = r#"
        <Multi_key> <space>		: "abc
    "#;
    assert_eq!(e(input), LexerError::UnterminatedString);
    let input = r#"
        <Multi_key> <space>		: "abc"#;
    assert_eq!(e(input), LexerError::UnterminatedString);
}

#[test]
fn unexpected_byte() {
    let input = r#"
        <abc> %
    "#;
    assert_eq!(e(input), LexerError::UnexpectedByte(b'%'));
}

#[test]
fn unknown_escape_sequence() {
    let mut interner = Interner::default();
    let abc = i(&mut interner, "abc");
    let s = i(&mut interner, "a\\ub");
    let input = r#"
        <abc> "a\ub"
    "#;
    assert_eq!(
        l(&mut interner, input),
        [Token::Keysym(abc), Token::String(s),]
    );
}

#[test]
fn stringstring() {
    let mut interner = Interner::default();
    let abc = i(&mut interner, "abc");
    let def = i(&mut interner, "def");
    let input = r#""abc""def""#;
    assert_eq!(
        l(&mut interner, input),
        [Token::String(abc), Token::String(def),]
    );
}

#[test]
fn escapes() {
    let mut interner = Interner::default();
    let abc = i(&mut interner, "a\"Mc\u{7f}");
    let input = r#"
        "a\"\x4dc\177"
    "#;
    assert_eq!(l(&mut interner, input), [Token::String(abc)]);
}
