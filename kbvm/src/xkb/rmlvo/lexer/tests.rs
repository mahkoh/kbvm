use {
    crate::xkb::{
        code::Code,
        interner::{Interned, Interner},
        rmlvo::{
            lexer::{Lexer, LexerError},
            token::Token,
        },
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
    Lexer::new(&empty_path(), &code, 0).lex_line(interner, &mut output)?;
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
    let chars = [('=', token![=]), ('*', token![*]), ('!', token![!])];
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
    let input = r#"


        = = =
    "#;
    assert_eq!(q(input), [token![=], token![=], token![=]]);
}

#[test]
fn comments() {
    let input = r#"
        // hello

        // world

        = = =
    "#;
    assert_eq!(q(input), [token![=], token![=], token![=]]);
}

#[test]
fn escape() {
    let input = r#"
        = \
        = \
        = \
    "#;
    assert_eq!(q(input), [token![=], token![=], token![=]]);
}

#[test]
fn unusual_whitespace() {
    let input = "= \r\t = \t\r =";
    assert_eq!(q(input), [token![=], token![=], token![=]]);
}

#[test]
fn multiple_lines() {
    let input = r#"
        = = \
        =
        ! ! !


        *

    "#;
    let mut interner = Interner::default();
    let code = Code::new(&input.as_bytes().to_vec().into());
    let mut lexer = Lexer::new(&empty_path(), &code, 0);
    let mut output = vec![];
    lexer.lex_line(&mut interner, &mut output).unwrap();
    assert_eq!(output, [token![=], token![=], token![=]]);
    output.clear();
    lexer.lex_line(&mut interner, &mut output).unwrap();
    assert_eq!(output, [token![!], token![!], token![!]]);
    output.clear();
    lexer.lex_line(&mut interner, &mut output).unwrap();
    assert_eq!(output, [token![*]]);
    output.clear();
    lexer.lex_line(&mut interner, &mut output).unwrap();
    assert!(output.is_empty());
}

#[test]
fn identifier() {
    let input = r#"
       abcd xxx
       $defg!{[}+\
       =

    "#;
    let mut interner = Interner::default();
    let abcd = i(&mut interner, "abcd");
    let xxx = i(&mut interner, "xxx");
    let defg = i(&mut interner, "defg!{[}+");
    let code = Code::new(&input.as_bytes().to_vec().into());
    let mut lexer = Lexer::new(&empty_path(), &code, 0);
    let mut output = vec![];
    lexer.lex_line(&mut interner, &mut output).unwrap();
    assert_eq!(output, [Token::Ident(abcd), Token::Ident(xxx)]);
    output.clear();
    lexer.lex_line(&mut interner, &mut output).unwrap();
    assert_eq!(output, [Token::GroupName(defg), token![=]]);
    output.clear();
    lexer.lex_line(&mut interner, &mut output).unwrap();
    assert!(output.is_empty());
}

#[test]
fn invalid_escape() {
    assert_eq!(e("\\n"), LexerError::UnexpectedByte(b'n'));
}

#[test]
fn invalid_ident() {
    assert_eq!(e("\0"), LexerError::UnexpectedByte(b'\0'));
}

#[test]
fn unterminated_group() {
    assert_eq!(e("$"), LexerError::EmptyMacroName);
}
