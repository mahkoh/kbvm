use crate::xkb::{
    code::Code,
    interner::{Interned, Interner},
    kccgst::{
        lexer::{
            Lexer,
            LexerError::{
                self, InvalidIntegerLiteral, UnexpectedByte, UnterminatedKeyName,
                UnterminatedString,
            },
        },
        token::{
            Punctuation,
            Token::{self, Float, Ident, Integer, KeyName, String},
        },
    },
    span::{SpanExt, Spanned},
};

fn l_(interner: &mut Interner, input: &str) -> Result<Vec<Spanned<Token>>, Spanned<LexerError>> {
    let mut output = vec![];
    let code = Code::new(&input.as_bytes().to_vec().into());
    Lexer::new(None, &code, 0).lex_item(interner, &mut output)?;
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
    let chars = [
        (';', token![;]),
        ('{', Punctuation::Obrace.into()),
        ('}', Punctuation::Cbrace.into()),
        ('=', token![=]),
        ('[', Punctuation::Obracket.into()),
        (']', Punctuation::Cbracket.into()),
        ('(', Punctuation::Oparen.into()),
        (')', Punctuation::Cparen.into()),
        ('.', token![.]),
        (',', token![,]),
        ('+', token![+]),
        ('-', token![-]),
        ('*', token![*]),
        ('/', token![/]),
        ('!', token![!]),
        ('~', token![~]),
    ];
    for (c1, t1) in chars {
        if t1 == token![;] {
            continue;
        }
        for (c2, t2) in chars {
            if (t1, t2) == (token![/], token![/]) {
                continue;
            }
            let input = format!("{c1}{c2}");
            let expected = [t1.spanned(0, 1), t2.spanned(1, 2)];
            assert_eq!(q(&input), expected);
        }
    }
}

#[test]
fn number() {
    let mut interner = Interner::default();

    let input = "123";
    let interned = i(&mut interner, input);
    assert_eq!(
        l(&mut interner, input),
        [Integer(interned, 123).spanned(0, 3)]
    );

    let input = "123.456";
    let interned = i(&mut interner, input);
    assert_eq!(
        l(&mut interner, input),
        [Float(interned, 123.456).spanned(0, 7)]
    );

    let input = "123.456.789";
    let interned1 = i(&mut interner, "123.456");
    let interned2 = i(&mut interner, "789");
    assert_eq!(
        l(&mut interner, input),
        [
            Float(interned1, 123.456).spanned(0, 7),
            token![.].spanned(7, 8),
            Integer(interned2, 789).spanned(8, 11),
        ]
    );

    let input = "0x123fg";
    let interned1 = i(&mut interner, "0x123f");
    let interned2 = i(&mut interner, "g");
    assert_eq!(
        l(&mut interner, input),
        [
            Integer(interned1, 0x123f).spanned(0, 6),
            Ident(interned2).spanned(6, 7),
        ]
    );

    let input = "0x123a.456";
    let interned1 = i(&mut interner, "0x123a");
    let interned2 = i(&mut interner, "456");
    assert_eq!(
        l(&mut interner, input),
        [
            Integer(interned1, 0x123a).spanned(0, 6),
            token![.].spanned(6, 7),
            Integer(interned2, 456).spanned(7, 10),
        ]
    );

    let input = "1111111111111111111111111111111111111";
    assert!(matches!(e(input).val, InvalidIntegerLiteral));
}

#[test]
fn comment() {
    let mut interner = Interner::default();
    let input = "abc // comment\ndef # comment\nghi";
    let abc = i(&mut interner, "abc");
    let def = i(&mut interner, "def");
    let ghi = i(&mut interner, "ghi");
    assert_eq!(
        l(&mut interner, input),
        [
            Ident(abc).spanned(0, 3),
            Ident(def).spanned(15, 18),
            Ident(ghi).spanned(29, 32),
        ],
    );
}

#[test]
fn ident() {
    let mut interner = Interner::default();

    let input = "abc";
    let interned = i(&mut interner, "abc");
    assert_eq!(l(&mut interner, input), [Ident(interned).spanned(0, 3),],);

    let input = "abc.";
    let interned = i(&mut interner, "abc");
    assert_eq!(
        l(&mut interner, input),
        [Ident(interned).spanned(0, 3), token![.].spanned(3, 4),],
    );

    let input = "Abc";
    let interned = i(&mut interner, "Abc");
    assert_eq!(l(&mut interner, input), [Ident(interned).spanned(0, 3),],);

    let input = "_aBc";
    let interned = i(&mut interner, "_aBc");
    assert_eq!(l(&mut interner, input), [Ident(interned).spanned(0, 4),],);

    let input = "_1aBc";
    let interned = i(&mut interner, "_1aBc");
    assert_eq!(l(&mut interner, input), [Ident(interned).spanned(0, 5),],);

    let input = "_1abc";
    let interned = i(&mut interner, "_1abc");
    assert_eq!(l(&mut interner, input), [Ident(interned).spanned(0, 5),],);

    let input = "1abc";
    let interned1 = i(&mut interner, "1");
    let interned2 = i(&mut interner, "abc");
    assert_eq!(
        l(&mut interner, input),
        [
            Integer(interned1, 1).spanned(0, 1),
            Ident(interned2).spanned(1, 4),
        ],
    );
}

#[test]
fn string() {
    let mut interner = Interner::default();

    let input = "\"abc\"";
    let interned = i(&mut interner, "abc");
    assert_eq!(l(&mut interner, input), [String(interned).spanned(0, 5),],);

    let input = r#"   "abc\""  "#;
    let interned = i(&mut interner, "abc\\\"");
    assert_eq!(
        l(&mut interner, input),
        [String(interned).spanned(3, 10),],
        "{:?}",
        interner,
    );

    let input = r#"   "abc\"   "#;
    assert_eq!(e(input), UnterminatedString.spanned(3, 12),);
}

#[test]
fn key_name() {
    let mut interner = Interner::default();

    let input = "<abc>";
    let interned = i(&mut interner, "abc");
    assert_eq!(l(&mut interner, input), [KeyName(interned).spanned(0, 5),],);

    let input = r#"<abc "#;
    assert_eq!(e(input), UnterminatedKeyName.spanned(0, 4),);

    let input = r#"<!{}#>"#;
    let interned = i(&mut interner, "!{}#");
    assert_eq!(l(&mut interner, input), [KeyName(interned).spanned(0, 6),],);
}

#[test]
fn unexpected_byte() {
    let input = "{\0";
    assert_eq!(e(input), UnexpectedByte(0).spanned(1, 2));
}
