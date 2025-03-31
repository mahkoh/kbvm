use {
    crate::xkb::{
        code::Code,
        code_map::CodeMap,
        diagnostic::{Diagnostic, DiagnosticKind, DiagnosticSink},
        interner::Interner,
        span::SpanExt,
        string_cooker::StringCooker,
    },
    bstr::ByteSlice,
    std::sync::Arc,
};

fn cook_(s: &str) -> (String, Vec<Diagnostic>) {
    let mut interner = Interner::default();
    let mut map = CodeMap::default();
    let code = Code::new(&Arc::new(s.as_bytes().to_vec()));
    let span = map.add(None, None, &code);
    let interned = interner.intern(&code.to_slice());
    let mut diagnostics = vec![];
    let mut sink = DiagnosticSink::new(&mut diagnostics);
    let mut cooker = StringCooker::default();
    let res = cooker.cook(&mut map, &mut sink, &mut interner, interned.spanned2(span));
    let res = interner.get(res).to_str().unwrap().to_owned();
    (res, diagnostics)
}

fn cook(s: &str) -> String {
    let (res, diagnostics) = cook_(s);
    assert!(diagnostics.is_empty());
    res
}

#[test]
fn success() {
    assert_eq!(cook(r#""#), "");
    assert_eq!(cook(r#"abc"#), "abc");
    assert_eq!(cook(r#"\\\n\t\r\b\f\v\e"#), "\\\n\t\r\x08\x0c\x0b\x1b");
    assert_eq!(cook(r#"\1"#), (0o1 as char).to_string());
    assert_eq!(cook(r#"\12"#), (0o12 as char).to_string());
    assert_eq!(cook(r#"\123"#), (0o123 as char).to_string());
    assert_eq!(cook(r#"\1234"#), format!("{}4", 0o123 as char));
    assert_eq!(cook(r#"\u{65e5}"#), format!("日"));
    assert_eq!(cook(r#"a\u{65e5}b"#), format!("a日b"));
}

#[test]
fn warning() {
    let (c, d) = cook_(r#"\n\777\n"#);
    assert_eq!(c, "\n\n");
    assert_eq!(d.len(), 1);
    assert_eq!(d[0].kind(), DiagnosticKind::OctalStringEscapeOverflow);

    let (c, d) = cook_(r#"\n\q\n"#);
    assert_eq!(c, "\n\n");
    assert_eq!(d.len(), 1);
    assert_eq!(d[0].kind(), DiagnosticKind::UnknownEscapeSequence);

    let (c, d) = cook_(r#"\n\u{65e5"#);
    assert_eq!(c, "\n");
    assert_eq!(d.len(), 1);
    assert_eq!(d[0].kind(), DiagnosticKind::UnterminatedUnicodeEscape);

    let (c, d) = cook_(r#"\n\u_65e5}"#);
    assert_eq!(c, "\n");
    assert_eq!(d.len(), 1);
    assert_eq!(d[0].kind(), DiagnosticKind::UnopenedUnicodeEscape);

    let (c, d) = cook_(r#"\n\u{x}\n"#);
    assert_eq!(c, "\n\n");
    assert_eq!(d.len(), 1);
    assert_eq!(
        d[0].kind(),
        DiagnosticKind::InvalidUnicodeEscapeRepresentation
    );

    let (c, d) = cook_(r#"\n\u{aaaaaaaa}\n"#);
    assert_eq!(c, "\n\n");
    assert_eq!(d.len(), 1);
    assert_eq!(d[0].kind(), DiagnosticKind::InvalidUnicodeCodepoint);
}
