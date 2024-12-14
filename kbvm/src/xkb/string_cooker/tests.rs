use {
    crate::xkb::{
        code::Code,
        interner::Interner,
        string_cooker::{StringCooker, StringCookerDiagnostic, StringCookerError},
    },
    bstr::ByteSlice,
    std::sync::Arc,
};

fn cook_(s: &str) -> (String, Vec<StringCookerDiagnostic>) {
    let mut interner = Interner::default();
    let code = Code::new(&Arc::new(s.as_bytes().to_vec()));
    let interned = interner.intern(&code.to_slice());
    let mut diagnostics = vec![];
    let mut cooker = StringCooker::default();
    let res = cooker.cook(&mut diagnostics, &mut interner, interned);
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
}

#[test]
fn warning() {
    let (c, d) = cook_(r#"\n\777\n"#);
    assert_eq!(c, "\n\n");
    assert_eq!(
        d,
        [StringCookerDiagnostic {
            range: 3..6,
            error: StringCookerError::OctalOverflow,
        },]
    );

    let (c, d) = cook_(r#"\n\q\n"#);
    assert_eq!(c, "\n\n");
    assert_eq!(
        d,
        [StringCookerDiagnostic {
            range: 3..4,
            error: StringCookerError::UnknownSequence(b'q'),
        },]
    );
}
