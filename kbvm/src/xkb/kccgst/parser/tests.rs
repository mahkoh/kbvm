use {
    crate::xkb::{
        code::Code,
        code_map::CodeMap,
        diagnostic::DiagnosticSink,
        interner::Interner,
        kccgst::{
            formatter::{Format, Formatter},
            lexer::{Lexer, LexerError},
            parser::{snoop_ty_and_name, Parser},
            token::Token,
        },
        meaning::MeaningCache,
        span::Spanned,
    },
    bstr::ByteSlice,
    std::sync::Arc,
    walkdir::WalkDir,
};

fn l(
    map: &mut CodeMap,
    i: &mut Interner,
    code: &Code,
) -> Result<Vec<Vec<Spanned<Token>>>, Spanned<LexerError>> {
    let span = map.add(None, None, &code);
    let mut res = vec![];
    let mut lexer = Lexer::new(None, &code, span.lo);
    loop {
        let mut tokens = vec![];
        lexer.lex_item(i, &mut tokens)?;
        if tokens.is_empty() {
            break;
        }
        res.push(tokens);
    }
    Ok(res)
}

fn s(interner: &Interner, t: &impl Format) -> Vec<u8> {
    let mut out = vec![];
    let mut f = Formatter::new(interner, &mut out);
    t.format(&mut f).unwrap();
    out
}

#[test]
fn round_trip() {
    let mut interner = Interner::default();
    let mut meaning_cache = MeaningCache::default();
    for f in ["compat", "geometry", "keycodes", "symbols", "types"] {
        let dir = format!("{}/../xkeyboard-config/{f}", env!("CARGO_MANIFEST_DIR"));
        let dir = WalkDir::new(&dir);
        for f in dir {
            let f = f.unwrap();
            if f.path().is_file() {
                if let Some(f) = f.path().file_name() {
                    if f == "README" || f == "meson.build" {
                        continue;
                    }
                }
                let path = Arc::new(f.into_path());
                let input = std::fs::read(path.as_path()).unwrap();
                let code = Code::new(&Arc::new(input));
                let mut map = CodeMap::default();
                let mut diag = vec![];
                let mut diagnostics = DiagnosticSink::new(&mut diag);
                let tokens = match l(&mut map, &mut interner, &code) {
                    Ok(t) => t,
                    Err(e) => panic!("{}: {:?}", path.display(), e),
                };
                for tokens in &tokens {
                    let (default, ty, name) =
                        snoop_ty_and_name(tokens, &interner, &mut meaning_cache).unwrap();
                    eprintln!(
                        "{}: {:?} {:?} {:?}",
                        path.display(),
                        default,
                        ty,
                        name.map(|n| interner.get(n.val).as_bstr())
                    );
                    let item = Parser {
                        tokens: &tokens,
                        interner: &interner,
                        meaning_cache: &mut meaning_cache,
                        pos: 0,
                        diagnostic_delta: 0,
                    }
                    .parse_item();
                    let item = match item {
                        Ok(i) => i,
                        Err(e) => {
                            diagnostics.push(&mut map, e.val.diagnostic_kind(), e);
                            for d in diag {
                                eprintln!("{}", d.with_code());
                            }
                            panic!();
                        }
                    };
                    let s = Arc::new(s(&interner, &item.val));
                    // println!("{}", s.as_bstr());
                    let code = Code::new(&s);
                    let t2 = match l(&mut map, &mut interner, &code) {
                        Ok(t) => t,
                        Err(e) => panic!("{}: {:?}", path.display(), e),
                    };
                    assert_eq!(t2.len(), 1);
                    assert_eq!(&t2[0], tokens, "{}", path.display());
                }
            }
        }
    }
}

#[test]
fn check_error() {
    let mut interner = Interner::default();
    let mut meaning_cache = MeaningCache::default();
    let mut map = CodeMap::default();
    let input = r#"
        xkb_keycodes hurr {
            <abcd> = 0xffffffff1;
        };
    "#;
    let code = Code::new(&Arc::new(input.as_bytes().to_vec()));
    let tokens = l(&mut map, &mut interner, &code).unwrap();
    let _err = Parser {
        tokens: &tokens[0],
        interner: &interner,
        meaning_cache: &mut meaning_cache,
        pos: 0,
        diagnostic_delta: 0,
    }
    .parse_item()
    .unwrap_err();
    // let err = err.into_diagnostic(&mut map, Severity::Error);
    // panic!("{}", err.with_code());
}
