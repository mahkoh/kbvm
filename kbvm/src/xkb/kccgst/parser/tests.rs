use crate::xkb::code::Code;
use crate::xkb::code_map::CodeMap;
use crate::xkb::diagnostic::DiagnosticSink;
use crate::xkb::interner::Interner;
use crate::xkb::kccgst::formatter::Format;
use crate::xkb::kccgst::formatter::Formatter;
use crate::xkb::kccgst::lexer::Lexer;
use crate::xkb::kccgst::lexer::LexerError;
use crate::xkb::kccgst::parser::parse_item;
use crate::xkb::kccgst::parser::snoop_ty_and_name;
use crate::xkb::kccgst::token::Token;
use crate::xkb::meaning::MeaningCache;
use crate::xkb::span::Spanned;
use bstr::ByteSlice;
use std::sync::Arc;
use walkdir::WalkDir;

fn l(
    map: &mut CodeMap,
    i: &mut Interner,
    code: &Code,
) -> Result<Vec<Vec<Spanned<Token>>>, Spanned<LexerError>> {
    let span = map.add(None, None, code);
    let mut res = vec![];
    let mut lexer = Lexer::new(None, code, span.lo);
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
                    let item = parse_item(
                        &mut map,
                        &mut diagnostics,
                        &interner,
                        &mut meaning_cache,
                        tokens,
                        0,
                        &mut { u64::MAX },
                    );
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
