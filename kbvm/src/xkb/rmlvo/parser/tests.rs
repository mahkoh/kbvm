use crate::config::DEFAULT_INCLUDE_DIR;
use crate::xkb::code::Code;
use crate::xkb::code_map::CodeMap;
use crate::xkb::context::Environment;
use crate::xkb::diagnostic::Diagnostic;
use crate::xkb::diagnostic::DiagnosticSink;
use crate::xkb::interner::Interner;
use crate::xkb::meaning::MeaningCache;
use crate::xkb::rmlvo::formatter::Format;
use crate::xkb::rmlvo::formatter::Formatter;
use crate::xkb::rmlvo::lexer::Lexer;
use crate::xkb::rmlvo::parser::ParserCache;
use crate::xkb::rmlvo::parser::parse_line;
use bstr::ByteSlice;
use std::path::Path;
use std::sync::Arc;
use walkdir::WalkDir;

fn test_round_trip(interner: &mut Interner, meaning_cache: &mut MeaningCache, path: &Path) {
    let path = Arc::new(path.to_path_buf());
    let input = std::fs::read(path.as_path()).unwrap();
    let code = Code::new(&Arc::new(input));
    let mut map = CodeMap::default();
    let span = map.add(Some(&path), None, &code);
    let mut diag = vec![];
    let mut diagnostics = DiagnosticSink::new(&mut diag);
    let mut lexer = Lexer::new(&path, &code, span.lo);
    let mut tokens = vec![];
    let mut cache = ParserCache::default();
    println!("{}:", path.display());
    let mut out = vec![];
    loop {
        lexer.lex_line(interner, &mut tokens).unwrap();
        if tokens.is_empty() {
            break;
        }
        let line = parse_line(
            &mut map,
            &mut diagnostics,
            interner,
            &mut cache,
            meaning_cache,
            &tokens,
            &Environment::default(),
        );
        tokens.clear();
        let line = match line {
            Ok(l) => l,
            Err(e) => {
                let diag = Diagnostic::new(&mut map, e.val.diagnostic_kind(), e.val, e.span);
                panic!("{}", diag.with_code());
            }
        };
        let mut formatter = Formatter::new(interner, &mut out);
        line.val.format(&mut formatter).unwrap();
    }
    println!("{}", out.as_bstr());
}

#[test]
fn round_trip() {
    let mut interner = Interner::default();
    let mut meaning_cache = MeaningCache::default();
    let dir = format!("{}/../xkeyboard-config/rules", env!("CARGO_MANIFEST_DIR"));
    let dir = WalkDir::new(&dir);
    for f in dir {
        let f = f.unwrap();
        if f.path().is_file() {
            if !f.path().as_os_str().as_encoded_bytes().ends_with(b".part") {
                continue;
            }
            test_round_trip(&mut interner, &mut meaning_cache, f.path());
        }
    }
    test_round_trip(
        &mut interner,
        &mut meaning_cache,
        format!("{}/rules/evdev", DEFAULT_INCLUDE_DIR).as_ref(),
    );
}
