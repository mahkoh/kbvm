use {
    crate::{
        config::DEFAULT_INCLUDE_DIR,
        xkb::{
            code::Code,
            code_map::CodeMap,
            diagnostic::{Diagnostic, DiagnosticKind, DiagnosticSink},
            interner::Interner,
            meaning::MeaningCache,
            rmlvo::{
                formatter::{Format, Formatter},
                lexer::Lexer,
                parser::{parse_line, ParserCache},
            },
        },
    },
    bstr::ByteSlice,
    std::{path::Path, sync::Arc},
    walkdir::WalkDir,
};

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
        );
        tokens.clear();
        let line = match line {
            Ok(l) => l,
            Err(e) => {
                let diag = Diagnostic::new(&mut map, DiagnosticKind::SyntaxError, e.val, e.span);
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
    if let Some(default_dir) = DEFAULT_INCLUDE_DIR {
        test_round_trip(
            &mut interner,
            &mut meaning_cache,
            format!("{}/rules/evdev", default_dir).as_ref(),
        );
    }
}
