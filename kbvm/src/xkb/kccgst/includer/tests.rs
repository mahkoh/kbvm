use {
    crate::xkb::{
        code::Code,
        code_loader::CodeLoader,
        code_map::CodeMap,
        diagnostic::DiagnosticSink,
        interner::Interner,
        kccgst::{
            ast_cache::AstCache,
            formatter::{Format, Formatter},
            lexer::Lexer,
            meaning::MeaningCache,
            parser::parse_item,
        },
    },
    bstr::ByteSlice,
    std::{path::Path, sync::Arc},
};
use crate::xkb::kccgst::embedder::embed;
use crate::xkb::kccgst::includer::resolve_includes;

#[test]
fn test() {
    let src = r#"
    xkb_keymap {
        xkb_symbols   { alternate "pc+us+inet(evdev)"	};
    };
    "#;
    let code = Code::new(&Arc::new(src.as_bytes().to_vec()));
    let mut map = CodeMap::default();
    let span = map.add(None, None, &code);
    let mut diagnostics = vec![];
    let mut cache = AstCache::default();
    let mut loader = CodeLoader::new(&[Arc::new(
        Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../xkeyboard-config")).to_path_buf(),
    )]);
    let mut interner = Interner::default();
    let mut meaning_cache = MeaningCache::default();
    let mut lexer = Lexer::new(None, &code, span.lo);
    let mut tokens = vec![];
    lexer.lex_item(&mut interner, &mut tokens).unwrap();
    let mut item = parse_item(&interner, &mut meaning_cache, &tokens, 0).unwrap();
    resolve_includes(
        &mut DiagnosticSink::new(&mut diagnostics),
        &mut map,
        &mut cache,
        &mut loader,
        &mut interner,
        &mut meaning_cache,
        &mut item.val,
    );
    embed(&mut item.val);

    let mut out = vec![];
    let mut formatter = Formatter::new(&interner, &mut out);
    item.val.format(&mut formatter).unwrap();
    println!("{}", out.as_bstr());

    for diagnostic in diagnostics {
        println!("{}", diagnostic.with_code());
    }
}
