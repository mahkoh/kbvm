use {
    crate::xkb::{
        code::Code,
        code_loader::CodeLoader,
        code_map::CodeMap,
        context::Environment,
        diagnostic::DiagnosticSink,
        interner::Interner,
        kccgst::formatter::{Format, Formatter},
        meaning::MeaningCache,
        rmlvo::resolver::{create_item, Group},
        span::SpanExt,
    },
    bstr::ByteSlice,
    std::{path::Path, sync::Arc},
};

#[test]
fn test() {
    let mut map = CodeMap::default();
    let mut meaning_cache = MeaningCache::default();
    let mut interner = Interner::default();
    let mut loader = CodeLoader::new(&[Arc::new(Path::new("/usr/share/X11/xkb").to_path_buf())]);
    let mut diag = vec![];
    let mut intern = |s: &str| {
        let code = Arc::new(s.as_bytes().to_vec());
        let code = Code::new(&code);
        let span = map.add(None, None, &code);
        interner.intern(&code.to_slice()).spanned2(span)
    };
    let mut remaining_runtime = u64::MAX;
    let rules = intern("evdev");
    let groups = [
        Group {
            layout: Some(intern("us").val),
            variant: Some(intern("dvorak").val),
        },
        Group {
            layout: Some(intern("ori").val),
            variant: Some(intern("abcd").val),
        },
    ];
    let options = [
        intern("caps:internal").val,
        intern("caps:shiftlock").val,
        intern("grab:break_actions").val,
        intern("ctrl:swapcaps_and_switch_layout").val,
    ];
    let model = Some(intern("sun_type6_jp").val);
    let item = create_item(
        &mut map,
        &mut interner,
        &mut loader,
        &mut DiagnosticSink::new(&mut diag),
        &mut meaning_cache,
        &mut remaining_runtime,
        rules,
        model,
        &options,
        &groups,
        1024,
        &Environment::default(),
    );
    let mut out = vec![];
    let mut formatter = Formatter::new(&interner, &mut out);
    item.format(&mut formatter).unwrap();
    for diag in diag {
        eprintln!("{}", diag.with_code());
    }
    eprintln!("{}", out.as_bstr());
}
