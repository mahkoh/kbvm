use {
    crate::xkb::{Context, diagnostic::WriteToStderr},
    std::path::Path,
};

#[test]
fn locale() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../compose-tests/xlocaledir")
        .display()
        .to_string();
    let mut context = Context::builder();
    context.clear();
    context.enable_environment(true);
    context.environment_accessor(move |s| match s {
        "XLOCALEDIR" => Some(root.clone()),
        _ => None,
    });
    let context = context.build();

    macro_rules! build {
        ($locale:expr) => {{
            let mut builder = context.compose_table_builder();
            builder.locale($locale);
            builder.build(WriteToStderr)
        }};
    }

    assert!(build!("en_US.UTF-8").is_some());
    assert!(build!("C.UTF-8").is_some());
    assert!(build!("univ.utf8").is_some());
    assert!(build!("C").is_some());
    assert!(build!("blabla").is_none());
}
