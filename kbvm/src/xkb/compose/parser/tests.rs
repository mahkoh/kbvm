use crate::xkb::{diagnostic::WriteToStderr, Context};

#[test]
fn round_trip() {
    let context = Context::builder().build();
    let mut builder = context.compose_table_builder();
    builder.file(format!(
        "{}/../compose-tests/xlocaledir/en_US.UTF-8/Compose",
        env!("CARGO_MANIFEST_DIR")
    ));
    let table = builder.build(WriteToStderr).unwrap();
    let buf = table.format().to_string();
    let mut builder = context.compose_table_builder();
    builder.buffer(&buf);
    let table2 = builder.build(WriteToStderr).unwrap();
    if table != table2 {
        eprintln!("original:  {}", table.format());
        eprintln!("roundtrip: {}", table2.format());
        panic!();
    }
    assert_eq!(table, table2);
    let mut count = 0;
    let mut iter = table.iter();
    while iter.next().is_some() {
        count += 1;
    }
    assert_eq!(count, 5756);
}
