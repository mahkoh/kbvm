use crate::xkb::{diagnostic::WriteToStderr, Context};

#[test]
fn test() {
    let context = Context::builder().build();
    let table = context.compose_table_builder().build(WriteToStderr);
    println!("{}", table.unwrap().format());
}
