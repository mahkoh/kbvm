use crate::xkb::Context;

#[test]
fn test() {
    let context = Context::builder().build();
    context.compose_from_locale("en_US");
}
