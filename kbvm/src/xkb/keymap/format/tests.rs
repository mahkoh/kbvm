use crate::xkb::{diagnostic::WriteToStderr, Context};

#[test]
fn rename_long_keys() {
    let context = Context::default();
    let map = context
        .keymap_from_bytes(WriteToStderr, None, include_str!("map1.xkb"))
        .unwrap();
    let actual = format!("{}\n", map.format().rename_long_keys(true));
    let expected = include_str!("map2.xkb");
    assert_eq!(actual, expected);
}

#[test]
fn single_line() {
    let context = Context::default();
    let map = context
        .keymap_from_bytes(WriteToStderr, None, include_str!("map1.xkb"))
        .unwrap();
    let actual = format!(
        "{}\n",
        map.format().rename_long_keys(true).single_line(true)
    );
    let expected = include_str!("map3.xkb");
    assert_eq!(actual, expected);
}
#[test]
fn lookup_only() {
    let context = Context::default();
    let map = context
        .keymap_from_bytes(WriteToStderr, None, include_str!("map4.xkb"))
        .unwrap();
    let actual = format!("{}\n", map.format().lookup_only(true));
    let expected = include_str!("map5.xkb");
    assert_eq!(actual, expected);
}
