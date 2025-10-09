use crate::xkb::{Context, diagnostic::WriteToStderr};

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

#[test]
fn multiple_actions() {
    let context = Context::default();
    let map = context
        .keymap_from_bytes(WriteToStderr, None, include_str!("map6.xkb"))
        .unwrap();
    let with_actions = format!("{}\n", map.format().multiple_actions_per_level(true));
    let expected = include_str!("map7.xkb");
    assert_eq!(with_actions, expected);
    let without_actions = format!("{}\n", map.format());
    let expected = include_str!("map8.xkb");
    assert_eq!(without_actions, expected);
}
