use crate::{
    syms,
    xkb::{diagnostic::WriteToStderr, Context},
    GroupIndex, Keycode, ModifierMask,
};

#[test]
fn clamp_negative() {
    const MAP: &str = r#"
        xkb_keymap {
            xkb_keycodes {
                <a> = 1;
                <b> = 2;
                <c> = 3;
            };
            xkb_symbols {
                key <a> {
                    groupsClamp,
                    [ a ], [ b ],
                };
                key <b> {
                    [ a ], [ b ],
                };
                key <c> {
                    groupsRedirect = Group3,
                    [ a ], [ b ],
                };
            };
        };
    "#;
    let map = Context::default()
        .keymap_from_bytes(WriteToStderr, None, MAP)
        .unwrap();
    let lookup = map.to_builder().build_lookup_table();
    let keysym = |kc| {
        lookup
            .lookup(GroupIndex(!0), ModifierMask::NONE, Keycode::from_x11(kc))
            .into_iter()
            .next()
            .unwrap()
            .keysym()
    };
    assert_eq!(keysym(1), syms::a);
    assert_eq!(keysym(2), syms::b);
    assert_eq!(keysym(3), syms::a);
}
