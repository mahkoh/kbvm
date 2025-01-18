use crate::Keycode;
use crate::xkb::Context;
use crate::xkb::diagnostic::WriteToStderr;

#[test]
fn absolute_path() {
    macro_rules! map {
        () => {
            r#"
                xkb_keymap {{
                    xkb_keycodes {{
                        <a> = 1;
                        <b> = 2;
                    }};
                    xkb_symbols {{
                        include "{}"
                    }};
                }};
            "#
        };
    }
    let path = format!("{}/src/xkb/code_loader/symbols.xkb", env!("CARGO_MANIFEST_DIR"));
    let map = format!(map!(), path);
    let mut context = Context::builder();
    context.clear();
    let context = context.build();
    let keymap = context.keymap_from_bytes(WriteToStderr, None, &map).unwrap();
    assert!(keymap.keys.contains_key(&Keycode::from_x11(1)));
    assert!(!keymap.keys.contains_key(&Keycode::from_x11(2)));
}
