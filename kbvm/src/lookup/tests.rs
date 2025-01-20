use crate::{
    builder::Builder,
    evdev,
    lookup::{Lookup, LookupTable},
    syms,
    xkb::{diagnostic::WriteToStderr, Context},
    GroupIndex, Keycode, Keysym, ModifierMask,
};

fn lookup_table(map: &str) -> LookupTable {
    let mut context = Context::builder();
    context.clear();
    context.enable_environment(false);
    context.append_path(&format!(
        "{}/../type-tests/include",
        env!("CARGO_MANIFEST_DIR")
    ));
    let map = context
        .build()
        .keymap_from_bytes(WriteToStderr, None, map)
        .unwrap();
    map.to_builder().build_lookup_table()
}

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
    let lookup = lookup_table(MAP);
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

fn create_custom_lookup<A, F>(
    map: &str,
    transform: F,
) -> impl Fn(Keycode, ModifierMask, A) -> (Keysym, char)
where
    F: for<'a> Fn(Lookup<'a>, A) -> Lookup<'a>,
{
    let lookup = lookup_table(map);
    move |kc, mask, a| {
        let lookup = lookup.lookup(GroupIndex::ZERO, mask, kc);
        let lookup = transform(lookup, a);
        lookup
            .into_iter()
            .map(|p| (p.keysym(), p.char().unwrap()))
            .next()
            .unwrap()
    }
}

#[test]
fn no_ctrl_fallback() {
    const MAP: &str = r#"
        xkb_keymap {
            xkb_keycodes {
                include "generated"
            };
            xkb_symbols {
                key <a> { [ kana_A ], [ a ] };
            };
        };
    "#;
    let prod = create_custom_lookup(MAP, |lookup, fallback| lookup.with_ctrl_fallback(fallback));
    assert_eq!(
        prod(evdev::A, ModifierMask::NONE, false),
        (syms::kana_A, 'ア'),
    );
    assert_eq!(
        prod(evdev::A, ModifierMask::NONE, true),
        (syms::kana_A, 'ア'),
    );
    assert_eq!(
        prod(evdev::A, ModifierMask::CONTROL, false),
        (syms::kana_A, 'ア'),
    );
    assert_eq!(
        prod(evdev::A, ModifierMask::CONTROL, true),
        (syms::a, '\u{1}'),
    );
}

#[test]
fn no_ctrl_transform() {
    const MAP: &str = r#"
        xkb_keymap {
            xkb_keycodes {
                include "generated"
            };
            xkb_symbols {
                key <a> { [ kana_A ], [ a ] };
                key <b> { [ b ] };
            };
        };
    "#;
    let prod = create_custom_lookup(MAP, |lookup, transform| {
        lookup.with_ctrl_transform(transform)
    });
    assert_eq!(
        prod(evdev::A, ModifierMask::NONE, false),
        (syms::kana_A, 'ア'),
    );
    assert_eq!(
        prod(evdev::A, ModifierMask::NONE, true),
        (syms::kana_A, 'ア'),
    );
    assert_eq!(
        prod(evdev::A, ModifierMask::CONTROL, false),
        (syms::kana_A, 'ア'),
    );
    assert_eq!(
        prod(evdev::A, ModifierMask::CONTROL, true),
        (syms::a, '\u{1}'),
    );
    assert_eq!(prod(evdev::B, ModifierMask::NONE, false), (syms::b, 'b'),);
    assert_eq!(prod(evdev::B, ModifierMask::NONE, true), (syms::b, 'b'),);
    assert_eq!(prod(evdev::B, ModifierMask::CONTROL, false), (syms::b, 'b'),);
    assert_eq!(
        prod(evdev::B, ModifierMask::CONTROL, true),
        (syms::b, '\u{2}'),
    );
}

#[test]
fn no_caps_transform() {
    const MAP: &str = r#"
        xkb_keymap {
            xkb_keycodes {
                include "generated"
            };
            xkb_symbols {
                key <a> { type = "ALPHABETIC", [ a, u ] };
                key <b> { [ b ] };
            };
        };
    "#;
    let prod = create_custom_lookup(MAP, |lookup, transform| {
        lookup.with_caps_transform(transform)
    });
    assert_eq!(prod(evdev::A, ModifierMask::NONE, false), (syms::a, 'a'),);
    assert_eq!(prod(evdev::A, ModifierMask::NONE, true), (syms::a, 'a'),);
    assert_eq!(prod(evdev::A, ModifierMask::LOCK, false), (syms::u, 'u'),);
    assert_eq!(prod(evdev::A, ModifierMask::LOCK, true), (syms::u, 'u'),);
    assert_eq!(prod(evdev::B, ModifierMask::NONE, false), (syms::b, 'b'),);
    assert_eq!(prod(evdev::B, ModifierMask::NONE, true), (syms::b, 'b'),);
    assert_eq!(prod(evdev::B, ModifierMask::LOCK, false), (syms::b, 'b'),);
    assert_eq!(prod(evdev::B, ModifierMask::LOCK, true), (syms::B, 'B'),);
}

#[test]
fn setters() {
    let table = Builder::default().build_lookup_table();
    let lookup = table.lookup(GroupIndex::ZERO, ModifierMask::NONE, evdev::A);
    assert_eq!(lookup.use_ctrl_fallback, true);
    assert_eq!(lookup.do_ctrl_transform, true);
    assert_eq!(lookup.do_caps_transform, true);
    {
        let mut lookup = lookup;
        lookup.set_ctrl_fallback(false);
        assert_eq!(lookup.use_ctrl_fallback, false);
    }
    {
        let mut lookup = lookup;
        lookup.set_ctrl_transform(false);
        assert_eq!(lookup.do_ctrl_transform, false);
    }
    {
        let mut lookup = lookup;
        lookup.set_caps_transform(false);
        assert_eq!(lookup.do_caps_transform, false);
    }
    {
        let lookup = lookup.with_ctrl_fallback(false);
        assert_eq!(lookup.use_ctrl_fallback, false);
    }
    {
        let lookup = lookup.with_ctrl_transform(false);
        assert_eq!(lookup.do_ctrl_transform, false);
    }
    {
        let lookup = lookup.with_caps_transform(false);
        assert_eq!(lookup.do_caps_transform, false);
    }
}

#[test]
fn remaining_mods() {
    const MAP: &str = r#"
        xkb_keymap {
            xkb_keycodes {
                include "generated"
            };
            xkb_types {
                type "KEYPAD" {
                    modifiers = Shift + Mod2;
                    map[Mod2] = Level2;
                    preserve[Shift] = Shift;
                };
            };
            xkb_symbols {
                key <a> { [ a, A ] };
                key <kp4> {
                    type = "KEYPAD",
                    [ leftarrow, 4 ],
                };
            };
        };
    "#;
    let lookup = lookup_table(MAP);
    let prod = |kc, mask| {
        let lookup = lookup.lookup(GroupIndex::ZERO, mask, kc);
        lookup
            .into_iter()
            .map(|p| (p.keysym(), lookup.remaining_mods()))
            .next()
            .unwrap()
    };
    assert_eq!(
        prod(evdev::A, ModifierMask::NONE),
        (syms::a, ModifierMask::NONE),
    );
    assert_eq!(
        prod(evdev::A, ModifierMask::CONTROL),
        (syms::a, ModifierMask::CONTROL),
    );
    assert_eq!(
        prod(evdev::A, ModifierMask::SHIFT),
        (syms::A, ModifierMask::NONE),
    );
    assert_eq!(
        prod(evdev::A, ModifierMask::SHIFT | ModifierMask::CONTROL),
        (syms::A, ModifierMask::CONTROL),
    );
    assert_eq!(
        prod(evdev::KP4, ModifierMask::NONE),
        (syms::leftarrow, ModifierMask::NONE),
    );
    assert_eq!(
        prod(evdev::KP4, ModifierMask::MOD2),
        (syms::_4, ModifierMask::NONE),
    );
    assert_eq!(
        prod(evdev::KP4, ModifierMask::SHIFT),
        (syms::leftarrow, ModifierMask::SHIFT),
    );
    assert_eq!(
        prod(evdev::KP4, ModifierMask::SHIFT | ModifierMask::MOD2),
        (syms::leftarrow, ModifierMask::NONE),
    );
    assert_eq!(
        prod(evdev::KP4, ModifierMask::CONTROL),
        (syms::leftarrow, ModifierMask::CONTROL),
    );
    assert_eq!(
        prod(evdev::KP4, ModifierMask::CONTROL | ModifierMask::MOD2),
        (syms::_4, ModifierMask::CONTROL),
    );
    assert_eq!(
        prod(evdev::KP4, ModifierMask::CONTROL | ModifierMask::SHIFT),
        (syms::leftarrow, ModifierMask::CONTROL | ModifierMask::SHIFT),
    );
    assert_eq!(
        prod(
            evdev::KP4,
            ModifierMask::CONTROL | ModifierMask::SHIFT | ModifierMask::MOD2
        ),
        (syms::leftarrow, ModifierMask::CONTROL),
    );
}

#[test]
fn did_transform() {
    const MAP: &str = r#"
        xkb_keymap {
            xkb_keycodes {
                include "generated"
            };
            xkb_symbols {
                key <a> { [ a ] };
                key <b> { [ kana_A ], [ a ] };
            };
        };
    "#;
    let lookup = lookup_table(MAP);
    let prod = |kc, mask| {
        let lookup = lookup.lookup(GroupIndex::ZERO, mask, kc);
        lookup.into_iter().next().unwrap()
    };
    assert!(!prod(evdev::A, ModifierMask::NONE).did_ctrl_fallback());
    assert!(!prod(evdev::A, ModifierMask::NONE).did_ctrl_transform());
    assert!(!prod(evdev::A, ModifierMask::NONE).did_caps_transform());
    assert!(!prod(evdev::A, ModifierMask::LOCK).did_ctrl_fallback());
    assert!(!prod(evdev::A, ModifierMask::LOCK).did_ctrl_transform());
    assert!(prod(evdev::A, ModifierMask::LOCK).did_caps_transform());
    assert!(!prod(evdev::A, ModifierMask::CONTROL).did_ctrl_fallback());
    assert!(prod(evdev::A, ModifierMask::CONTROL).did_ctrl_transform());
    assert!(!prod(evdev::A, ModifierMask::CONTROL).did_caps_transform());
    assert!(prod(evdev::B, ModifierMask::CONTROL).did_ctrl_fallback());
    assert!(prod(evdev::B, ModifierMask::CONTROL).did_ctrl_transform());
    assert!(!prod(evdev::B, ModifierMask::CONTROL).did_caps_transform());
    assert!(prod(evdev::B, ModifierMask::CONTROL | ModifierMask::LOCK).did_ctrl_fallback());
    assert!(prod(evdev::B, ModifierMask::CONTROL | ModifierMask::LOCK).did_ctrl_transform());
    assert!(prod(evdev::B, ModifierMask::CONTROL | ModifierMask::LOCK).did_caps_transform());
}

#[test]
fn effective_group() {
    const MAP: &str = r#"
        xkb_keymap {
            xkb_keycodes {
                include "generated"
            };
            xkb_symbols {
                key <a> { [ a ] };
                key <b> { [ a ], [ b ] };
                key <c> { groupsClamp, [ a ], [ b ] };
                key <d> { groupsRedirect = Group2, [ a ], [ b ] };
                key <e> { };
            };
        };
    "#;
    let lookup = lookup_table(MAP);
    let prod = |kc, idx: u32| lookup.effective_group(GroupIndex(idx), kc).map(|g| g.0);
    assert_eq!(prod(evdev::A, 0), Some(0));
    assert_eq!(prod(evdev::A, 1), Some(0));
    assert_eq!(prod(evdev::A, !0), Some(0));
    assert_eq!(prod(evdev::B, 0), Some(0));
    assert_eq!(prod(evdev::B, 1), Some(1));
    assert_eq!(prod(evdev::B, 2), Some(0));
    assert_eq!(prod(evdev::B, 3), Some(1));
    assert_eq!(prod(evdev::B, !0), Some(1));
    assert_eq!(prod(evdev::C, 0), Some(0));
    assert_eq!(prod(evdev::C, 1), Some(1));
    assert_eq!(prod(evdev::C, 2), Some(1));
    assert_eq!(prod(evdev::C, 3), Some(1));
    assert_eq!(prod(evdev::C, !0), Some(0));
    assert_eq!(prod(evdev::C, !0 - 1), Some(0));
    assert_eq!(prod(evdev::D, 0), Some(0));
    assert_eq!(prod(evdev::D, 1), Some(1));
    assert_eq!(prod(evdev::D, 2), Some(1));
    assert_eq!(prod(evdev::D, 3), Some(1));
    assert_eq!(prod(evdev::D, !0), Some(1));
    assert_eq!(prod(evdev::D, !0 - 1), Some(1));
    assert_eq!(prod(evdev::E, 0), None);
    assert_eq!(prod(evdev::E, 1), None);
    assert_eq!(prod(evdev::E, 2), None);
    assert_eq!(prod(evdev::E, 3), None);
    assert_eq!(prod(evdev::E, !0), None);
    assert_eq!(prod(evdev::E, !0 - 1), None);
}

#[test]
fn repeats() {
    const MAP: &str = r#"
        xkb_keymap {
            xkb_keycodes {
                include "generated"
            };
            xkb_symbols {
                key <a> { [ a ] };
                key <b> { repeats = false, [ a ] };
            };
        };
    "#;
    let lookup = lookup_table(MAP);
    assert_eq!(lookup.repeats(evdev::A), true);
    assert_eq!(lookup.repeats(evdev::B), false);
    assert_eq!(lookup.repeats(evdev::C), true);
}
