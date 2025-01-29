use crate::{
    builder::Redirect,
    syms,
    xkb::{
        controls::ControlMask,
        diagnostic::WriteToStderr,
        group::GroupMask,
        group_component::GroupComponent,
        indicator::IndicatorIdx,
        keymap::{actions::ModsSetAction, Action, Indicator, KeyBehavior},
        mod_component::ModComponentMask,
        Context,
    },
    Components, ControlsMask, GroupDelta, GroupIndex, ModifierMask,
};

#[test]
fn from_lookup() {
    for map in [include_str!("map1.xkb"), include_str!("map3.xkb")] {
        let keymap = Context::default()
            .keymap_from_bytes(WriteToStderr, None, map)
            .unwrap();
        let lookup = keymap.to_builder().build_lookup_table();
        let out = lookup.to_xkb_keymap();
        let str = format!("{:#}\n", out.format());
        println!("{}", str);
        assert_eq!(map, str);
    }
}

#[test]
fn inspection() {
    let keymap = Context::default()
        .keymap_from_bytes(WriteToStderr, None, include_str!("map2.xkb"))
        .unwrap();
    {
        let mut mods = keymap.virtual_modifiers();
        let m1 = mods.next().unwrap();
        assert_eq!(m1.name(), "A");
        assert_eq!(m1.mask().0, 0x1);
        assert_eq!(mods.next(), None);
    }
    {
        let mut keys = keymap.keys();
        let k1 = keys.next().unwrap();
        assert_eq!(k1.name(), "a");
        assert_eq!(k1.keycode().raw(), 1);
        assert_eq!(k1.repeats(), false);
        assert_eq!(k1.redirect(), Redirect::Clamp);
        assert_eq!(k1.behavior(), Some(&KeyBehavior::Lock));
        {
            let mut groups = k1.groups();
            let g1 = groups.next().unwrap();
            assert_eq!(g1, None);
            let g2 = groups.next().unwrap().unwrap();
            assert_eq!(g2.ty().mask(), ModifierMask::MOD1 | ModifierMask::MOD2);
            {
                let mut mappings = g2.ty().mappings();
                let m1 = mappings.next().unwrap();
                assert_eq!(m1.mask(), ModifierMask::MOD1);
                assert_eq!(m1.preserved(), ModifierMask::NONE);
                assert_eq!(m1.consumed(), ModifierMask::MOD1);
                assert_eq!(m1.level(), 0);
                let m2 = mappings.next().unwrap();
                assert_eq!(m2.mask(), ModifierMask::MOD1 | ModifierMask::MOD2);
                assert_eq!(m2.preserved(), ModifierMask::MOD1);
                assert_eq!(m2.consumed(), ModifierMask::MOD2);
                assert_eq!(m2.level(), 1);
                assert_eq!(mappings.next(), None);
            }
            {
                let mut levels = g2.levels();
                let l1 = levels.next().unwrap();
                assert_eq!(l1.symbols(), [syms::a, syms::b]);
                assert_eq!(l1.actions(), []);
                let l2 = levels.next().unwrap();
                assert_eq!(l2.symbols(), []);
                assert_eq!(
                    l2.actions(),
                    [Action::ModsSet(ModsSetAction {
                        clear_locks: false,
                        modifiers: ModifierMask::MOD1,
                    })]
                );
                assert_eq!(levels.next(), None);
            }
            assert_eq!(groups.next(), None);
        }
        assert_eq!(keys.next(), None);
    }
    {
        let mut indicators = keymap.indicators();
        let i1 = indicators.next().unwrap();
        assert_eq!(i1.name(), "A");
        assert_eq!(indicators.next(), None);
    }
}

#[test]
fn indicator_matcher() {
    let mut i = Indicator {
        virt: false,
        index: IndicatorIdx::ONE,
        name: Default::default(),
        modifier_mask: ModifierMask::MOD1 | ModifierMask::MOD3,
        group_mask: Default::default(),
        controls: Default::default(),
        mod_components: ModComponentMask::EFFECTIVE,
        group_component: Default::default(),
    };
    {
        let m = i.matcher();
        let mut c = Components::default();
        assert_eq!(m.matches(&c), false);
        c.mods = ModifierMask::MOD1;
        assert_eq!(m.matches(&c), true);
        c.mods = ModifierMask::MOD2;
        assert_eq!(m.matches(&c), false);
        c.mods_pressed = ModifierMask::MOD1;
        assert_eq!(m.matches(&c), false);
    }
    {
        i.mod_components = ModComponentMask::NONE;
        let m = i.matcher();
        let mut c = Components::default();
        assert_eq!(m.matches(&c), false);
        c.mods = ModifierMask::MOD1;
        assert_eq!(m.matches(&c), false);
        c.mods = ModifierMask::MOD2;
        assert_eq!(m.matches(&c), false);
        c.mods_pressed = ModifierMask::MOD1;
        assert_eq!(m.matches(&c), false);
    }
    {
        i.mod_components = ModComponentMask::BASE;
        let m = i.matcher();
        let mut c = Components::default();
        assert_eq!(m.matches(&c), false);
        c.mods_pressed = ModifierMask::MOD2;
        c.mods_latched = ModifierMask::MOD1;
        c.mods_locked = ModifierMask::MOD1;
        c.mods = ModifierMask::MOD1;
        assert_eq!(m.matches(&c), false);
        c.mods_pressed = ModifierMask::MOD1;
        assert_eq!(m.matches(&c), true);
    }
    {
        i.mod_components = ModComponentMask::LATCHED;
        let m = i.matcher();
        let mut c = Components::default();
        assert_eq!(m.matches(&c), false);
        c.mods_pressed = ModifierMask::MOD1;
        c.mods_latched = ModifierMask::MOD2;
        c.mods_locked = ModifierMask::MOD1;
        c.mods = ModifierMask::MOD1;
        assert_eq!(m.matches(&c), false);
        c.mods_latched = ModifierMask::MOD1;
        assert_eq!(m.matches(&c), true);
    }
    {
        i.mod_components = ModComponentMask::LOCKED;
        let m = i.matcher();
        let mut c = Components::default();
        assert_eq!(m.matches(&c), false);
        c.mods_pressed = ModifierMask::MOD1;
        c.mods_latched = ModifierMask::MOD1;
        c.mods_locked = ModifierMask::MOD2;
        c.mods = ModifierMask::MOD1;
        assert_eq!(m.matches(&c), false);
        c.mods_locked = ModifierMask::MOD1;
        assert_eq!(m.matches(&c), true);
    }
    {
        i.mod_components = ModComponentMask::LATCHED | ModComponentMask::LOCKED;
        let m = i.matcher();
        let mut c = Components::default();
        assert_eq!(m.matches(&c), false);
        c.mods_pressed = ModifierMask::MOD1;
        c.mods_latched = ModifierMask::MOD2;
        c.mods_locked = ModifierMask::MOD2;
        c.mods = ModifierMask::MOD1;
        assert_eq!(m.matches(&c), false);
        c.mods_locked = ModifierMask::MOD1;
        assert_eq!(m.matches(&c), true);
        c.mods_latched = ModifierMask::MOD1;
        c.mods_locked = ModifierMask::MOD2;
        assert_eq!(m.matches(&c), true);
    }
    {
        i.mod_components = ModComponentMask::NONE;
        i.group_component = GroupComponent::Effective;
        i.group_mask = GroupMask(2);
        let m = i.matcher();
        let mut c = Components::default();
        assert_eq!(m.matches(&c), false);
        c.group_pressed = GroupDelta(1);
        c.group_latched = GroupDelta(1);
        c.group_locked = GroupIndex(1);
        c.group = GroupIndex(2);
        assert_eq!(m.matches(&c), false);
        c.group = GroupIndex(1);
        assert_eq!(m.matches(&c), true);
    }
    {
        i.group_component = GroupComponent::Base;
        let m = i.matcher();
        let mut c = Components::default();
        assert_eq!(m.matches(&c), false);
        c.group_pressed = GroupDelta(0);
        c.group_latched = GroupDelta(1);
        c.group_locked = GroupIndex(1);
        c.group = GroupIndex(1);
        assert_eq!(m.matches(&c), false);
        c.group_pressed = GroupDelta(9);
        assert_eq!(m.matches(&c), true);
    }
    {
        i.group_component = GroupComponent::Latched;
        let m = i.matcher();
        let mut c = Components::default();
        assert_eq!(m.matches(&c), false);
        c.group_pressed = GroupDelta(1);
        c.group_latched = GroupDelta(0);
        c.group_locked = GroupIndex(1);
        c.group = GroupIndex(1);
        assert_eq!(m.matches(&c), false);
        c.group_latched = GroupDelta(9);
        assert_eq!(m.matches(&c), true);
    }
    {
        i.group_component = GroupComponent::Locked;
        let m = i.matcher();
        let mut c = Components::default();
        assert_eq!(m.matches(&c), false);
        c.group_pressed = GroupDelta(1);
        c.group_latched = GroupDelta(1);
        c.group_locked = GroupIndex(2);
        c.group = GroupIndex(1);
        assert_eq!(m.matches(&c), false);
        c.group_locked = GroupIndex(1);
        assert_eq!(m.matches(&c), true);
    }
    {
        i.group_component = GroupComponent::None;
        i.controls = ControlMask::OVERLAY1;
        let m = i.matcher();
        let mut c = Components::default();
        assert_eq!(m.matches(&c), false);
        c.controls = ControlsMask(ControlMask::OVERLAY2.0 as u32);
        assert_eq!(m.matches(&c), false);
        c.controls |= ControlsMask(ControlMask::OVERLAY1.0 as u32);
        assert_eq!(m.matches(&c), true);
        c.controls = ControlsMask(ControlMask::OVERLAY1.0 as u32);
        assert_eq!(m.matches(&c), true);
    }
}
