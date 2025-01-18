use crate::{state_machine::Event, Components, GroupDelta, GroupIndex, ModifierMask};

#[test]
fn apply_event() {
    let mut c = Components::default();
    assert!(c.apply_event(Event::ModsPressed(ModifierMask(0x1))));
    assert_eq!(c.mods_pressed.0, 0x1);
    assert!(c.apply_event(Event::ModsLatched(ModifierMask(0x2))));
    assert_eq!(c.mods_latched.0, 0x2);
    assert!(c.apply_event(Event::ModsLocked(ModifierMask(0x4))));
    assert_eq!(c.mods_locked.0, 0x4);
    assert!(c.apply_event(Event::ModsEffective(ModifierMask(0x8))));
    assert_eq!(c.mods.0, 0x8);
    assert!(c.apply_event(Event::GroupPressed(GroupDelta(0x1))));
    assert_eq!(c.group_pressed.0, 0x1);
    assert!(c.apply_event(Event::GroupLatched(GroupDelta(0x2))));
    assert_eq!(c.group_latched.0, 0x2);
    assert!(c.apply_event(Event::GroupLocked(GroupIndex(0x4))));
    assert_eq!(c.group_locked.0, 0x4);
    assert!(c.apply_event(Event::GroupEffective(GroupIndex(0x8))));
    assert_eq!(c.group.0, 0x8);
    assert!(!c.apply_event(Event::GroupEffective(GroupIndex(0x8))));
    assert_eq!(c.group.0, 0x8);
}

#[test]
fn update_effective() {
    let mut c = Components::default();

    c.mods_pressed = ModifierMask(0x1);
    c.mods_latched = ModifierMask(0x2);
    c.mods_locked = ModifierMask(0x4);
    c.group_pressed = GroupDelta(0x1);
    c.group_latched = GroupDelta(0x2);
    c.group_locked = GroupIndex(0x4);
    c.update_effective();
    assert_eq!(c.mods.0, 0x7);
    assert_eq!(c.group.0, 0x7);

    c.mods_pressed = ModifierMask(0x1);
    c.mods_latched = ModifierMask(0x3);
    c.mods_locked = ModifierMask(0x7);
    c.group_pressed = GroupDelta(0x1);
    c.group_latched = GroupDelta(0x3);
    c.group_locked = GroupIndex(0x7);
    c.update_effective();
    assert_eq!(c.mods.0, 0x7);
    assert_eq!(c.group.0, 0xB);
}
