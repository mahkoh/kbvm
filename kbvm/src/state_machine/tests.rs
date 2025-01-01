use crate::{
    builder::Builder,
    group_type::GroupType,
    modifier::{ModifierIndex, ModifierMask},
    routine::Routine,
    state_machine::{Keycode, State},
};

const KEY_A: Keycode = Keycode(0x10);
const KEY_SHIFT: Keycode = Keycode(0x11);
const KEY_LOCK: Keycode = Keycode(0x12);

const SHIFT: ModifierIndex = ModifierIndex::new(0x0).unwrap();
const SHIFT_MASK: ModifierMask = SHIFT.to_mask();

const LOCK: ModifierIndex = ModifierIndex::new(0x1).unwrap();
const LOCK_MASK: ModifierMask = LOCK.to_mask();

#[test]
fn test() {
    let one_layer = GroupType::builder(ModifierMask::default()).build();
    let alphanumeric = GroupType::builder(SHIFT_MASK | LOCK_MASK)
        .map(SHIFT_MASK, 1)
        .map(LOCK_MASK, 1)
        .build();

    let mut builder = Builder::default();
    {
        let mut key = builder.add_key(KEY_A);
        let mut group = key.add_group(0, &alphanumeric);
        let _layer = group.add_layer(0);
        let _layer = group.add_layer(1);
    }
    {
        let mut routine = Routine::builder();
        let [r0] = routine.allocate_vars();
        routine
            .load_lit(r0, SHIFT_MASK.0)
            .pressed_mods_inc(r0)
            .on_release()
            .pressed_mods_dec(r0);
        let routine = routine.build();

        builder
            .add_key(KEY_SHIFT)
            .add_group(0, &one_layer)
            .add_layer(0)
            .routine(&routine);
    }
    {
        let mut routine = Routine::builder();
        let [action_mods, locked_current, locked_already_pressed, locked_after_pressed, locked_released] =
            routine.allocate_vars();
        routine
            .load_lit(action_mods, LOCK_MASK.0)
            .pressed_mods_inc(action_mods)
            .locked_mods_load(locked_current)
            .bit_or(locked_after_pressed, action_mods, locked_current)
            .locked_mods_store(locked_after_pressed)
            .on_release()
            .pressed_mods_dec(action_mods)
            .bit_and(locked_already_pressed, action_mods, locked_current)
            .locked_mods_load(locked_current)
            .bit_nand(locked_released, locked_current, locked_already_pressed)
            .locked_mods_store(locked_released);
        let routine = routine.build();

        builder
            .add_key(KEY_LOCK)
            .add_group(0, &one_layer)
            .add_layer(0)
            .routine(&routine);
    }
    let state_machine = builder.build_state_machine();
    let mut state = State::default();
    let mut key = |key: Keycode, down: bool| {
        let mut events = vec![];
        state_machine.handle_key(&mut state, &mut events, key, down);
        events
    };
    let events = key(KEY_SHIFT, true);
    println!("{:#?}", events);
    let events = key(KEY_LOCK, true);
    println!("{:#?}", events);
    let events = key(KEY_LOCK, false);
    println!("{:#?}", events);
    let events = key(KEY_SHIFT, false);
    println!("{:#?}", events);
    let events = key(KEY_LOCK, true);
    println!("{:#?}", events);
    let events = key(KEY_LOCK, false);
    println!("{:#?}", events);
}
