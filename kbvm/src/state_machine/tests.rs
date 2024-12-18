use crate::{
    builder::Builder,
    group_type::GroupType,
    modifier::{ModifierIndex, ModifierMask},
    routine::{
        Register::{self, *},
        Routine,
    },
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
        let on_press = Routine::builder()
            .load_lit(R0, SHIFT_MASK.0)
            .pressed_mods_inc(R0)
            .build();
        let on_release = Routine::builder().pressed_mods_dec(R0).build();

        builder
            .add_key(KEY_SHIFT)
            .add_group(0, &one_layer)
            .add_layer(0)
            .on_press(&on_press)
            .on_release(&on_release);
    }
    {
        const ACTION_MODS: Register = R0;
        const LOCKED_CURRENT: Register = R1;
        const LOCKED_ALREADY_PRESSED: Register = R2;
        const LOCKED_AFTER_PRESSED: Register = R3;
        const LOCKED_RELEASED: Register = R4;
        let on_press = Routine::builder()
            .load_lit(ACTION_MODS, LOCK_MASK.0)
            .pressed_mods_inc(ACTION_MODS)
            .locked_mods_load(LOCKED_CURRENT)
            .bit_or(LOCKED_AFTER_PRESSED, ACTION_MODS, LOCKED_CURRENT)
            .locked_mods_store(LOCKED_AFTER_PRESSED)
            .build();
        let on_release = Routine::builder()
            .pressed_mods_dec(ACTION_MODS)
            .bit_and(LOCKED_ALREADY_PRESSED, ACTION_MODS, LOCKED_CURRENT)
            .locked_mods_load(LOCKED_CURRENT)
            .bit_nand(LOCKED_RELEASED, LOCKED_CURRENT, LOCKED_ALREADY_PRESSED)
            .locked_mods_store(LOCKED_RELEASED)
            .build();

        builder
            .add_key(KEY_LOCK)
            .add_group(0, &one_layer)
            .add_layer(0)
            .on_press(&on_press)
            .on_release(&on_release);
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
