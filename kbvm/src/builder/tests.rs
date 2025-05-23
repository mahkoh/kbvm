use crate::{
    builder::{Builder, GroupBuilder, KeyBuilder, LevelBuilder},
    keysym::generated::syms,
    routine::{Routine, RoutineBuilder},
    state_machine::{
        Direction::{Down, Up},
        Event,
    },
    GroupType, Keycode, ModifierMask,
};

#[test]
fn radio_group() {
    let mut builder = Builder::default();
    let down_global = builder.add_global();
    let gt = GroupType::builder(ModifierMask::NONE).build();
    for i in 1..4 {
        let routine = {
            let mut b = RoutineBuilder::default();
            let [key, down, eq] = b.allocate_vars();
            let same_key = b
                .load_lit(key, i)
                .load_global(down, down_global)
                .eq(eq, key, down)
                .prepare_skip_if(eq);
            let no_down = b.prepare_skip_if_not(down);
            b.key_up(down)
                .finish_skip(no_down)
                .key_down(key)
                .store_global(down_global, key)
                .finish_skip(same_key);
            b.build()
        };
        let mut level = LevelBuilder::new(0);
        level.routine(&routine);
        let mut gb = GroupBuilder::new(0, &gt);
        gb.add_level(level);
        let mut kb = KeyBuilder::new(Keycode::from_x11(i));
        kb.add_group(gb);
        builder.add_key(kb);
    }
    let sm = builder.build_state_machine();
    let mut state = sm.create_state();
    let a = Keycode::from_x11(1);
    let b = Keycode::from_x11(2);
    let c = Keycode::from_x11(3);
    let mut key = |kc, dir| {
        let mut events = vec![];
        sm.handle_key(&mut state, &mut events, kc, dir);
        println!("{:#?}", events);
        events
    };
    assert_eq!(key(a, Down), [Event::KeyDown(a)]);
    assert_eq!(key(a, Up), []);
    assert_eq!(key(b, Down), [Event::KeyUp(a), Event::KeyDown(b)]);
    assert_eq!(key(b, Up), []);
    assert_eq!(key(c, Down), [Event::KeyUp(b), Event::KeyDown(c)]);
    assert_eq!(key(c, Up), []);
    assert_eq!(key(c, Down), []);
    assert_eq!(key(c, Up), []);
    assert_eq!(key(b, Down), [Event::KeyUp(c), Event::KeyDown(b)]);
    assert_eq!(key(b, Up), []);
}

#[test]
fn repeat_set_mods_key() {
    let a = Keycode::from_x11(1);
    let b = Keycode::from_x11(2);

    let mut builder = Builder::default();
    let group_type = GroupType::builder(ModifierMask::NONE).build();
    {
        let global = builder.add_global();
        let routine = {
            let mut builder = Routine::builder();
            let [is_later, var] = builder.allocate_vars();
            let anchor = builder
                .load_global(is_later, global)
                .prepare_skip_if(is_later);
            let anchor = builder
                .load_lit(var, 1)
                .store_global(global, var)
                .mods_pressed_store(var)
                .finish_skip(anchor)
                .load_lit(var, a.0)
                .key_down(var)
                .on_release()
                .key_up(var)
                .prepare_skip_if(is_later);
            builder
                .load_lit(var, 0)
                .mods_pressed_store(var)
                .finish_skip(anchor);
            builder.build()
        };
        let mut level_builder = LevelBuilder::new(0);
        level_builder.keysyms(&[syms::a]).routine(&routine);
        let mut group_builder = GroupBuilder::new(0, &group_type);
        group_builder.add_level(level_builder);
        let mut key_builder = KeyBuilder::new(a);
        key_builder.add_group(group_builder);
        builder.add_key(key_builder);
    }
    {
        let routine = {
            let mut builder = Routine::builder();
            let [kc, mods] = builder.allocate_vars();
            builder
                .last_pressed_mods_load(mods)
                .mods_pressed_inc(mods)
                .last_key_load(kc)
                .key_down(kc)
                .on_release()
                .key_up(kc)
                .mods_pressed_dec(mods);
            builder.build()
        };
        let mut key_builder = KeyBuilder::new(b);
        key_builder.routine(&routine);
        builder.add_key(key_builder);
    }
    let sm = builder.build_state_machine();
    let mut state = sm.create_state();
    let mut key = |kc, dir| {
        let mut events = vec![];
        sm.handle_key(&mut state, &mut events, kc, dir);
        println!("{:#?}", events);
        events
    };
    use Event::*;
    assert_eq!(
        key(a, Down),
        [
            ModsPressed(ModifierMask(1)),
            ModsEffective(ModifierMask(1)),
            KeyDown(a),
        ]
    );
    assert_eq!(
        key(a, Up),
        [
            KeyUp(a),
            ModsPressed(ModifierMask(0)),
            ModsEffective(ModifierMask(0)),
        ]
    );
    assert_eq!(key(b, Down), [KeyDown(a)]);
    assert_eq!(key(b, Up), [KeyUp(a)]);
}
