use crate::{
    builder::{Builder, GroupBuilder, KeyBuilder, LevelBuilder},
    routine::RoutineBuilder,
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
