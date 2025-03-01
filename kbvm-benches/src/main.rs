use kbvm::{
    evdev::A,
    state_machine::Direction,
    xkb::{diagnostic::WriteToLog, Context},
};

fn main() {
    let builder = Context::builder()
        .build()
        .keymap_from_bytes(WriteToLog, None, MAP.as_bytes())
        .unwrap()
        .to_builder();
    let state_machine = builder.build_state_machine();
    let mut state = state_machine.create_state();
    let mut events = vec![];
    for _ in 0..100000000 {
        state_machine.handle_key(&mut state, &mut events, A, Direction::Down);
        state_machine.handle_key(&mut state, &mut events, A, Direction::Up);
        events.clear();
    }
}

const MAP: &str = r#"
xkb_keymap {
    xkb_keycodes {
          <a>         = 38;
          <leftshift> = 50;
          <capslock>  = 66;
    };
    xkb_symbols {
        key <a> { [ a, A ] };
        key <leftshift> { [ SetMods(modifiers=Shift)  ] };
        key <capslock> { [ LockMods(modifiers=Lock) ] };
    };
};
"#;
