use {
    criterion::{criterion_group, criterion_main, Criterion},
    kbvm::{
        evdev::{A, CAPSLOCK, LEFTSHIFT},
        state_machine::Direction,
        xkb::{diagnostic::WriteToLog, Context},
    },
    libxkbcommon_test_linker::XState,
};
use kbvm::evdev::{B, C};

fn shift_press_release(c: &mut Criterion) {
    let builder = Context::builder()
        .build()
        .keymap_from_bytes(WriteToLog, None, MAP.as_bytes())
        .unwrap()
        .to_builder();
    let state_machine = builder.build_state_machine();
    let mut state = state_machine.create_state();
    let mut xkbc = XState::new(MAP);
    let mut events = vec![];
    c.bench_function("kbvm - press/release - a", |b| {
        for k in [A, B, C] {
            state_machine.handle_key(&mut state, &mut events, k, Direction::Down);
        }
        for k in [A, B, C] {
            state_machine.handle_key(&mut state, &mut events, k, Direction::Up);
        }
        events.clear();
        b.iter(|| {
            for _ in 0..100 {
                state_machine.handle_key(&mut state, &mut events, A, Direction::Down);
                state_machine.handle_key(&mut state, &mut events, A, Direction::Up);
                events.clear();
            }
        })
    });
    c.bench_function("xkbc - press/release - a", |b| {
        b.iter(|| {
            for _ in 0..100 {
                xkbc.handle_key(A.to_raw(), true);
                xkbc.handle_key(A.to_raw(), false);
            }
        })
    });
    c.bench_function("kbvm - press/release - shift", |b| {
        for k in [A, B, C] {
            state_machine.handle_key(&mut state, &mut events, k, Direction::Down);
        }
        for k in [A, B, C] {
            state_machine.handle_key(&mut state, &mut events, k, Direction::Up);
        }
        events.clear();
        b.iter(|| {
            for _ in 0..100 {
                state_machine.handle_key(&mut state, &mut events, LEFTSHIFT, Direction::Down);
                state_machine.handle_key(&mut state, &mut events, LEFTSHIFT, Direction::Up);
                events.clear();
            }
        })
    });
    c.bench_function("xkbc - press/release - shift", |b| {
        b.iter(|| {
            for _ in 0..100 {
                xkbc.handle_key(LEFTSHIFT.to_raw(), true);
                xkbc.handle_key(LEFTSHIFT.to_raw(), false);
            }
        })
    });
    c.bench_function("kbvm - press/release - lock", |b| {
        for k in [A, B, C] {
            state_machine.handle_key(&mut state, &mut events, k, Direction::Down);
        }
        for k in [A, B, C] {
            state_machine.handle_key(&mut state, &mut events, k, Direction::Up);
        }
        events.clear();
        b.iter(|| {
            state_machine.handle_key(&mut state, &mut events, CAPSLOCK, Direction::Down);
            state_machine.handle_key(&mut state, &mut events, CAPSLOCK, Direction::Up);
            events.clear();
        })
    });
    c.bench_function("xkbc - press/release - lock", |b| {
        b.iter(|| {
            xkbc.handle_key(CAPSLOCK.to_raw(), true);
            xkbc.handle_key(CAPSLOCK.to_raw(), false);
        })
    });
    c.bench_function("kbvm - press/release - shift/a", |b| {
        for k in [A, B, C] {
            state_machine.handle_key(&mut state, &mut events, k, Direction::Down);
        }
        for k in [A, B, C] {
            state_machine.handle_key(&mut state, &mut events, k, Direction::Up);
        }
        events.clear();
        b.iter(|| {
            state_machine.handle_key(&mut state, &mut events, LEFTSHIFT, Direction::Down);
            state_machine.handle_key(&mut state, &mut events, A, Direction::Down);
            state_machine.handle_key(&mut state, &mut events, A, Direction::Up);
            state_machine.handle_key(&mut state, &mut events, LEFTSHIFT, Direction::Up);
            events.clear();
        })
    });
    c.bench_function("xkbc - press/release - shift/a", |b| {
        b.iter(|| {
            xkbc.handle_key(LEFTSHIFT.to_raw(), true);
            xkbc.handle_key(A.to_raw(), true);
            xkbc.handle_key(A.to_raw(), false);
            xkbc.handle_key(LEFTSHIFT.to_raw(), false);
        })
    });
    c.bench_function("kbvm - press/release - lock/a", |b| {
        for k in [A, B, C] {
            state_machine.handle_key(&mut state, &mut events, k, Direction::Down);
        }
        for k in [A, B, C] {
            state_machine.handle_key(&mut state, &mut events, k, Direction::Up);
        }
        events.clear();
        b.iter(|| {
            state_machine.handle_key(&mut state, &mut events, CAPSLOCK, Direction::Down);
            state_machine.handle_key(&mut state, &mut events, CAPSLOCK, Direction::Up);
            state_machine.handle_key(&mut state, &mut events, A, Direction::Down);
            state_machine.handle_key(&mut state, &mut events, A, Direction::Up);
            state_machine.handle_key(&mut state, &mut events, CAPSLOCK, Direction::Down);
            state_machine.handle_key(&mut state, &mut events, CAPSLOCK, Direction::Up);
            events.clear();
        })
    });
    c.bench_function("xkbc - press/release - lock/a", |b| {
        b.iter(|| {
            xkbc.handle_key(CAPSLOCK.to_raw(), true);
            xkbc.handle_key(CAPSLOCK.to_raw(), false);
            xkbc.handle_key(A.to_raw(), true);
            xkbc.handle_key(A.to_raw(), false);
            xkbc.handle_key(CAPSLOCK.to_raw(), true);
            xkbc.handle_key(CAPSLOCK.to_raw(), false);
        })
    });
}

criterion_group!(benches, shift_press_release,);
criterion_main!(benches);

const MAP: &str = r#"
xkb_keymap {
    xkb_keycodes {
          <a>         = 38;
          <leftshift> = 50;
          <capslock>  = 66;
    };
    xkb_types {
        type "one_layer" {
            modifiers = none;
        };
        type "alphanumeric" {
            modifiers  = Shift+Lock;
            map[Shift] = Level2;
            map[Lock]  = Level2;
        };
    };
    xkb_compatibility { };
    xkb_symbols {
        key <leftshift> { actions[Group1] = [ SetMods(modifiers=Shift)  ] };
        key <capslock>  { actions[Group1] = [ LockMods(modifiers=Shift) ] };
    };
};
"#;
