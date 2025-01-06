use {
    criterion::{criterion_group, criterion_main, Criterion},
    kbvm::{
        builder::Builder,
        group_type::GroupType,
        modifier::{ModifierIndex, ModifierMask},
        routine::{
            Register,
            Register::{R0, R1, R2, R3, R4},
            Routine,
        },
        state_machine::{Keycode, State},
    },
    libxkbcommon_test_linker::XState,
};

const KEY_A: Keycode = Keycode(0x10);
const KEY_SHIFT: Keycode = Keycode(0x11);
const KEY_LOCK: Keycode = Keycode(0x12);

const SHIFT: ModifierIndex = ModifierIndex(0x0);
const SHIFT_MASK: ModifierMask = SHIFT.to_mask();

const LOCK: ModifierIndex = ModifierIndex(0x1);
const LOCK_MASK: ModifierMask = LOCK.to_mask();

fn shift_press_release(c: &mut Criterion) {
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
    let mut xkbc = XState::new(MAP);
    let mut events = vec![];
    c.bench_function("kbvm - press/release - a", |b| {
        b.iter(|| {
            state_machine.handle_key(&mut state, &mut events, KEY_A, true);
            state_machine.handle_key(&mut state, &mut events, KEY_A, false);
            events.clear();
        })
    });
    c.bench_function("xkbc - press/release - a", |b| {
        b.iter(|| {
            xkbc.handle_key(KEY_A.0, true);
            xkbc.handle_key(KEY_A.0, false);
        })
    });
    c.bench_function("kbvm - press/release - shift", |b| {
        b.iter(|| {
            state_machine.handle_key(&mut state, &mut events, KEY_SHIFT, true);
            state_machine.handle_key(&mut state, &mut events, KEY_SHIFT, false);
            events.clear();
        })
    });
    c.bench_function("xkbc - press/release - shift", |b| {
        b.iter(|| {
            xkbc.handle_key(KEY_SHIFT.0, true);
            xkbc.handle_key(KEY_SHIFT.0, false);
        })
    });
    c.bench_function("kbvm - press/release - lock", |b| {
        b.iter(|| {
            state_machine.handle_key(&mut state, &mut events, KEY_LOCK, true);
            state_machine.handle_key(&mut state, &mut events, KEY_LOCK, false);
            events.clear();
        })
    });
    c.bench_function("xkbc - press/release - lock", |b| {
        b.iter(|| {
            xkbc.handle_key(KEY_LOCK.0, true);
            xkbc.handle_key(KEY_LOCK.0, false);
        })
    });
    c.bench_function("kbvm - press/release - shift/a", |b| {
        b.iter(|| {
            state_machine.handle_key(&mut state, &mut events, KEY_SHIFT, true);
            state_machine.handle_key(&mut state, &mut events, KEY_A, true);
            state_machine.handle_key(&mut state, &mut events, KEY_A, false);
            state_machine.handle_key(&mut state, &mut events, KEY_SHIFT, false);
            events.clear();
        })
    });
    c.bench_function("xkbc - press/release - shift/a", |b| {
        b.iter(|| {
            xkbc.handle_key(KEY_SHIFT.0, true);
            xkbc.handle_key(KEY_A.0, true);
            xkbc.handle_key(KEY_A.0, false);
            xkbc.handle_key(KEY_SHIFT.0, false);
        })
    });
    c.bench_function("kbvm - press/release - lock/a", |b| {
        b.iter(|| {
            state_machine.handle_key(&mut state, &mut events, KEY_LOCK, true);
            state_machine.handle_key(&mut state, &mut events, KEY_LOCK, false);
            state_machine.handle_key(&mut state, &mut events, KEY_A, true);
            state_machine.handle_key(&mut state, &mut events, KEY_A, false);
            state_machine.handle_key(&mut state, &mut events, KEY_LOCK, true);
            state_machine.handle_key(&mut state, &mut events, KEY_LOCK, false);
            events.clear();
        })
    });
    c.bench_function("xkbc - press/release - lock/a", |b| {
        b.iter(|| {
            xkbc.handle_key(KEY_LOCK.0, true);
            xkbc.handle_key(KEY_LOCK.0, false);
            xkbc.handle_key(KEY_A.0, true);
            xkbc.handle_key(KEY_A.0, false);
            xkbc.handle_key(KEY_LOCK.0, true);
            xkbc.handle_key(KEY_LOCK.0, false);
        })
    });
}

criterion_group!(benches, shift_press_release,);
criterion_main!(benches);

const MAP: &str = r#"
xkb_keymap {
    xkb_keycodes {
          <A>     =  16;
          <SHIFT> =  17;
          <LOCK>  =  18;
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
        key <SHIFT> { actions[Group1] = [ SetMods(modifiers=Shift)  ] };
        key <LOCK>  { actions[Group1] = [ LockMods(modifiers=Shift) ] };
    };
};
"#;
