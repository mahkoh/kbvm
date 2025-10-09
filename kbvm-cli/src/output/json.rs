use {
    crate::{evdev::keycode_to_name, output::Output},
    kbvm::{ControlsMask, GroupDelta, GroupIndex, Keycode, Keysym, ModifierMask, xkb::Keymap},
    serde::Serialize,
    std::{
        io::{Write, stdout},
        time::SystemTime,
    },
};

pub struct Json;

#[derive(Serialize)]
struct Msg<'a> {
    unix_timestamp_ms: u128,
    #[serde(flatten)]
    ty: Type<'a>,
}

#[derive(Serialize)]
#[serde(rename_all = "snake_case")]
#[serde(tag = "type")]
enum Type<'a> {
    Keymap {
        map: String,
    },
    KeyDown {
        keycode: u32,
        name: Option<&'static str>,
    },
    KeyUp {
        keycode: u32,
        name: Option<&'static str>,
    },
    Keysym {
        keysym: u32,
        char: Option<char>,
    },
    ModsPressed {
        mods: u32,
    },
    ModsLatched {
        mods: u32,
    },
    ModsLocked {
        mods: u32,
    },
    Mods {
        mods: u32,
    },
    GroupPressed {
        group: i32,
    },
    GroupLatched {
        group: i32,
    },
    GroupLocked {
        group: i32,
    },
    Group {
        group: i32,
    },
    Controls {
        controls: u32,
    },
    ComposePending {
        keysym: u32,
    },
    ComposeAborted {
        keysym: u32,
    },
    Composed {
        keysym: Option<u32>,
        string: Option<&'a str>,
        original_keysym: u32,
    },
    StateReset,
}

impl Json {
    fn msg(&self, ty: Type<'_>) {
        let msg = Msg {
            unix_timestamp_ms: SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap()
                .as_millis(),
            ty,
        };
        let mut stdout = stdout().lock();
        serde_json::to_writer(&mut stdout, &msg).unwrap();
        stdout.write_all(b"\n").unwrap();
        stdout.flush().unwrap();
    }
}

impl Output for Json {
    fn keymap(&mut self, keymap: &Keymap) {
        self.msg(Type::Keymap {
            map: format!("{}", keymap.format().multiple_actions_per_level(true)),
        });
    }

    fn key_down(&mut self, keycode: Keycode) {
        self.msg(Type::KeyDown {
            keycode: keycode.to_evdev(),
            name: keycode_to_name(keycode),
        });
    }

    fn key_up(&mut self, keycode: Keycode) {
        self.msg(Type::KeyUp {
            keycode: keycode.to_evdev(),
            name: keycode_to_name(keycode),
        });
    }

    fn keysym(&mut self, keysym: Keysym, char: Option<char>) {
        self.msg(Type::Keysym {
            keysym: keysym.0,
            char,
        });
    }

    fn mods_pressed(&mut self, mods: ModifierMask) {
        self.msg(Type::ModsPressed { mods: mods.0 });
    }

    fn mods_latched(&mut self, mods: ModifierMask) {
        self.msg(Type::ModsLatched { mods: mods.0 });
    }

    fn mods_locked(&mut self, mods: ModifierMask) {
        self.msg(Type::ModsLocked { mods: mods.0 });
    }

    fn mods(&mut self, mods: ModifierMask) {
        self.msg(Type::Mods { mods: mods.0 });
    }

    fn group_pressed(&mut self, group: GroupDelta) {
        self.msg(Type::GroupPressed {
            group: group.0 as i32,
        });
    }

    fn group_latched(&mut self, group: GroupDelta) {
        self.msg(Type::GroupLatched {
            group: group.0 as i32,
        });
    }

    fn group_locked(&mut self, group: GroupIndex) {
        self.msg(Type::GroupLocked {
            group: group.0 as i32,
        });
    }

    fn group(&mut self, group: GroupIndex) {
        self.msg(Type::Group {
            group: group.0 as i32,
        });
    }

    fn controls(&mut self, group: ControlsMask) {
        self.msg(Type::Controls { controls: group.0 });
    }

    fn compose_pending(&mut self, keysym: Keysym) {
        self.msg(Type::ComposePending { keysym: keysym.0 });
    }

    fn compose_aborted(&mut self, keysym: Keysym) {
        self.msg(Type::ComposeAborted { keysym: keysym.0 });
    }

    fn composed(&mut self, keysym: Option<Keysym>, string: Option<&str>, original_keysym: Keysym) {
        self.msg(Type::Composed {
            keysym: keysym.map(|k| k.0),
            string,
            original_keysym: original_keysym.0,
        });
    }

    fn state_reset(&mut self) {
        self.msg(Type::StateReset);
    }
}
