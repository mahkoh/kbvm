use {
    crate::{evdev::keycode_to_name, output::Output},
    debug_fn::debug_fn,
    kbvm::{xkb::Keymap, ControlsMask, GroupDelta, GroupIndex, Keycode, Keysym, ModifierMask},
    owo_colors::{OwoColorize, Stream::Stdout},
    std::{
        fmt::{Display, Formatter},
        time::{Duration, Instant, SystemTime},
    },
};

pub struct Ansi {
    theme: Theme,
    last_event: Option<Instant>,
}

impl Ansi {
    pub fn new(theme: Theme) -> Self {
        Self {
            theme,
            last_event: None,
        }
    }

    fn now(&self) -> Now2 {
        Now2(self.theme)
    }

    fn key_down_up(&self, down: bool) -> KeyDirection {
        KeyDirection {
            down,
            theme: self.theme,
        }
    }

    fn handle_delta(&mut self) {
        let now = Instant::now();
        if let Some(last_event) = self.last_event {
            if now - last_event > Duration::from_millis(200) {
                println!("{} {}", self.now(), "------------".color_time(self.theme));
            }
        }
        self.last_event = Some(now);
    }

    fn mods(&mut self, name: &str, mods: ModifierMask) {
        self.handle_delta();
        println!(
            "{} {:12} = 0x{:08x}",
            self.now(),
            name.color_mods(self.theme),
            mods.0,
        );
    }

    fn group(&mut self, name: &str, group: u32) {
        self.handle_delta();
        println!(
            "{} {:13} = {}",
            self.now(),
            name.color_group(self.theme),
            group as i32,
        );
    }
}

impl Output for Ansi {
    fn keymap(&mut self, keymap: &Keymap) {
        self.handle_delta();
        println!(
            "{} {} {:#}",
            self.now(),
            "keymap".color_time(self.theme),
            keymap.format(),
        );
    }

    fn key_down(&mut self, keycode: Keycode) {
        self.handle_delta();
        println!(
            "{} {} {}",
            self.now(),
            self.key_down_up(true),
            KeyName(keycode)
        );
    }

    fn key_up(&mut self, keycode: Keycode) {
        self.handle_delta();
        println!(
            "{} {} {}",
            self.now(),
            self.key_down_up(false),
            KeyName(keycode)
        );
    }

    fn keysym(&mut self, sym: Keysym, char: Option<char>) {
        self.handle_delta();
        print!("{}   {} {sym}", self.now(), "sym".color_sym(self.theme));
        if let Some(c) = char {
            print!(" {c:?}");
        }
        println!();
    }

    fn mods_pressed(&mut self, mods: ModifierMask) {
        self.mods("mods_pressed", mods);
    }

    fn mods_latched(&mut self, mods: ModifierMask) {
        self.mods("mods_latched", mods);
    }

    fn mods_locked(&mut self, mods: ModifierMask) {
        self.mods("mods_locked", mods);
    }

    fn mods(&mut self, mods: ModifierMask) {
        self.mods("mods", mods);
    }

    fn group_pressed(&mut self, group: GroupDelta) {
        self.group("group_pressed", group.0);
    }

    fn group_latched(&mut self, group: GroupDelta) {
        self.group("group_latched", group.0);
    }

    fn group_locked(&mut self, group: GroupIndex) {
        self.group("group_locked", group.0);
    }

    fn group(&mut self, group: GroupIndex) {
        self.group("group", group.0);
    }

    fn controls(&mut self, group: ControlsMask) {
        self.handle_delta();
        println!(
            "{} {} = {group:?}",
            self.now(),
            "controls".color_controls(self.theme),
        );
    }

    fn compose_pending(&mut self, keysym: Keysym) {
        self.handle_delta();
        println!(
            "{}   {} ({keysym})",
            self.now(),
            "compose pending".color_compose(self.theme),
        );
    }

    fn compose_aborted(&mut self, keysym: Keysym) {
        self.handle_delta();
        println!(
            "{}   {} ({keysym})",
            self.now(),
            "compose aborted".color_compose(self.theme),
        );
    }

    fn composed(&mut self, keysym: Option<Keysym>, string: Option<&str>, original_keysym: Keysym) {
        self.handle_delta();
        print!("{}   {}", self.now(), "composed".color_compose(self.theme));
        if let Some(sym) = keysym {
            print!(" {}", sym);
        }
        if let Some(string) = string {
            print!(" {:?}", string);
        }
        println!(" ({})", original_keysym);
    }

    fn state_reset(&mut self) {
        self.handle_delta();
        println!("{} {}", self.now(), "state reset".color_time(self.theme));
    }
}

#[derive(Copy, Clone)]
pub enum Theme {
    Dark,
}

fn apply<T>(t: &T, rgb: [u8; 3]) -> impl Display + use<'_, T>
where
    T: Display,
{
    let r = rgb[0];
    let g = rgb[1];
    let b = rgb[2];
    t.if_supports_color(Stdout, move |c| c.truecolor(r, g, b))
}

trait Colorize {
    fn color_time(&self, theme: Theme) -> impl Display;
    fn color_key_up_down(&self, theme: Theme) -> impl Display;
    fn color_sym(&self, theme: Theme) -> impl Display;
    fn color_mods(&self, theme: Theme) -> impl Display;
    fn color_group(&self, theme: Theme) -> impl Display;
    fn color_controls(&self, theme: Theme) -> impl Display;
    fn color_compose(&self, theme: Theme) -> impl Display;
}

impl<T> Colorize for T
where
    T: Display,
{
    fn color_time(&self, theme: Theme) -> impl Display {
        let c = match theme {
            Theme::Dark => [100, 100, 100],
        };
        apply(self, c)
    }

    fn color_key_up_down(&self, theme: Theme) -> impl Display {
        let c = match theme {
            Theme::Dark => [100, 100, 255],
        };
        apply(self, c)
    }

    fn color_sym(&self, theme: Theme) -> impl Display {
        let c = match theme {
            Theme::Dark => [100, 255, 255],
        };
        apply(self, c)
    }

    fn color_mods(&self, theme: Theme) -> impl Display {
        let c = match theme {
            Theme::Dark => [255, 255, 100],
        };
        apply(self, c)
    }

    fn color_group(&self, theme: Theme) -> impl Display {
        let c = match theme {
            Theme::Dark => [255, 100, 100],
        };
        apply(self, c)
    }

    fn color_controls(&self, theme: Theme) -> impl Display {
        let c = match theme {
            Theme::Dark => [255, 100, 255],
        };
        apply(self, c)
    }

    fn color_compose(&self, theme: Theme) -> impl Display {
        let c = match theme {
            Theme::Dark => [100, 255, 100],
        };
        apply(self, c)
    }
}

pub struct Now;

impl Display for Now {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let millis = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_millis()
            % 1_000_000_000;
        write!(f, "[{:07}.{:03}]", millis / 1000, millis % 1000)
    }
}

pub struct Now2(Theme);

impl Display for Now2 {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let millis = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_millis()
            % 1_000_000_000;
        debug_fn(move |f| write!(f, "[{:07}.{:03}]", millis / 1000, millis % 1000))
            .color_time(self.0)
            .fmt(f)
    }
}

pub struct KeyName(pub Keycode);

impl Display for KeyName {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match keycode_to_name(self.0) {
            None => write!(f, "`{}`", self.0.to_evdev()),
            Some(n) => f.write_str(n),
        }
    }
}

struct KeyDirection {
    down: bool,
    theme: Theme,
}

impl Display for KeyDirection {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let s = match self.down {
            true => "key down",
            false => "key up  ",
        };
        let res = s.color_key_up_down(self.theme).fmt(f);
        res
    }
}
