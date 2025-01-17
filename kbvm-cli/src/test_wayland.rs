use {
    crate::{
        output::{
            ansi::{Ansi, Theme},
            json::Json,
            Output,
        },
        utils::read_path,
    },
    bitflags::Flags,
    clap::{Args, ValueEnum},
    error_reporter::Report,
    hashbrown::{hash_map::Entry, HashMap},
    kbvm::{
        lookup::LookupTable,
        state_machine::{self, Direction, Event, StateMachine},
        xkb::{
            compose::{self, ComposeTable, FeedResult},
            diagnostic::WriteToLog,
            Context,
        },
        Components, Keycode,
    },
    memmap2::MmapOptions,
    wayland_client::{
        delegate_noop,
        protocol::{
            wl_buffer::WlBuffer,
            wl_callback::{self, WlCallback},
            wl_compositor::WlCompositor,
            wl_keyboard::{self, KeyState, WlKeyboard},
            wl_registry::{self, WlRegistry},
            wl_seat::{self, WlSeat},
            wl_surface::WlSurface,
        },
        Connection, Dispatch, QueueHandle, WEnum,
    },
    wayland_protocols::{
        wp::{
            single_pixel_buffer::v1::client::wp_single_pixel_buffer_manager_v1::WpSinglePixelBufferManagerV1,
            viewporter::client::{wp_viewport::WpViewport, wp_viewporter::WpViewporter},
        },
        xdg::{
            decoration::zv1::client::{
                zxdg_decoration_manager_v1::ZxdgDecorationManagerV1,
                zxdg_toplevel_decoration_v1::{self, ZxdgToplevelDecorationV1},
            },
            shell::client::{
                xdg_surface::{self, XdgSurface},
                xdg_toplevel::{self, XdgToplevel},
                xdg_wm_base::{self, XdgWmBase},
            },
        },
    },
};

#[derive(Args, Debug, Default)]
pub struct TestWaylandArgs {
    /// Print the compiled keymap.
    #[clap(long)]
    print_keymap: bool,
    #[clap(flatten)]
    compose: ComposeGroup,
    /// Print messages as newline-separated JSON.
    #[clap(long)]
    json: bool,
    /// Instead of using the wl_keyboard.modifiers event, treat the wl_keyboard.key events
    /// as raw input and run your own state machine.
    ///
    /// The state machine is reset whenever the window gains focus.
    #[clap(long)]
    state_machine: bool,
    /// Use the keymap stored in this file instead of the keymap sent by the compositor.
    ///
    /// Implies `--state-machine`.
    ///
    /// If the file name is `-`, the keymap is read from stdin. This can be used with the
    /// following pattern:
    ///
    /// kbvm expand-rmlvo --layout ru,de | kbvm test-wayland --keymap -
    #[clap(long)]
    keymap: Option<String>,
    /// Enable or disable colored output.
    ///
    /// By default, colored output is used if stdout refers to a terminal.
    #[clap(value_enum, long, require_equals = true, num_args = 0..=1, default_missing_value = "always")]
    color: Option<Color>,
}

#[derive(ValueEnum, Debug, Clone)]
enum Color {
    Never,
    Always,
    Auto,
}

#[derive(Args, Debug, Default)]
#[group(multiple = false)]
struct ComposeGroup {
    /// Disable handling of compose sequences.
    #[clap(long)]
    no_compose: bool,
    /// Use this compose file instead of the default one.
    #[clap(long)]
    compose_file: Option<String>,
}

enum ComposeSetting {
    Disabled,
    Default,
    Path(String),
}

pub fn main(args: TestWaylandArgs) {
    if let Some(color) = args.color {
        match color {
            Color::Never => owo_colors::set_override(false),
            Color::Always => owo_colors::set_override(true),
            Color::Auto => {}
        }
    }
    let con = match Connection::connect_to_env() {
        Ok(c) => c,
        Err(e) => {
            log::error!("could not connect to compositor: {}", Report::new(e));
            std::process::exit(1);
        }
    };
    let mut queue = con.new_event_queue::<State>();
    let qh = queue.handle();
    let _registry = con.display().get_registry(&qh, ());
    con.display().sync(&qh, InitialRoundtrip);
    let context = Context::default();
    let mut output: Box<dyn Output> = match args.json {
        true => Box::new(Json),
        false => Box::new(Ansi::new(Theme::Dark)),
    };
    let (lookup_table, state_machine) = create_global_lt_and_sm(
        &context,
        &mut *output,
        args.print_keymap,
        args.keymap.as_deref(),
    );
    let mut state = State {
        qh,
        registry: Default::default(),
        keyboards: Default::default(),
        context,
        window: None,
        output,
        compose: match args.compose.compose_file {
            None => match args.compose.no_compose {
                true => ComposeSetting::Disabled,
                _ => ComposeSetting::Default,
            },
            Some(p) => ComposeSetting::Path(p),
        },
        print_keymap: args.print_keymap,
        create_state_machine: args.state_machine || args.keymap.is_some(),
        lookup_table,
        state_machine,
    };
    loop {
        queue.blocking_dispatch(&mut state).unwrap();
    }
}

fn create_global_lt_and_sm(
    context: &Context,
    output: &mut dyn Output,
    print_keymap: bool,
    path: Option<&str>,
) -> (Option<LookupTable>, Option<StateMachine>) {
    let mut state_machine = None;
    let mut lookup_table = None;
    if let Some(path) = path {
        let (path, source) = read_path(path);
        let keymap = context.keymap_from_bytes(WriteToLog, Some(path.as_ref()), &source);
        let keymap = match keymap {
            Ok(m) => m,
            Err(_) => {
                log::error!("could not compile keymap");
                std::process::exit(1);
            }
        };
        if print_keymap {
            output.keymap(&keymap);
        }
        let builder = keymap.to_builder();
        state_machine = Some(builder.build_state_machine());
        lookup_table = Some(builder.build_lookup_table());
    }
    (lookup_table, state_machine)
}

struct State {
    qh: QueueHandle<State>,
    registry: Registry,
    keyboards: HashMap<u32, Keyboard>,
    context: Context,
    window: Option<Window>,
    output: Box<dyn Output>,
    compose: ComposeSetting,
    print_keymap: bool,
    create_state_machine: bool,
    lookup_table: Option<LookupTable>,
    state_machine: Option<StateMachine>,
}

struct Window {
    surface: WlSurface,
    buffer: WlBuffer,
    viewport: WpViewport,
    width: i32,
    height: i32,
}

#[derive(Default)]
struct Registry {
    wl_compositor: Option<WlCompositor>,
    wl_seats: HashMap<u32, RawSeat>,
    wp_viewporter: Option<WpViewporter>,
    xdg_wm_base: Option<XdgWmBase>,
    wp_single_pixel_buffer_manager_v1: Option<WpSinglePixelBufferManagerV1>,
    zxdg_decoration_manager_v1: Option<ZxdgDecorationManagerV1>,
}

struct Keyboard {
    wl_keyboard: WlKeyboard,
    lookup: Option<LookupTable>,
    state_machine: Option<Sm>,
    components: Components,
    compose: Option<Compose>,
}

struct Sm {
    state_machine: StateMachine,
    state: state_machine::State,
    events: Vec<Event>,
}

struct Compose {
    table: ComposeTable,
    state: compose::State,
}

struct RawSeat {
    seat: WlSeat,
    capabilities: wl_seat::Capability,
    name: String,
}

struct InitialRoundtrip;

delegate_noop!(State: ignore WlCompositor);
delegate_noop!(State: ignore WpSinglePixelBufferManagerV1);
delegate_noop!(State: ignore ZxdgDecorationManagerV1);
delegate_noop!(State: ignore ZxdgToplevelDecorationV1);
delegate_noop!(State: ignore WpViewporter);
delegate_noop!(State: ignore WlSurface);
delegate_noop!(State: ignore WlBuffer);
delegate_noop!(State: ignore WpViewport);

impl State {
    fn handle_seat_capabilities(&mut self, name: u32) {
        let Some(seat) = self.registry.wl_seats.get(&name) else {
            return;
        };
        let has_kb = seat.capabilities.contains(wl_seat::Capability::Keyboard);
        match self.keyboards.entry(name) {
            Entry::Occupied(_) if has_kb => {}
            Entry::Occupied(e) => {
                let kb = e.remove();
                kb.wl_keyboard.release();
            }
            Entry::Vacant(e) if has_kb => {
                let create_compose = |path: Option<&str>| {
                    let mut builder = self.context.compose_table_builder();
                    if let Some(path) = path {
                        builder.file(path);
                    }
                    builder.build(WriteToLog).map(|table| Compose {
                        state: table.state(),
                        table,
                    })
                };
                let compose = match &self.compose {
                    ComposeSetting::Disabled => None,
                    ComposeSetting::Default => create_compose(None),
                    ComposeSetting::Path(p) => create_compose(Some(p)),
                };
                let kb = seat.seat.get_keyboard(&self.qh, name);
                e.insert(Keyboard {
                    wl_keyboard: kb,
                    lookup: self.lookup_table.clone(),
                    state_machine: self.state_machine.clone().map(|sm| Sm {
                        state: sm.create_state(),
                        state_machine: sm,
                        events: vec![],
                    }),
                    components: Default::default(),
                    compose,
                });
            }
            Entry::Vacant(_) => {}
        }
    }
}

impl Dispatch<WlRegistry, ()> for State {
    fn event(
        state: &mut Self,
        proxy: &WlRegistry,
        event: wl_registry::Event,
        _data: &(),
        _conn: &Connection,
        qh: &QueueHandle<Self>,
    ) {
        use wl_registry::Event;
        match event {
            Event::Global {
                name,
                interface,
                version,
            } => match interface.as_str() {
                "wl_compositor" => {
                    state.registry.wl_compositor = Some(proxy.bind(name, version, qh, ()))
                }
                "xdg_wm_base" => {
                    state.registry.xdg_wm_base = Some(proxy.bind(name, version, qh, ()))
                }
                "wp_viewporter" => {
                    state.registry.wp_viewporter = Some(proxy.bind(name, version, qh, ()))
                }
                "wp_single_pixel_buffer_manager_v1" => {
                    state.registry.wp_single_pixel_buffer_manager_v1 =
                        Some(proxy.bind(name, version, qh, ()))
                }
                "zxdg_decoration_manager_v1" => {
                    state.registry.zxdg_decoration_manager_v1 =
                        Some(proxy.bind(name, version, qh, ()))
                }
                "wl_seat" => {
                    let seat = RawSeat {
                        seat: proxy.bind(name, version, qh, name),
                        capabilities: wl_seat::Capability::empty(),
                        name: name.to_string(),
                    };
                    state.registry.wl_seats.insert(name, seat);
                }
                _ => {}
            },
            Event::GlobalRemove { name } => {
                let Some(seat) = state.registry.wl_seats.remove(&name) else {
                    return;
                };
                seat.seat.release();
            }
            _ => {}
        }
    }
}

impl Dispatch<XdgWmBase, ()> for State {
    fn event(
        _state: &mut Self,
        proxy: &XdgWmBase,
        event: xdg_wm_base::Event,
        _data: &(),
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
    ) {
        use xdg_wm_base::Event;
        match event {
            Event::Ping { serial } => {
                proxy.pong(serial);
            }
            _ => {}
        }
    }
}

impl Dispatch<WlSeat, u32> for State {
    fn event(
        state: &mut Self,
        _proxy: &WlSeat,
        event: wl_seat::Event,
        name: &u32,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
    ) {
        let Some(seat) = state.registry.wl_seats.get_mut(name) else {
            return;
        };
        use wl_seat::Event;
        match event {
            Event::Capabilities { capabilities } => {
                seat.capabilities = capabilities.unwrap();
                state.handle_seat_capabilities(*name);
            }
            Event::Name { name } => {
                seat.name = name;
            }
            _ => {}
        }
    }
}

impl Dispatch<XdgSurface, ()> for State {
    fn event(
        state: &mut Self,
        proxy: &XdgSurface,
        event: xdg_surface::Event,
        _data: &(),
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
    ) {
        let window = state.window.as_ref().unwrap();
        use xdg_surface::Event;
        match event {
            Event::Configure { serial } => {
                proxy.ack_configure(serial);
                window.viewport.set_destination(window.width, window.height);
                window.surface.attach(Some(&window.buffer), 0, 0);
                window.surface.commit();
            }
            _ => {}
        }
    }
}

impl Dispatch<XdgToplevel, ()> for State {
    fn event(
        state: &mut Self,
        _proxy: &XdgToplevel,
        event: xdg_toplevel::Event,
        _data: &(),
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
    ) {
        let window = state.window.as_mut().unwrap();
        use xdg_toplevel::Event;
        match event {
            Event::Configure {
                mut width,
                mut height,
                ..
            } => {
                if width == 0 {
                    width = 800;
                }
                if height == 0 {
                    height = 600;
                }
                window.width = width;
                window.height = height;
            }
            Event::Close => {
                std::process::exit(0);
            }
            _ => {}
        }
    }
}

impl Dispatch<WlCallback, InitialRoundtrip> for State {
    fn event(
        state: &mut Self,
        _proxy: &WlCallback,
        event: wl_callback::Event,
        _name: &InitialRoundtrip,
        _conn: &Connection,
        qh: &QueueHandle<Self>,
    ) {
        use wl_callback::Event;
        match event {
            Event::Done { .. } => {
                let r = &state.registry;
                macro_rules! check_singleton {
                    ($name:ident) => {
                        if r.$name.is_none() {
                            log::error!("compositor does not expose {} global", stringify!($name));
                            std::process::exit(1);
                        }
                    };
                }
                check_singleton!(xdg_wm_base);
                check_singleton!(wl_compositor);
                check_singleton!(wp_viewporter);
                check_singleton!(wp_single_pixel_buffer_manager_v1);
                let surface = r.wl_compositor.as_ref().unwrap().create_surface(qh, ());
                let viewport = r
                    .wp_viewporter
                    .as_ref()
                    .unwrap()
                    .get_viewport(&surface, qh, ());
                let xdg_surface = r
                    .xdg_wm_base
                    .as_ref()
                    .unwrap()
                    .get_xdg_surface(&surface, qh, ());
                let xdg_toplevel = xdg_surface.get_toplevel(qh, ());
                if let Some(dm) = &r.zxdg_decoration_manager_v1 {
                    let decoration = dm.get_toplevel_decoration(&xdg_toplevel, qh, ());
                    decoration.set_mode(zxdg_toplevel_decoration_v1::Mode::ServerSide);
                }
                surface.commit();
                let buffer = state
                    .registry
                    .wp_single_pixel_buffer_manager_v1
                    .as_ref()
                    .unwrap()
                    .create_u32_rgba_buffer(0, 0, 0, !0, qh, ());
                state.window = Some(Window {
                    surface,
                    buffer,
                    viewport,
                    width: 800,
                    height: 600,
                });
            }
            _ => {}
        }
    }
}

impl Dispatch<WlKeyboard, u32> for State {
    fn event(
        state: &mut Self,
        _proxy: &WlKeyboard,
        event: wl_keyboard::Event,
        &name: &u32,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
    ) {
        let Some(keyboard) = state.keyboards.get_mut(&name) else {
            return;
        };
        use wl_keyboard::Event;
        match event {
            Event::Keymap { fd, size, .. } => {
                if state.state_machine.is_some() {
                    return;
                }
                let map = unsafe { MmapOptions::new().len(size as usize).map(&fd).unwrap() };
                let map = state.context.keymap_from_bytes(WriteToLog, None, &map);
                let map = match map {
                    Ok(map) => map,
                    Err(_) => {
                        log::error!("Could not parse keymap");
                        keyboard.lookup = None;
                        keyboard.state_machine = None;
                        return;
                    }
                };
                if state.print_keymap {
                    state.output.keymap(&map);
                }
                let builder = map.to_builder();
                if state.create_state_machine {
                    let sm = builder.build_state_machine();
                    keyboard.state_machine = Some(Sm {
                        state: sm.create_state(),
                        state_machine: sm,
                        events: vec![],
                    });
                }
                let lookup = builder.build_lookup_table();
                keyboard.lookup = Some(lookup);
            }
            Event::Key {
                key,
                state: WEnum::Value(key_state),
                ..
            } => {
                let mut handle_logical_key =
                    |output: &mut dyn Output,
                     components: &Components,
                     key: Keycode,
                     key_state: KeyState| {
                        if key_state == KeyState::Released {
                            output.key_up(key);
                            return;
                        }
                        output.key_down(key);
                        let Some(lookup) = &keyboard.lookup else {
                            return;
                        };
                        for key in lookup.lookup(components.group, components.mods, key) {
                            let sym = key.keysym();
                            'handle_sym: {
                                if let Some(compose) = &mut keyboard.compose {
                                    let res = compose.table.feed(&mut compose.state, sym);
                                    if let Some(res) = res {
                                        match res {
                                            FeedResult::Pending => output.compose_pending(sym),
                                            FeedResult::Aborted => output.compose_aborted(sym),
                                            FeedResult::Composed { string, keysym } => {
                                                output.composed(keysym, string, sym);
                                            }
                                        }
                                        break 'handle_sym;
                                    }
                                }
                                output.keysym(key.keysym(), key.char());
                            }
                        }
                    };
                let key = Keycode::from_evdev(key);
                if let Some(sm) = &mut keyboard.state_machine {
                    let direction = match key_state {
                        KeyState::Released => Direction::Up,
                        KeyState::Pressed => Direction::Down,
                        _ => return,
                    };
                    sm.events.clear();
                    sm.state_machine
                        .handle_key(&mut sm.state, &mut sm.events, key, direction);
                    let components = &mut keyboard.components;
                    macro_rules! component {
                        ($field:ident, $p:expr) => {{
                            state.output.$field($p);
                            components.$field = $p;
                            continue;
                        }};
                    }
                    for &event in &sm.events {
                        use state_machine::Event;
                        let (key, key_state) = match event {
                            Event::KeyDown(kc) => (kc, KeyState::Pressed),
                            Event::KeyUp(kc) => (kc, KeyState::Released),
                            Event::ModsPressed(p) => component!(mods_pressed, p),
                            Event::ModsLatched(p) => component!(mods_latched, p),
                            Event::ModsLocked(p) => component!(mods_locked, p),
                            Event::ModsEffective(p) => component!(mods, p),
                            Event::GroupPressed(p) => component!(group_pressed, p),
                            Event::GroupLatched(p) => component!(group_latched, p),
                            Event::GroupLocked(p) => component!(group_locked, p),
                            Event::GroupEffective(p) => component!(group, p),
                        };
                        handle_logical_key(&mut *state.output, components, key, key_state);
                    }
                    return;
                }
                handle_logical_key(&mut *state.output, &keyboard.components, key, key_state);
            }
            Event::Modifiers {
                mods_depressed: mods_pressed,
                mods_latched,
                mods_locked,
                group: group_locked,
                ..
            } => {
                if keyboard.state_machine.is_some() {
                    return;
                }
                if keyboard.components.group_locked.0 != group_locked {
                    keyboard.components.group_locked.0 = group_locked;
                    keyboard.components.update_effective();
                    state.output.group_locked(keyboard.components.group_locked);
                    state.output.group(keyboard.components.group);
                }
                macro_rules! update {
                    ($field:ident) => {
                        if keyboard.components.$field.0 != $field {
                            keyboard.components.$field.0 = $field;
                            state.output.$field(keyboard.components.$field);
                        }
                    };
                }
                update!(mods_pressed);
                update!(mods_latched);
                update!(mods_locked);
                let old_mods = keyboard.components.mods;
                keyboard.components.update_effective();
                if old_mods != keyboard.components.mods {
                    state.output.mods(keyboard.components.mods);
                }
            }
            Event::Enter { .. } => {
                if let Some(sm) = &mut keyboard.state_machine {
                    sm.state = sm.state_machine.create_state();
                    keyboard.components = Components::default();
                    state.output.state_reset();
                }
            }
            _ => {}
        }
    }
}

/// Some people don't understand that simple things should be simple.
trait WEnumGarbageExt<T> {
    fn unwrap(self) -> T;
}

impl<T> WEnumGarbageExt<T> for WEnum<T>
where
    T: Flags<Bits = u32>,
{
    fn unwrap(self) -> T {
        match self {
            WEnum::Value(t) => t,
            WEnum::Unknown(f) => T::from_bits_retain(f),
        }
    }
}
