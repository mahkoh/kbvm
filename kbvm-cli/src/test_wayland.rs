use {
    crate::{
        output::{
            Output,
            ansi::{Ansi, Theme},
            json::Json,
        },
        utils::read_path,
        wayland_protocols::{
            single_pixel_buffer_v1::wp_single_pixel_buffer_manager_v1::WpSinglePixelBufferManagerV1,
            viewporter::{wp_viewport::WpViewport, wp_viewporter::WpViewporter},
            wayland::{
                wl_buffer::WlBuffer,
                wl_compositor::WlCompositor,
                wl_display::WlDisplay,
                wl_fixes::WlFixes,
                wl_keyboard::{
                    WlKeyboard, WlKeyboardEventHandler, WlKeyboardKeyState, WlKeyboardKeymapFormat,
                    WlKeyboardRef,
                },
                wl_registry::{WlRegistry, WlRegistryEventHandler, WlRegistryRef},
                wl_seat::{WlSeat, WlSeatCapability, WlSeatEventHandler, WlSeatRef},
                wl_shm::WlShm,
                wl_surface::{WlSurface, WlSurfaceRef},
            },
            xdg_decoration_unstable_v1::{
                zxdg_decoration_manager_v1::ZxdgDecorationManagerV1,
                zxdg_toplevel_decoration_v1::ZxdgToplevelDecorationV1Mode,
            },
            xdg_shell::{
                xdg_surface::{XdgSurface, XdgSurfaceEventHandler, XdgSurfaceRef},
                xdg_toplevel::{XdgToplevel, XdgToplevelEventHandler, XdgToplevelRef},
                xdg_wm_base::XdgWmBase,
            },
        },
    },
    clap::{Args, ValueEnum, ValueHint},
    error_reporter::Report,
    hashbrown::HashMap,
    kbvm::{
        Components, Keycode,
        lookup::LookupTable,
        state_machine::{self, Direction, Event, StateMachine},
        xkb::{
            Context,
            compose::{self, ComposeTable, FeedResult},
            diagnostic::WriteToLog,
        },
    },
    memmap2::MmapOptions,
    std::{cell::RefCell, os::fd::OwnedFd, rc::Rc},
    wl_client::{
        Libwayland,
        proxy::{self, OwnedProxy},
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
    #[clap(long, value_hint = ValueHint::FilePath)]
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
    #[clap(long, value_hint = ValueHint::FilePath)]
    compose_file: Option<String>,
}

enum ComposeSetting {
    Disabled,
    Default,
    Path(String),
}

#[derive(Default)]
struct Singletons {
    wl_fixes: Option<WlFixes>,
    wl_compositor: Option<WlCompositor>,
    wl_shm: Option<WlShm>,
    wp_viewporter: Option<WpViewporter>,
    xdg_wm_base: Option<XdgWmBase>,
    wp_single_pixel_buffer_manager_v1: Option<WpSinglePixelBufferManagerV1>,
    zxdg_decoration_manager_v1: Option<ZxdgDecorationManagerV1>,
}

struct State {
    wl_registry: WlRegistry,
    _wl_fixes: Option<WlFixes>,
    _wl_compositor: WlCompositor,
    _wl_shm: Option<WlShm>,
    _wp_viewporter: WpViewporter,
    _xdg_wm_base: XdgWmBase,
    _wp_single_pixel_buffer_manager_v1: Option<WpSinglePixelBufferManagerV1>,
    _zxdg_decoration_manager_v1: Option<ZxdgDecorationManagerV1>,
    context: Context,
    wl_surface: WlSurface,
    wl_buffer: WlBuffer,
    wp_viewport: WpViewport,
    xdg_surface: XdgSurface,
    _xdg_toplevel: XdgToplevel,
    compose: ComposeSetting,
    print_keymap: bool,
    create_state_machine: bool,
    lookup_table: Option<LookupTable>,
    state_machine: Option<StateMachine>,
    mutable: RefCell<StateMutable>,
}

struct StateMutable {
    seats: HashMap<u32, Rc<RefCell<Seat>>>,
    width: i32,
    height: i32,
    output: Box<dyn Output>,
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

struct Seat {
    state: Rc<State>,
    wl_seat: WlSeat,
    name: String,
    keyboard: Option<WlKeyboard>,
}

struct Keyboard {
    state: Rc<State>,
    lookup: Option<LookupTable>,
    state_machine: Option<Sm>,
    components: Components,
    compose: Option<Compose>,
}

pub fn main(args: TestWaylandArgs) {
    if let Some(color) = args.color {
        match color {
            Color::Never => owo_colors::set_override(false),
            Color::Always => owo_colors::set_override(true),
            Color::Auto => {}
        }
    }
    let libwayland = match Libwayland::open() {
        Ok(l) => l,
        Err(e) => {
            log::error!("could not open libwayland: {}", Report::new(e));
            std::process::exit(1);
        }
    };
    let con = match libwayland.connect_to_default_display() {
        Ok(c) => c,
        Err(e) => {
            log::error!("could not connect to compositor: {}", Report::new(e));
            std::process::exit(1);
        }
    };
    let queue = con.create_local_queue(c"kbvm");
    let wl_display: WlDisplay = queue.display();
    let mut r = get_singletons(&wl_display);
    macro_rules! check_singleton {
        ($($name:ident),*) => {
            if $(r.$name.is_none())&&* {
                $(
                    log::error!("compositor does not expose {} global", stringify!($name));
                )*
                std::process::exit(1);
            }
        };
    }
    check_singleton!(xdg_wm_base);
    check_singleton!(wl_compositor);
    check_singleton!(wp_viewporter);
    #[cfg(target_os = "linux")]
    check_singleton!(wp_single_pixel_buffer_manager_v1, wl_shm);
    #[cfg(not(target_os = "linux"))]
    check_singleton!(wp_single_pixel_buffer_manager_v1);
    let wl_compositor = r.wl_compositor.take().unwrap();
    let wp_viewporter = r.wp_viewporter.take().unwrap();
    let xdg_wm_base = r.xdg_wm_base.take().unwrap();
    let wl_shm = r.wl_shm.take();
    let zxdg_decoration_manager_v1 = r.zxdg_decoration_manager_v1.take();
    let wp_single_pixel_buffer_manager_v1 = r.wp_single_pixel_buffer_manager_v1.take();
    let wl_surface = wl_compositor.create_surface();
    let wp_viewport = wp_viewporter.get_viewport(&wl_surface);
    let xdg_surface = xdg_wm_base.get_xdg_surface(&wl_surface);
    let xdg_toplevel = xdg_surface.get_toplevel();
    xdg_toplevel.set_title("KBVM");
    if let Some(dm) = &zxdg_decoration_manager_v1 {
        let decoration = dm.get_toplevel_decoration(&xdg_toplevel);
        decoration.set_mode(ZxdgToplevelDecorationV1Mode::SERVER_SIDE);
    }
    wl_surface.commit();
    #[allow(unused_labels)]
    let wl_buffer = 'buffer: {
        #[cfg(target_os = "linux")]
        if wp_single_pixel_buffer_manager_v1.is_none() {
            break 'buffer create_shm_buffer(wl_shm.as_ref());
        }
        wp_single_pixel_buffer_manager_v1
            .as_ref()
            .unwrap()
            .create_u32_rgba_buffer(0, 0, 0, !0)
    };
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
    let wl_registry = wl_display.get_registry();
    let state = Rc::new(State {
        wl_registry: wl_registry.clone(),
        _wl_fixes: r.wl_fixes,
        _wl_compositor: wl_compositor,
        _wl_shm: wl_shm,
        _wp_viewporter: wp_viewporter,
        _xdg_wm_base: xdg_wm_base.clone(),
        _wp_single_pixel_buffer_manager_v1: wp_single_pixel_buffer_manager_v1,
        _zxdg_decoration_manager_v1: zxdg_decoration_manager_v1,
        context: Default::default(),
        wl_surface,
        wl_buffer,
        wp_viewport,
        xdg_surface: xdg_surface.clone(),
        _xdg_toplevel: xdg_toplevel.clone(),
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
        mutable: RefCell::new(StateMutable {
            seats: Default::default(),
            width: 800,
            height: 600,
            output,
        }),
    });
    proxy::set_event_handler_local(
        &xdg_wm_base,
        XdgWmBase::on_ping(|wm, serial| wm.pong(serial)),
    );
    proxy::set_event_handler_local(&wl_registry, state.clone());
    proxy::set_event_handler_local(&xdg_toplevel, state.clone());
    proxy::set_event_handler_local(&xdg_surface, state.clone());
    loop {
        if let Err(e) = queue.dispatch_blocking() {
            panic!("{}", Report::new(e));
        }
    }
}

fn get_singletons(wl_display: &WlDisplay) -> Singletons {
    let wl = wl_display.get_registry();
    let queue = proxy::queue(wl_display);
    let registry = RefCell::new(Singletons::default());
    queue.dispatch_scope_blocking(|scope| {
        scope.set_event_handler_local(
            &wl,
            WlRegistry::on_global(|_, name, interface, version| {
                let reg = &mut *registry.borrow_mut();
                macro_rules! bind {
                    ($max_ver:expr) => {
                        Some(wl.bind(name, version.min($max_ver)))
                    };
                }
                match interface {
                    WlFixes::INTERFACE => reg.wl_fixes = bind!(1),
                    WlCompositor::INTERFACE => reg.wl_compositor = bind!(1),
                    WlShm::INTERFACE => reg.wl_shm = bind!(1),
                    XdgWmBase::INTERFACE => reg.xdg_wm_base = bind!(1),
                    WpViewporter::INTERFACE => reg.wp_viewporter = bind!(1),
                    ZxdgDecorationManagerV1::INTERFACE => reg.zxdg_decoration_manager_v1 = bind!(1),
                    WpSinglePixelBufferManagerV1::INTERFACE => {
                        reg.wp_single_pixel_buffer_manager_v1 = bind!(1)
                    }
                    _ => {}
                }
            }),
        );
        queue.dispatch_roundtrip_blocking().unwrap();
    });
    let r = registry.into_inner();
    if let Some(fixes) = &r.wl_fixes {
        fixes.destroy_registry(&wl);
    }
    proxy::destroy(&wl);
    r
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

impl WlRegistryEventHandler for Rc<State> {
    fn global(&self, _slf: &WlRegistryRef, name: u32, interface: &str, version: u32) {
        if interface != WlSeat::INTERFACE {
            return;
        }
        let wl_seat = self.wl_registry.bind::<WlSeat>(name, version);
        let seat = Rc::new(RefCell::new(Seat {
            state: self.clone(),
            wl_seat: wl_seat.clone(),
            name: String::new(),
            keyboard: None,
        }));
        proxy::set_event_handler_local(&wl_seat, seat.clone());
        self.mutable.borrow_mut().seats.insert(name, seat);
    }

    fn global_remove(&self, _slf: &WlRegistryRef, name: u32) {
        let slf = &mut *self.mutable.borrow_mut();
        let Some(seat) = slf.seats.remove(&name) else {
            return;
        };
        let seat = &mut *seat.borrow_mut();
        if let Some(kb) = &seat.keyboard {
            kb.release();
        }
        seat.wl_seat.release();
    }
}

impl WlSeatEventHandler for Rc<RefCell<Seat>> {
    fn capabilities(&self, _slf: &WlSeatRef, capabilities: WlSeatCapability) {
        let slf = &mut *self.borrow_mut();
        let has_kb = capabilities.contains(WlSeatCapability::KEYBOARD);
        if has_kb {
            if slf.keyboard.is_none() {
                let state = &*slf.state;
                let create_compose = |path: Option<&str>| {
                    let mut builder = state.context.compose_table_builder();
                    if let Some(path) = path {
                        builder.file(path);
                    }
                    builder.build(WriteToLog).map(|table| Compose {
                        state: table.create_state(),
                        table,
                    })
                };
                let compose = match &state.compose {
                    ComposeSetting::Disabled => None,
                    ComposeSetting::Default => create_compose(None),
                    ComposeSetting::Path(p) => create_compose(Some(p)),
                };
                let wl_keyboard = slf.wl_seat.get_keyboard();
                let keyboard = RefCell::new(Keyboard {
                    state: slf.state.clone(),
                    lookup: state.lookup_table.clone(),
                    state_machine: state.state_machine.clone().map(|sm| Sm {
                        state: sm.create_state(),
                        state_machine: sm,
                        events: vec![],
                    }),
                    components: Default::default(),
                    compose,
                });
                proxy::set_event_handler_local(&wl_keyboard, keyboard);
                slf.keyboard = Some(wl_keyboard);
            }
        } else {
            if let Some(kb) = slf.keyboard.take() {
                kb.release();
            }
        }
    }

    fn name(&self, _slf: &WlSeatRef, name: &str) {
        self.borrow_mut().name = name.to_string();
    }
}

impl XdgSurfaceEventHandler for Rc<State> {
    fn configure(&self, _slf: &XdgSurfaceRef, serial: u32) {
        let slf = &mut *self.mutable.borrow_mut();
        self.xdg_surface.ack_configure(serial);
        self.wp_viewport.set_destination(slf.width, slf.height);
        self.wl_surface.attach(Some(&self.wl_buffer), 0, 0);
        self.wl_surface.commit();
    }
}

impl XdgToplevelEventHandler for Rc<State> {
    fn configure(&self, _slf: &XdgToplevelRef, mut width: i32, mut height: i32, _states: &[u8]) {
        let slf = &mut *self.mutable.borrow_mut();
        if width == 0 {
            width = 800;
        }
        if height == 0 {
            height = 600;
        }
        slf.width = width;
        slf.height = height;
    }

    fn close(&self, _slf: &XdgToplevelRef) {
        std::process::exit(0);
    }
}

impl WlKeyboardEventHandler for RefCell<Keyboard> {
    fn keymap(
        &self,
        _slf: &WlKeyboardRef,
        _format: WlKeyboardKeymapFormat,
        fd: OwnedFd,
        size: u32,
    ) {
        let slf = &mut *self.borrow_mut();
        let state = &*slf.state;
        if state.state_machine.is_some() {
            return;
        }
        let mutable = &mut *state.mutable.borrow_mut();
        let map = unsafe { MmapOptions::new().len(size as usize).map(&fd).unwrap() };
        let map = state.context.keymap_from_bytes(WriteToLog, None, &map);
        let map = match map {
            Ok(map) => map,
            Err(_) => {
                log::error!("Could not parse keymap");
                slf.lookup = None;
                slf.state_machine = None;
                return;
            }
        };
        if state.print_keymap {
            mutable.output.keymap(&map);
        }
        let builder = map.to_builder();
        if state.create_state_machine {
            let sm = builder.build_state_machine();
            slf.state_machine = Some(Sm {
                state: sm.create_state(),
                state_machine: sm,
                events: vec![],
            });
        }
        let lookup = builder.build_lookup_table();
        slf.lookup = Some(lookup);
    }

    fn enter(
        &self,
        _slf: &WlKeyboardRef,
        _serial: u32,
        _surface: Option<&WlSurfaceRef>,
        _keys: &[u8],
    ) {
        let slf = &mut *self.borrow_mut();
        let mutable = &mut *slf.state.mutable.borrow_mut();
        if let Some(sm) = &mut slf.state_machine {
            sm.state = sm.state_machine.create_state();
            slf.components = Components::default();
            mutable.output.state_reset();
        }
    }

    fn key(
        &self,
        _slf: &WlKeyboardRef,
        _serial: u32,
        _time: u32,
        key: u32,
        key_state: WlKeyboardKeyState,
    ) {
        let slf = &mut *self.borrow_mut();
        let mutable = &mut *slf.state.mutable.borrow_mut();
        let mut handle_logical_key =
            |output: &mut dyn Output,
             components: &Components,
             key: Keycode,
             key_state: WlKeyboardKeyState| {
                if key_state == WlKeyboardKeyState::RELEASED {
                    output.key_up(key);
                    return;
                }
                output.key_down(key);
                let Some(lookup) = &slf.lookup else {
                    return;
                };
                for key in lookup.lookup(components.group, components.mods, key) {
                    let sym = key.keysym();
                    'handle_sym: {
                        if let Some(compose) = &mut slf.compose {
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
        if let Some(sm) = &mut slf.state_machine {
            let direction = match key_state {
                WlKeyboardKeyState::RELEASED => Direction::Up,
                WlKeyboardKeyState::PRESSED => Direction::Down,
                _ => return,
            };
            sm.events.clear();
            sm.state_machine
                .handle_key(&mut sm.state, &mut sm.events, key, direction);
            let components = &mut slf.components;
            macro_rules! component {
                ($field:ident, $p:expr) => {{
                    mutable.output.$field($p);
                    components.$field = $p;
                    continue;
                }};
            }
            for &event in &sm.events {
                use state_machine::Event;
                let (key, key_state) = match event {
                    Event::KeyDown(kc) => (kc, WlKeyboardKeyState::PRESSED),
                    Event::KeyUp(kc) => (kc, WlKeyboardKeyState::RELEASED),
                    Event::ModsPressed(p) => component!(mods_pressed, p),
                    Event::ModsLatched(p) => component!(mods_latched, p),
                    Event::ModsLocked(p) => component!(mods_locked, p),
                    Event::ModsEffective(p) => component!(mods, p),
                    Event::GroupPressed(p) => component!(group_pressed, p),
                    Event::GroupLatched(p) => component!(group_latched, p),
                    Event::GroupLocked(p) => component!(group_locked, p),
                    Event::GroupEffective(p) => component!(group, p),
                    Event::Controls(p) => component!(controls, p),
                };
                handle_logical_key(&mut *mutable.output, components, key, key_state);
            }
            return;
        }
        handle_logical_key(&mut *mutable.output, &slf.components, key, key_state);
    }

    fn modifiers(
        &self,
        _slf: &WlKeyboardRef,
        _serial: u32,
        mods_pressed: u32,
        mods_latched: u32,
        mods_locked: u32,
        group: u32,
    ) {
        let slf = &mut *self.borrow_mut();
        if slf.state_machine.is_some() {
            return;
        }
        let mutable = &mut *slf.state.mutable.borrow_mut();
        if slf.components.group_locked.0 != group {
            slf.components.group_locked.0 = group;
            slf.components.update_effective();
            mutable.output.group_locked(slf.components.group_locked);
            mutable.output.group(slf.components.group);
        }
        macro_rules! update {
            ($field:ident) => {
                if slf.components.$field.0 != $field {
                    slf.components.$field.0 = $field;
                    mutable.output.$field(slf.components.$field);
                }
            };
        }
        update!(mods_pressed);
        update!(mods_latched);
        update!(mods_locked);
        let old_mods = slf.components.mods;
        slf.components.update_effective();
        if old_mods != slf.components.mods {
            mutable.output.mods(slf.components.mods);
        }
    }
}

#[cfg(target_os = "linux")]
fn create_shm_buffer(wl_shm: Option<&WlShm>) -> WlBuffer {
    use {
        crate::wayland_protocols::wayland::wl_shm::WlShmFormat,
        std::{io::Write, os::fd::AsFd},
        uapi::c,
    };
    let mem = uapi::memfd_create("buffer", c::MFD_CLOEXEC | c::MFD_ALLOW_SEALING);
    let mut mem = match mem {
        Ok(m) => m,
        Err(e) => {
            log::error!("could not create memfd: {}", Report::new(e));
            std::process::exit(1);
        }
    };
    if let Err(e) = mem.write_all(&[0, 0, 0, !0]) {
        log::error!("could not write to memfd: {}", e);
        std::process::exit(1);
    }
    let _ = uapi::fcntl_add_seals(mem.raw(), c::F_SEAL_SHRINK);
    let pool = wl_shm.unwrap().create_pool(mem.as_fd(), 4);
    pool.create_buffer(0, 1, 1, 4, WlShmFormat::XRGB8888)
}
