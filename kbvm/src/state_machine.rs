//! The compositor-side [`StateMachine`].
//!
//! This module contains types to map physical key events to logical events.
//!
//! The main entry point to this module is the [`StateMachine`] type.
//!
//! # Example
//!
//! ```
//! # use kbvm::state_machine::StateMachine;
//! # use kbvm::xkb::Context;
//! # use kbvm::xkb::diagnostic::WriteToLog;
//! #
//! fn create_state_machine(keymap: &[u8]) -> StateMachine {
//!     let context = Context::default();
//!     context
//!         .keymap_from_bytes(WriteToLog, None, keymap)
//!         .unwrap()
//!         .to_builder()
//!         .build_state_machine()
//! }
//! ```

#[cfg(test)]
mod tests;

#[allow(unused_imports)]
use crate::evdev;
use {
    crate::{
        builder::Redirect,
        components::Components,
        group::{GroupDelta, GroupIndex},
        modifier::{NUM_MODS, NUM_MODS_MASK},
        routine::{run, Flag, Lo, Register, Routine, StateEventHandler},
        state_machine::hidden::Keycode,
        GroupType, ModifierMask,
    },
    isnt::std_1::primitive::IsntSliceExt,
    linearize::StaticMap,
    std::{
        fmt::{Debug, Formatter},
        sync::Arc,
    },
};

/// The compositor-side state machine.
///
/// This type encodes the business logic turning libinput key events into `wl_keyboard`
/// events. The documentation of [`Self::handle_key`] has an example showing how to
/// integrate this type into a wayland compositor.
///
/// # Example
///
/// ```
/// # use kbvm::state_machine::StateMachine;
/// # use kbvm::xkb::Context;
/// # use kbvm::xkb::diagnostic::WriteToLog;
/// #
/// fn create_state_machine(keymap: &[u8]) -> StateMachine {
///     let context = Context::default();
///     context
///         .keymap_from_bytes(WriteToLog, None, keymap)
///         .unwrap()
///         .to_builder()
///         .build_state_machine()
/// }
/// ```
#[derive(Debug, Clone)]
pub struct StateMachine {
    pub(crate) num_groups: u32,
    pub(crate) num_globals: usize,
    // pub(crate) keys: HashMap<Keycode, KeyGroups>,
    pub(crate) keys: Vec<Option<KeyGroups>>,
}

#[derive(Debug, Clone)]
pub(crate) struct KeyGroups {
    pub(crate) groups: Box<[Option<KeyGroup>]>,
    pub(crate) redirect: Redirect,
}

#[derive(Debug, Clone)]
pub(crate) struct KeyGroup {
    pub(crate) ty: GroupType,
    pub(crate) levels: Box<[KeyLevel]>,
}

#[derive(Default, Debug, Clone)]
pub(crate) struct KeyLevel {
    pub(crate) routine: Option<Routine>,
}

/// The state of a state machine.
///
/// This type can be created via [`StateMachine::create_state`] and every [`State`]
/// object should only be used with the state machine it was created from.
#[derive(Clone, Debug)]
pub struct State {
    globals: Box<[u32]>,
    layer2: Vec<Layer2Base>,
    layer3: Vec<Layer3>,
    mods_pressed_count: [u32; NUM_MODS],
    components: Components,
    actuation: u64,
}

/// An event emitted by a [`StateMachine`].
///
/// This event might cause the [`Components`] of the state to change. You can easily
/// apply this change by calling [`Components::apply_event`].
///
/// # Effective Modifiers and Group
///
/// When the pressed/latched/locked modifiers or group change, this might also affect the
/// effective modifiers/group since the effective modifiers are defined as
///
/// ```text
/// mods_effective = mods_pressed | mods_latched | mods_locked
/// ```
///
/// and the effective group is defined as
///
/// ```text
/// group_effective = group_pressed + group_latched + group_locked
/// ```
///
/// If this happens, the event changing the pressed/latched/locked modifiers or group is
/// always followed by an event changing the effective modifiers or group. For example,
/// if a key press changes the pressed and latched modifiers, you might see the following
/// sequence of events:
///
/// - `ModsPressed(ModifierMask::SHIFT)`
/// - `ModsLatched(ModifierMask::CONTROL)`
/// - `ModsEffective(ModifierMask::SHIFT | ModifierMask::CONTROL)`
#[derive(Copy, Clone, Eq, PartialEq)]
pub enum Event {
    /// A key was logically pressed.
    KeyDown(Keycode),
    /// A key was logically released.
    KeyUp(Keycode),
    /// The pressed modifiers have changed.
    ModsPressed(ModifierMask),
    /// The latched modifiers have changed.
    ModsLatched(ModifierMask),
    /// The locked modifiers have changed.
    ModsLocked(ModifierMask),
    /// The effective modifiers have changed.
    ModsEffective(ModifierMask),
    /// The pressed group has changed.
    GroupPressed(GroupDelta),
    /// The latched group has changed.
    GroupLatched(GroupDelta),
    /// The locked group has changed.
    GroupLocked(GroupIndex),
    /// The effective group has changed.
    GroupEffective(GroupIndex),
}

impl Debug for Event {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Event::ModsPressed(m) => write!(f, "mods_pressed = {m:?}"),
            Event::ModsLatched(m) => write!(f, "mods_latched = {m:?}"),
            Event::ModsLocked(m) => write!(f, "mods_locked = {m:?}"),
            Event::ModsEffective(m) => write!(f, "mods_effective = {m:?}"),
            Event::GroupPressed(g) => write!(f, "group_pressed = {g:?}"),
            Event::GroupLatched(g) => write!(f, "group_latched = {g:?}"),
            Event::GroupLocked(g) => write!(f, "group_locked = {g:?}"),
            Event::GroupEffective(g) => write!(f, "group_effective = {g:?}"),
            Event::KeyDown(k) => write!(f, "key_down({})", k.0),
            Event::KeyUp(k) => write!(f, "key_up({})", k.0),
        }
    }
}

struct Layer2Handler<'a> {
    num_groups: u32,
    mods_pressed_count: &'a mut [u32; NUM_MODS],
    pub_state: &'a mut Components,
    any_state_changed: bool,
    acc_state: Components,
    events: &'a mut Vec<Event>,
    layer3: &'a mut Vec<Layer3>,
}

impl Layer2Handler<'_> {
    #[inline(always)]
    fn flush_state(&mut self) {
        if !self.any_state_changed {
            return;
        }
        self.flush_state_();
    }

    #[inline(never)]
    fn flush_state_(&mut self) {
        self.any_state_changed = false;
        let acs = &mut self.acc_state;
        acs.mods = acs.mods_pressed | acs.mods_latched | acs.mods_locked;
        acs.group = acs.group_locked + acs.group_pressed + acs.group_latched;
        macro_rules! wrap_group {
            ($value:expr) => {
                if $value.0 >= self.num_groups {
                    let tmp = $value.0 as i32 as i64;
                    $value.0 = if tmp < 0 {
                        (tmp % self.num_groups as i64 + self.num_groups as i64) as u32
                    } else {
                        (tmp % self.num_groups as i64) as u32
                    };
                }
            };
        }
        wrap_group!(acs.group_locked);
        wrap_group!(acs.group);
        macro_rules! flush {
            ($($camel:ident, $field:ident;)*) => {
                $(
                    if acs.$field != self.pub_state.$field {
                        self.pub_state.$field = acs.$field;
                        self.events.push(Event::$camel(acs.$field));
                    }
                )*
            };
        }
        flush! {
            ModsPressed, mods_pressed;
            ModsLatched, mods_latched;
            ModsLocked, mods_locked;
            ModsEffective, mods;
            GroupPressed, group_pressed;
            GroupLatched, group_latched;
            GroupLocked, group_locked;
            GroupEffective, group;
        }
    }
}

impl StateEventHandler for Layer2Handler<'_> {
    #[inline]
    fn mods_pressed_inc(&mut self, mods: ModifierMask) {
        let mut changed = false;
        for idx in mods {
            let count = &mut self.mods_pressed_count[idx.raw() as usize & NUM_MODS_MASK];
            if *count == 0 {
                *count = 1;
                changed = true;
            } else {
                *count = count.saturating_add(1);
            }
        }
        if changed {
            self.any_state_changed = true;
            self.acc_state.mods_pressed |= mods;
        }
    }

    #[inline]
    fn mods_pressed_dec(&mut self, mods: ModifierMask) {
        let mut changed = ModifierMask(0);
        for idx in mods {
            let count = &mut self.mods_pressed_count[idx.raw() as usize & NUM_MODS_MASK];
            if *count == 1 {
                *count = 0;
                changed |= idx.to_mask();
            } else {
                *count = count.saturating_sub(1);
            }
        }
        if changed.0 != 0 {
            self.any_state_changed = true;
            self.acc_state.mods_pressed &= !changed;
        }
    }

    #[inline]
    fn mods_pressed_load(&self) -> u32 {
        self.acc_state.mods_pressed.0
    }

    #[inline]
    fn mods_pressed_store(&mut self, val: u32) {
        self.any_state_changed = true;
        self.acc_state.mods_pressed.0 = val;
    }

    #[inline]
    fn mods_latched_load(&self) -> u32 {
        self.acc_state.mods_latched.0
    }

    #[inline]
    fn mods_latched_store(&mut self, val: u32) {
        self.any_state_changed = true;
        self.acc_state.mods_latched.0 = val;
    }

    #[inline]
    fn mods_locked_load(&self) -> u32 {
        self.acc_state.mods_locked.0
    }

    #[inline]
    fn mods_locked_store(&mut self, val: u32) {
        self.any_state_changed = true;
        self.acc_state.mods_locked.0 = val;
    }

    #[inline]
    fn group_pressed_load(&self) -> u32 {
        self.acc_state.group_pressed.0
    }

    #[inline]
    fn group_pressed_store(&mut self, val: u32) {
        self.any_state_changed = true;
        self.acc_state.group_pressed.0 = val;
    }

    #[inline]
    fn group_latched_load(&self) -> u32 {
        self.acc_state.group_latched.0
    }

    #[inline]
    fn group_latched_store(&mut self, val: u32) {
        self.any_state_changed = true;
        self.acc_state.group_latched.0 = val;
    }

    #[inline]
    fn group_locked_load(&self) -> u32 {
        self.acc_state.group_locked.0
    }

    #[inline]
    fn group_locked_store(&mut self, val: u32) {
        self.any_state_changed = true;
        self.acc_state.group_locked.0 = val;
    }

    #[inline(always)]
    fn key_down(&mut self, keycode: Keycode) {
        // println!("down {:?}", self.layer3);
        let mut slot = None;
        for key in &mut *self.layer3 {
            if key.key == Some(keycode) {
                key.rc = key.rc.saturating_add(1);
                return;
            } else if key.key.is_none() {
                slot = Some(key);
            }
        }
        let layer3 = match slot {
            Some(layer3) => layer3,
            _ => {
                self.layer3.push(Layer3 { key: None, rc: 0 });
                self.layer3.last_mut().unwrap()
            }
        };
        layer3.key = Some(keycode);
        layer3.rc = 1;
        self.flush_state();
        self.events.push(Event::KeyDown(keycode));
    }

    #[inline(always)]
    fn key_up(&mut self, keycode: Keycode) {
        // println!("up   {:?}", self.layer3);
        'find_key: {
            for key in &mut *self.layer3 {
                if key.key != Some(keycode) {
                    continue;
                }
                key.rc -= 1;
                if key.rc != 0 {
                    return;
                }
                key.key = None;
                break 'find_key;
            }
            return;
        }
        self.flush_state();
        self.events.push(Event::KeyUp(keycode));
    }
}

pub(crate) mod hidden {
    #[allow(unused_imports)]
    use crate::evdev;
    use kbvm_proc::CloneWithDelta;

    /// A keycode.
    ///
    /// Keycodes represent physical keys. On Linux, they usually correspond to evdev
    /// events which are in turn modeled after the USB standard. The [`evdev`] module
    /// contains constants for evdev keycodes.
    #[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, CloneWithDelta, Default)]
    pub struct Keycode(pub(crate) u32);
}

impl Keycode {
    /// Returns the raw `u32` representing this keycode.
    ///
    /// This value should only be used in [`Routine`]s.
    pub const fn raw(self) -> u32 {
        self.0
    }

    /// Creates a keycode from an X11 keycode.
    #[inline]
    pub const fn from_x11(kc: u32) -> Self {
        Self(kc)
    }

    /// Converts the keycode to an X11 keycode.
    ///
    /// If this keycode was not created via [`Keycode::from_x11`], then the conversion is
    /// performed on a best-effort basis.
    #[inline]
    pub const fn to_x11(self) -> u32 {
        self.0
    }

    /// Creates a keycode from an evdev code.
    #[inline]
    pub const fn from_evdev(kc: u32) -> Self {
        Self(kc.saturating_add(8))
    }

    /// Converts the keycode to an evdev code.
    ///
    /// If this keycode was not created via [`Keycode::from_evdev`], then the conversion
    /// is performed on a best-effort basis.
    #[inline]
    pub const fn to_evdev(self) -> u32 {
        self.0.saturating_sub(8)
    }
}

#[derive(Clone, Debug)]
struct Layer2Base {
    key: Option<Keycode>,
    layer2: Box<Layer2>,
}

#[derive(Default, Clone, Debug)]
struct Layer2 {
    actuation: u64,
    rc: u32,
    registers_log: StaticMap<Register, u32>,
    flags: StaticMap<Flag, u32>,
    on_release: Option<Arc<[Lo]>>,
    spill: Box<[u32]>,
}

#[derive(Debug, Clone)]
struct Layer3 {
    key: Option<Keycode>,
    rc: u32,
}

/// The direction of a key event.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
#[non_exhaustive]
pub enum Direction {
    /// The key was released.
    Up,
    /// The key was pressed.
    Down,
}

impl StateMachine {
    /// Creates a new [`State`] object that can be used with this state machine.
    pub fn create_state(&self) -> State {
        State {
            globals: vec![0; self.num_globals].into_boxed_slice(),
            layer2: Default::default(),
            layer3: Default::default(),
            mods_pressed_count: Default::default(),
            components: Default::default(),
            actuation: Default::default(),
        }
    }

    /// Turns a key press/release event into [`Event`]s and updates the [`State`].
    ///
    /// The `state` should have been created with [`Self::create_state`] of this object.
    /// Otherwise this function might panic.
    ///
    /// The key-press events are reference counted. This means that a key will remain
    /// pressed until the number of key-release events matches the number of key-press
    /// events. For example
    ///
    /// ```text
    /// key_press(A)    -  emits events for the key press
    /// key_press(A)    -  produces no events
    /// key_release(A)  -  produces no events
    /// key_release(A)  -  emits events for the key release
    /// ```
    ///
    /// This means that callers of this function should ensure that the number of
    /// key-release events eventually matches the number of key-press events for the same
    /// key. For example:
    ///
    /// - If a keyboard is disconnected while a key is pressed, the compositor might want
    ///   to synthesize a key-release.
    /// - If a libei client disconnects while a key is pressed, dito.
    ///
    /// In turn, this call will only ever emit symmetric [`KeyDown`](Event::KeyDown) and
    /// [`KeyUp`](Event::KeyUp) events. That is, a `KeyUp` event will not be emitted
    /// unless it is preceded by a corresponding `KeyDown` event and no `KeyDown` event
    /// will be emitted if the key is already down.
    ///
    /// Compositors should use this function as follows:
    ///
    /// 1. Receive keyboard events from libinput or libei.
    /// 2. Feed the event into this function.
    /// 3. Discard the original libinput/libei event.
    /// 4. Process the events emitted via the `events` parameter.
    ///
    /// It is important that the compositor does not forward the libinput/libei events
    /// directly to clients. For example, the state machine might be configured to
    /// redirect the [`evdev::CAPSLOCK`] key to the [`evdev::LEFTCTRL`] key. Instead, the
    /// compositor must use the `KeyDown` and `KeyUp` events that are emitted via the
    /// `events` parameter.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::{Components, Keycode};
    /// # use kbvm::state_machine::{Direction, Event, State, StateMachine};
    /// # fn send_components_to_clients(components: &Components) { }
    /// # fn send_key_event_to_clients(keycode: Keycode, down: bool) { }
    /// #
    /// fn handle_key_event(
    ///     state_machine: &StateMachine,
    ///     state: &mut State,
    ///     key: Keycode,
    ///     direction: Direction,
    ///     components: &mut Components,
    /// ) {
    ///     let mut events = vec!();
    ///     state_machine.handle_key(state, &mut events, key, direction);
    ///     let mut components_changed = false;
    ///     for event in events {
    ///         components_changed |= components.apply_event(event);
    ///         let (keycode, down) = match event {
    ///             Event::KeyDown(kc) => (kc, true),
    ///             Event::KeyUp(kc) => (kc, false),
    ///             _ => continue,
    ///         };
    ///         if components_changed {
    ///             components_changed = false;
    ///             send_components_to_clients(components);
    ///         }
    ///         send_key_event_to_clients(keycode, down);
    ///     }
    ///     if components_changed {
    ///         send_components_to_clients(components);
    ///     }
    /// }
    /// ```
    pub fn handle_key(
        &self,
        state: &mut State,
        events: &mut Vec<Event>,
        key: Keycode,
        direction: Direction,
    ) {
        self.handle_key_(state, events, key, direction);
        state.actuation += 1;
    }

    fn handle_key_(
        &self,
        state: &mut State,
        events: &mut Vec<Event>,
        key: Keycode,
        direction: Direction,
    ) {
        let mut slot = None;
        for base in &mut state.layer2 {
            if base.key != Some(key) {
                if base.key.is_none() {
                    slot = Some(base);
                }
                continue;
            }
            let layer2 = &mut base.layer2;
            if direction == Direction::Down {
                layer2.rc = layer2.rc.saturating_add(1);
                return;
            }
            layer2.rc -= 1;
            if layer2.rc != 0 {
                return;
            }
            layer2.flags[Flag::LaterKeyActuated] = (layer2.actuation < state.actuation) as u32;
            let mut handler = Layer2Handler {
                num_groups: self.num_groups,
                mods_pressed_count: &mut state.mods_pressed_count,
                acc_state: state.components,
                pub_state: &mut state.components,
                events,
                any_state_changed: false,
                layer3: &mut state.layer3,
            };
            if let Some(release) = &layer2.on_release {
                run(
                    &mut handler,
                    release,
                    &mut layer2.registers_log,
                    &mut state.globals,
                    &mut layer2.flags,
                    &mut layer2.spill,
                );
                handler.flush_state();
            } else {
                handler.key_up(key);
            }
            base.key = None;
            return;
        }
        if direction == Direction::Up {
            return;
        }
        let group = state.components.group;
        let mods = state.components.mods;
        let mut on_press = None;
        let mut on_release = None;
        let mut spill = 0;
        if let Some(Some(key_groups)) = self.keys.get(key.0 as usize) {
            if key_groups.groups.is_not_empty() {
                let group = key_groups.redirect.apply(group, key_groups.groups.len());
                if let Some(key_group) = &key_groups.groups[group] {
                    let mapping = key_group.ty.map(mods);
                    let level = mapping.level;
                    if let Some(key_level) = key_group.levels.get(level) {
                        if let Some(routine) = &key_level.routine {
                            on_press = Some(&routine.on_press);
                            on_release = Some(routine.on_release.clone());
                            spill = routine.spill;
                        }
                    }
                }
            }
        }
        let base = match slot {
            Some(slot) => slot,
            _ => {
                state.layer2.push(Layer2Base {
                    key: None,
                    layer2: Default::default(),
                });
                state.layer2.last_mut().unwrap()
            }
        };
        base.key = Some(key);
        let layer2 = &mut base.layer2;
        layer2.rc = 1;
        layer2.actuation = state.actuation + 1;
        layer2.registers_log = Default::default();
        layer2.flags = Default::default();
        layer2.on_release = on_release;
        if spill > 0 {
            if spill <= layer2.spill.len() {
                layer2.spill[..spill].fill(0);
            } else {
                layer2.spill = vec![0; spill].into_boxed_slice();
            }
        }
        let mut handler = Layer2Handler {
            num_groups: self.num_groups,
            mods_pressed_count: &mut state.mods_pressed_count,
            acc_state: state.components,
            pub_state: &mut state.components,
            events,
            any_state_changed: false,
            layer3: &mut state.layer3,
        };
        if let Some(on_press) = on_press {
            run(
                &mut handler,
                on_press,
                &mut layer2.registers_log,
                &mut state.globals,
                &mut layer2.flags,
                &mut layer2.spill,
            );
        } else {
            handler.key_down(key);
            if handler.acc_state.any_latched() {
                handler.mods_latched_store(0);
                handler.group_latched_store(0);
            }
        }
        handler.flush_state();
    }
}
