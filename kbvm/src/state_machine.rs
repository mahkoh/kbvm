#[cfg(test)]
mod tests;

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

#[derive(Debug)]
pub struct StateMachine {
    pub(crate) num_groups: u32,
    pub(crate) num_globals: usize,
    // pub(crate) keys: HashMap<Keycode, KeyGroups>,
    pub(crate) keys: Vec<Option<KeyGroups>>,
}

#[derive(Debug)]
pub(crate) struct KeyGroups {
    pub(crate) groups: Box<[Option<KeyGroup>]>,
    pub(crate) redirect: Redirect,
}

#[derive(Debug)]
pub(crate) struct KeyGroup {
    pub(crate) ty: GroupType,
    pub(crate) levels: Box<[KeyLevel]>,
}

#[derive(Default, Debug)]
pub(crate) struct KeyLevel {
    pub(crate) routine: Option<Routine>,
}

pub struct State {
    globals: Box<[u32]>,
    layer2: Vec<Layer2Base>,
    layer3: Vec<Layer3>,
    mods_pressed_count: [u32; NUM_MODS],
    components: Components,
    actuation: u64,
}

#[derive(Copy, Clone)]
pub enum LogicalEvent {
    ModsPressed(ModifierMask),
    ModsLatched(ModifierMask),
    ModsLocked(ModifierMask),
    ModsEffective(ModifierMask),
    GroupPressed(GroupDelta),
    GroupLatched(GroupDelta),
    GroupLocked(GroupIndex),
    GroupEffective(GroupIndex),
    KeyDown(Keycode),
    KeyUp(Keycode),
}

impl Debug for LogicalEvent {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            LogicalEvent::ModsPressed(m) => write!(f, "mods_pressed = {m:?}"),
            LogicalEvent::ModsLatched(m) => write!(f, "mods_latched = {m:?}"),
            LogicalEvent::ModsLocked(m) => write!(f, "mods_locked = {m:?}"),
            LogicalEvent::ModsEffective(m) => write!(f, "mods_effective = {m:?}"),
            LogicalEvent::GroupPressed(g) => write!(f, "group_pressed = {g:?}"),
            LogicalEvent::GroupLatched(g) => write!(f, "group_latched = {g:?}"),
            LogicalEvent::GroupLocked(g) => write!(f, "group_locked = {g:?}"),
            LogicalEvent::GroupEffective(g) => write!(f, "group_effective = {g:?}"),
            LogicalEvent::KeyDown(k) => write!(f, "key_down({})", k.0),
            LogicalEvent::KeyUp(k) => write!(f, "key_up({})", k.0),
        }
    }
}

struct Layer2Handler<'a> {
    num_groups: u32,
    mods_pressed_count: &'a mut [u32; NUM_MODS],
    pub_state: &'a mut Components,
    any_state_changed: bool,
    acc_state: Components,
    events: &'a mut Vec<LogicalEvent>,
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
                        self.events.push(LogicalEvent::$camel(acs.$field));
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
        self.events.push(LogicalEvent::KeyDown(keycode));
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
        self.events.push(LogicalEvent::KeyUp(keycode));
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

struct Layer2Base {
    key: Option<Keycode>,
    layer2: Box<Layer2>,
}

#[derive(Default)]
struct Layer2 {
    actuation: u64,
    rc: u32,
    registers_log: StaticMap<Register, u32>,
    flags: StaticMap<Flag, u32>,
    on_release: Option<Arc<[Lo]>>,
    spill: Box<[u32]>,
}

#[derive(Debug)]
struct Layer3 {
    key: Option<Keycode>,
    rc: u32,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub enum Direction {
    Up,
    Down,
}

impl StateMachine {
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

    pub fn handle_key(
        &self,
        state: &mut State,
        events: &mut Vec<LogicalEvent>,
        key: Keycode,
        direction: Direction,
    ) {
        self.handle_key_(state, events, key, direction);
        state.actuation += 1;
    }

    fn handle_key_(
        &self,
        state: &mut State,
        events: &mut Vec<LogicalEvent>,
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

impl State {
    #[inline]
    pub fn components(&self) -> Components {
        self.components
    }
}
