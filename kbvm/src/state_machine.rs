#[cfg(test)]
mod tests;

use {
    crate::{
        group_type::GroupType,
        modifier::{ModifierMask, NUM_MODS, NUM_MODS_MASK},
        routine::{run, Component, Flag, Global, Lo, Register, Routine, StateEventHandler},
    },
    hashbrown::HashMap,
    kbvm_proc::CloneWithDelta,
    linearize::StaticMap,
    std::{
        fmt::{Debug, Formatter},
        sync::Arc,
    },
};

#[derive(Debug)]
pub struct StateMachine {
    pub(crate) keys: HashMap<Keycode, KeyGroups>,
    // pub(crate) keys: Vec<KeyGroups>,
}

#[derive(Default, Debug)]
pub(crate) struct KeyGroups {
    pub(crate) groups: Box<[Option<KeyGroup>]>,
}

#[derive(Debug)]
pub(crate) struct KeyGroup {
    pub(crate) ty: GroupType,
    pub(crate) layers: Box<[KeyLayer]>,
}

#[derive(Default, Debug)]
pub(crate) struct KeyLayer {
    pub(crate) routine: Option<Routine>,
}

#[derive(Default)]
pub struct State {
    globals: StaticMap<Global, u32>,
    active: Vec<ActiveKey>,
    mods_pressed_count: [u32; NUM_MODS],
    state_log: StateLog,
}

#[derive(Copy, Clone, Default)]
struct StateLog {
    mods_pressed: ModifierMask,
    mods_latched: ModifierMask,
    mods_locked: ModifierMask,
    mods_effective: ModifierMask,
    group_pressed: u32,
    group_latched: u32,
    group_locked: u32,
    group_effective: u32,
}

#[derive(Copy, Clone)]
pub enum LogicalEvent {
    ModsPressed(ModifierMask),
    ModsLatched(ModifierMask),
    ModsLocked(ModifierMask),
    ModsEffective(ModifierMask),
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
            LogicalEvent::KeyDown(k) => write!(f, "key_down({})", k.0),
            LogicalEvent::KeyUp(k) => write!(f, "key_up({})", k.0),
        }
    }
}

struct LogHandler<'a> {
    mods_pressed_count: &'a mut [u32; NUM_MODS],
    pub_state: &'a mut StateLog,
    acc_state: StateLog,
    events: &'a mut Vec<LogicalEvent>,
}

impl LogHandler<'_> {
    fn flush_state(&mut self) {
        macro_rules! flush {
            ($($camel:ident, $field:ident;)*) => {
                $(
                    if self.acc_state.$field != self.pub_state.$field {
                        self.pub_state.$field = self.acc_state.$field;
                        self.events.push(LogicalEvent::$camel(self.acc_state.$field));
                    }
                )*
            };
        }
        self.acc_state.mods_effective =
            self.acc_state.mods_pressed | self.acc_state.mods_latched | self.acc_state.mods_locked;
        flush! {
            ModsPressed, mods_pressed;
            ModsLatched, mods_latched;
            ModsLocked, mods_locked;
            ModsEffective, mods_effective;
        }
    }
}

impl StateEventHandler for LogHandler<'_> {
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
            self.acc_state.mods_pressed &= !changed;
        }
    }

    #[inline]
    fn component_load(&self, component: Component) -> u32 {
        match component {
            Component::ModsPressed => self.acc_state.mods_pressed.0,
            Component::ModsLatched => self.acc_state.mods_latched.0,
            Component::ModsLocked => self.acc_state.mods_locked.0,
            Component::GroupPressed => self.acc_state.group_pressed,
            Component::GroupLatched => self.acc_state.group_latched,
            Component::GroupLocked => self.acc_state.group_locked,
        }
    }

    #[inline]
    fn component_store(&mut self, component: Component, val: u32) {
        match component {
            Component::ModsPressed => self.acc_state.mods_pressed.0 = val,
            Component::ModsLatched => self.acc_state.mods_latched.0 = val,
            Component::ModsLocked => self.acc_state.mods_locked.0 = val,
            Component::GroupPressed => {}
            Component::GroupLatched => {}
            Component::GroupLocked => {}
        }
    }

    #[inline(always)]
    fn key_down(&mut self, keycode: Keycode) {
        self.flush_state();
        self.events.push(LogicalEvent::KeyDown(keycode));
    }

    #[inline(always)]
    fn key_up(&mut self, keycode: Keycode) {
        self.flush_state();
        self.events.push(LogicalEvent::KeyUp(keycode));
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, CloneWithDelta)]
pub struct Keycode(pub u32);

struct ActiveKey {
    key: Keycode,
    down_log: u32,
    registers_log: StaticMap<Register, u32>,
    flags: StaticMap<Flag, u32>,
    on_release: Option<Arc<[Lo]>>,
    spill: Box<[u32]>,
}

impl StateMachine {
    pub fn handle_key(
        &self,
        state: &mut State,
        events: &mut Vec<LogicalEvent>,
        key: Keycode,
        down: bool,
    ) {
        for i in 0..state.active.len() {
            let active = &mut state.active[i];
            if active.key == key {
                if down {
                    active.down_log = active.down_log.saturating_add(1);
                    return;
                }
                if active.down_log == 1 {
                    if let Some(release) = &active.on_release {
                        let mut handler = LogHandler {
                            mods_pressed_count: &mut state.mods_pressed_count,
                            acc_state: state.state_log,
                            pub_state: &mut state.state_log,
                            events,
                        };
                        run(
                            &mut handler,
                            &release,
                            &mut active.registers_log,
                            &mut state.globals,
                            &mut active.flags,
                            &mut active.spill,
                        );
                        handler.flush_state();
                    } else {
                        events.push(LogicalEvent::KeyUp(key));
                    }
                    state.active.swap_remove(i);
                    return;
                }
                active.down_log -= 1;
                return;
            }
        }
        if !down {
            return;
        }
        let group = state.state_log.group_effective;
        let mods = state.state_log.mods_effective;
        let mut on_press = None;
        let mut on_release = None;
        let mut spill = Box::new([]) as Box<[u32]>;
        if let Some(key_groups) = self.keys.get(&key) {
            if let Some(Some(key_group)) = key_groups.groups.get(group as usize) {
                let mapping = key_group.ty.map(mods);
                let layer = mapping.layer;
                if let Some(key_layer) = key_group.layers.get(layer) {
                    if let Some(routine) = &key_layer.routine {
                        on_press = Some(&routine.on_press);
                        on_release = Some(routine.on_release.clone());
                        if routine.spill > 0 {
                            spill = vec![0; routine.spill].into_boxed_slice();
                        }
                    }
                }
            }
        }
        let mut active = ActiveKey {
            key,
            down_log: 1,
            registers_log: Default::default(),
            flags: Default::default(),
            on_release,
            spill,
        };
        let mut handler = LogHandler {
            mods_pressed_count: &mut state.mods_pressed_count,
            acc_state: state.state_log,
            pub_state: &mut state.state_log,
            events,
        };
        if let Some(on_press) = on_press {
            run(
                &mut handler,
                on_press,
                &mut active.registers_log,
                &mut state.globals,
                &mut active.flags,
                &mut [],
            );
        } else {
            handler.key_down(key);
            handler.component_store(Component::ModsLatched, 0);
        }
        handler.flush_state();
        state.active.push(active);
    }
}
