#[cfg(test)]
mod tests;

use {
    crate::{
        group_type::GroupType,
        modifier::{ModifierMask, NUM_MODS, NUM_MODS_MASK},
        routine::{run, Component, Flag, Global, Register, Routine, StateEventHandler},
    },
    hashbrown::HashMap,
    kbvm_proc::CloneWithDelta,
    linearize::StaticMap,
    std::fmt::{Debug, Formatter},
};

pub struct StateMachine {
    pub(crate) keys: HashMap<Keycode, KeyGroups>,
    // pub(crate) keys: Vec<KeyGroups>,
}

#[derive(Default)]
pub(crate) struct KeyGroups {
    pub(crate) groups: Box<[Option<KeyGroup>]>,
}

pub(crate) struct KeyGroup {
    pub(crate) ty: GroupType,
    pub(crate) layers: Box<[KeyLayer]>,
}

#[derive(Default)]
pub(crate) struct KeyLayer {
    pub(crate) routine: Option<Routine>,
}

#[derive(Default)]
pub struct State {
    globals: StaticMap<Global, u32>,
    active: Vec<ActiveKey>,
    state_log: StateLog,
}

#[derive(Default)]
struct StateLog {
    mods_pressed_count: [u32; NUM_MODS],
    mods_pressed: ModifierMask,
    mods_latched: ModifierMask,
    mods_locked: ModifierMask,
    mods_effective: ModifierMask,
    _group_pressed: u32,
    _group_latched: u32,
    _group_locked: u32,
    group_effective: u32,
}

pub enum LogicalEvent {
    ModsPressed(ModifierMask),
    ModsLatched(ModifierMask),
    ModsLocked(ModifierMask),
    ModsEffective(ModifierMask),
}

impl Debug for LogicalEvent {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let (name, mods) = match self {
            LogicalEvent::ModsPressed(m) => ("mods_pressed", m),
            LogicalEvent::ModsLatched(m) => ("mods_latched", m),
            LogicalEvent::ModsLocked(m) => ("mods_locked", m),
            LogicalEvent::ModsEffective(m) => ("mods_effective", m),
        };
        write!(f, "{name} = {mods:?}")
    }
}

struct LogHandler<'a> {
    state: &'a mut StateLog,
    events: &'a mut Vec<LogicalEvent>,
}

impl LogHandler<'_> {
    fn update_effective_mods(&mut self) {
        let mods = self.state.mods_pressed | self.state.mods_latched | self.state.mods_locked;
        if mods != self.state.mods_effective {
            self.state.mods_effective = mods;
            self.events.push(LogicalEvent::ModsEffective(mods));
        }
    }
}

impl StateEventHandler for LogHandler<'_> {
    #[inline]
    fn mods_pressed_inc(&mut self, mods: ModifierMask) {
        let mut changed = false;
        for idx in mods {
            let count = &mut self.state.mods_pressed_count[idx.raw() as usize & NUM_MODS_MASK];
            if *count == 0 {
                *count = 1;
                changed = true;
            } else {
                *count = count.saturating_add(1);
            }
        }
        if changed {
            self.state.mods_pressed |= mods;
            self.events
                .push(LogicalEvent::ModsPressed(self.state.mods_pressed));
            self.update_effective_mods();
        }
    }

    #[inline]
    fn mods_pressed_dec(&mut self, mods: ModifierMask) {
        let mut changed = ModifierMask(0);
        for idx in mods {
            let count = &mut self.state.mods_pressed_count[idx.raw() as usize & NUM_MODS_MASK];
            if *count == 1 {
                *count = 0;
                changed |= idx.to_mask();
            } else {
                *count = count.saturating_sub(1);
            }
        }
        if changed.0 != 0 {
            self.state.mods_pressed &= !changed;
            self.events
                .push(LogicalEvent::ModsPressed(self.state.mods_pressed));
            self.update_effective_mods();
        }
    }

    #[inline]
    fn component_load(&self, component: Component) -> u32 {
        match component {
            Component::ModsPressed => self.state.mods_pressed.0,
            Component::ModsLatched => self.state.mods_latched.0,
            Component::ModsLocked => self.state.mods_locked.0,
        }
    }

    #[inline]
    fn component_store(&mut self, component: Component, val: u32) {
        macro_rules! store {
            ($($camel:ident, $snake:ident;)*) => {
                match component {
                    $(
                        Component::$camel => {
                            if self.state.$snake.0 != val {
                                self.state.$snake.0 = val;
                                self.events
                                    .push(LogicalEvent::$camel(self.state.$snake));
                                self.update_effective_mods();
                            }
                        }
                    )*
                }
            };
        }
        store! {
            ModsPressed, mods_pressed;
            ModsLatched, mods_latched;
            ModsLocked, mods_locked;
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, CloneWithDelta)]
pub struct Keycode(pub u32);

struct ActiveKey {
    key: Keycode,
    down_log: u32,
    registers_log: StaticMap<Register, u32>,
    flags: StaticMap<Flag, u32>,
    on_release: Option<Routine>,
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
                            state: &mut state.state_log,
                            events,
                        };
                        run(
                            &mut handler,
                            &release.on_release,
                            &mut active.registers_log,
                            &mut state.globals,
                            &mut active.flags,
                            &mut [],
                        );
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
        let mut key_layer_opt = None;
        if let Some(key_groups) = self.keys.get(&key) {
            // if (key.0 as usize) < self.keys.len() {
            //     let key_groups = &self.keys[key.0 as usize];
            if let Some(Some(key_group)) = key_groups.groups.get(group as usize) {
                let mapping = key_group.ty.map(mods);
                let layer = mapping.layer;
                if let Some(key_layer) = key_group.layers.get(layer) {
                    key_layer_opt = Some(key_layer);
                }
            }
        }
        let mut active = ActiveKey {
            key,
            down_log: 1,
            registers_log: Default::default(),
            flags: Default::default(),
            on_release: key_layer_opt.and_then(|l| l.routine.clone()),
        };
        if let Some(key_layer) = key_layer_opt {
            if let Some(routine) = &key_layer.routine {
                let mut handler = LogHandler {
                    state: &mut state.state_log,
                    events,
                };
                run(
                    &mut handler,
                    &routine.on_press,
                    &mut active.registers_log,
                    &mut state.globals,
                    &mut active.flags,
                    &mut [],
                );
            }
        }
        state.active.push(active);
    }
}
