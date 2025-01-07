use crate::{modifier::ModifierMask, state_machine::LogicalEvent};

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
#[non_exhaustive]
pub struct Components {
    pub mods_pressed: ModifierMask,
    pub mods_latched: ModifierMask,
    pub mods_locked: ModifierMask,
    pub mods_effective: ModifierMask,
    pub group_pressed: u32,
    pub group_latched: u32,
    pub group_locked: u32,
    pub group_effective: u32,
}

impl Components {
    pub fn apply_event(&mut self, event: LogicalEvent) -> bool {
        macro_rules! change {
            ($field:ident, $val:expr) => {{
                let changed = self.$field != $val;
                self.$field = $val;
                changed
            }};
        }
        match event {
            LogicalEvent::ModsPressed(v) => change!(mods_pressed, v),
            LogicalEvent::ModsLatched(v) => change!(mods_latched, v),
            LogicalEvent::ModsLocked(v) => change!(mods_locked, v),
            LogicalEvent::ModsEffective(v) => change!(mods_effective, v),
            LogicalEvent::GroupPressed(v) => change!(group_pressed, v),
            LogicalEvent::GroupLatched(v) => change!(group_latched, v),
            LogicalEvent::GroupLocked(v) => change!(group_locked, v),
            LogicalEvent::GroupEffective(v) => change!(group_effective, v),
            _ => false,
        }
    }
}
