use crate::{
    group::{GroupDelta, GroupIndex},
    state_machine::LogicalEvent,
    ModifierMask,
};

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
#[non_exhaustive]
pub struct Components {
    pub mods_pressed: ModifierMask,
    pub mods_latched: ModifierMask,
    pub mods_locked: ModifierMask,
    pub mods: ModifierMask,
    pub group_pressed: GroupDelta,
    pub group_latched: GroupDelta,
    pub group_locked: GroupIndex,
    pub group: GroupIndex,
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
            LogicalEvent::ModsEffective(v) => change!(mods, v),
            LogicalEvent::GroupPressed(v) => change!(group_pressed, v),
            LogicalEvent::GroupLatched(v) => change!(group_latched, v),
            LogicalEvent::GroupLocked(v) => change!(group_locked, v),
            LogicalEvent::GroupEffective(v) => change!(group, v),
            _ => false,
        }
    }

    pub(crate) fn any_latched(&self) -> bool {
        self.group_latched.0 != 0 || self.mods_latched.0 != 0
    }
}
