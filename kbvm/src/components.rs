#[cfg(test)]
mod tests;

#[allow(unused_imports)]
use crate::state_machine::StateMachine;
use crate::{
    controls::ControlsMask,
    group::{GroupDelta, GroupIndex},
    state_machine::Event,
    ModifierMask,
};

/// The active modifiers/group of a keyboard.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
#[non_exhaustive]
pub struct Components {
    /// The pressed modifiers.
    pub mods_pressed: ModifierMask,
    /// The latched modifiers.
    pub mods_latched: ModifierMask,
    /// The locked modifiers.
    pub mods_locked: ModifierMask,
    /// The effective modifiers.
    ///
    /// This is defined as the bitwise OR of the pressed, latched, and locked modifiers.
    pub mods: ModifierMask,
    /// The pressed group.
    pub group_pressed: GroupDelta,
    /// The latched group.
    pub group_latched: GroupDelta,
    /// The locked group.
    ///
    /// Under wayland, this field corresponds to the `group` argument in
    /// `wl_keyboard.modifiers` events.
    pub group_locked: GroupIndex,
    /// The effective group.
    ///
    /// This is defined as the sum of the pressed, latched, and locked group.
    pub group: GroupIndex,
    /// The controls.
    pub controls: ControlsMask,
}

impl Components {
    /// Applies an [`Event`] from a [`StateMachine`] to these components.
    ///
    /// This function is intended for compositors. Clients receiving components via
    /// `wl_keyboard` events should instead use [`Self::update_effective`]. See the
    /// example therein.
    pub fn apply_event(&mut self, event: Event) -> bool {
        macro_rules! change {
            ($field:ident, $val:expr) => {{
                let changed = self.$field != $val;
                self.$field = $val;
                changed
            }};
        }
        match event {
            Event::ModsPressed(v) => change!(mods_pressed, v),
            Event::ModsLatched(v) => change!(mods_latched, v),
            Event::ModsLocked(v) => change!(mods_locked, v),
            Event::ModsEffective(v) => change!(mods, v),
            Event::GroupPressed(v) => change!(group_pressed, v),
            Event::GroupLatched(v) => change!(group_latched, v),
            Event::GroupLocked(v) => change!(group_locked, v),
            Event::GroupEffective(v) => change!(group, v),
            Event::Controls(v) => change!(controls, v),
            _ => false,
        }
    }

    /// Updates the effective modifiers and group in terms of the pressed/latched/locked
    /// modifiers/group.
    ///
    /// This function is intended for clients receiving the pressed/latched/locked
    /// modifiers and the locked group via `wl_keyboard` events.
    ///
    /// If you're using the [`Self::apply_event`] function to process events generated by
    /// a [`StateMachine`], then you don't have to use this function because the event
    /// stream will contain events updating the effective modifiers/group directly.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::{Components, GroupIndex, ModifierMask};
    /// #
    /// fn handle_wl_keyboard_modifiers(
    ///     components: &mut Components,
    ///     mods_pressed: ModifierMask,
    ///     mods_latched: ModifierMask,
    ///     mods_locked: ModifierMask,
    ///     group: GroupIndex,
    /// ) {
    ///     components.mods_pressed = mods_pressed;
    ///     components.mods_latched = mods_latched;
    ///     components.mods_locked = mods_locked;
    ///     components.group_locked = group;
    ///     components.update_effective();
    /// }
    /// ```
    pub fn update_effective(&mut self) {
        self.group = self.group_locked + self.group_pressed + self.group_latched;
        self.mods = self.mods_pressed | self.mods_latched | self.mods_locked;
    }

    pub(crate) fn any_latched(&self) -> bool {
        self.group_latched.0 != 0 || self.mods_latched.0 != 0
    }
}
