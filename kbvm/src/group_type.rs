//! [`GroupType`] helpers.

#[allow(unused_imports)]
use crate::lookup::{Lookup, LookupTable};
use {
    crate::{GroupType, ModifierMask},
    hashbrown::HashMap,
    std::sync::Arc,
};

pub(crate) mod hidden {
    use {crate::group_type::Data, std::sync::Arc};
    #[allow(unused_imports)]
    use {crate::group_type::GroupTypeBuilder, crate::lookup::Lookup, crate::ModifierMask};

    /// The type of a key group.
    ///
    /// Each key group has a type that determines how the active modifiers are mapped to
    /// a key level.
    ///
    /// A type has a mask that determines which modifiers are considered by this mapping
    /// process. For example, if the mask is [`ModifierMask::SHIFT`], then the state of
    /// the control modifier is irrelevant.
    ///
    /// # Example (two-level symbolic keys)
    ///
    /// ```
    /// # use kbvm::{GroupType, ModifierMask};
    /// let _ty = GroupType::builder(ModifierMask::SHIFT).map(ModifierMask::SHIFT, 1).build();
    /// ```
    ///
    /// This type only considers the shift modifier and maps it to level 1. If shift is
    /// not pressed, level 0 is used.
    ///
    /// This is the type usually used for two-level symbolic keys. For example, the `1`
    /// key on US keyboard layouts that emits `!` when used with shift.
    ///
    /// # Example (two-level alphabetic keys)
    ///
    /// ```
    /// # use kbvm::{GroupType, ModifierMask};
    /// let _ty = GroupType::builder(ModifierMask::SHIFT | ModifierMask::LOCK)
    ///     .map(ModifierMask::SHIFT, 1)
    ///     .map(ModifierMask::LOCK, 1)
    ///     .build();
    /// ```
    ///
    /// This type considers the shift and the capslock modifiers and maps each individual
    /// modifier to level 1. If neither modifier is active, the level is 0. If both
    /// modifiers are active, the level is also 0, allowing the shift key to cancel the
    /// effects of capslock.
    ///
    /// This is the type usually used for two-level alphabetic keys. For example, the `a`
    /// key on US keyboard layouts that emits `A` when either shift or capslock are active
    /// but reverts back to `a` when shift is pressed while capslock is active.
    ///
    /// # Preserved modifiers
    ///
    /// Usually, mapping modifiers to a level is said to *consume* all modifiers in the
    /// type's modifier mask.
    ///
    /// For example, consider the `9` key on German keyboard layout:
    ///
    /// ```text
    /// ┌─────────┐
    /// │ )       │
    /// │         │
    /// │ 9     ] │
    /// └─────────┘
    /// ```
    ///
    /// Depending on the modifiers, this key produces the following keysyms:
    ///
    /// - `None`: `9`
    /// - `Shift`: `)`
    /// - `AltGr`: `]` (here, `AltGr` is usually an alias for [`ModifierMask::MOD5`])
    ///
    /// The group type for this key thus uses the modifier mask `Shift + AltGr`.
    ///
    /// Now let's say that an application has a keyboard shortcut assigned to `Ctrl + ]`.
    /// To trigger the shortcut, the user would have to press the `Ctrl`, `AltGr`, and
    /// `9` keys on their keyboard.
    ///
    /// This would cause the effective modifier mask at the time of the key press to be
    /// `Ctrl + AltGr`. During the mapping process, the `AltGr` modifier is consumed by
    /// the [`GroupType`]. Therefore, the remaining modifiers are `Ctrl`. Since the
    /// produced keysym is `]`, the keyboard shortcut is triggered.
    ///
    /// If `AltGr` were not consumed, then the keyboard shortcut would not trigger since
    /// the application does not have a shortcut assigned to `Ctrl + AltGr + ]`.
    ///
    /// There are, however, some less common situations where you do not want a modifier
    /// to be consumed.
    ///
    /// For example, on Microsoft Windows, keypad keys consider the `shift` modifier but
    /// do not consume it. Consider the `←` key on the keypad:
    ///
    /// ```text
    /// ┌─────────┐
    /// │ 4       │
    /// │         │
    /// │ ←       │
    /// └─────────┘
    /// ```
    ///
    /// This uses the following mapping:
    ///
    /// - `None`: `←`
    /// - `NumLock`: `4`
    /// - `Shift`: `←` (preserve `Shift`)
    /// - `Shift + NumLock`: `←`
    ///
    /// The group type for this key thus uses the modifier mask `Shift + NumLock` and the
    /// shift key can be used to cancel the effects of the num lock key.
    ///
    /// As you can see in the list above, if `NumLock` is not active, we want the `Shift`
    /// modifier to be preserved such that applications can use `Shift + ←` as a shortcut.
    ///
    /// You can accomplish this by using the [`GroupTypeBuilder::map_preserve`] function.
    #[derive(Clone, Debug)]
    pub struct GroupType {
        pub(crate) data: Arc<Data>,
    }
}

#[derive(Debug)]
pub(crate) struct Data {
    pub(crate) mask: ModifierMask,
    pub(crate) cases: Vec<Case>,
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct Case {
    pub(crate) mods: ModifierMask,
    pub(crate) level: usize,
    pub(crate) consumed: ModifierMask,
}

#[derive(Debug)]
pub(crate) struct KeyTypeMapping {
    pub(crate) level: usize,
    pub(crate) consumed: ModifierMask,
}

impl GroupType {
    /// Creates a [`GroupTypeBuilder`].
    ///
    /// The mask is the mask that will be considered during the mapping process. See the
    /// documentation of [`Self`] for details.
    pub fn builder(mask: ModifierMask) -> GroupTypeBuilder {
        GroupTypeBuilder {
            mask,
            cases: Default::default(),
        }
    }

    pub(crate) fn map(&self, mut mods: ModifierMask) -> KeyTypeMapping {
        mods &= self.data.mask;
        for case in &self.data.cases {
            if mods == case.mods {
                return KeyTypeMapping {
                    level: case.level,
                    consumed: mods & case.consumed,
                };
            }
        }
        KeyTypeMapping {
            level: 0,
            consumed: mods,
        }
    }
}

/// A builder for a [`GroupType`].
///
/// This type is created via [`GroupType::builder`].
///
/// See the documentation of [`GroupType`] for more information.
#[derive(Clone, Debug)]
pub struct GroupTypeBuilder {
    mask: ModifierMask,
    cases: HashMap<ModifierMask, Case>,
}

impl GroupTypeBuilder {
    /// Builds the [`GroupType`].
    pub fn build(&self) -> GroupType {
        GroupType {
            data: Arc::new(Data {
                mask: self.mask,
                cases: self.cases.values().copied().collect(),
            }),
        }
    }

    /// Adds a mapping from modifiers to a key level.
    ///
    /// If these modifiers already have a mapping, it is replaced.
    ///
    /// This call consumes all modifiers in the types modifier mask during the lookup
    /// process. It is equivalent to `self.map_preserve(mods, ModifierMask::NONE, level)`.
    pub fn map(&mut self, mods: ModifierMask, level: usize) -> &mut Self {
        self.map_preserve(mods, ModifierMask(0), level)
    }

    /// Add a mapping from modifiers to a key level while preserving some modifiers.
    ///
    /// If these modifiers already have a mapping, it is replaced.
    ///
    /// During the mapping process, the modifiers in `preserved` are not consumed. That
    /// is, if the modifiers in a call to [`LookupTable::lookup`] contained any of the
    /// modifiers from `preserved`, then [`Lookup::remaining_mods`] will also contain
    /// those modifiers.
    pub fn map_preserve(
        &mut self,
        mods: ModifierMask,
        preserve: ModifierMask,
        level: usize,
    ) -> &mut Self {
        self.cases.insert(
            mods,
            Case {
                mods,
                level,
                consumed: !preserve,
            },
        );
        self
    }
}
