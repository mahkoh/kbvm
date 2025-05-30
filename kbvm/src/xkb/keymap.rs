//! XKB keymaps.
//!
//! This module contains types representing the components of an XKB keymap.
//!
//! The entry point to this module is the [`Keymap`].

mod format;
mod from_lookup;
mod from_resolved;
pub mod iterators;
#[cfg(test)]
mod tests;
mod to_builder;

pub use crate::xkb::keymap::format::Formatter;
use {
    crate::{
        builder::Redirect,
        group::{GroupDelta, GroupIndex},
        xkb::{
            controls::ControlMask,
            group::{GroupIdx, GroupMask},
            group_component::GroupComponent,
            indicator::IndicatorIdx,
            keymap::{
                actions::{
                    ControlsLockAction, ControlsSetAction, GroupLatchAction, GroupLockAction,
                    GroupSetAction, ModsLatchAction, ModsLockAction, ModsSetAction,
                    RedirectKeyAction,
                },
                iterators::{Groups, Indicators, Keys, Levels, Mappings, VirtualModifiers},
            },
            level::Level,
            mod_component::ModComponentMask,
            radio_group::RadioGroup,
            resolved::GroupsRedirect,
        },
        Components, ControlsMask, Keysym, ModifierIndex, ModifierMask,
    },
    hashbrown::DefaultHashBuilder,
    indexmap::IndexMap,
    smallvec::SmallVec,
    std::sync::Arc,
};
#[expect(unused_imports)]
use {
    crate::{lookup::LookupTable, xkb::Context},
    std::fmt::Display,
};

/// A fully-resolved XKB keymap.
///
/// This object is usually created from a [`Context`] but can also be created via
/// [`LookupTable::to_xkb_keymap`].
///
/// # Creating a keymap from XKB source
///
/// ```xkb
/// xkb_keymap {
///     xkb_keycodes {
///         <a> = 38;
///         <leftshift> = 50;
///     };
///     xkb_symbols {
///         key <a> {
///             [ a, A ],
///         };
///         key <leftshift> {
///             [ Shift_L ],
///             [ SetMods(mods = Shift) ],
///         };
///     };
/// };
/// ```
///
/// ```no_run
/// # use kbvm::xkb::Context;
/// # use kbvm::xkb::diagnostic::WriteToLog;
/// # const MAP: &str = "...";
/// let context = Context::default();
/// let keymap = context.keymap_from_bytes(WriteToLog, None, MAP.as_bytes()).unwrap();
/// ```
///
/// # Creating a keymap from RMLVO names
///
/// ```
/// # use kbvm::xkb::Context;
/// # use kbvm::xkb::diagnostic::WriteToLog;
/// # use kbvm::xkb::rmlvo::Group;
/// let context = Context::default();
/// let keymap = context.keymap_from_names(
///     WriteToLog,
///     None,
///     None,
///     Some(&[Group {
///         layout: "de",
///         variant: "neo",
///     }]),
///     None,
/// );
/// ```
#[derive(Debug, PartialEq)]
pub struct Keymap {
    pub(crate) name: Option<Arc<String>>,
    pub(crate) max_keycode: u32,
    pub(crate) indicators: Vec<Indicator>,
    pub(crate) keycodes: Vec<Keycode>,
    pub(crate) types: Vec<Arc<KeyType>>,
    pub(crate) virtual_modifiers: Vec<VirtualModifier>,
    pub(crate) mod_maps: Vec<(ModifierIndex, ModMapValue)>,
    pub(crate) group_names: Vec<(GroupIdx, Arc<String>)>,
    pub(crate) keys: IndexMap<crate::Keycode, Key, DefaultHashBuilder>,
}

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) struct ModMapValue {
    pub(crate) key_name: Arc<String>,
    pub(crate) key_sym: Option<Keysym>,
}

/// An indicator.
///
/// # Example
///
/// ```xkb
/// xkb_compat {
///     indicator "Caps Lock" {
///         modifiers = Lock;
///         whichModState = Effective;
///     };
/// };
/// ```
#[derive(Debug, PartialEq)]
pub struct Indicator {
    pub(crate) virt: bool,
    pub(crate) index: IndicatorIdx,
    pub(crate) name: Arc<String>,
    pub(crate) modifier_mask: ModifierMask,
    pub(crate) group_mask: GroupMask,
    pub(crate) controls: ControlMask,
    pub(crate) mod_components: ModComponentMask,
    pub(crate) group_component: GroupComponent,
}

/// An indicator matcher that determines if an indicator should be active.
pub struct IndicatorMatcher {
    mods_pressed: u32,
    mods_latched: u32,
    mods_locked: u32,
    mods: u32,
    group_pressed: bool,
    group_not_pressed: bool,
    group_latched: bool,
    group_not_latched: bool,
    group_locked: u32,
    group: u32,
    controls: u32,
}

#[derive(Debug, PartialEq)]
pub(crate) struct Keycode {
    pub(crate) name: Arc<String>,
    pub(crate) keycode: crate::Keycode,
}

/// A virtual modifier.
///
/// # Example
///
/// ```xkb
/// xkb_compat {
///     virtual_modifiers V1 = 0x100;
/// };
/// ```
#[derive(Debug, PartialEq)]
pub struct VirtualModifier {
    pub(crate) name: Arc<String>,
    pub(crate) values: ModifierMask,
}

/// A key type.
///
/// # Example
///
/// ```xkb
/// xkb_types {
///     type "X" {
///         modifiers = Shift+Mod1;
///         map[Shift] = Level2;
///         map[Mod1] = Level2;
///     };
/// };
/// ```
#[derive(Debug, PartialEq)]
pub struct KeyType {
    pub(crate) name: Arc<String>,
    pub(crate) modifiers: ModifierMask,
    pub(crate) mappings: Vec<KeyTypeMapping>,
    pub(crate) level_names: Vec<(Level, Arc<String>)>,
}

/// A key-type mapping.
///
/// # Example
///
/// ```xkb
/// xkb_types {
///     type "X" {
///         modifiers = Shift+Mod1;
///         map[Shift] = Level2;
///         map[Mod1] = Level2;
///     };
/// };
/// ```
///
/// This object might refer to either `map[Shift]` or `map[Mod1]`.
#[derive(Debug, PartialEq)]
pub struct KeyTypeMapping {
    pub(crate) modifiers: ModifierMask,
    pub(crate) preserved: ModifierMask,
    pub(crate) level: Level,
}

/// An XKB action.
///
/// # Example
///
/// ```xkb
/// xkb_symbols {
///     key <leftshift> {
///         [ SetMods(mods = Shift) ],
///     };
/// };
/// ```
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum Action {
    /// A `SetMods` action.
    ModsSet(ModsSetAction),
    /// A `LatchMods` action.
    ModsLatch(ModsLatchAction),
    /// A `LockMods` action.
    ModsLock(ModsLockAction),
    /// A `SetGroup` action.
    GroupSet(GroupSetAction),
    /// A `LatchGroup` action.
    GroupLatch(GroupLatchAction),
    /// A `LockGroup` action.
    GroupLock(GroupLockAction),
    /// A `RedirectKey` action.
    RedirectKey(RedirectKeyAction),
    /// A `SetControls` action.
    ControlsSet(ControlsSetAction),
    /// A `LockControls` action.
    ControlsLock(ControlsLockAction),
}

/// The XKB actions supported by KBVM.
pub mod actions {
    use {
        crate::{
            xkb::{controls::ControlMask, group},
            ModifierMask,
        },
        std::sync::Arc,
    };

    /// A `SetMods` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <leftshift> {
    ///         [ SetMods(mods = Shift) ],
    ///     };
    /// };
    /// ```
    #[derive(Clone, Debug, PartialEq)]
    pub struct ModsSetAction {
        pub(crate) clear_locks: bool,
        pub(crate) modifiers: ModifierMask,
    }

    /// A `LatchMods` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <leftshift> {
    ///         [ LatchMods(mods = Shift) ],
    ///     };
    /// };
    /// ```
    #[derive(Clone, Debug, PartialEq)]
    pub struct ModsLatchAction {
        pub(crate) clear_locks: bool,
        pub(crate) latch_to_lock: bool,
        pub(crate) modifiers: ModifierMask,
    }

    /// A `LockMods` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <leftshift> {
    ///         [ LockMods(mods = Shift) ],
    ///     };
    /// };
    /// ```
    #[derive(Clone, Debug, PartialEq)]
    pub struct ModsLockAction {
        pub(crate) modifiers: ModifierMask,
        pub(crate) lock: bool,
        pub(crate) unlock: bool,
    }

    /// A `SetGroup` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <compose> {
    ///         [ SetGroup(group = +1) ],
    ///     };
    /// };
    /// ```
    #[derive(Clone, Debug, PartialEq)]
    pub struct GroupSetAction {
        pub(crate) group: group::GroupChange,
        pub(crate) clear_locks: bool,
    }

    /// A `LatchGroup` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <compose> {
    ///         [ LatchGroup(group = +1) ],
    ///     };
    /// };
    /// ```
    #[derive(Clone, Debug, PartialEq)]
    pub struct GroupLatchAction {
        pub(crate) group: group::GroupChange,
        pub(crate) clear_locks: bool,
        pub(crate) latch_to_lock: bool,
    }

    /// A `LockGroup` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <compose> {
    ///         [ LockGroup(group = +1) ],
    ///     };
    /// };
    /// ```
    #[derive(Clone, Debug, PartialEq)]
    pub struct GroupLockAction {
        pub(crate) group: group::GroupChange,
    }

    /// A `RedirectKey` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ RedirectKey(key = b) ],
    ///     };
    /// };
    /// ```
    ///
    /// # Implementation
    ///
    /// KBVM implements this action as follows:
    ///
    /// The new key code is emitted in `KeyDown` and `KeyUp` events instead of the
    /// original key code.
    ///
    /// If the [`Self::modifier_mask`] of the action is not empty, then it additionally
    /// modifies the components before and after the `KeyDown` and `KeyUp` events as
    /// follows:
    ///
    /// - Before the event:
    ///
    ///   1. Save the pressed, latched, and locked modifiers.
    ///   2. Set the latched and locked modifiers to 0.
    ///   3. Set the pressed modifiers to the effective modifiers.
    ///   4. Clear the modifiers from [`Self::mods_to_clear`] from the pressed modifiers.
    ///   5. Set the modifiers from [`Self::mods_to_set`] in the pressed modifiers.
    ///
    /// - After the event:
    ///
    ///   1. Restore the pressed, latched, and locked modifiers.
    #[derive(Clone, Debug, PartialEq)]
    pub struct RedirectKeyAction {
        pub(crate) key_name: Arc<String>,
        pub(crate) keycode: crate::Keycode,
        pub(crate) mods_to_set: ModifierMask,
        pub(crate) mods_to_clear: ModifierMask,
    }

    /// A `SetControls` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <leftshift> {
    ///         [ SetControls(controls = Overlay1) ],
    ///     };
    /// };
    /// ```
    #[derive(Clone, Debug, PartialEq)]
    pub struct ControlsSetAction {
        pub(crate) controls: ControlMask,
    }

    /// A `LockControls` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <leftshift> {
    ///         [ LockControls(controls = Overlay1) ],
    ///     };
    /// };
    /// ```
    #[derive(Clone, Debug, PartialEq)]
    pub struct ControlsLockAction {
        pub(crate) controls: ControlMask,
        pub(crate) lock: bool,
        pub(crate) unlock: bool,
    }
}

/// A group change performed by an [`Action`].
pub enum GroupChange {
    /// An absolute change of a group.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <compose> {
    ///         [ SetGroup(group = Group2) ],
    ///     };
    /// };
    /// ```
    Absolute(GroupIndex),
    /// A relative change of a group.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <compose> {
    ///         [ SetGroup(group = +1) ],
    ///     };
    /// };
    /// ```
    Relative(GroupDelta),
}

/// A key.
///
/// # Example
///
/// ```xkb
/// xkb_symbols {
///     key <a> { [ a, A ] };
/// };
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct Key {
    pub(crate) key_name: Arc<String>,
    pub(crate) keycode: crate::Keycode,
    pub(crate) groups: Vec<Option<KeyGroup>>,
    pub(crate) repeat: bool,
    pub(crate) behavior: Option<KeyBehavior>,
    pub(crate) redirect: GroupsRedirect,
}

/// A key behavior.
///
/// # Example
///
/// ```xkb
/// xkb_symbols {
///     key <a> {
///         locks = true,
///         [ a, A ],
///     };
/// };
/// ```
///
/// The key behavior is `KeyBehavior::Lock`.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum KeyBehavior {
    /// The key locks.
    Lock,
    /// The key is affected by an overlay control.
    Overlay(OverlayBehavior),
    /// The key is affected by a radio-group control.
    RadioGroup(RadioGroupBehavior),
}

/// The overlay that affects a key behavior.
///
/// # Example
///
/// ```xkb
/// xkb_symbols {
///     key <a> {
///         overlay2 = <b>,
///     };
/// };
/// ```
///
/// The overlay is `Overlay2`.
#[derive(Copy, Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum KeyOverlay {
    /// The first overlay.
    Overlay1,
    /// The second overlay.
    Overlay2,
}

/// An overlay behavior.
///
/// # Example
///
/// ```xkb
/// xkb_symbols {
///     key <a> {
///         overlay1 = <b>,
///         [ a, A ],
///     };
/// };
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct OverlayBehavior {
    pub(crate) overlay: KeyOverlay,
    pub(crate) key_name: Arc<String>,
    pub(crate) keycode: crate::Keycode,
}

/// A radio-group behavior.
///
/// # Example
///
/// ```xkb
/// xkb_symbols {
///     key <a> {
///         radiogroup = 1,
///         [ a, A ],
///     };
/// };
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct RadioGroupBehavior {
    pub(crate) allow_none: bool,
    pub(crate) radio_group: RadioGroup,
}

/// A key group.
///
/// # Example
///
/// ```xkb
/// xkb_symbols {
///     key <a> {
///         [ a, A ],
///         [ b, B ],
///     };
/// };
/// ```
///
/// This object might refer to either `[ a, A ]` or `[ b, B ]`.
#[derive(Clone, Debug, PartialEq)]
pub struct KeyGroup {
    pub(crate) key_type: Arc<KeyType>,
    pub(crate) levels: Vec<KeyLevel>,
}

/// A key level.
///
/// # Example
///
/// ```xkb
/// xkb_symbols {
///     key <a> {
///         symbols[Group1] = [ a,          A                    ],
///         actions[Group1] = [ NoAction(), SetMods(mods = Mod1) ],
///         symbols[Group2] = [ b,          B                    ],
///     };
/// };
/// ```
///
/// This object might refer to any of
///
/// - `a`,
/// - `A` / `SetMods(mods = Mod1)`,
/// - `b`, or
/// - `B`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct KeyLevel {
    pub(crate) symbols: SmallVec<[Keysym; 1]>,
    pub(crate) actions: SmallVec<[Action; 1]>,
}

impl Keymap {
    /// Returns an iterator over the virtual modifiers of this map.
    ///
    /// Note that the real modifiers are not included in this iterator. The real modifiers
    /// use the following hard-coded assignments:
    ///
    /// | name         | index | mask   |
    /// | ------------ | ----- | ------ |
    /// | Shift        | `0`   | `0x01` |
    /// | Lock         | `1`   | `0x02` |
    /// | Control      | `2`   | `0x04` |
    /// | Mod1/Alt     | `3`   | `0x08` |
    /// | Mod2/NumLock | `4`   | `0x10` |
    /// | Mod3         | `5`   | `0x20` |
    /// | Mod4/Logo    | `6`   | `0x40` |
    /// | Mod5         | `7`   | `0x80` |
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     virtual_modifiers A;
    ///     virtual_modifiers B;
    ///     virtual_modifiers C;
    /// };
    /// ```
    ///
    /// The iterator returns 3 elements, one for A, one for B, and one for C.
    pub fn virtual_modifiers(&self) -> VirtualModifiers<'_> {
        VirtualModifiers {
            modifiers: self.virtual_modifiers.iter(),
        }
    }

    /// Returns an iterator over the keys of this map.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> { [ a, A ] };
    ///     key <leftshift> { [ SetMods(mods = Shift) ] };
    /// };
    /// ```
    ///
    /// The iterator returns 2 elements, one for `<a>` and one for `<leftshift>`.
    pub fn keys(&self) -> Keys<'_> {
        Keys {
            keys: self.keys.values(),
        }
    }

    /// Returns an iterator over the indicators of this map.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "Caps Lock" {
    ///         modifiers = Lock;
    ///     };
    ///     indicator "Num Lock" {
    ///         modifiers = Mod2;
    ///     };
    /// };
    /// ```
    ///
    /// The iterator returns 2 elements, one for `Caps Lock` and one for `Num Lock`.
    pub fn indicators(&self) -> Indicators<'_> {
        Indicators {
            indicators: self.indicators.iter(),
        }
    }

    /// Returns a type that can be used to format the map in XKB text format.
    ///
    /// # Warning
    ///
    /// When using this function to create a keymap for Xwayland, compositors should
    /// enable both [`Formatter::lookup_only`] and [`Formatter::rename_long_keys`].
    ///
    /// `lookup_only` prevents key actions and key behaviors from being included in the
    /// map. This works around a bug in Xwayland where Xwayland will execute the actions
    /// and behaviors instead of only passing the key events to X clients.
    ///
    /// `rename_long_keys` is only necessary if the original keymap contains key names
    /// that are longer than 4 bytes. Xwayland cannot handle such key names.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::xkb::Keymap;
    /// fn pretty_print_keymap(keymap: &Keymap) -> String {
    ///     format!("{}\n", keymap.format())
    /// }
    /// ```
    pub fn format(&self) -> Formatter<'_> {
        Formatter {
            keymap: self,
            single_line: false,
            lookup_only: false,
            multiple_actions_per_level: false,
            rename_long_keys: false,
        }
    }
}

impl Indicator {
    pub(crate) fn dummy() -> Self {
        Self {
            virt: false,
            index: IndicatorIdx::ONE,
            name: Arc::new("DUMMY".to_string()),
            modifier_mask: Default::default(),
            group_mask: Default::default(),
            controls: Default::default(),
            mod_components: Default::default(),
            group_component: Default::default(),
        }
    }
}

impl VirtualModifier {
    pub(crate) fn dummy() -> Self {
        Self {
            name: Arc::new("Dummy".to_string()),
            values: Default::default(),
        }
    }

    /// Returns the name of the modifier.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     virtual_modifiers V1 = 0x100;
    /// };
    /// ```
    ///
    /// The function returns `"V1"`.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the modifier mask that the modifier maps to.
    ///
    /// # Example 1
    ///
    /// ```xkb
    /// xkb_compat {
    ///     virtual_modifiers V1 = 0x100;
    /// };
    /// ```
    ///
    /// The function returns `0x100`.
    ///
    /// # Example 2
    ///
    /// ```xkb
    /// xkb_compat {
    ///     virtual_modifiers Alt;
    ///
    ///     interpret Alt_L {
    ///         virtualmodifier = Alt;
    ///     };
    /// };
    ///
    /// xkb_symbols {
    ///     key <leftalt> { [ Alt_L ] };
    ///     modmap Mod1 { <leftalt> };
    /// };
    /// ```
    ///
    /// The function returns [`ModifierMask::MOD1`].
    pub fn mask(&self) -> ModifierMask {
        self.values
    }
}

impl Key {
    /// Returns the name of the key.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <abcd> { [ a, A ] };
    /// };
    /// ```
    ///
    /// The function returns `abcd`.
    pub fn name(&self) -> &str {
        &self.key_name
    }

    /// Returns the keycode of the key.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keycodes {
    ///     <a> = 38;
    /// };
    /// xkb_symbols {
    ///     key <a> { [ a, A ] };
    /// };
    /// ```
    ///
    /// The function returns `38`.
    pub fn keycode(&self) -> crate::Keycode {
        self.keycode
    }

    /// Returns whether the key repeats.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         repeats = false,
    ///         [ a, A ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `false`.
    pub fn repeats(&self) -> bool {
        self.repeat
    }

    /// Returns the group-redirect setting of this key.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         groupsClamp,
    ///         [ a, A ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns [`Redirect::Clamp`].
    pub fn redirect(&self) -> Redirect {
        self.redirect.to_redirect()
    }

    /// Returns the behavior of the key.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         locks = true,
    ///         [ a, A ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `KeyBehavior::Lock`.
    pub fn behavior(&self) -> Option<&KeyBehavior> {
        self.behavior.as_ref()
    }

    /// Returns an iterator over the groups of this key.
    ///
    /// # Example 1
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ a, A ],
    ///         [ b, B ],
    ///     };
    /// };
    /// ```
    ///
    /// The iterator returns two elements, one for `[ a, A ]` and one for `[ b, B ]`.
    ///
    /// # Example 2
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         symbols[Group1] = [ a, A ],
    ///         symbols[Group3] = [ b, B ],
    ///     };
    /// };
    /// ```
    ///
    /// The iterator returns three elements:
    ///
    /// - `Some([ a, A ])`
    /// - `None`
    /// - `Some([ b, B ])`
    pub fn groups(&self) -> Groups<'_> {
        Groups {
            groups: self.groups.iter(),
        }
    }
}

impl KeyGroup {
    /// Returns the type of this group.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     type "X" {
    ///         modifiers = Shift;
    ///         map[Shift] = Level2;
    ///     };
    /// };
    /// xkb_symbols {
    ///     key <a> {
    ///         type[Group1] = "X";
    ///         symbols[Group1] = [ a, A ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns the `X` type.
    pub fn ty(&self) -> &KeyType {
        &self.key_type
    }

    /// Returns an iterator over the levels of this group.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ a, A ],
    ///     };
    /// };
    /// ```
    ///
    /// The iterator returns two elements, one for `a` and one for `A`.
    pub fn levels(&self) -> Levels<'_> {
        Levels {
            levels: self.levels.iter(),
        }
    }
}

impl KeyType {
    /// Returns the modifier mask of this key type.
    ///
    /// Modifiers outside of this mask are completely ignored by this key type. That is,
    /// they are masked out before considering which level to map to and are never
    /// consumed.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     type "X" {
    ///         modifiers = Shift+Mod1;
    ///         map[Shift] = Level2;
    ///         map[Mod1] = Level2;
    ///     };
    /// };
    /// ```
    ///
    /// The function returns [`ModifierMask::SHIFT | ModifierMask::MOD1`](ModifierMask).
    pub fn mask(&self) -> ModifierMask {
        self.modifiers
    }

    /// Returns an iterator over the mappings of this key type.
    ///
    /// Mappings that are not explicitly defined are not returned. Such mappings map to
    /// level 1 and consume all input modifiers.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     type "X" {
    ///         modifiers = Shift+Mod1;
    ///         map[Shift] = Level2;
    ///         map[Mod1] = Level2;
    ///     };
    /// };
    /// ```
    ///
    /// The iterator returns one mapping for `map[Shift]` and one mapping for
    /// `map[Mod1]`.
    ///
    /// Note that no mappings are returned for `map[None]` and `map[Shift+Mod1]`.
    pub fn mappings(&self) -> Mappings<'_> {
        Mappings {
            mappings: self.mappings.iter(),
        }
    }
}

impl KeyTypeMapping {
    /// Returns the modifier mask of this mapping.
    ///
    /// After masking with the [type mask](KeyType::mask), the effective modifiers must
    /// match this mask exactly for the mapping to be applicable.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     type "X" {
    ///         modifiers = Shift+Mod1;
    ///         map[Shift] = Level2;
    ///         map[Mod1] = Level2;
    ///     };
    /// };
    /// ```
    ///
    /// If this mapping is `map[Shift]`, then this function returns
    /// [`ModifierMask::SHIFT`].
    pub fn mask(&self) -> ModifierMask {
        self.modifiers
    }

    /// Returns the preserved modifiers of this mapping.
    ///
    /// If this mapping is applicable, then the preserved modifiers are not consumed by
    /// the mapping and might be used for keyboard shortcuts or keysym transformations.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     type "X" {
    ///         modifiers = Control+Shift;
    ///         map[Control+Shift] = Level2;
    ///         preserve[Control+Shift] = Control;
    ///     };
    /// };
    /// ```
    ///
    /// If this mapping is `map[Control+Shift]`, then function returns
    /// [`ModifierMask::CONTROL`].
    pub fn preserved(&self) -> ModifierMask {
        self.modifiers & self.preserved
    }

    /// Returns the consumed modifiers of this mapping.
    ///
    /// If this mapping is applicable, then the consumed modifiers should no longer be
    /// considered for keyboard shortcuts and keysym transformations.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     type "X" {
    ///         modifiers = Control+Shift;
    ///         map[Control+Shift] = Level2;
    ///         preserve[Control+Shift] = Control;
    ///     };
    /// };
    /// ```
    ///
    /// If this mapping is `map[Control+Shift]`, then function returns
    /// [`ModifierMask::SHIFT`].
    pub fn consumed(&self) -> ModifierMask {
        self.modifiers & !self.preserved
    }

    /// Returns the 0-based level that this mapping maps to.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     type "X" {
    ///         modifiers = Shift;
    ///         map[Shift] = Level2;
    ///     };
    /// };
    /// ```
    ///
    /// If this mapping is `map[Shift]`, then function returns `1`.
    pub fn level(&self) -> usize {
        self.level.to_offset()
    }
}

impl KeyLevel {
    /// Returns the symbols of this level.
    ///
    /// Note that returning more than 1 keysym is an extension that does not work for X
    /// applications.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [
    ///             a,
    ///             { A, B },
    ///         ],
    ///     };
    /// };
    /// ```
    ///
    /// If this object refers to the first level, then this function returns `&[syms::a]`.
    ///
    /// If this object refers to the second level, then this function returns
    /// `&[syms::A, syms::B]`.
    pub fn symbols(&self) -> &[Keysym] {
        &self.symbols
    }

    /// Returns the actions of this level.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [
    ///             SetMods(mods = Mod1),
    ///             {
    ///                 SetMods(mods = Mod2),
    ///                 LockGroup(group = 2),
    ///             },
    ///         ],
    ///     };
    /// };
    /// ```
    ///
    /// If this object refers to the first level, then this function returns
    /// `&[SetMods(mods = Mod1)]`.
    ///
    /// If this object refers to the second level, then this function returns
    /// `&[SetMods(mods = Mod2), LockGroup(group = 2)]`.
    pub fn actions(&self) -> &[Action] {
        &self.actions
    }
}

impl ModsSetAction {
    /// Returns whether this action has the `clearLocks` flag set.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <leftshift> {
    ///         [ SetMods(mods = Shift, clearLocks) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `true`.
    pub fn clear_locks(&self) -> bool {
        self.clear_locks
    }

    /// Returns the modifier mask of this action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <leftshift> {
    ///         [ SetMods(mods = Shift) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `Shift`.
    pub fn mask(&self) -> ModifierMask {
        self.modifiers
    }
}

impl ModsLatchAction {
    /// Returns whether this action has the `clearLocks` flag set.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <leftshift> {
    ///         [ LatchMods(mods = Shift, clearLocks) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `true`.
    pub fn clear_locks(&self) -> bool {
        self.clear_locks
    }

    /// Returns whether this action has the `latchToLock` flag set.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <leftshift> {
    ///         [ LatchMods(mods = Shift, latchToLock) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `true`.
    pub fn latch_to_lock(&self) -> bool {
        self.latch_to_lock
    }

    /// Returns the modifier mask of this action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <leftshift> {
    ///         [ LatchMods(mods = Shift) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `Shift`.
    pub fn mask(&self) -> ModifierMask {
        self.modifiers
    }
}

impl ModsLockAction {
    /// Returns whether this action will lock the modifiers.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <leftshift> {
    ///         [ LockMods(mods = Shift, affect = unlock) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `false`.
    pub fn lock(&self) -> bool {
        self.lock
    }

    /// Returns whether this action will unlock previously-locked modifiers.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <leftshift> {
    ///         [ LockMods(mods = Shift, affect = lock) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `false`.
    pub fn unlock(&self) -> bool {
        self.unlock
    }

    /// Returns the modifier mask of this action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <leftshift> {
    ///         [ LockMods(mods = Shift) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `Shift`.
    pub fn mask(&self) -> ModifierMask {
        self.modifiers
    }
}

impl GroupSetAction {
    /// Returns whether this action has the `clearLocks` flag set.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <compose> {
    ///         [ SetGroup(group = +1, clearLocks) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `true`.
    pub fn clear_locks(&self) -> bool {
        self.clear_locks
    }

    /// Returns the group change of this action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <compose> {
    ///         [ SetGroup(group = -1) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `GroupChange::Relative(-1)`.
    pub fn group(&self) -> GroupChange {
        self.group.to_group_change()
    }
}

impl GroupLatchAction {
    /// Returns whether this action has the `clearLocks` flag set.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <compose> {
    ///         [ LatchGroup(group = +1, clearLocks) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `true`.
    pub fn clear_locks(&self) -> bool {
        self.clear_locks
    }

    /// Returns whether this action has the `latchToLock` flag set.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <compose> {
    ///         [ LatchGroup(group = +1, latchToLock) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `true`.
    pub fn latch_to_lock(&self) -> bool {
        self.latch_to_lock
    }

    /// Returns the group change of this action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <compose> {
    ///         [ LatchGroup(group = -1) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `GroupChange::Relative(-1)`.
    pub fn group(&self) -> GroupChange {
        self.group.to_group_change()
    }
}

impl GroupLockAction {
    /// Returns the group change of this action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <compose> {
    ///         [ LockGroup(group = -1) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `GroupChange::Relative(-1)`.
    pub fn group(&self) -> GroupChange {
        self.group.to_group_change()
    }
}

impl RedirectKeyAction {
    /// Returns the name of the key that this action redirects to.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ RedirectKey(key = <b>) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `"b"`.
    pub fn key_name(&self) -> &str {
        &self.key_name
    }

    /// Returns the keycode of the key that this action redirects to.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ RedirectKey(key = <b>) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns the keycode of `<b>`.
    pub fn keycode(&self) -> crate::Keycode {
        self.keycode
    }

    /// Returns the mods that will be set by this action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ RedirectKey(key = <b>, mods = Shift+Mod1) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `ModifierMask::SHIFT | ModifierMask::MOD1`.
    pub fn mods_to_set(&self) -> ModifierMask {
        self.mods_to_set
    }

    /// Returns the mods that will be cleared by this action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ RedirectKey(key = <b>, clearMods = Shift+Mod1) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `ModifierMask::SHIFT | ModifierMask::MOD1`.
    pub fn mods_to_clear(&self) -> ModifierMask {
        self.mods_to_clear
    }

    /// Returns the mods that are affected by this action.
    ///
    /// This is a shorthand for `self.mods_to_set() | self.mods_to_clear()`.
    pub fn modifier_mask(&self) -> ModifierMask {
        self.mods_to_set | self.mods_to_clear
    }
}

impl ControlsSetAction {
    /// Returns the controls mask of this action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <leftshift> {
    ///         [ SetControls(mods = Overlay1) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns the mask for `Overlay1`.
    pub fn mask(&self) -> ControlsMask {
        ControlsMask(self.controls.0 as u32)
    }
}

impl ControlsLockAction {
    /// Returns whether this action will lock the controls.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <leftshift> {
    ///         [ LockControls(controls = Overlay1, affect = unlock) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `false`.
    pub fn lock(&self) -> bool {
        self.lock
    }

    /// Returns whether this action will unlock previously-locked controls.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <leftshift> {
    ///         [ LockControls(controls = Overlay1, affect = lock) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `false`.
    pub fn unlock(&self) -> bool {
        self.unlock
    }

    /// Returns the controls mask of this action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <leftshift> {
    ///         [ LockControls(mods = Overlay1) ],
    ///     };
    /// };
    /// ```
    ///
    /// The function returns the mask for `Overlay1`.
    pub fn mask(&self) -> ControlsMask {
        ControlsMask(self.controls.0 as u32)
    }
}

impl OverlayBehavior {
    /// Returns the overlay that controls this behavior.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         overlay2 = <b>,
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `KeyOverlay::Overlay2`.
    pub fn overlay(&self) -> KeyOverlay {
        self.overlay
    }

    /// Returns the name of the key that this behavior redirects to if the overlay is
    /// active.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         overlay1 = <b>,
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `"b"`.
    pub fn key_name(&self) -> &str {
        &self.key_name
    }

    /// Returns the keycode of the key that this action redirects to if the overlay is
    /// active.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         overlay1 = <b>,
    ///     };
    /// };
    /// ```
    ///
    /// The function returns the keycode of `<b>`.
    pub fn keycode(&self) -> crate::Keycode {
        self.keycode
    }
}

impl RadioGroupBehavior {
    /// Returns whether pressing this key can release the pressed key.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         allownone,
    ///         radiogroup = 1,
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `true`.
    pub fn allow_none(&self) -> bool {
        self.allow_none
    }

    /// Returns the group that this key belongs to. This is a value between 1 and 32.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         radiogroup = 1,
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `1`.
    pub fn group(&self) -> u32 {
        self.radio_group.raw()
    }
}

impl Indicator {
    /// The name of the `Num Lock` indicator.
    pub const NUM_LOCK: &str = "Num Lock";
    /// The name of the `Caps Lock` indicator.
    pub const CAPS_LOCK: &str = "Caps Lock";
    /// The name of the `Scroll Lock` indicator.
    pub const SCROLL_LOCK: &str = "Scroll Lock";
    /// The name of the `Compose` indicator.
    pub const COMPOSE: &str = "Compose";
    /// The name of the `Kana` indicator.
    pub const KANA: &str = "Kana";

    /// Returns the name of the indicator.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "Caps Lock" {
    ///         modifiers = Lock;
    ///         whichModState = Effective;
    ///     };
    /// };
    /// ```
    ///
    /// The function returns `"Caps Lock"`.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the matcher for the indicator.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "Caps Lock" {
    ///         modifiers = Lock;
    ///         whichModState = Effective;
    ///     };
    /// };
    /// ```
    ///
    /// The matcher will match if the effective group contains the Lock modifier.
    pub fn matcher(&self) -> IndicatorMatcher {
        macro_rules! mods {
            ($comp:ident) => {
                self.mod_components
                    .contains(ModComponentMask::$comp)
                    .then_some(self.modifier_mask.0)
                    .unwrap_or_default()
            };
        }
        macro_rules! group_flag {
            ($comp:ident, $tt:tt) => {
                self.group_component == GroupComponent::$comp
                && self.group_mask.0 $tt 0
            };
        }
        macro_rules! group_mask {
            ($comp:ident) => {
                (self.group_component == GroupComponent::$comp)
                    .then_some(self.group_mask.0)
                    .unwrap_or_default()
            };
        }
        IndicatorMatcher {
            mods_pressed: mods!(BASE),
            mods_latched: mods!(LATCHED),
            mods_locked: mods!(LOCKED),
            mods: mods!(EFFECTIVE),
            group_pressed: group_flag!(Base, !=),
            group_not_pressed: group_flag!(Base, ==),
            group_latched: group_flag!(Latched, !=),
            group_not_latched: group_flag!(Latched, ==),
            group_locked: group_mask!(Locked),
            group: group_mask!(Effective),
            controls: self.controls.0 as u32,
        }
    }
}

impl IndicatorMatcher {
    /// Returns whether this indicator should be illuminated.
    pub fn matches(&self, components: &Components) -> bool {
        let mut res = 0;
        res |= self.mods_pressed & components.mods_pressed.0;
        res |= self.mods_latched & components.mods_latched.0;
        res |= self.mods_locked & components.mods_locked.0;
        res |= self.mods & components.mods.0;
        res |= (self.group_pressed & (components.group_pressed.0 != 0)) as u32;
        res |= (self.group_not_pressed & (components.group_pressed.0 == 0)) as u32;
        res |= (self.group_latched & (components.group_latched.0 != 0)) as u32;
        res |= (self.group_not_latched & (components.group_latched.0 == 0)) as u32;
        if components.group_locked.0 < u32::BITS {
            res |= self.group_locked & (1 << components.group_locked.0);
        }
        if components.group.0 < u32::BITS {
            res |= self.group & (1 << components.group.0);
        }
        res |= self.controls & components.controls.0;
        res != 0
    }
}
