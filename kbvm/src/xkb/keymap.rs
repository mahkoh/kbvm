//! XKB keymaps.
//!
//! This module contains types representing the components of an XKB keymap.

mod format;
mod from_lookup;
mod from_resolved;
pub mod iterators;
mod to_builder;

use {
    crate::{
        builder::Redirect,
        group::{GroupDelta, GroupIndex},
        keysym::Keysym,
        modifier::{ModifierIndex, ModifierMask},
        state_machine,
        xkb::{
            controls::ControlMask,
            group::{GroupIdx, GroupMask},
            group_component::GroupComponent,
            indicator::IndicatorIdx,
            keymap::{
                actions::{
                    GroupLatchAction, GroupLockAction, GroupSetAction, ModsLatchAction,
                    ModsLockAction, ModsSetAction,
                },
                iterators::{Groups, Keys, Levels, Mappings, VirtualModifiers},
            },
            level::Level,
            mod_component::ModComponentMask,
            resolved::GroupsRedirect,
        },
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
/// # Example
///
/// ```xkb
/// xkb_keymap {
///     xkb_keycodes {
///         <a> = 38;
///         <leftshift> = 50;
///     };
///     xkb_types { };
///     xkb_compat { };
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
/// # Formatting
///
/// This type implements [`Display`] which will format the map in XKB text format. By
/// default, the map will be formatted in a single line. You can enable multi-line
/// formatting by using the alternate modifier `#`.
///
/// ```
/// # use kbvm::xkb::Keymap;
/// fn pretty_print_keymap(keymap: &Keymap) -> String {
///     format!("{keymap:#}")
/// }
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
    pub(crate) keys: IndexMap<state_machine::Keycode, Key, DefaultHashBuilder>,
}

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) struct ModMapValue {
    pub(crate) key_name: Arc<String>,
    pub(crate) key_sym: Option<Keysym>,
}

#[derive(Debug, PartialEq)]
pub(crate) struct Indicator {
    pub(crate) virt: bool,
    pub(crate) index: IndicatorIdx,
    pub(crate) name: Arc<String>,
    pub(crate) modifier_mask: ModifierMask,
    pub(crate) group_mask: GroupMask,
    pub(crate) controls: ControlMask,
    pub(crate) mod_components: ModComponentMask,
    pub(crate) group_components: GroupComponent,
}

#[derive(Debug, PartialEq)]
pub(crate) struct Keycode {
    pub(crate) name: Arc<String>,
    pub(crate) keycode: state_machine::Keycode,
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
}

/// The XKB actions supported by KBVM.
pub mod actions {
    use crate::{modifier::ModifierMask, xkb::group};

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
    pub(crate) key_code: state_machine::Keycode,
    pub(crate) groups: Vec<Option<KeyGroup>>,
    pub(crate) repeat: bool,
    pub(crate) redirect: GroupsRedirect,
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
            group_components: Default::default(),
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
    pub fn keycode(&self) -> state_machine::Keycode {
        self.key_code
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
    /// # Example 1
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
    /// `map[Shift]`.
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
