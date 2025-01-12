#[expect(unused_imports)]
use crate::builder::Builder;
use {
    crate::{
        builder::Redirect, group::GroupIndex, group_type::GroupType, keysym::Keysym,
        modifier::ModifierMask, state_machine::Keycode,
    },
    hashbrown::HashMap,
    smallvec::SmallVec,
    std::fmt::{Debug, Formatter},
};

/// A keysym lookup table.
///
/// This type allows looking up keysyms and characters generated by key-press events. It
/// is created by calling [`Builder::build_lookup_table`].
///
/// # Example
///
/// ```
/// # use kbvm::group::GroupIndex;
/// # use kbvm::lookup::LookupTable;
/// # use kbvm::modifier::ModifierMask;
/// # use kbvm::state_machine::Keycode;
/// fn get_string(
///     table: &LookupTable,
///     group: GroupIndex,
///     modifiers: ModifierMask,
///     keycode: Keycode,
/// ) -> String {
///     table
///         .lookup(group, modifiers, keycode)
///         .into_iter()
///         .flat_map(|p| p.char())
///         .collect()
/// }
/// ```
///
/// # Key Repeat
///
/// A key can be configured to repeat. In this case, the client should periodically emulate
/// key-press events until the next key is pressed or this key is released.
///
/// This property can be accessed with the [`Lookup::repeats`] function.
///
/// ```
/// # use kbvm::group::GroupIndex;
/// # use kbvm::lookup::LookupTable;
/// # use kbvm::modifier::ModifierMask;
/// # use kbvm::state_machine::Keycode;
/// fn need_key_repeat_events(
///     table: &LookupTable,
///     group: GroupIndex,
///     modifiers: ModifierMask,
///     keycode: Keycode,
/// ) -> bool {
///     table
///         .lookup(group, modifiers, keycode)
///         .repeats()
/// }
/// ```
///
/// # Consumed Modifiers
///
/// The lookup process might consume some of the input modifiers. These modifiers should be ignored
/// when the produced keysyms are used in keyboard shortcuts.
///
/// This remaining modifiers can be accessed with the [`Lookup::remaining_mods`] function.
///
/// ```
/// # use kbvm::group::GroupIndex;
/// # use kbvm::keysym::Keysym;
/// # use kbvm::lookup::LookupTable;
/// # use kbvm::modifier::ModifierMask;
/// # use kbvm::state_machine::Keycode;
/// fn get_keysyms_for_shortcuts(
///     table: &LookupTable,
///     group: GroupIndex,
///     modifiers: ModifierMask,
///     keycode: Keycode,
/// ) -> impl Iterator<Item = (ModifierMask, Keysym)> + use<'_> {
///     let lookup = table.lookup(group, modifiers, keycode);
///     lookup.into_iter().map(move |p| (lookup.remaining_mods(), p.keysym()))
/// }
/// ```
///
/// # Caps Transformation
///
/// If the remaining modifiers contain the caps modifier, the keysym iterator will automatically
/// perform caps transformation. That is, all keysyms will be replaced by their uppercase version.
///
/// This behavior can be disabled by using [`Lookup::with_caps_transform`] and
/// [`Lookup::set_caps_transform`].
///
/// Caps transformation is applied before ctrl transformation.
///
/// ```
/// # use kbvm::group::GroupIndex;
/// # use kbvm::keysym::Keysym;
/// # use kbvm::lookup::{KeysymProps, LookupTable};
/// # use kbvm::modifier::ModifierMask;
/// # use kbvm::state_machine::Keycode;
/// fn disable_caps_transform(
///     table: &LookupTable,
///     group: GroupIndex,
///     modifiers: ModifierMask,
///     keycode: Keycode,
/// ) -> impl Iterator<Item = KeysymProps> + use<'_> {
///     table
///         .lookup(group, modifiers, keycode)
///         .with_caps_transform(false)
///         .into_iter()
/// }
/// ```
///
/// # Ctrl Transformation
///
/// If the remaining modifiers contain the ctrl modifier, the keysym iterator will automatically
/// perform ctrl transformation. This transform only affects the produced characters but not the
/// produced keysyms.
///
/// This behavior can be disabled by using [`Lookup::with_ctrl_transform`] and
/// [`Lookup::set_ctrl_transform`].
///
/// Caps transformation is applied before ctrl transformation.
///
/// ```
/// # use kbvm::group::GroupIndex;
/// # use kbvm::keysym::Keysym;
/// # use kbvm::lookup::{KeysymProps, LookupTable};
/// # use kbvm::modifier::ModifierMask;
/// # use kbvm::state_machine::Keycode;
/// fn disable_ctrl_transform(
///     table: &LookupTable,
///     group: GroupIndex,
///     modifiers: ModifierMask,
///     keycode: Keycode,
/// ) -> impl Iterator<Item = char> + use<'_> {
///     table
///         .lookup(group, modifiers, keycode)
///         .with_ctrl_transform(false)
///         .into_iter()
///         .flat_map(|p| p.char())
/// }
/// ```
///
/// Ctrl transformation applies the following modification:
///
/// ```
/// # let input = 0u8;
/// let output = match input {
///    b'@'..=b'~' => input & 0x1f,
///    b' ' | b'2' => 0,
///    b'3'..=b'7' => input - 0x1b,
///    b'8' => 0x7f,
///    b'/' => 0x1f,
///    _ => input,
/// };
/// ```
///
/// # Ctrl Transformation Fallback
///
/// If
///
/// - the preconditions for ctrl transformation are met, and
/// - the key press would otherwise produce a single keysym, and
/// - that keysym has a value greater than 127, and
/// - the key has another group that, when used with the same modifiers, would produce a single
///   keysym less than or equal to 127,
///
/// then, instead of using the original keysym, the keysym form the first such group is used.
///
/// Caps and ctrl transformations are then applied to that keysym.
///
/// This behavior can be disabled by using [`Lookup::with_ctrl_fallback`] and
/// [`Lookup::set_ctrl_fallback`].
///
/// ```
/// # use kbvm::group::GroupIndex;
/// # use kbvm::keysym::Keysym;
/// # use kbvm::lookup::{KeysymProps, LookupTable};
/// # use kbvm::modifier::ModifierMask;
/// # use kbvm::state_machine::Keycode;
/// fn disable_ctrl_fallback(
///     table: &LookupTable,
///     group: GroupIndex,
///     modifiers: ModifierMask,
///     keycode: Keycode,
/// ) -> impl Iterator<Item = char> + use<'_> {
///     table
///         .lookup(group, modifiers, keycode)
///         .with_ctrl_fallback(false)
///         .into_iter()
///         .flat_map(|p| p.char())
/// }
/// ```
#[derive(Clone, Debug)]
pub struct LookupTable {
    pub(crate) ctrl: Option<ModifierMask>,
    pub(crate) caps: Option<ModifierMask>,
    pub(crate) keys: HashMap<Keycode, KeyGroups>,
}

#[derive(Default, Clone, Debug)]
pub(crate) struct KeyGroups {
    pub(crate) repeats: bool,
    pub(crate) redirect: Redirect,
    pub(crate) groups: Box<[Option<KeyGroup>]>,
}

#[derive(Clone, Debug)]
pub(crate) struct KeyGroup {
    pub(crate) ty: GroupType,
    pub(crate) layers: Box<[KeyLayer]>,
}

#[derive(Default, Clone, Debug)]
pub(crate) struct KeyLayer {
    pub(crate) symbols: SmallVec<[Keysym; 1]>,
}

#[derive(Copy, Clone)]
pub struct Lookup<'a> {
    original_mods: ModifierMask,
    remaining_mods: ModifierMask,
    use_ctrl_fallback: bool,
    do_ctrl_transform: bool,
    do_caps_transform: bool,
    repeats: bool,
    lookup: &'a LookupTable,
    groups: &'a [Option<KeyGroup>],
    syms: &'a [Keysym],
}

#[derive(Clone, Debug)]
pub struct KeysymsIter<'a> {
    did_ctrl_fallback: bool,
    do_ctrl_transform: bool,
    do_caps_transform: bool,
    syms: &'a [Keysym],
}

/// A keysym produced by the lookup process.
#[derive(Copy, Clone, Debug)]
pub struct KeysymProps {
    keysym: Keysym,
    char: Option<char>,
    did_ctrl_fallback: bool,
    did_ctrl_transform: bool,
    did_caps_transform: bool,
}

impl Lookup<'_> {
    /// Enables or disables Ctrl Transformation Fallback.
    ///
    /// See the documentation of the type for more details.
    pub fn with_ctrl_fallback(mut self, fallback: bool) -> Self {
        self.use_ctrl_fallback = fallback;
        self
    }

    /// Enables or disables Ctrl Transformation Fallback.
    ///
    /// See the documentation of the type for more details.
    pub fn set_ctrl_fallback(&mut self, fallback: bool) {
        self.use_ctrl_fallback = fallback;
    }

    /// Enables or disables Ctrl Transformation.
    ///
    /// See the documentation of the type for more details.
    pub fn with_ctrl_transform(mut self, transform: bool) -> Self {
        self.do_ctrl_transform = transform;
        self
    }

    /// Enables or disables Ctrl Transformation.
    ///
    /// See the documentation of the type for more details.
    pub fn set_ctrl_transform(&mut self, transform: bool) {
        self.do_ctrl_transform = transform;
    }

    /// Enables or disables Caps Transformation.
    ///
    /// See the documentation of the type for more details.
    pub fn with_caps_transform(mut self, transform: bool) -> Self {
        self.do_caps_transform = transform;
        self
    }

    /// Enables or disables Caps Transformation.
    ///
    /// See the documentation of the type for more details.
    pub fn set_caps_transform(&mut self, transform: bool) {
        self.do_caps_transform = transform;
    }

    /// Returns whether the key repeats.
    pub fn repeats(&self) -> bool {
        self.repeats
    }

    /// Returns the remaining modifiers.
    ///
    /// These are the modifiers that should be used for keyboard shortcuts.
    pub fn remaining_mods(&self) -> ModifierMask {
        self.remaining_mods
    }
}

impl KeysymProps {
    /// The keysym.
    pub fn keysym(&self) -> Keysym {
        self.keysym
    }

    /// The character produced by the lookup.
    ///
    /// If ctrl transformation was applied to the lookup, then this character is different
    /// from [Keysym::char].
    pub fn char(&self) -> Option<char> {
        self.char
    }

    /// Whether ctrl fallback was applied to this keysym.
    pub fn did_ctrl_fallback(&self) -> bool {
        self.did_ctrl_fallback
    }

    /// Whether ctrl transformation was applied to this keysym and changed the character.
    pub fn did_ctrl_transform(&self) -> bool {
        self.did_ctrl_transform
    }

    /// Whether caps transformation was applied to this keysym and changed the keysym.
    pub fn did_caps_transform(&self) -> bool {
        self.did_caps_transform
    }
}

impl Debug for Lookup<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Keysyms")
            .field("original_mods", &self.original_mods)
            .field("remaining_mods", &self.remaining_mods)
            .field("use_ctrl_fallback", &self.use_ctrl_fallback)
            .field("do_ctrl_transform", &self.do_ctrl_transform)
            .field("do_caps_transform", &self.do_caps_transform)
            .field("repeats", &self.repeats)
            .finish_non_exhaustive()
    }
}

impl LookupTable {
    pub fn lookup(&self, group: GroupIndex, mods: ModifierMask, keycode: Keycode) -> Lookup<'_> {
        let mut consumed = ModifierMask::default();
        let mut groups = &[][..];
        let mut syms = &[][..];
        let mut repeats = true;
        if let Some(key) = self.keys.get(&keycode) {
            repeats = key.repeats;
            groups = &key.groups;
            if key.groups.len() > 0 {
                let group = key.redirect.apply(group, key.groups.len());
                if let Some(group) = &key.groups[group] {
                    let mapping = group.ty.map(mods);
                    consumed = mapping.consumed;
                    // println!("{:?}", group.ty);
                    // println!("{:?}", mapping);
                    if let Some(layer) = group.layers.get(mapping.layer) {
                        syms = &layer.symbols;
                    }
                }
            }
        }
        Lookup {
            original_mods: mods,
            remaining_mods: mods & !consumed,
            use_ctrl_fallback: true,
            do_ctrl_transform: true,
            do_caps_transform: true,
            lookup: self,
            repeats,
            groups,
            syms,
        }
    }

    pub fn effective_layout(&self, group: GroupIndex, keycode: Keycode) -> Option<GroupIndex> {
        if let Some(key) = self.keys.get(&keycode) {
            if key.groups.len() > 0 {
                let group = key.redirect.apply(group, key.groups.len());
                return Some(GroupIndex(group as u32));
            }
        }
        None
    }
}

impl<'a> IntoIterator for Lookup<'a> {
    type Item = KeysymProps;
    type IntoIter = KeysymsIter<'a>;

    fn into_iter(self) -> Self::IntoIter {
        let mut do_ctrl_transform = false;
        if let Some(mask) = self.lookup.ctrl {
            if self.do_ctrl_transform && self.remaining_mods.contains(mask) {
                do_ctrl_transform = true;
            }
        }
        let mut do_caps_transform = false;
        if let Some(mask) = self.lookup.caps {
            if self.do_caps_transform && self.remaining_mods.contains(mask) {
                do_caps_transform = true;
            }
        }
        let mut syms = self.syms;
        let mut did_ctrl_fallback = false;
        if self.use_ctrl_fallback && do_ctrl_transform && syms.len() == 1 && syms[0].0 > 127 {
            for group in self.groups.iter().flatten() {
                let layer = group.ty.map(self.original_mods).layer;
                if let Some(layer) = group.layers.get(layer) {
                    if layer.symbols.len() == 1 && layer.symbols[0].0 <= 127 {
                        syms = &layer.symbols;
                        did_ctrl_fallback = true;
                        break;
                    }
                }
            }
        }
        KeysymsIter {
            did_ctrl_fallback,
            do_ctrl_transform,
            do_caps_transform,
            syms,
        }
    }
}

impl Iterator for KeysymsIter<'_> {
    type Item = KeysymProps;

    fn next(&mut self) -> Option<Self::Item> {
        let mut sym = self.syms.first().copied()?;
        self.syms = &self.syms[1..];
        let mut did_caps_transform = false;
        if self.do_caps_transform {
            let prev = sym;
            sym = sym.to_uppercase();
            did_caps_transform = prev != sym;
        }
        let mut char = None;
        let mut did_ctrl_transform = false;
        if self.do_ctrl_transform && sym.0 <= 127 {
            let s = sym.0 as u8;
            'transform: {
                let c = match s {
                    b'@'..=b'~' | b' ' => s & 0x1f,
                    b'2' => 0,
                    b'3'..=b'7' => s - 0x1b,
                    b'8' => 0x7f,
                    b'/' => 0x1f,
                    _ => break 'transform,
                };
                char = Some(c as char);
                did_ctrl_transform = true;
            }
        }
        if char.is_none() {
            char = sym.char();
        }
        Some(KeysymProps {
            keysym: sym,
            char,
            did_ctrl_fallback: self.did_ctrl_fallback,
            did_ctrl_transform,
            did_caps_transform,
        })
    }
}
