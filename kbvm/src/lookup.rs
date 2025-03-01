//! The client-side [`LookupTable`].
//!
//! This module contains types to map key events to keysyms and characters.
//!
//! The main entry point to this module is the [`LookupTable`] type.
//!
//! # Example
//!
//! ```
//! # use kbvm::{GroupIndex, Keycode, ModifierMask};
//! # use kbvm::lookup::LookupTable;
//! # use kbvm::xkb::Context;
//! # use kbvm::xkb::diagnostic::WriteToLog;
//! fn create_lookup_table(keymap: &[u8]) -> LookupTable {
//!     let context = Context::default();
//!     let keymap = context.keymap_from_bytes(WriteToLog, None, keymap).unwrap();
//!     keymap.to_builder().build_lookup_table()
//! }
//!
//! fn key_press_to_string(
//!     lookup_table: &LookupTable,
//!     group: GroupIndex,
//!     mods: ModifierMask,
//!     keycode: Keycode,
//! ) -> String {
//!     lookup_table
//!         .lookup(group, mods, keycode)
//!         .into_iter()
//!         .flat_map(|p| p.char())
//!         .collect()
//! }
//! ```

#[cfg(test)]
mod tests;

#[expect(unused_imports)]
use crate::{builder::Builder, Components};
use {
    crate::{
        builder::Redirect, group::GroupIndex, key_storage::KeyStorage, GroupType, Keycode, Keysym,
        ModifierMask,
    },
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
/// # use kbvm::GroupIndex;
/// # use kbvm::lookup::LookupTable;
/// # use kbvm::ModifierMask;
/// # use kbvm::Keycode;
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
/// key-press events until the next repeating key is pressed or this key is released.
///
/// This property can be accessed with the [`Lookup::repeats`] function.
///
/// ```
/// # use kbvm::GroupIndex;
/// # use kbvm::lookup::LookupTable;
/// # use kbvm::ModifierMask;
/// # use kbvm::Keycode;
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
/// # use kbvm::GroupIndex;
/// # use kbvm::Keysym;
/// # use kbvm::lookup::LookupTable;
/// # use kbvm::ModifierMask;
/// # use kbvm::Keycode;
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
/// ```
/// # use kbvm::GroupIndex;
/// # use kbvm::Keysym;
/// # use kbvm::lookup::{KeysymProps, LookupTable};
/// # use kbvm::ModifierMask;
/// # use kbvm::Keycode;
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
/// ```
/// # use kbvm::GroupIndex;
/// # use kbvm::Keysym;
/// # use kbvm::lookup::{KeysymProps, LookupTable};
/// # use kbvm::ModifierMask;
/// # use kbvm::Keycode;
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
/// This is sometimes used to allow users of non-latin keyboard layouts to use latin
/// keysyms in keyboard shortcuts.
///
/// This behavior can be disabled by using [`Lookup::with_ctrl_fallback`] and
/// [`Lookup::set_ctrl_fallback`].
///
/// ```
/// # use kbvm::GroupIndex;
/// # use kbvm::Keysym;
/// # use kbvm::lookup::{KeysymProps, LookupTable};
/// # use kbvm::ModifierMask;
/// # use kbvm::Keycode;
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
    pub(crate) keys: KeyStorage<KeyGroups>,
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
    pub(crate) levels: Box<[KeyLevel]>,
}

#[derive(Default, Clone, Debug)]
pub(crate) struct KeyLevel {
    pub(crate) symbols: SmallVec<[Keysym; 1]>,
}

/// The result of a [`LookupTable::lookup`] call.
///
/// This object represents the result of a call to [`LookupTable::lookup`]. To get the
/// produced keysyms, use the [`IntoIterator`] implementation of this type.
///
/// You can modify the output of the iterator by calling the various functions of this
/// type. This is described in detail in the [`LookupTable`] documentation.
///
/// # Example
///
/// ```
/// # use kbvm::{evdev, GroupIndex, ModifierMask};
/// # use kbvm::lookup::LookupTable;
/// fn lookup(table: &LookupTable) {
///     for output in table.lookup(GroupIndex::ZERO, ModifierMask::SHIFT, evdev::A) {
///         println!("{:?}", output.keysym());
///     }
/// }
/// ```
///
/// This might print
///
/// ```text
/// A
/// ```
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

/// An iterator over [`KeysymProps`].
///
/// This type is created via the [`IntoIterator`] implementation of [`Lookup`].
#[derive(Clone, Debug)]
pub struct LookupIter<'a> {
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
    /// See the documentation of [`LookupTable`] for more details.
    pub fn with_ctrl_fallback(mut self, fallback: bool) -> Self {
        self.use_ctrl_fallback = fallback;
        self
    }

    /// Enables or disables Ctrl Transformation Fallback.
    ///
    /// See the documentation of [`LookupTable`] for more details.
    pub fn set_ctrl_fallback(&mut self, fallback: bool) {
        self.use_ctrl_fallback = fallback;
    }

    /// Enables or disables Ctrl Transformation.
    ///
    /// See the documentation of [`LookupTable`] for more details.
    pub fn with_ctrl_transform(mut self, transform: bool) -> Self {
        self.do_ctrl_transform = transform;
        self
    }

    /// Enables or disables Ctrl Transformation.
    ///
    /// See the documentation of [`LookupTable`] for more details.
    pub fn set_ctrl_transform(&mut self, transform: bool) {
        self.do_ctrl_transform = transform;
    }

    /// Enables or disables Caps Transformation.
    ///
    /// See the documentation of [`LookupTable`] for more details.
    pub fn with_caps_transform(mut self, transform: bool) -> Self {
        self.do_caps_transform = transform;
        self
    }

    /// Enables or disables Caps Transformation.
    ///
    /// See the documentation of [`LookupTable`] for more details.
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
    /// from [`Keysym::char`].
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
    /// Looks up the keysyms associated with a key press.
    ///
    /// The `group` and `mods` should be the effective group and effective modifiers
    /// computed by the compositor.
    ///
    /// In wayland, the effective modifiers are the bitwise OR of the pressed, latched,
    /// and locked modifiers. This can be managed with the [`Components`] type.
    ///
    /// # Example
    ///
    /// An application using the wayland-client crate might use this function as follows:
    ///
    /// ```ignore
    /// struct State {
    ///     lookup_table: LookupTable,
    ///     group: GroupIndex,
    ///     mods: ModifierMask,
    /// }
    ///
    /// impl Dispatch<WlKeyboard, ()> for State {
    ///     fn event(
    ///         state: &mut State,
    ///         _: &WlKeyboard,
    ///         event: wl_keyboard::Event,
    ///         _: &(),
    ///         _: &Connection,
    ///         _: &QueueHandle<State>,
    ///     ) {
    ///         use wl_keyboard::Event;
    ///         match event {
    ///             Event::Modifiers {
    ///                 mods_depressed, mods_latched, mods_locked, group, ..
    ///             } => {
    ///                 state.group = GroupIndex(group);
    ///                 state.mods = ModifierMask(mods_depressed | mods_latched | mods_locked);
    ///             }
    ///             Event::Key { key, state: WEnum::Value(WlKeyState::Pressed), .. } => {
    ///                 let key = Keycode::from_evdev(key);
    ///                 for keysym in state.lookup_table.lookup(state.group, state.mods, key) {
    ///                     println!("{keysym:?}");
    ///                 }
    ///             }
    ///             _ => { },
    ///         }
    ///     }
    /// }
    /// ```
    pub fn lookup(&self, group: GroupIndex, mods: ModifierMask, keycode: Keycode) -> Lookup<'_> {
        let mut consumed = ModifierMask::default();
        let mut groups = &[][..];
        let mut syms = &[][..];
        let mut repeats = true;
        if let Some(key) = self.keys.get(keycode) {
            repeats = key.repeats;
            groups = &key.groups;
            if key.groups.len() > 0 {
                let group = key.redirect.apply(group, key.groups.len());
                if let Some(group) = &key.groups[group] {
                    let mapping = group.ty.map(mods);
                    consumed = mapping.consumed;
                    // println!("{:?}", group.ty);
                    // println!("{:?}", mapping);
                    if let Some(level) = group.levels.get(mapping.level) {
                        syms = &level.symbols;
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

    /// Returns the effective group that will be used for the keycode.
    ///
    /// The `group` parameter should be the effective group computed by the compositor.
    /// This function will then return the effective group for the specific key.
    ///
    /// These values can differ when the key has fewer groups than the maximum number of
    /// groups in the keymap. In this case, the effective group is calculated using
    /// the [`Redirect`] setting of the key.
    pub fn effective_group(&self, group: GroupIndex, keycode: Keycode) -> Option<GroupIndex> {
        if let Some(key) = self.keys.get(keycode) {
            if key.groups.len() > 0 {
                let group = key.redirect.apply(group, key.groups.len());
                return Some(GroupIndex(group as u32));
            }
        }
        None
    }

    /// Returns whether the key repeats.
    pub fn repeats(&self, keycode: Keycode) -> bool {
        if let Some(key) = self.keys.get(keycode) {
            return key.repeats;
        }
        true
    }
}

impl<'a> IntoIterator for Lookup<'a> {
    type Item = KeysymProps;
    type IntoIter = LookupIter<'a>;

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
                let level = group.ty.map(self.original_mods).level;
                if let Some(level) = group.levels.get(level) {
                    if level.symbols.len() == 1 && level.symbols[0].0 <= 127 {
                        syms = &level.symbols;
                        did_ctrl_fallback = true;
                        break;
                    }
                }
            }
        }
        LookupIter {
            did_ctrl_fallback,
            do_ctrl_transform,
            do_caps_transform,
            syms,
        }
    }
}

impl Iterator for LookupIter<'_> {
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
