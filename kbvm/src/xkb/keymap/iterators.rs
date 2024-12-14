//! Iterators over components of a keymap.

#[expect(unused_imports)]
use crate::xkb::keymap::{KeyType, Keymap};
use {
    crate::xkb::keymap::{Indicator, Key, KeyGroup, KeyLevel, KeyTypeMapping, VirtualModifier},
    indexmap::map::Values,
    std::slice::Iter,
};

/// An iterator over the virtual modifiers of a keymap.
///
/// Created using [`Keymap::virtual_modifiers`].
pub struct VirtualModifiers<'a> {
    pub(super) modifiers: Iter<'a, VirtualModifier>,
}

/// An iterator over the keys of a keymap.
///
/// Created using [`Keymap::keys`].
pub struct Keys<'a> {
    pub(super) keys: Values<'a, crate::Keycode, Key>,
}

/// An iterator over the groups of a key.
///
/// Created using [`Key::groups`].
pub struct Groups<'a> {
    pub(super) groups: Iter<'a, Option<KeyGroup>>,
}

/// An iterator over the levels of a group.
///
/// Created using [`KeyGroup::levels`].
pub struct Levels<'a> {
    pub(super) levels: Iter<'a, KeyLevel>,
}

/// An iterator over the mappings of a key type.
///
/// Created using [`KeyType::mappings`].
pub struct Mappings<'a> {
    pub(super) mappings: Iter<'a, KeyTypeMapping>,
}

/// An iterator over the indicators of a keymap.
///
/// Created using [`Keymap::indicators`].
pub struct Indicators<'a> {
    pub(super) indicators: Iter<'a, Indicator>,
}

impl<'a> Iterator for Mappings<'a> {
    type Item = &'a KeyTypeMapping;

    fn next(&mut self) -> Option<Self::Item> {
        self.mappings.next()
    }
}

impl<'a> Iterator for Levels<'a> {
    type Item = &'a KeyLevel;

    fn next(&mut self) -> Option<Self::Item> {
        self.levels.next()
    }
}

impl<'a> Iterator for Groups<'a> {
    type Item = Option<&'a KeyGroup>;

    fn next(&mut self) -> Option<Self::Item> {
        self.groups.next().map(|v| v.as_ref())
    }
}

impl<'a> Iterator for VirtualModifiers<'a> {
    type Item = &'a VirtualModifier;

    fn next(&mut self) -> Option<Self::Item> {
        self.modifiers.next()
    }
}

impl<'a> Iterator for Keys<'a> {
    type Item = &'a Key;

    fn next(&mut self) -> Option<Self::Item> {
        self.keys.next()
    }
}

impl<'a> Iterator for Indicators<'a> {
    type Item = &'a Indicator;

    fn next(&mut self) -> Option<Self::Item> {
        self.indicators.next()
    }
}
