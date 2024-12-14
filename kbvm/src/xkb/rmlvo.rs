//! RMLVO helpers and types.

use {crate::xkb::format::FormatFormat, std::fmt::Display};

#[macro_use]
mod macros;
#[cfg(test)]
mod formatter;
pub(crate) mod lexer;
pub(crate) mod parser;
pub(crate) mod resolver;
mod token;

/// An RMLVO group consisting of a layout and a variant.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct Group<'a> {
    /// The layout of the group.
    pub layout: &'a str,
    /// The variant of the group.
    pub variant: &'a str,
}

impl<'a> Group<'a> {
    /// Creates a Group iterator from comma-separated lists of layouts and variants.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::xkb::rmlvo::Group;
    /// let layout = "us,il,ru,de,jp";
    /// let variant = ",,phonetic,neo";
    /// let groups: Vec<_> = Group::from_layouts_and_variants(layout, variant).collect();
    /// assert_eq!(
    ///     groups,
    ///     [
    ///         Group { layout: "us", variant: "" },
    ///         Group { layout: "il", variant: "" },
    ///         Group { layout: "ru", variant: "phonetic" },
    ///         Group { layout: "de", variant: "neo" },
    ///         Group { layout: "jp", variant: "" },
    ///     ],
    /// );
    /// ```
    pub fn from_layouts_and_variants(
        layout: &'a str,
        variant: &'a str,
    ) -> impl Iterator<Item = Self> {
        let layouts = layout.split(',');
        let mut variants = variant.split(',');
        layouts.map(move |layout| Group {
            layout: layout.trim(),
            variant: variants.next().unwrap_or_default().trim(),
        })
    }
}

/// An include merge mode.
///
/// # Example
///
/// ```xkb
/// xkb_symbols {
///     override "pc"
///     override "de(neo)"
///     augment "inet(evdev)"
/// };
/// ```
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum MergeMode {
    /// In case of conflict, the previous definition is retained.
    Augment,
    /// In case of conflict, the new definition overrides the old one.
    Override,
}

/// An include statement in an XKB section.
///
/// This is part of [`Expanded`].
///
/// # Example
///
/// ```xkb
/// xkb_symbols {
///     override "pc"
///     override "de(neo)"
///     augment "inet(evdev)"
/// };
/// ```
#[derive(Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub struct Element {
    /// The merge mode.
    pub merge_mode: MergeMode,
    /// The include string.
    pub include: String,
}

/// Expanded RMLVO names.
///
/// # Example
///
/// ```xkb
/// xkb_keymap {
///     xkb_keycodes {
///         override "evdev"
///         override "aliases(qwertz)"
///     };
///     xkb_types {
///         override "complete"
///     };
///     xkb_compat {
///         override "complete"
///         override "caps(caps_lock)"
///         override "misc(assign_shift_left_action)"
///         override "level5(level5_lock)"
///     };
///     xkb_symbols {
///         override "pc"
///         override "de(neo)"
///         override "inet(evdev)"
///     };
///     xkb_geometry {
///         override "pc(pc105)"
///     };
/// };
/// ```
#[derive(Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub struct Expanded {
    /// The elements of the `xkb_keycodes` section.
    pub keycodes: Vec<Element>,
    /// The elements of the `xkb_types` section.
    pub types: Vec<Element>,
    /// The elements of the `xkb_compat` section.
    pub compat: Vec<Element>,
    /// The elements of the `xkb_symbols` section.
    pub symbols: Vec<Element>,
    /// The elements of the `xkb_geometry` section.
    pub geometry: Vec<Element>,
}

impl Expanded {
    /// Returns a type that can be used to format the expanded map in XKB text format.
    ///
    /// See the type documentation for an example.
    ///
    /// By default, the map will be formatted in a single line. You can enable multi-line
    /// formatting by using the alternate modifier `#`.
    pub fn format(&self) -> impl Display + use<'_> {
        FormatFormat(self)
    }
}
