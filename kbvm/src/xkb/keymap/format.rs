#[cfg(test)]
mod tests;

use {
    crate::xkb::Keymap,
    std::fmt::{Debug, Display},
};

/// A formatter for keymaps.
///
/// # Example
///
/// ```
/// # use kbvm::xkb::Keymap;
/// fn create_keymap_for_xwayland(keymap: &Keymap) -> String {
///     format!("{}\n", keymap.format().lookup_only(true).rename_long_keys(true))
/// }
/// ```
pub struct Formatter<'a> {
    pub(crate) keymap: &'a Keymap,
    pub(crate) single_line: bool,
    pub(crate) lookup_only: bool,
    pub(crate) rename_long_keys: bool,
}

impl Formatter<'_> {
    /// Enables or disables formatting the keymap as a single line.
    ///
    /// By default, the keymap is pretty-printed in multiple lines.
    pub fn single_line(mut self, val: bool) -> Self {
        self.single_line = val;
        self
    }

    /// Enables or disables formatting of key behaviors and key actions.
    ///
    /// By default, key behaviors and key actions are included in the map.
    ///
    /// This should be enabled for keymaps sent by compositors to Xwayland to work around
    /// an Xwayland bug. See [`Keymap::format`].
    pub fn lookup_only(mut self, val: bool) -> Self {
        self.lookup_only = val;
        self
    }

    /// Enables or disables the automatic renaming of key names longer than 4 bytes.
    ///
    /// By default, key names are used unchanged.
    ///
    /// This should be enabled for keymaps sent by compositors to Xwayland since Xwayland
    /// cannot handle key names longer than 4 bytes.
    pub fn rename_long_keys(mut self, val: bool) -> Self {
        self.rename_long_keys = val;
        self
    }
}

impl Debug for Formatter<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        Display::fmt(self, f)
    }
}
