use kbvm_proc::CloneWithDelta;

/// A keycode.
///
/// Keycodes represent physical keys. On Linux, they usually correspond to evdev
/// events which are in turn modeled after the USB standard. The [`evdev`] module
/// contains constants for evdev keycodes.
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, CloneWithDelta, Default)]
pub struct Keycode(pub(crate) u32);

impl Keycode {
    /// Returns the raw `u32` representing this keycode.
    ///
    /// This value should only be used in [`Routine`]s.
    pub const fn raw(self) -> u32 {
        self.0
    }

    /// Creates a keycode from an X11 keycode.
    #[inline]
    pub const fn from_x11(kc: u32) -> Self {
        Self(kc)
    }

    /// Converts the keycode to an X11 keycode.
    ///
    /// If this keycode was not created via [`Keycode::from_x11`], then the conversion is
    /// performed on a best-effort basis.
    #[inline]
    pub const fn to_x11(self) -> u32 {
        self.0
    }

    /// Creates a keycode from an evdev code.
    #[inline]
    pub const fn from_evdev(kc: u32) -> Self {
        Self(kc.saturating_add(8))
    }

    /// Converts the keycode to an evdev code.
    ///
    /// If this keycode was not created via [`Keycode::from_evdev`], then the conversion
    /// is performed on a best-effort basis.
    #[inline]
    pub const fn to_evdev(self) -> u32 {
        self.0.saturating_sub(8)
    }
}
