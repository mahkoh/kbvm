//! Modifier helpers.

#[cfg(test)]
mod tests;

use {
    crate::modifier::hidden::{ModifierIndex, ModifierMask},
    std::{
        fmt::{Debug, Formatter},
        ops::{BitAnd, BitAndAssign, BitOr, BitOrAssign, BitXor, BitXorAssign, Not},
    },
};

pub(crate) mod hidden {
    use kbvm_proc::CloneWithDelta;

    /// A modifier mask.
    ///
    /// KBVM does not inherently assign meaning to the bits in the mask. However, in the
    /// XKB ecosystem, the bits in the least significant byte has the following meaning:
    ///
    /// - `0x01` - [`Self::SHIFT`]
    /// - `0x02` - [`Self::LOCK`]
    /// - `0x04` - [`Self::CONTROL`]
    /// - `0x08` - [`Self::MOD1`], [`Self::ALT`]
    /// - `0x10` - [`Self::MOD2`], [`Self::NUM_LOCK`]
    /// - `0x20` - [`Self::MOD3`]
    /// - `0x40` - [`Self::MOD4`], [`Self::SUPER`]
    /// - `0x80` - [`Self::MOD5`]
    ///
    /// # Iterating
    ///
    /// This type implements [`IntoIterator`] which allows you to iterate over the set
    /// bits in the mask.
    #[derive(Copy, Clone, Eq, PartialEq, Hash, Default, CloneWithDelta)]
    pub struct ModifierMask(pub u32);

    /// A modifier index.
    ///
    /// This is a number between 0 and 31 that indicates a bit in a modifier mask.
    ///
    /// See the documentation of [`ModifierMask`] for possible meanings of the indices.
    #[derive(Copy, Clone, Eq, PartialEq, Hash, CloneWithDelta, Ord, PartialOrd)]
    pub struct ModifierIndex(pub(super) u32);
}

/// An iterator over the modifiers in a [`ModifierMask`].
///
/// It is constructed by using the [`IntoIterator`] implementation of [`ModifierMask`].
///
/// # Example
///
/// ```
/// # use kbvm::{ModifierIndex, ModifierMask};
/// let mask = ModifierMask::SHIFT | ModifierMask::CONTROL;
/// let mut iter = mask.into_iter();
/// assert_eq!(iter.next(), Some(ModifierIndex::SHIFT));
/// assert_eq!(iter.next(), Some(ModifierIndex::CONTROL));
/// assert_eq!(iter.next(), None);
/// ```
#[derive(Clone, Debug)]
pub struct ModifierMaskIter(u32);

impl ModifierIndex {
    /// The `0` index, corresponding to the XKB `shift` modifier.
    pub const SHIFT: Self = Self::new(0).unwrap();
    /// The `1` index, corresponding to the XKB `lock` modifier.
    pub const LOCK: Self = Self::new(1).unwrap();
    /// The `2` index, corresponding to the XKB `control` modifier.
    pub const CONTROL: Self = Self::new(2).unwrap();
    /// The `3` index, corresponding to the XKB `mod1` modifier.
    pub const MOD1: Self = Self::new(3).unwrap();
    /// This is an alias for [`Self::MOD1`].
    ///
    /// This association exists by convention and is hard-coded in xkbcommon.
    pub const ALT: Self = Self::new(3).unwrap();
    /// The `4` index, corresponding to the XKB `mod2` modifier.
    pub const MOD2: Self = Self::new(4).unwrap();
    /// This is an alias for [`Self::MOD2`].
    ///
    /// This association exists by convention and is hard-coded in xkbcommon.
    pub const NUM_LOCK: Self = Self::new(4).unwrap();
    /// The `5` index, corresponding to the XKB `mod3` modifier.
    pub const MOD3: Self = Self::new(5).unwrap();
    /// The `6` index, corresponding to the XKB `mod4` modifier.
    pub const MOD4: Self = Self::new(6).unwrap();
    /// This is an alias for [`Self::MOD4`].
    ///
    /// This association exists by convention and is hard-coded in xkbcommon.
    pub const SUPER: Self = Self::new(6).unwrap();
    /// The `7` index, corresponding to the XKB `mod5` modifier.
    pub const MOD5: Self = Self::new(7).unwrap();

    /// Creates a new [`ModifierIndex`].
    ///
    /// Returns `None` if the index is larger than 31.
    pub const fn new(index: u32) -> Option<Self> {
        if index >= u32::BITS {
            None
        } else {
            Some(Self(index))
        }
    }

    /// Returns the numeric index.
    ///
    /// This value is guaranteed to be less than 32.
    pub const fn raw(self) -> u32 {
        self.0
    }

    /// Returns the mask corresponding to this index.
    ///
    /// ```
    /// # use kbvm::{ModifierIndex, ModifierMask};
    /// assert_eq!(ModifierIndex::SHIFT.to_mask(), ModifierMask::SHIFT);
    /// ```
    pub const fn to_mask(self) -> ModifierMask {
        ModifierMask(1 << self.0)
    }
}

impl ModifierMask {
    /// The empty mask, `Self(0)`.
    pub const NONE: Self = Self(0);
    /// The `0x01` mask, corresponding to the XKB `shift` modifier.
    pub const SHIFT: Self = ModifierIndex::SHIFT.to_mask();
    /// The `0x02` mask, corresponding to the XKB `lock` modifier.
    pub const LOCK: Self = ModifierIndex::LOCK.to_mask();
    /// The `0x04` mask, corresponding to the XKB `control` modifier.
    pub const CONTROL: Self = ModifierIndex::CONTROL.to_mask();
    /// The `0x08` mask, corresponding to the XKB `mod1` modifier.
    pub const MOD1: Self = ModifierIndex::MOD1.to_mask();
    /// This is an alias for [`Self::MOD1`].
    ///
    /// This association exists by convention and is hard-coded in xkbcommon.
    pub const ALT: Self = Self::MOD1;
    /// The `0x10` mask, corresponding to the XKB `mod2` modifier.
    pub const MOD2: Self = ModifierIndex::MOD2.to_mask();
    /// This is an alias for [`Self::MOD2`].
    ///
    /// This association exists by convention and is hard-coded in xkbcommon.
    pub const NUM_LOCK: Self = Self::MOD2;
    /// The `0x20` mask, corresponding to the XKB `mod3` modifier.
    pub const MOD3: Self = ModifierIndex::MOD3.to_mask();
    /// The `0x40` mask, corresponding to the XKB `mod4` modifier.
    pub const MOD4: Self = ModifierIndex::MOD4.to_mask();
    /// This is an alias for [`Self::MOD4`].
    ///
    /// This association exists by convention and is hard-coded in xkbcommon.
    pub const SUPER: Self = Self::MOD4;
    /// The `0x80` mask, corresponding to the XKB `mod5` modifier.
    pub const MOD5: Self = ModifierIndex::MOD5.to_mask();

    /// Returns whether this mask completely contains the other mask.
    ///
    /// This is a shorthand for `self & other == other`.
    pub const fn contains(self, other: ModifierMask) -> bool {
        self.0 & other.0 == other.0
    }
}

impl BitOr for ModifierMask {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl BitAnd for ModifierMask {
    type Output = Self;

    fn bitand(self, rhs: Self) -> Self::Output {
        Self(self.0 & rhs.0)
    }
}

impl BitXor for ModifierMask {
    type Output = Self;

    fn bitxor(self, rhs: Self) -> Self::Output {
        Self(self.0 ^ rhs.0)
    }
}

impl BitOrAssign for ModifierMask {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

impl BitAndAssign for ModifierMask {
    fn bitand_assign(&mut self, rhs: Self) {
        self.0 &= rhs.0;
    }
}

impl BitXorAssign for ModifierMask {
    fn bitxor_assign(&mut self, rhs: Self) {
        self.0 ^= rhs.0;
    }
}

impl Not for ModifierMask {
    type Output = Self;

    fn not(self) -> Self::Output {
        Self(!self.0)
    }
}

impl IntoIterator for ModifierMask {
    type Item = ModifierIndex;
    type IntoIter = ModifierMaskIter;

    fn into_iter(self) -> Self::IntoIter {
        ModifierMaskIter(self.0)
    }
}

impl Iterator for ModifierMaskIter {
    type Item = ModifierIndex;

    fn next(&mut self) -> Option<Self::Item> {
        let idx = self.0.trailing_zeros();
        if idx < u32::BITS {
            self.0 ^= 1 << idx;
            Some(ModifierIndex(idx))
        } else {
            None
        }
    }
}

impl Debug for ModifierMask {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "0x{:08x}", self.0)
    }
}

impl Debug for ModifierIndex {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        self.0.fmt(f)
    }
}

pub(crate) const NUM_MODS: usize = u32::BITS as usize;
pub(crate) const NUM_MODS_MASK: usize = NUM_MODS - 1;
