use {
    kbvm_proc::CloneWithDelta,
    std::{
        fmt::{Debug, Formatter},
        ops::{BitAnd, BitAndAssign, BitOr, BitOrAssign, BitXor, BitXorAssign, Not},
    },
};

#[derive(Copy, Clone, Eq, PartialEq, Hash, Default, CloneWithDelta)]
pub struct ModifierMask(pub u32);

#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug, CloneWithDelta)]
pub struct ModifierIndex(pub u32);

#[derive(Clone)]
pub struct ModifierMaskIter(u32);

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

impl ModifierIndex {
    pub const fn to_mask(self) -> ModifierMask {
        ModifierMask(1 << self.0)
    }
}

pub const NUM_MODS: usize = u32::BITS as usize;
pub const NUM_MODS_MASK: usize = NUM_MODS - 1;
