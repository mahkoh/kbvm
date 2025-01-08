use {
    kbvm_proc::CloneWithDelta,
    std::{
        fmt::{Debug, Formatter},
        ops::{BitAnd, BitAndAssign, BitOr, BitOrAssign, BitXor, BitXorAssign, Not},
    },
};

#[derive(Copy, Clone, Eq, PartialEq, Hash, Default, CloneWithDelta)]
pub struct ModifierMask(pub u32);

#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug, CloneWithDelta, Ord, PartialOrd)]
pub struct ModifierIndex(u32);

#[derive(Clone)]
pub struct ModifierMaskIter(u32);

impl ModifierIndex {
    pub(crate) const SHIFT: Self = Self::new(0).unwrap();
    pub(crate) const LOCK: Self = Self::new(1).unwrap();
    pub(crate) const CONTROL: Self = Self::new(2).unwrap();
    pub(crate) const MOD1: Self = Self::new(3).unwrap();
    pub(crate) const MOD2: Self = Self::new(4).unwrap();
    pub(crate) const MOD3: Self = Self::new(5).unwrap();
    pub(crate) const MOD4: Self = Self::new(6).unwrap();
    pub(crate) const MOD5: Self = Self::new(7).unwrap();

    pub const fn new(index: u32) -> Option<Self> {
        if index >= u32::BITS {
            None
        } else {
            Some(Self(index))
        }
    }

    pub const fn raw(self) -> u32 {
        self.0
    }

    pub const fn to_mask(self) -> ModifierMask {
        ModifierMask(1 << self.0)
    }
}

impl ModifierMask {
    pub(crate) const NONE: Self = Self(0);
    pub const SHIFT: Self = ModifierIndex::SHIFT.to_mask();
    pub const LOCK: Self = ModifierIndex::LOCK.to_mask();
    pub const CONTROL: Self = ModifierIndex::CONTROL.to_mask();
    pub const MOD1: Self = ModifierIndex::MOD1.to_mask();
    pub const MOD2: Self = ModifierIndex::MOD2.to_mask();
    pub const MOD3: Self = ModifierIndex::MOD3.to_mask();
    pub const MOD4: Self = ModifierIndex::MOD4.to_mask();
    pub const MOD5: Self = ModifierIndex::MOD5.to_mask();

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

pub const NUM_MODS: usize = u32::BITS as usize;
pub const NUM_MODS_MASK: usize = NUM_MODS - 1;
