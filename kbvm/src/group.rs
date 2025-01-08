use std::{
    fmt::{Debug, Formatter},
    ops::Add,
};

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Default)]
#[repr(transparent)]
pub struct GroupIndex(pub u32);

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Default)]
#[repr(transparent)]
pub struct GroupDelta(pub u32);

impl Debug for GroupDelta {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0 as i32)
    }
}

impl Debug for GroupIndex {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl Add<GroupDelta> for GroupIndex {
    type Output = Self;

    fn add(self, rhs: GroupDelta) -> Self::Output {
        Self(self.0.wrapping_add(rhs.0))
    }
}

impl Add<GroupDelta> for GroupDelta {
    type Output = Self;

    fn add(self, rhs: GroupDelta) -> Self::Output {
        Self(self.0.wrapping_add(rhs.0))
    }
}
