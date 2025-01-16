#[allow(unused_imports)]
use crate::builder::Redirect;
use std::{
    fmt::{Debug, Formatter},
    ops::Add,
};

/// A 0-based index into the groups of a keyboard.
///
/// Since different keys can have different numbers of groups, there is no such thing as
/// an invalid group index. Instead, when using the group index, it is brought into range
/// according to the [`Redirect`] setting of the key.
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Default)]
#[repr(transparent)]
pub struct GroupIndex(pub u32);

/// A group delta.
///
/// This delta can be applied to a [`GroupIndex`] to get a new `GroupIndex`. This is the
/// type of the pressed and latched group.
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Default)]
#[repr(transparent)]
pub struct GroupDelta(pub u32);

impl GroupIndex {
    pub const ZERO: GroupIndex = GroupIndex(0);
}

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
