use {
    crate::{
        group::{GroupDelta, GroupIndex},
        xkb::keymap,
    },
    kbvm_proc::CloneWithDelta,
};

#[derive(Copy, Clone, Debug, CloneWithDelta, PartialEq)]
pub(crate) struct GroupIdx(u32);

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub(crate) struct GroupMask(pub(crate) u32);

#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) enum GroupChange {
    Absolute(GroupIdx),
    Rel(i32),
}

impl GroupChange {
    pub(crate) fn to_group_change(self) -> keymap::GroupChange {
        match self {
            GroupChange::Absolute(idx) => keymap::GroupChange::Absolute(GroupIndex(idx.raw() - 1)),
            GroupChange::Rel(delta) => keymap::GroupChange::Relative(GroupDelta(delta as u32)),
        }
    }
}

impl Default for GroupChange {
    fn default() -> Self {
        Self::Rel(0)
    }
}

impl GroupIdx {
    pub(crate) const ONE: GroupIdx = GroupIdx::new(1).unwrap();

    pub(crate) const fn new(group: u32) -> Option<Self> {
        if group < 1 || group > u32::BITS {
            return None;
        }
        Some(Self(group))
    }

    pub(crate) const fn raw(self) -> u32 {
        self.0
    }

    pub(crate) const fn to_offset(self) -> usize {
        self.0 as usize - 1
    }
}
