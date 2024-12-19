use kbvm_proc::CloneWithDelta;

#[derive(Copy, Clone, Debug, CloneWithDelta, Eq, PartialEq)]
pub(crate) struct IndicatorIdx(u32);

impl IndicatorIdx {
    pub const ONE: Self = Self::new(1).unwrap();

    pub(crate) const fn new(idx: u32) -> Option<Self> {
        if idx < 1 || idx > u32::BITS {
            return None;
        }
        Some(Self(idx))
    }

    pub(crate) const fn raw(self) -> u32 {
        self.0
    }

    pub(crate) const fn to_offset(self) -> usize {
        self.0 as usize - 1
    }
}
