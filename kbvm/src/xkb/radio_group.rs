#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub(crate) struct RadioGroup(u32);

impl RadioGroup {
    pub(crate) const fn new(radio_group: u32) -> Option<RadioGroup> {
        if radio_group < 1 || radio_group > u32::BITS {
            return None;
        }
        Some(Self(radio_group))
    }

    pub(crate) const fn raw(self) -> u32 {
        self.0
    }

    pub(crate) const fn to_mask(self) -> u32 {
        1 << (self.0 - 1)
    }
}
