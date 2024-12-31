#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub(crate) struct Level(u32);

impl Level {
    pub(crate) const fn new(level: u32) -> Option<Level> {
        if level < 1 {
            return None;
        }
        Some(Self(level))
    }

    pub(crate) const fn raw(self) -> u32 {
        self.0
    }

    pub(crate) const fn to_offset(self) -> usize {
        self.0 as usize - 1
    }
}
