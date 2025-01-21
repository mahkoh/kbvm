/// A bitmask of control flags.
///
/// KBVM does not assign a meaning to the individual bits but the XKB implementation uses
/// some of the bits to implement XKB controls.
#[derive(Copy, Clone, Eq, PartialEq, Hash, Default)]
pub struct ControlsMask(pub u32);

bitmask!(ControlsMask);
