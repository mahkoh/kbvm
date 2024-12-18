#[derive(Copy, Clone, Default, Eq, PartialEq)]
pub(crate) struct GroupComponentMask(u8);

keyed_bitfield! {
    GroupComponentMask:
    0 => BASE => Base,
    1 => LATCHED => Latched,
    2 => LOCKED => Locked,
    3 => EFFECTIVE => Effective | Any,
}
