#[derive(Copy, Clone, Default, Eq, PartialEq)]
pub(crate) struct ModComponentMask(u8);

keyed_bitfield! {
    ModComponentMask:
    0 => BASE => Base,
    1 => LATCHED => Latched,
    2 => LOCKED => Locked,
    3 => EFFECTIVE => Effective | Compat | Any,
}
