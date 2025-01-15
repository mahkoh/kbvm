//! [`Keycode`] constants for evdev keys.
//!
//! See [input-event-code.h] for the authoritative source for these constants.
//!
//! [input-event-code.h]: https://github.com/torvalds/linux/blob/master/include/uapi/linux/input-event-codes.h

#[allow(unused_imports)]
use crate::Keycode;
#[rustfmt::skip]
pub use generated::*;

mod generated;
