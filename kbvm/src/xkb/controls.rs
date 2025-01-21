#[derive(Copy, Clone, Default, PartialEq, Eq)]
pub(crate) struct ControlMask(pub(crate) u16);

keyed_bitfield! {
    ControlMask:
    0 => REPEAT_KEYS => RepeatKeys | Repeat | AutoRepeat,
    1 => SLOW_KEYS => SlowKeys,
    2 => BOUNCE_KEYS => BounceKeys,
    3 => STICKY_KEYS => StickyKeys,
    4 => MOUSE_KEYS => MouseKeys,
    5 => MOUSE_KEYS_ACCEL => MouseKeysAccel,
    6 => ACCESS_X_KEYS => AccessXKeys,
    7 => ACCESS_X_TIMEOUT => AccessXTimeout,
    8 => ACCESS_X_FEEDBACK => AccessXFeedback,
    9 => AUDIBLE_BELL => AudibleBell,
    10 => OVERLAY1 => Overlay1,
    11 => OVERLAY2 => Overlay2,
    12 => IGNORE_GROUP_LOCK => IgnoreGroupLock,
}
