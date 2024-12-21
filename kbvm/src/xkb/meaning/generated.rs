use super::*;

pub(super) const LONGEST: usize = 25;

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[allow(non_camel_case_types)]
pub(crate) enum Meaning {
    __Unknown,
    Accel,
    Accelerate,
    AccessXFeedback,
    AccessXKeys,
    AccessXTimeout,
    Action,
    ActionMessage,
    Actions,
    Affect,
    Alias,
    All,
    AllOf,
    Allowexplicit,
    Allownone,
    ALPHABETIC,
    AlphanumericKeys,
    Alternate,
    AlternateGroup,
    Any,
    Anylevel,
    AnyOf,
    AnyOfOrNone,
    AudibleBell,
    Augment,
    AutoRepeat,
    Base,
    Both,
    BounceKeys,
    Button,
    Clampgroups,
    ClearLocks,
    Clearmodifiers,
    Clearmods,
    Compat,
    Control,
    Controls,
    Count,
    Ctrls,
    Data,
    Default,
    Defaultbutton,
    Dev,
    DevBtn,
    DevButton,
    Device,
    DeviceBtn,
    DeviceButton,
    DeviceVal,
    DeviceValuator,
    DevVal,
    DevValuator,
    Dfltbtn,
    Driveskbd,
    Driveskeyboard,
    Effective,
    Exactly,
    False,
    FOUR_LEVEL,
    FOUR_LEVEL_ALPHABETIC,
    FOUR_LEVEL_KEYPAD,
    FOUR_LEVEL_SEMIALPHABETIC,
    FunctionKeys,
    GenerateKeyEvent,
    GenKeyEvent,
    Geometry,
    Group,
    Groupname,
    Groups,
    Groupsclamp,
    Groupsredirect,
    Groupswrap,
    Hidden,
    IgnoreGroupLock,
    Include,
    Increment,
    Index,
    Indicator,
    Indicatordriveskbd,
    Indicatordriveskeyboard,
    Interpret,
    ISOLock,
    Kc,
    Key,
    Keycode,
    Keycodes,
    KEYPAD,
    KeypadKeys,
    Keys,
    Latched,
    LatchGroup,
    LatchMods,
    LatchToLock,
    Leddriveskbd,
    Leddriveskeyboard,
    Level1,
    LevelName,
    Levelname,
    Levelone,
    Lock,
    LockControls,
    LockDevBtn,
    LockDevButton,
    LockDeviceBtn,
    LockDeviceButton,
    Locked,
    LockGroup,
    Locking,
    LockMods,
    LockPointerBtn,
    LockPointerButton,
    LockPtrBtn,
    LockPtrButton,
    Locks,
    Logo,
    Map,
    Maximum,
    Message,
    MessageAction,
    Minimum,
    Mod1,
    Mod2,
    Mod3,
    Mod4,
    Mod5,
    ModifierKeys,
    ModifierMap,
    Modifiers,
    ModMap,
    Modmap,
    Modmapmods,
    Mods,
    MouseKeys,
    MouseKeysAccel,
    MovePointer,
    MovePtr,
    Name,
    Neither,
    No,
    NoAction,
    None,
    NoneOf,
    Nosymbol,
    Off,
    On,
    ONE_LEVEL,
    Outline,
    Overlay,
    Overlay1,
    Overlay2,
    Override,
    Partial,
    Permanentradiogroup,
    PointerButton,
    Preserve,
    Private,
    PtrBtn,
    Radiogroup,
    Redirect,
    Redirectgroups,
    RedirectKey,
    Repeat,
    Repeating,
    RepeatKeys,
    Repeats,
    Replace,
    Report,
    Row,
    Same,
    SameServer,
    Screen,
    Section,
    SetControls,
    SetGroup,
    SetMods,
    SetPointerDefault,
    SetPtrDflt,
    Shape,
    Shift,
    SlowKeys,
    Solid,
    StickyKeys,
    SwitchScreen,
    Symbols,
    Terminate,
    TerminateServer,
    Text,
    True,
    TWO_LEVEL,
    Type,
    Types,
    Unlock,
    Usemodmap,
    Usemodmapmods,
    Value,
    Virtual,
    Virtualmod,
    Virtualmodifier,
    VirtualModifiers,
    Virtualmodifiers,
    Virtualmods,
    Vmods,
    Voidsymbol,
    Whichgroupstate,
    Whichmodifierstate,
    Whichmodstate,
    Wrapgroups,
    X,
    XkbCompat,
    XkbCompatibility,
    XkbCompatibilityMap,
    XkbCompatMap,
    XkbGeometry,
    XkbKeycodes,
    XkbKeymap,
    XkbLayout,
    XkbSemantics,
    XkbSymbols,
    XkbTypes,
    Y,
    Yes,
}

impl Meaning {
    pub(crate) fn name(self) -> &'static str {
        match self {
            Self::__Unknown => "__Unknown",
            Self::Accel => "accel",
            Self::Accelerate => "accelerate",
            Self::AccessXFeedback => "AccessXFeedback",
            Self::AccessXKeys => "AccessXKeys",
            Self::AccessXTimeout => "AccessXTimeout",
            Self::Action => "action",
            Self::ActionMessage => "ActionMessage",
            Self::Actions => "actions",
            Self::Affect => "affect",
            Self::Alias => "alias",
            Self::All => "all",
            Self::AllOf => "AllOf",
            Self::Allowexplicit => "allowexplicit",
            Self::Allownone => "allownone",
            Self::ALPHABETIC => "ALPHABETIC",
            Self::AlphanumericKeys => "alphanumeric_keys",
            Self::Alternate => "alternate",
            Self::AlternateGroup => "alternate_group",
            Self::Any => "any",
            Self::Anylevel => "anylevel",
            Self::AnyOf => "AnyOf",
            Self::AnyOfOrNone => "AnyOfOrNone",
            Self::AudibleBell => "AudibleBell",
            Self::Augment => "augment",
            Self::AutoRepeat => "AutoRepeat",
            Self::Base => "base",
            Self::Both => "both",
            Self::BounceKeys => "BounceKeys",
            Self::Button => "button",
            Self::Clampgroups => "clampgroups",
            Self::ClearLocks => "clearLocks",
            Self::Clearmodifiers => "clearmodifiers",
            Self::Clearmods => "clearmods",
            Self::Compat => "compat",
            Self::Control => "Control",
            Self::Controls => "controls",
            Self::Count => "count",
            Self::Ctrls => "ctrls",
            Self::Data => "data",
            Self::Default => "default",
            Self::Defaultbutton => "defaultbutton",
            Self::Dev => "dev",
            Self::DevBtn => "DevBtn",
            Self::DevButton => "DevButton",
            Self::Device => "device",
            Self::DeviceBtn => "DeviceBtn",
            Self::DeviceButton => "DeviceButton",
            Self::DeviceVal => "DeviceVal",
            Self::DeviceValuator => "DeviceValuator",
            Self::DevVal => "DevVal",
            Self::DevValuator => "DevValuator",
            Self::Dfltbtn => "dfltbtn",
            Self::Driveskbd => "driveskbd",
            Self::Driveskeyboard => "driveskeyboard",
            Self::Effective => "effective",
            Self::Exactly => "Exactly",
            Self::False => "false",
            Self::FOUR_LEVEL => "FOUR_LEVEL",
            Self::FOUR_LEVEL_ALPHABETIC => "FOUR_LEVEL_ALPHABETIC",
            Self::FOUR_LEVEL_KEYPAD => "FOUR_LEVEL_KEYPAD",
            Self::FOUR_LEVEL_SEMIALPHABETIC => "FOUR_LEVEL_SEMIALPHABETIC",
            Self::FunctionKeys => "function_keys",
            Self::GenerateKeyEvent => "generateKeyEvent",
            Self::GenKeyEvent => "genKeyEvent",
            Self::Geometry => "geometry",
            Self::Group => "group",
            Self::Groupname => "groupname",
            Self::Groups => "groups",
            Self::Groupsclamp => "groupsclamp",
            Self::Groupsredirect => "groupsredirect",
            Self::Groupswrap => "groupswrap",
            Self::Hidden => "hidden",
            Self::IgnoreGroupLock => "IgnoreGroupLock",
            Self::Include => "include",
            Self::Increment => "increment",
            Self::Index => "index",
            Self::Indicator => "indicator",
            Self::Indicatordriveskbd => "indicatordriveskbd",
            Self::Indicatordriveskeyboard => "indicatordriveskeyboard",
            Self::Interpret => "interpret",
            Self::ISOLock => "ISOLock",
            Self::Kc => "kc",
            Self::Key => "key",
            Self::Keycode => "keycode",
            Self::Keycodes => "keycodes",
            Self::KEYPAD => "KEYPAD",
            Self::KeypadKeys => "keypad_keys",
            Self::Keys => "keys",
            Self::Latched => "latched",
            Self::LatchGroup => "LatchGroup",
            Self::LatchMods => "LatchMods",
            Self::LatchToLock => "latchToLock",
            Self::Leddriveskbd => "leddriveskbd",
            Self::Leddriveskeyboard => "leddriveskeyboard",
            Self::Level1 => "level1",
            Self::LevelName => "level_name",
            Self::Levelname => "levelname",
            Self::Levelone => "levelone",
            Self::Lock => "Lock",
            Self::LockControls => "LockControls",
            Self::LockDevBtn => "LockDevBtn",
            Self::LockDevButton => "LockDevButton",
            Self::LockDeviceBtn => "LockDeviceBtn",
            Self::LockDeviceButton => "LockDeviceButton",
            Self::Locked => "locked",
            Self::LockGroup => "LockGroup",
            Self::Locking => "locking",
            Self::LockMods => "LockMods",
            Self::LockPointerBtn => "LockPointerBtn",
            Self::LockPointerButton => "LockPointerButton",
            Self::LockPtrBtn => "LockPtrBtn",
            Self::LockPtrButton => "LockPtrButton",
            Self::Locks => "locks",
            Self::Logo => "logo",
            Self::Map => "map",
            Self::Maximum => "maximum",
            Self::Message => "Message",
            Self::MessageAction => "MessageAction",
            Self::Minimum => "minimum",
            Self::Mod1 => "Mod1",
            Self::Mod2 => "Mod2",
            Self::Mod3 => "Mod3",
            Self::Mod4 => "Mod4",
            Self::Mod5 => "Mod5",
            Self::ModifierKeys => "modifier_keys",
            Self::ModifierMap => "modifier_map",
            Self::Modifiers => "modifiers",
            Self::ModMap => "mod_map",
            Self::Modmap => "modmap",
            Self::Modmapmods => "modmapmods",
            Self::Mods => "mods",
            Self::MouseKeys => "MouseKeys",
            Self::MouseKeysAccel => "MouseKeysAccel",
            Self::MovePointer => "MovePointer",
            Self::MovePtr => "MovePtr",
            Self::Name => "name",
            Self::Neither => "neither",
            Self::No => "no",
            Self::NoAction => "NoAction",
            Self::None => "none",
            Self::NoneOf => "NoneOf",
            Self::Nosymbol => "nosymbol",
            Self::Off => "off",
            Self::On => "on",
            Self::ONE_LEVEL => "ONE_LEVEL",
            Self::Outline => "outline",
            Self::Overlay => "overlay",
            Self::Overlay1 => "Overlay1",
            Self::Overlay2 => "Overlay2",
            Self::Override => "override",
            Self::Partial => "partial",
            Self::Permanentradiogroup => "permanentradiogroup",
            Self::PointerButton => "PointerButton",
            Self::Preserve => "preserve",
            Self::Private => "Private",
            Self::PtrBtn => "PtrBtn",
            Self::Radiogroup => "radiogroup",
            Self::Redirect => "Redirect",
            Self::Redirectgroups => "redirectgroups",
            Self::RedirectKey => "RedirectKey",
            Self::Repeat => "repeat",
            Self::Repeating => "repeating",
            Self::RepeatKeys => "RepeatKeys",
            Self::Repeats => "repeats",
            Self::Replace => "replace",
            Self::Report => "report",
            Self::Row => "row",
            Self::Same => "same",
            Self::SameServer => "sameServer",
            Self::Screen => "screen",
            Self::Section => "section",
            Self::SetControls => "SetControls",
            Self::SetGroup => "SetGroup",
            Self::SetMods => "SetMods",
            Self::SetPointerDefault => "SetPointerDefault",
            Self::SetPtrDflt => "SetPtrDflt",
            Self::Shape => "shape",
            Self::Shift => "Shift",
            Self::SlowKeys => "SlowKeys",
            Self::Solid => "solid",
            Self::StickyKeys => "StickyKeys",
            Self::SwitchScreen => "SwitchScreen",
            Self::Symbols => "symbols",
            Self::Terminate => "Terminate",
            Self::TerminateServer => "TerminateServer",
            Self::Text => "text",
            Self::True => "true",
            Self::TWO_LEVEL => "TWO_LEVEL",
            Self::Type => "type",
            Self::Types => "types",
            Self::Unlock => "unlock",
            Self::Usemodmap => "usemodmap",
            Self::Usemodmapmods => "usemodmapmods",
            Self::Value => "value",
            Self::Virtual => "virtual",
            Self::Virtualmod => "virtualmod",
            Self::Virtualmodifier => "virtualmodifier",
            Self::VirtualModifiers => "virtual_modifiers",
            Self::Virtualmodifiers => "virtualmodifiers",
            Self::Virtualmods => "virtualmods",
            Self::Vmods => "vmods",
            Self::Voidsymbol => "voidsymbol",
            Self::Whichgroupstate => "whichgroupstate",
            Self::Whichmodifierstate => "whichmodifierstate",
            Self::Whichmodstate => "whichmodstate",
            Self::Wrapgroups => "wrapgroups",
            Self::X => "x",
            Self::XkbCompat => "xkb_compat",
            Self::XkbCompatibility => "xkb_compatibility",
            Self::XkbCompatibilityMap => "xkb_compatibility_map",
            Self::XkbCompatMap => "xkb_compat_map",
            Self::XkbGeometry => "xkb_geometry",
            Self::XkbKeycodes => "xkb_keycodes",
            Self::XkbKeymap => "xkb_keymap",
            Self::XkbLayout => "xkb_layout",
            Self::XkbSemantics => "xkb_semantics",
            Self::XkbSymbols => "xkb_symbols",
            Self::XkbTypes => "xkb_types",
            Self::Y => "y",
            Self::Yes => "yes",
        }
    }

    pub(super) fn lowercase(self) -> &'static str {
        match self {
            Self::__Unknown => "__Unknown",
            Self::Accel => "accel",
            Self::Accelerate => "accelerate",
            Self::AccessXFeedback => "accessxfeedback",
            Self::AccessXKeys => "accessxkeys",
            Self::AccessXTimeout => "accessxtimeout",
            Self::Action => "action",
            Self::ActionMessage => "actionmessage",
            Self::Actions => "actions",
            Self::Affect => "affect",
            Self::Alias => "alias",
            Self::All => "all",
            Self::AllOf => "allof",
            Self::Allowexplicit => "allowexplicit",
            Self::Allownone => "allownone",
            Self::ALPHABETIC => "alphabetic",
            Self::AlphanumericKeys => "alphanumeric_keys",
            Self::Alternate => "alternate",
            Self::AlternateGroup => "alternate_group",
            Self::Any => "any",
            Self::Anylevel => "anylevel",
            Self::AnyOf => "anyof",
            Self::AnyOfOrNone => "anyofornone",
            Self::AudibleBell => "audiblebell",
            Self::Augment => "augment",
            Self::AutoRepeat => "autorepeat",
            Self::Base => "base",
            Self::Both => "both",
            Self::BounceKeys => "bouncekeys",
            Self::Button => "button",
            Self::Clampgroups => "clampgroups",
            Self::ClearLocks => "clearlocks",
            Self::Clearmodifiers => "clearmodifiers",
            Self::Clearmods => "clearmods",
            Self::Compat => "compat",
            Self::Control => "control",
            Self::Controls => "controls",
            Self::Count => "count",
            Self::Ctrls => "ctrls",
            Self::Data => "data",
            Self::Default => "default",
            Self::Defaultbutton => "defaultbutton",
            Self::Dev => "dev",
            Self::DevBtn => "devbtn",
            Self::DevButton => "devbutton",
            Self::Device => "device",
            Self::DeviceBtn => "devicebtn",
            Self::DeviceButton => "devicebutton",
            Self::DeviceVal => "deviceval",
            Self::DeviceValuator => "devicevaluator",
            Self::DevVal => "devval",
            Self::DevValuator => "devvaluator",
            Self::Dfltbtn => "dfltbtn",
            Self::Driveskbd => "driveskbd",
            Self::Driveskeyboard => "driveskeyboard",
            Self::Effective => "effective",
            Self::Exactly => "exactly",
            Self::False => "false",
            Self::FOUR_LEVEL => "four_level",
            Self::FOUR_LEVEL_ALPHABETIC => "four_level_alphabetic",
            Self::FOUR_LEVEL_KEYPAD => "four_level_keypad",
            Self::FOUR_LEVEL_SEMIALPHABETIC => "four_level_semialphabetic",
            Self::FunctionKeys => "function_keys",
            Self::GenerateKeyEvent => "generatekeyevent",
            Self::GenKeyEvent => "genkeyevent",
            Self::Geometry => "geometry",
            Self::Group => "group",
            Self::Groupname => "groupname",
            Self::Groups => "groups",
            Self::Groupsclamp => "groupsclamp",
            Self::Groupsredirect => "groupsredirect",
            Self::Groupswrap => "groupswrap",
            Self::Hidden => "hidden",
            Self::IgnoreGroupLock => "ignoregrouplock",
            Self::Include => "include",
            Self::Increment => "increment",
            Self::Index => "index",
            Self::Indicator => "indicator",
            Self::Indicatordriveskbd => "indicatordriveskbd",
            Self::Indicatordriveskeyboard => "indicatordriveskeyboard",
            Self::Interpret => "interpret",
            Self::ISOLock => "isolock",
            Self::Kc => "kc",
            Self::Key => "key",
            Self::Keycode => "keycode",
            Self::Keycodes => "keycodes",
            Self::KEYPAD => "keypad",
            Self::KeypadKeys => "keypad_keys",
            Self::Keys => "keys",
            Self::Latched => "latched",
            Self::LatchGroup => "latchgroup",
            Self::LatchMods => "latchmods",
            Self::LatchToLock => "latchtolock",
            Self::Leddriveskbd => "leddriveskbd",
            Self::Leddriveskeyboard => "leddriveskeyboard",
            Self::Level1 => "level1",
            Self::LevelName => "level_name",
            Self::Levelname => "levelname",
            Self::Levelone => "levelone",
            Self::Lock => "lock",
            Self::LockControls => "lockcontrols",
            Self::LockDevBtn => "lockdevbtn",
            Self::LockDevButton => "lockdevbutton",
            Self::LockDeviceBtn => "lockdevicebtn",
            Self::LockDeviceButton => "lockdevicebutton",
            Self::Locked => "locked",
            Self::LockGroup => "lockgroup",
            Self::Locking => "locking",
            Self::LockMods => "lockmods",
            Self::LockPointerBtn => "lockpointerbtn",
            Self::LockPointerButton => "lockpointerbutton",
            Self::LockPtrBtn => "lockptrbtn",
            Self::LockPtrButton => "lockptrbutton",
            Self::Locks => "locks",
            Self::Logo => "logo",
            Self::Map => "map",
            Self::Maximum => "maximum",
            Self::Message => "message",
            Self::MessageAction => "messageaction",
            Self::Minimum => "minimum",
            Self::Mod1 => "mod1",
            Self::Mod2 => "mod2",
            Self::Mod3 => "mod3",
            Self::Mod4 => "mod4",
            Self::Mod5 => "mod5",
            Self::ModifierKeys => "modifier_keys",
            Self::ModifierMap => "modifier_map",
            Self::Modifiers => "modifiers",
            Self::ModMap => "mod_map",
            Self::Modmap => "modmap",
            Self::Modmapmods => "modmapmods",
            Self::Mods => "mods",
            Self::MouseKeys => "mousekeys",
            Self::MouseKeysAccel => "mousekeysaccel",
            Self::MovePointer => "movepointer",
            Self::MovePtr => "moveptr",
            Self::Name => "name",
            Self::Neither => "neither",
            Self::No => "no",
            Self::NoAction => "noaction",
            Self::None => "none",
            Self::NoneOf => "noneof",
            Self::Nosymbol => "nosymbol",
            Self::Off => "off",
            Self::On => "on",
            Self::ONE_LEVEL => "one_level",
            Self::Outline => "outline",
            Self::Overlay => "overlay",
            Self::Overlay1 => "overlay1",
            Self::Overlay2 => "overlay2",
            Self::Override => "override",
            Self::Partial => "partial",
            Self::Permanentradiogroup => "permanentradiogroup",
            Self::PointerButton => "pointerbutton",
            Self::Preserve => "preserve",
            Self::Private => "private",
            Self::PtrBtn => "ptrbtn",
            Self::Radiogroup => "radiogroup",
            Self::Redirect => "redirect",
            Self::Redirectgroups => "redirectgroups",
            Self::RedirectKey => "redirectkey",
            Self::Repeat => "repeat",
            Self::Repeating => "repeating",
            Self::RepeatKeys => "repeatkeys",
            Self::Repeats => "repeats",
            Self::Replace => "replace",
            Self::Report => "report",
            Self::Row => "row",
            Self::Same => "same",
            Self::SameServer => "sameserver",
            Self::Screen => "screen",
            Self::Section => "section",
            Self::SetControls => "setcontrols",
            Self::SetGroup => "setgroup",
            Self::SetMods => "setmods",
            Self::SetPointerDefault => "setpointerdefault",
            Self::SetPtrDflt => "setptrdflt",
            Self::Shape => "shape",
            Self::Shift => "shift",
            Self::SlowKeys => "slowkeys",
            Self::Solid => "solid",
            Self::StickyKeys => "stickykeys",
            Self::SwitchScreen => "switchscreen",
            Self::Symbols => "symbols",
            Self::Terminate => "terminate",
            Self::TerminateServer => "terminateserver",
            Self::Text => "text",
            Self::True => "true",
            Self::TWO_LEVEL => "two_level",
            Self::Type => "type",
            Self::Types => "types",
            Self::Unlock => "unlock",
            Self::Usemodmap => "usemodmap",
            Self::Usemodmapmods => "usemodmapmods",
            Self::Value => "value",
            Self::Virtual => "virtual",
            Self::Virtualmod => "virtualmod",
            Self::Virtualmodifier => "virtualmodifier",
            Self::VirtualModifiers => "virtual_modifiers",
            Self::Virtualmodifiers => "virtualmodifiers",
            Self::Virtualmods => "virtualmods",
            Self::Vmods => "vmods",
            Self::Voidsymbol => "voidsymbol",
            Self::Whichgroupstate => "whichgroupstate",
            Self::Whichmodifierstate => "whichmodifierstate",
            Self::Whichmodstate => "whichmodstate",
            Self::Wrapgroups => "wrapgroups",
            Self::X => "x",
            Self::XkbCompat => "xkb_compat",
            Self::XkbCompatibility => "xkb_compatibility",
            Self::XkbCompatibilityMap => "xkb_compatibility_map",
            Self::XkbCompatMap => "xkb_compat_map",
            Self::XkbGeometry => "xkb_geometry",
            Self::XkbKeycodes => "xkb_keycodes",
            Self::XkbKeymap => "xkb_keymap",
            Self::XkbLayout => "xkb_layout",
            Self::XkbSemantics => "xkb_semantics",
            Self::XkbSymbols => "xkb_symbols",
            Self::XkbTypes => "xkb_types",
            Self::Y => "y",
            Self::Yes => "yes",
        }
    }
}

pub(super) static LOWERCASE_TO_MEANING: PhfMap<[u8], Meaning> = PhfMap {
    key: 12913932095322966823,
    disps: &[(0, 0), (0, 36), (0, 20), (0, 24), (0, 94), (0, 4), (0, 0), (0, 2), (0, 3), (1, 9), (0, 138), (0, 55), (0, 171), (2, 0), (0, 73), (1, 5), (0, 8), (0, 2), (0, 16), (0, 1), (0, 6), (0, 0), (0, 0), (0, 1), (0, 29), (0, 13), (0, 1), (0, 3), (0, 0), (0, 59), (0, 19), (0, 3), (0, 107), (0, 4), (3, 41), (0, 17), (0, 32), (0, 84), (0, 23), (0, 22), (0, 5), (0, 36), (1, 0), (0, 13), (0, 43), (0, 36), (0, 9), (0, 79), (0, 85), (0, 51), (0, 0), (1, 56), (0, 106), (0, 25), (0, 0), (0, 3), (0, 84), (1, 60), (0, 134), (0, 33), (0, 33), (1, 181), (0, 85), (1, 13)],
    map: &[Meaning::Repeat, Meaning::Keycodes, Meaning::SetGroup, Meaning::LockDevButton, Meaning::XkbKeycodes, Meaning::AudibleBell, Meaning::SlowKeys, Meaning::GenerateKeyEvent, Meaning::Preserve, Meaning::MovePointer, Meaning::Control, Meaning::FOUR_LEVEL_SEMIALPHABETIC, Meaning::SwitchScreen, Meaning::NoneOf, Meaning::Keys, Meaning::Redirect, Meaning::Clearmodifiers, Meaning::On, Meaning::DevBtn, Meaning::XkbTypes, Meaning::XkbCompatibilityMap, Meaning::SetControls, Meaning::Unlock, Meaning::Levelone, Meaning::FOUR_LEVEL_KEYPAD, Meaning::Key, Meaning::Redirectgroups, Meaning::PtrBtn, Meaning::Wrapgroups, Meaning::AlphanumericKeys, Meaning::Off, Meaning::AnyOfOrNone, Meaning::Groupname, Meaning::All, Meaning::Usemodmap, Meaning::Increment, Meaning::SetPointerDefault, Meaning::Row, Meaning::MessageAction, Meaning::Permanentradiogroup, Meaning::Accel, Meaning::Dev, Meaning::XkbSymbols, Meaning::LockPointerBtn, Meaning::True, Meaning::LockGroup, Meaning::Solid, Meaning::Latched, Meaning::Default, Meaning::Overlay2, Meaning::Indicatordriveskbd, Meaning::MouseKeysAccel, Meaning::RepeatKeys, Meaning::Alias, Meaning::No, Meaning::Shift, Meaning::Groupswrap, Meaning::Geometry, Meaning::Repeating, Meaning::Mod3, Meaning::Type, Meaning::Driveskbd, Meaning::FOUR_LEVEL, Meaning::PointerButton, Meaning::VirtualModifiers, Meaning::Mod5, Meaning::Both, Meaning::XkbCompatMap, Meaning::AccessXTimeout, Meaning::Dfltbtn, Meaning::Locking, Meaning::XkbGeometry, Meaning::Nosymbol, Meaning::Whichgroupstate, Meaning::Virtual, Meaning::Count, Meaning::LatchToLock, Meaning::Mod1, Meaning::XkbKeymap, Meaning::Usemodmapmods, Meaning::XkbCompatibility, Meaning::Leddriveskeyboard, Meaning::Radiogroup, Meaning::Text, Meaning::Device, Meaning::MouseKeys, Meaning::Overlay, Meaning::Groups, Meaning::None, Meaning::Maximum, Meaning::LockPointerButton, Meaning::DeviceBtn, Meaning::LevelName, Meaning::Allowexplicit, Meaning::Lock, Meaning::LockPtrButton, Meaning::Locks, Meaning::Include, Meaning::DevValuator, Meaning::LockControls, Meaning::SameServer, Meaning::Symbols, Meaning::Yes, Meaning::Indicator, Meaning::Types, Meaning::ModifierKeys, Meaning::Augment, Meaning::Virtualmodifiers, Meaning::Defaultbutton, Meaning::ActionMessage, Meaning::DeviceButton, Meaning::X, Meaning::Logo, Meaning::Private, Meaning::Minimum, Meaning::Controls, Meaning::Vmods, Meaning::Accelerate, Meaning::KEYPAD, Meaning::Name, Meaning::Data, Meaning::KeypadKeys, Meaning::Driveskeyboard, Meaning::AnyOf, Meaning::Outline, Meaning::Replace, Meaning::Section, Meaning::Hidden, Meaning::Any, Meaning::Neither, Meaning::Alternate, Meaning::Repeats, Meaning::NoAction, Meaning::SetMods, Meaning::ModifierMap, Meaning::False, Meaning::Partial, Meaning::Terminate, Meaning::Override, Meaning::TerminateServer, Meaning::Value, Meaning::Whichmodstate, Meaning::StickyKeys, Meaning::Overlay1, Meaning::Clampgroups, Meaning::MovePtr, Meaning::XkbLayout, Meaning::AlternateGroup, Meaning::Modifiers, Meaning::DevButton, Meaning::Same, Meaning::Screen, Meaning::Shape, Meaning::Button, Meaning::ONE_LEVEL, Meaning::Virtualmod, Meaning::Map, Meaning::Affect, Meaning::TWO_LEVEL, Meaning::Leddriveskbd, Meaning::Message, Meaning::Groupsredirect, Meaning::Effective, Meaning::Clearmods, Meaning::DeviceValuator, Meaning::Voidsymbol, Meaning::LatchGroup, Meaning::AllOf, Meaning::Kc, Meaning::ModMap, Meaning::Action, Meaning::Mods, Meaning::Exactly, Meaning::Virtualmodifier, Meaning::Levelname, Meaning::Virtualmods, Meaning::DeviceVal, Meaning::Base, Meaning::ClearLocks, Meaning::AccessXFeedback, Meaning::Y, Meaning::Allownone, Meaning::LockDeviceButton, Meaning::FunctionKeys, Meaning::XkbSemantics, Meaning::XkbCompat, Meaning::Anylevel, Meaning::ALPHABETIC, Meaning::Keycode, Meaning::Modmap, Meaning::FOUR_LEVEL_ALPHABETIC, Meaning::ISOLock, Meaning::SetPtrDflt, Meaning::GenKeyEvent, Meaning::RedirectKey, Meaning::Groupsclamp, Meaning::Ctrls, Meaning::BounceKeys, Meaning::LockDevBtn, Meaning::AccessXKeys, Meaning::LatchMods, Meaning::Locked, Meaning::LockMods, Meaning::Actions, Meaning::Mod2, Meaning::Compat, Meaning::Mod4, Meaning::AutoRepeat, Meaning::Index, Meaning::IgnoreGroupLock, Meaning::Group, Meaning::Report, Meaning::Indicatordriveskeyboard, Meaning::Level1, Meaning::DevVal, Meaning::Interpret, Meaning::Modmapmods, Meaning::LockPtrBtn, Meaning::LockDeviceBtn, Meaning::Whichmodifierstate],
    _phantom: core::marker::PhantomData,
};

pub(super) static STRING_TO_MEANING: PhfMap<[u8], Meaning> = PhfMap {
    key: 12913932095322966823,
    disps: &[(0, 1), (0, 9), (0, 0), (0, 29), (0, 43), (0, 20), (0, 12), (0, 16), (0, 13), (0, 19), (0, 0), (0, 0), (0, 24), (0, 0), (0, 79), (0, 53), (0, 3), (0, 0), (0, 16), (0, 7), (0, 32), (0, 2), (0, 21), (0, 3), (0, 48), (0, 18), (0, 0), (0, 0), (0, 57), (0, 17), (0, 2), (0, 37), (0, 34), (0, 10), (0, 5), (0, 30), (1, 1), (0, 45), (0, 0), (0, 9), (0, 12), (2, 139), (1, 2), (0, 62), (0, 9), (0, 13), (0, 159), (0, 1), (0, 201), (0, 169), (0, 3), (0, 101), (0, 60), (0, 5), (0, 29), (0, 60), (6, 177), (0, 8), (0, 33), (0, 120), (4, 170), (0, 72), (0, 0), (2, 129)],
    map: &[Meaning::Indicatordriveskeyboard, Meaning::Repeat, Meaning::False, Meaning::Clearmodifiers, Meaning::Indicatordriveskbd, Meaning::Virtualmodifiers, Meaning::Redirectgroups, Meaning::SetGroup, Meaning::Mod1, Meaning::XkbKeycodes, Meaning::Preserve, Meaning::Driveskbd, Meaning::Control, Meaning::Leddriveskeyboard, Meaning::Geometry, Meaning::Kc, Meaning::AudibleBell, Meaning::Modmapmods, Meaning::Terminate, Meaning::LockPointerBtn, Meaning::RepeatKeys, Meaning::True, Meaning::XkbCompatibilityMap, Meaning::Symbols, Meaning::Shape, Meaning::XkbTypes, Meaning::PtrBtn, Meaning::Neither, Meaning::Default, Meaning::StickyKeys, Meaning::Wrapgroups, Meaning::Lock, Meaning::Value, Meaning::LatchToLock, Meaning::FOUR_LEVEL_SEMIALPHABETIC, Meaning::Overlay2, Meaning::Usemodmap, Meaning::Levelone, Meaning::Groupname, Meaning::SetMods, Meaning::Type, Meaning::DeviceVal, Meaning::Accel, Meaning::AlphanumericKeys, Meaning::Latched, Meaning::Permanentradiogroup, Meaning::None, Meaning::Dev, Meaning::TWO_LEVEL, Meaning::SameServer, Meaning::ALPHABETIC, Meaning::Message, Meaning::LatchGroup, Meaning::FOUR_LEVEL_KEYPAD, Meaning::MessageAction, Meaning::Groupswrap, Meaning::NoAction, Meaning::No, Meaning::LevelName, Meaning::ModifierKeys, Meaning::KEYPAD, Meaning::Index, Meaning::Solid, Meaning::Modifiers, Meaning::LockGroup, Meaning::Alias, Meaning::Action, Meaning::MovePtr, Meaning::XkbSymbols, Meaning::Both, Meaning::SetPointerDefault, Meaning::LockPointerButton, Meaning::FOUR_LEVEL, Meaning::XkbKeymap, Meaning::Dfltbtn, Meaning::Nosymbol, Meaning::Virtual, Meaning::XkbGeometry, Meaning::IgnoreGroupLock, Meaning::LockMods, Meaning::LockPtrButton, Meaning::AutoRepeat, Meaning::Redirect, Meaning::Whichgroupstate, Meaning::Repeating, Meaning::RedirectKey, Meaning::Text, Meaning::Radiogroup, Meaning::LockDevBtn, Meaning::Device, Meaning::All, Meaning::LatchMods, Meaning::Repeats, Meaning::Overlay, Meaning::LockDevButton, Meaning::BounceKeys, Meaning::LockDeviceBtn, Meaning::Accelerate, Meaning::Locks, Meaning::Groups, Meaning::Yes, Meaning::ActionMessage, Meaning::ONE_LEVEL, Meaning::Section, Meaning::AnyOfOrNone, Meaning::Indicator, Meaning::Types, Meaning::DeviceBtn, Meaning::Anylevel, Meaning::Group, Meaning::Report, Meaning::Allownone, Meaning::Replace, Meaning::Virtualmod, Meaning::X, Meaning::Mod2, Meaning::XkbCompatMap, Meaning::Vmods, Meaning::Actions, Meaning::Data, Meaning::Mod4, Meaning::Controls, Meaning::ClearLocks, Meaning::Outline, Meaning::AccessXTimeout, Meaning::Hidden, Meaning::Voidsymbol, Meaning::AlternateGroup, Meaning::Exactly, Meaning::XkbLayout, Meaning::Button, Meaning::ModifierMap, Meaning::Count, Meaning::Minimum, Meaning::Defaultbutton, Meaning::Driveskeyboard, Meaning::Leddriveskbd, Meaning::Allowexplicit, Meaning::Any, Meaning::Whichmodstate, Meaning::AllOf, Meaning::AnyOf, Meaning::Interpret, Meaning::Base, Meaning::MouseKeysAccel, Meaning::FOUR_LEVEL_ALPHABETIC, Meaning::Clampgroups, Meaning::ISOLock, Meaning::Row, Meaning::AccessXKeys, Meaning::Augment, Meaning::Increment, Meaning::Locking, Meaning::Name, Meaning::SetPtrDflt, Meaning::KeypadKeys, Meaning::Affect, Meaning::Shift, Meaning::Clearmods, Meaning::Include, Meaning::Key, Meaning::NoneOf, Meaning::Map, Meaning::Effective, Meaning::On, Meaning::MovePointer, Meaning::ModMap, Meaning::PointerButton, Meaning::DevBtn, Meaning::MouseKeys, Meaning::LockPtrBtn, Meaning::Maximum, Meaning::Overlay1, Meaning::Virtualmodifier, Meaning::Levelname, Meaning::Virtualmods, Meaning::Logo, Meaning::LockControls, Meaning::Groupsclamp, Meaning::Ctrls, Meaning::Screen, Meaning::GenKeyEvent, Meaning::VirtualModifiers, Meaning::Mod3, Meaning::Mods, Meaning::XkbSemantics, Meaning::DevButton, Meaning::Alternate, Meaning::XkbCompat, Meaning::FunctionKeys, Meaning::Groupsredirect, Meaning::SetControls, Meaning::AccessXFeedback, Meaning::Mod5, Meaning::Y, Meaning::Override, Meaning::DevVal, Meaning::DeviceButton, Meaning::Keycode, Meaning::Off, Meaning::SlowKeys, Meaning::Modmap, Meaning::Locked, Meaning::Same, Meaning::Keycodes, Meaning::Usemodmapmods, Meaning::LockDeviceButton, Meaning::Private, Meaning::Unlock, Meaning::DeviceValuator, Meaning::DevValuator, Meaning::Keys, Meaning::Level1, Meaning::Partial, Meaning::TerminateServer, Meaning::XkbCompatibility, Meaning::GenerateKeyEvent, Meaning::SwitchScreen, Meaning::Whichmodifierstate, Meaning::Compat],
    _phantom: core::marker::PhantomData,
};
