use {
    crate::generate_map,
    std::fmt::{Debug, Formatter},
};

struct Meaning {
    orig: &'static str,
    lowercase: String,
    camel: String,
}

pub fn main() {
    let output = generate_output();
    std::fs::write("kbvm/src/xkb/meaning/generated.rs", output).unwrap();
}

const STRINGS: [&str; 227] = [
    "accel",
    "accelerate",
    "AccessXFeedback",
    "AccessXKeys",
    "AccessXTimeout",
    "action",
    "ActionMessage",
    "actions",
    "affect",
    "alias",
    "all",
    "AllOf",
    "allowexplicit",
    "allownone",
    "ALPHABETIC",
    "alphanumeric_keys",
    "Alt",
    "alternate",
    "alternate_group",
    "<any>",
    "any",
    "anylevel",
    "AnyOf",
    "AnyOfOrNone",
    "AudibleBell",
    "augment",
    "AutoRepeat",
    "base",
    "both",
    "BounceKeys",
    "button",
    "Caps",
    "clampgroups",
    "clearLocks",
    "clearmodifiers",
    "clearmods",
    "compat",
    "Control",
    "controls",
    "count",
    "ctrl",
    "ctrls",
    "data",
    "default",
    "defaultbutton",
    "dev",
    "DevBtn",
    "DevButton",
    "device",
    "DeviceBtn",
    "DeviceButton",
    "DeviceVal",
    "DeviceValuator",
    "DevVal",
    "DevValuator",
    "dfltbtn",
    "driveskbd",
    "driveskeyboard",
    "effective",
    "Exactly",
    "false",
    "FOUR_LEVEL",
    "FOUR_LEVEL_ALPHABETIC",
    "FOUR_LEVEL_KEYPAD",
    "FOUR_LEVEL_SEMIALPHABETIC",
    "function_keys",
    "generateKeyEvent",
    "genKeyEvent",
    "geometry",
    "group",
    "groupname",
    "groups",
    "groupsclamp",
    "groupsredirect",
    "groupswrap",
    "hidden",
    "IgnoreGroupLock",
    "include",
    "increment",
    "index",
    "indicator",
    "indicatordriveskbd",
    "indicatordriveskeyboard",
    "interpret",
    "ISOLock",
    "kc",
    "key",
    "keycode",
    "keycodes",
    "KEYPAD",
    "keypad_keys",
    "keys",
    "latched",
    "LatchGroup",
    "LatchMods",
    "latchToLock",
    "leddriveskbd",
    "leddriveskeyboard",
    "level1",
    "level_name",
    "levelname",
    "levelone",
    "Lock",
    "LockControls",
    "LockDevBtn",
    "LockDevButton",
    "LockDeviceBtn",
    "LockDeviceButton",
    "locked",
    "LockGroup",
    "locking",
    "LockMods",
    "LockPointerBtn",
    "LockPointerButton",
    "LockPtrBtn",
    "LockPtrButton",
    "locks",
    "logo",
    "map",
    "maximum",
    "Message",
    "MessageAction",
    "Meta",
    "minimum",
    "Mod1",
    "Mod2",
    "Mod3",
    "Mod4",
    "Mod5",
    "modifier_keys",
    "modifier_map",
    "modifiers",
    "mod_map",
    "modmap",
    "modmapmods",
    "mods",
    "MouseKeys",
    "MouseKeysAccel",
    "MovePointer",
    "MovePtr",
    "name",
    "neither",
    "no",
    "NoAction",
    "<none>",
    "none",
    "NoneOf",
    "nosymbol",
    "off",
    "on",
    "ONE_LEVEL",
    "outline",
    "overlay",
    "Overlay1",
    "Overlay2",
    "override",
    "partial",
    "permanentradiogroup",
    "PointerButton",
    "preserve",
    "Private",
    "PtrBtn",
    "radiogroup",
    "Redirect",
    "redirectgroups",
    "RedirectKey",
    "repeat",
    "repeating",
    "RepeatKeys",
    "repeats",
    "replace",
    "report",
    "row",
    "same",
    "sameServer",
    "screen",
    "section",
    "SetControls",
    "SetGroup",
    "SetMods",
    "SetPointerDefault",
    "SetPtrDflt",
    "shape",
    "Shift",
    "SlowKeys",
    "solid",
    "<some>",
    "StickyKeys",
    "SwitchScreen",
    "symbols",
    "Terminate",
    "TerminateServer",
    "text",
    "true",
    "TWO_LEVEL",
    "type",
    "types",
    "unlock",
    "usemodmap",
    "usemodmapmods",
    "value",
    "virtual",
    "virtualmod",
    "virtualmodifier",
    "virtual_modifiers",
    "virtualmodifiers",
    "virtualmods",
    "vmods",
    "voidsymbol",
    "whichgroupstate",
    "whichmodifierstate",
    "whichmodstate",
    "wrapgroups",
    "x",
    "xkb_compat",
    "xkb_compatibility",
    "xkb_compatibility_map",
    "xkb_compat_map",
    "xkb_geometry",
    "xkb_keycodes",
    "xkb_keymap",
    "xkb_layout",
    "xkb_semantics",
    "xkb_symbols",
    "xkb_types",
    "y",
    "yes",
];

fn generate_output() -> String {
    let mut meanings = vec![];
    for string in STRINGS {
        meanings.push(Meaning {
            orig: string,
            lowercase: string.to_ascii_lowercase(),
            camel: to_camel(string),
        });
    }
    let longest = generate_longest(&meanings);
    let enum_ = generate_enum(&meanings);
    let lower_map = generate_lowercase_to_meaning(&meanings);
    let orig_map = generate_string_to_meaning(&meanings);
    let mut res = String::new();
    res.push_str("use super::*;\n");
    res.push_str("\n");
    res.push_str(&longest);
    res.push_str("\n");
    res.push_str(&enum_);
    res.push_str("\n");
    res.push_str(&lower_map);
    res.push_str("\n");
    res.push_str(&orig_map);
    res
}

fn generate_longest(meanings: &[Meaning]) -> String {
    let len = meanings.iter().map(|m| m.orig.len()).max().unwrap();
    format!("pub(super) const LONGEST: usize = {len};\n")
}

struct EnumDebug<'a>(&'a Meaning);
impl Debug for EnumDebug<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "Meaning::{}", self.0.camel)
    }
}

fn generate_string_to_meaning(meanings: &[Meaning]) -> String {
    let keys: Vec<_> = meanings
        .iter()
        .map(|meaning| meaning.orig.as_bytes())
        .collect();
    let mut values: Vec<_> = meanings.iter().map(EnumDebug).collect();
    generate_map("STRING_TO_MEANING", "[u8]", "Meaning", &keys, &mut values)
}

fn generate_lowercase_to_meaning(meanings: &[Meaning]) -> String {
    let keys: Vec<_> = meanings
        .iter()
        .map(|meaning| meaning.lowercase.as_bytes())
        .collect();
    let mut values: Vec<_> = meanings.iter().map(EnumDebug).collect();
    generate_map(
        "LOWERCASE_TO_MEANING",
        "[u8]",
        "Meaning",
        &keys,
        &mut values,
    )
}

fn to_camel(s: &str) -> String {
    let mut res = String::new();
    let mut is_first = true;
    for c in s.chars() {
        if c == '_' {
            is_first = true;
        } else if c == '<' {
            is_first = true;
            res.push_str("LessThan_");
        } else if c == '>' {
            is_first = true;
            res.push_str("_GreaterThan");
        } else if is_first {
            if c.is_ascii_uppercase() && !res.is_empty() {
                res.push('_');
                res.push(c);
            } else {
                res.push(c.to_ascii_uppercase());
            }
            is_first = false;
        } else {
            res.push(c);
        }
    }
    res
}

fn generate_enum(meanings: &[Meaning]) -> String {
    use std::fmt::Write;
    let mut res = String::new();
    res.push_str("#[derive(Copy, Clone, Eq, PartialEq, Debug)]\n");
    res.push_str("#[allow(non_camel_case_types)]\n");
    res.push_str("pub(crate) enum Meaning {\n");
    res.push_str("    __Unknown,\n");
    for meaning in meanings {
        writeln!(res, "    {},", meaning.camel).unwrap();
    }
    res.push_str("}\n");
    res.push_str("\n");
    res.push_str("impl Meaning {\n");
    res.push_str("    pub(crate) fn name(self) -> &'static str {\n");
    res.push_str("        match self {\n");
    res.push_str("            Self::__Unknown => \"__Unknown\",\n");
    for meaning in meanings {
        writeln!(
            res,
            "            Self::{} => \"{}\",",
            meaning.camel, meaning.orig
        )
        .unwrap();
    }
    res.push_str("        }\n");
    res.push_str("    }\n");
    res.push_str("\n");
    res.push_str("    pub(super) fn lowercase(self) -> &'static str {\n");
    res.push_str("        match self {\n");
    res.push_str("            Self::__Unknown => \"__Unknown\",\n");
    for meaning in meanings {
        writeln!(
            res,
            "            Self::{} => \"{}\",",
            meaning.camel, meaning.lowercase
        )
        .unwrap();
    }
    res.push_str("        }\n");
    res.push_str("    }\n");
    res.push_str("}\n");
    res
}
