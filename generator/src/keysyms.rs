use {
    crate::generate_map,
    indexmap::IndexMap,
    regex::Regex,
    std::{
        borrow::Cow,
        collections::{hash_map::Entry, HashMap},
        fmt::{Debug, Formatter},
    },
};

pub fn main() {
    let output = generate_output();
    std::fs::write("kbvm/src/keysym/generated.rs", output).unwrap();
}

fn generate_output() -> String {
    use std::fmt::Write;
    let mut keysyms = IndexMap::new();
    handle_header(&mut keysyms);
    handle_yaml(&mut keysyms);
    validate(&keysyms);
    keysyms.sort_unstable_by(|l, _, r, _| l.cmp(&r));
    assign_indices(&mut keysyms);
    let longest_keysym_name = get_longest_keysym_name(&keysyms);
    let lowercase = create_lowercase_mapping(&keysyms);
    let keysym_to_idx = generate_keysym_to_idx(&keysyms);
    let name_to_idx = generate_name_to_idx(&keysyms);
    let lower_name_to_idx = generate_lower_name_to_idx(&lowercase);
    let keysym_to_lower = generate_keysym_to_other_case_keysym(&keysyms, false);
    let keysym_to_upper = generate_keysym_to_other_case_keysym(&keysyms, true);
    let idx_to_char = generate_idx_to_char(&keysyms);
    let char_to_bespoke_idx = generate_char_to_bespoke_idx(&keysyms);
    let names = generate_names(&mut keysyms);
    let datas = generate_datas(&keysyms);
    let sym_consts = generate_sym_consts(&keysyms);
    let mut res = String::new();
    writeln!(res, "use super::*;\n").unwrap();
    writeln!(res, "#[cfg(test)]").unwrap();
    writeln!(res, "pub(super) const LEN: usize = {};\n", keysyms.len()).unwrap();
    writeln!(
        res,
        "pub(super) const LONGEST_NAME: usize = {longest_keysym_name};\n"
    )
    .unwrap();
    writeln!(res, "{names}").unwrap();
    writeln!(res, "{keysym_to_idx}").unwrap();
    writeln!(res, "{name_to_idx}").unwrap();
    writeln!(res, "{lower_name_to_idx}").unwrap();
    writeln!(res, "{keysym_to_upper}").unwrap();
    writeln!(res, "{keysym_to_lower}").unwrap();
    writeln!(res, "{idx_to_char}").unwrap();
    writeln!(res, "{char_to_bespoke_idx}").unwrap();
    writeln!(res, "{datas}").unwrap();
    writeln!(res, "{sym_consts}").unwrap();
    res
}

#[derive(Debug)]
struct KeysymInfo {
    keysym: u32,
    definitive_name: Option<&'static str>,
    code_point: Option<u32>,
    lower: Option<u32>,
    upper: Option<u32>,
    is_lower: bool,
    is_upper: bool,
    names: Vec<KeysymName>,
    definitive_idx: u16,
}

#[derive(Debug)]
struct KeysymName {
    name: Cow<'static, str>,
    idx: u16,
    name_start: usize,
    name_len: usize,
    alias_for: Option<&'static str>,
    deprecated: bool,
    #[expect(dead_code)]
    deprecation_reason: Option<&'static str>,
}

fn handle_header(output: &mut IndexMap<u32, KeysymInfo>) {
    let header = include_str!("../../libxkbcommon/include/xkbcommon/xkbcommon-keysyms.h");
    let regex = Regex::new(
        r#"(?xm)
        ^\#define\s+
        XKB_KEY_(?<name>\w+)\s+
        0x(?<id>[a-fA-F0-9]+)\b\s*
        (
            /\*\s*
            (
                (
                    (?<non_deprecated>non-)?(?<deprecated1>deprecated)
                    (\s+alias\s+for\s+(?<alias1>\w+))?
                    (\s+\((?<reason>[^)]+)\))?
                )
                |
                (
                    Same\s+as\s+XKB_KEY_(?<alias2>\w+)
                )
                |
                (?<deprecated2>
                    \(<?U
                )
            )
        )?
        "#,
    )
    .unwrap();
    for captures in regex.captures_iter(&header) {
        let name = captures.name("name").unwrap().as_str();
        let id = &captures["id"];
        let id = u32::from_str_radix(id, 16).unwrap();
        let deprecated = (captures.name("deprecated1").is_some()
            && captures.name("non_deprecated").is_none())
            || captures.name("deprecated2").is_some();
        let alias = captures
            .name("alias1")
            .or(captures.name("alias2"))
            .map(|m| m.as_str());
        let reason = captures.name("reason").map(|m| m.as_str());
        let infos = output.entry(id).or_insert(KeysymInfo {
            keysym: id,
            definitive_name: None,
            code_point: None,
            lower: None,
            upper: None,
            is_lower: false,
            is_upper: false,
            names: vec![],
            definitive_idx: 0,
        });
        infos.names.push(KeysymName {
            name: name.into(),
            idx: 0,
            alias_for: alias,
            deprecated,
            deprecation_reason: reason,
            name_start: 0,
            name_len: 0,
        });
        // if let Some(suffix) = name.strip_prefix("XF86") {
        //     infos.names.push(KeysymName {
        //         name: format!("XF86_{suffix}").into(),
        //         idx: 0,
        //         alias_for: Some(name),
        //         deprecated,
        //         deprecation_reason: reason,
        //     });
        // }
    }
}

fn handle_yaml(output: &mut IndexMap<u32, KeysymInfo>) {
    let yaml = include_str!("../../libxkbcommon/data/keysyms.yaml");
    let mut lines = yaml
        .lines()
        .filter_map(|mut l| {
            if let Some((v, _)) = l.split_once("#") {
                l = v;
            }
            l = l.trim_ascii_end();
            if l.is_empty() {
                None
            } else {
                Some(l)
            }
        })
        .peekable();
    while let Some(keysym) = lines.next() {
        let Some(keysym) = keysym.strip_prefix("0x") else {
            unreachable!("missing leading 0x in line {keysym:?}");
        };
        let Some(keysym) = keysym.strip_suffix(":") else {
            unreachable!("missing trailing : in line {keysym:?}");
        };
        let keysym = u32::from_str_radix(keysym, 16).unwrap();
        let mut name = None;
        let mut code_point = None;
        let mut lower = None;
        let mut upper = None;
        while let Some(line) = lines.peek() {
            let Some(line) = line.strip_prefix("  ") else {
                break;
            };
            lines.next();
            let Some((key, value)) = line.split_once(": ") else {
                unreachable!("missing : in line {line:?}");
            };
            let hex = |s: &str| {
                let Some(s) = s.strip_prefix("0x") else {
                    return None;
                };
                let Ok(val) = u32::from_str_radix(s, 16) else {
                    return None;
                };
                Some(val)
            };
            match key {
                "name" => name = Some(value),
                "code point" => {
                    let Some(val) = hex(value) else {
                        unreachable!("Unexpected code point {value:?}");
                    };
                    code_point = Some(val);
                }
                "lower" => {
                    let Some(val) = hex(value) else {
                        unreachable!("Unexpected lower {value:?}");
                    };
                    lower = Some(val);
                }
                "upper" => {
                    let Some(val) = hex(value) else {
                        unreachable!("Unexpected upper {value:?}");
                    };
                    upper = Some(val);
                }
                _ => unreachable!("Unexpected key `{key:?}`"),
            }
        }
        let Some(name) = name else {
            unreachable!("Missing name field");
        };
        let Some(info) = output.get_mut(&keysym) else {
            unreachable!("keysym.yaml contains keysym 0x{keysym:x} not present in the header");
        };
        info.definitive_name = Some(name);
        info.code_point = code_point;
        info.lower = lower;
        info.upper = upper;
        if let Some(cp) = code_point {
            if let Some(c) = char::from_u32(cp) {
                info.is_lower = c.is_lowercase();
                info.is_upper = c.is_uppercase();
            }
        }
    }
}

fn validate(output: &IndexMap<u32, KeysymInfo>) {
    let mut names = HashMap::new();
    for v in output.values() {
        if v.definitive_name.is_none() {
            unreachable!("Keysym is missing a definitive name: {v:#x?}");
        }
        if v.lower.is_some() && v.upper.is_some() {
            unreachable!("Keysym has both lower and upper variants: {v:#x?}");
        }
        if let Some(casing) = v.lower.or(v.upper) {
            if v.keysym >> 24 != 0x01 && v.keysym > 0xff_ff {
                unreachable!("Large non-unicode keysym has casing: {v:#x?}");
            }
            if casing >> 24 != 0x01 && !output.contains_key(&casing) {
                unreachable!("Other case does not exist: {v:#x?}");
            }
        }
        for name in &v.names {
            match names.entry(name.name.as_ref()) {
                Entry::Occupied(e) => {
                    unreachable!(
                        "Keysym name has been re-used {v:#x?} previously {:#x?}",
                        e.get()
                    );
                }
                Entry::Vacant(e) => {
                    e.insert(v);
                }
            }
        }
        if let Some(cp) = v.code_point {
            if char::from_u32(cp).is_none() {
                unreachable!("Keysym code point is invalid: {v:#x?}");
            }
        }
        if v.keysym >> 24 == 0x01 {
            let cp = v.keysym << 8 >> 8;
            if v.code_point != Some(cp) {
                unreachable!("Unicode keysym has a weird code point: {v:#x?}");
            }
            let Some(char) = char::from_u32(cp) else {
                unreachable!("Unicode keysym is not a valid code point: {v:#x?}");
            };
            macro_rules! check_casing {
                ($f:ident, $field:ident) => {
                    let mut transformed: Vec<_> = char.$f().collect();
                    if transformed.len() == 1 && transformed[0] == char {
                        transformed.clear();
                    }
                    if transformed.len() == 1 {
                        let c = transformed[0] as u32 | 0x01_00_00_00;
                        if v.$field != Some(c) {
                            unreachable!(
                                "Expected {} variant {c:x} for keysym: {v:#x?}",
                                stringify!($field)
                            );
                        }
                    } else {
                        if v.$field.is_some() {
                            unreachable!(
                                "Expected no {} variant for keysym: {v:#x?}",
                                stringify!($field)
                            );
                        }
                    }
                };
            }
            check_casing!(to_uppercase, upper);
            check_casing!(to_lowercase, lower);
        } else {
            if v.code_point.is_some() && v.keysym > 0xff_ff {
                unreachable!("Bespoke keysym with code point does not fit into u16: {v:#x?}");
            }
        }
    }
    for v in output.values() {
        let mut have_definitive_name = false;
        for name in &v.names {
            if name.name == v.definitive_name.unwrap() {
                have_definitive_name = true;
            }
            if let Some(alias) = name.alias_for {
                match names.get(&alias) {
                    None => {
                        unreachable!("Keysym alias {alias:?} does not exist: {v:#x?}");
                    }
                    Some(a) => {
                        if a.keysym != v.keysym {
                            unreachable!(
                                "Keysym name is alias for different keysym: {v:#x?}, {a:#x?}"
                            );
                        }
                    }
                }
            }
        }
        if !have_definitive_name {
            unreachable!("The definitive name is not one of the keysym names: {v:#x?}");
        }
    }
}

fn assign_indices(output: &mut IndexMap<u32, KeysymInfo>) {
    let mut i = 0;
    for ks in output.values_mut() {
        for name in &mut ks.names {
            name.idx = i;
            if name.name == ks.definitive_name.unwrap() {
                ks.definitive_idx = i;
            }
            i += 1;
        }
    }
}

fn get_longest_keysym_name(output: &IndexMap<u32, KeysymInfo>) -> usize {
    output
        .values()
        .flat_map(|v| v.names.iter().map(|n| n.name.len()))
        .max()
        .unwrap_or_default()
}

fn create_lowercase_mapping(input: &IndexMap<u32, KeysymInfo>) -> HashMap<String, u16> {
    let mut out = HashMap::new();
    for v in input.values() {
        for kname in &v.names {
            let name = kname.name.as_ref();
            match out.entry(name.to_ascii_lowercase()) {
                Entry::Vacant(e) => {
                    e.insert((kname, name));
                }
                Entry::Occupied(mut o) => {
                    if name > o.get().1 {
                        o.insert((kname, name));
                    }
                }
            }
        }
    }
    out.into_iter().map(|(k, v)| (k, v.0.idx)).collect()
}

fn generate_name_to_idx(output: &IndexMap<u32, KeysymInfo>) -> String {
    let mut keys = vec![];
    let mut values = vec![];
    for v in output.values() {
        for n in &v.names {
            keys.push(n.name.as_bytes());
            values.push(n.idx);
        }
    }
    generate_map("NAME_TO_IDX", "[u8]", "u16", &keys, &mut values)
}

fn generate_lower_name_to_idx(output: &HashMap<String, u16>) -> String {
    let mut keys = vec![];
    let mut values = vec![];
    for (name, idx) in output {
        keys.push(name.as_bytes());
        values.push(*idx);
    }
    generate_map("LOWER_NAME_TO_IDX", "[u8]", "u16", &keys, &mut values)
}

fn generate_keysym_to_idx(output: &IndexMap<u32, KeysymInfo>) -> String {
    let mut keys = vec![];
    let mut values = vec![];
    for v in output.values() {
        keys.push(v.keysym);
        values.push(v.definitive_idx);
    }
    generate_map("KEYSYM_TO_IDX", "u32", "u16", &keys, &mut values)
}

fn generate_keysym_to_other_case_keysym(output: &IndexMap<u32, KeysymInfo>, upper: bool) -> String {
    struct MappingDebugger(u32, u32);
    impl Debug for MappingDebugger {
        fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
            write!(
                f,
                "KeysymCaseMapping {{ keysym: 0x{:04x}, other: 0x{:08x} }}",
                self.0, self.1
            )
        }
    }
    let mut keys = vec![];
    let mut values = vec![];
    for v in output.values() {
        if v.keysym < 0x80 {
            continue;
        }
        if v.keysym > 0xff_ff {
            continue;
        }
        let o = match upper {
            false => v.lower,
            true => v.upper,
        };
        let Some(o) = o else {
            continue;
        };
        keys.push(v.keysym);
        values.push(MappingDebugger(v.keysym, o));
    }
    let name = match upper {
        false => "KEYSYM_TO_LOWER_KEYSYM",
        true => "KEYSYM_TO_UPPER_KEYSYM",
    };
    generate_map(name, "u32", "KeysymCaseMapping", &keys, &mut values)
}

fn generate_idx_to_char(output: &IndexMap<u32, KeysymInfo>) -> String {
    use std::fmt::Write;
    let mut values: Vec<_> = output
        .values()
        .filter_map(|v| {
            if v.keysym > 0xffff {
                return None;
            }
            v.code_point.map(|c| (v.keysym, c))
        })
        .collect();
    values.sort_by_key(|v| v.0);
    let mut res = String::new();
    writeln!(res, "pub(super) static KEYSYM_TO_CHAR: &[KeysymChar] = &[").unwrap();
    for (keysym, char) in values {
        let char = char::from_u32(char).unwrap();
        writeln!(res, "    KeysymChar {{").unwrap();
        writeln!(res, "        keysym: 0x{keysym:04x},").unwrap();
        writeln!(res, "        char: {char:?},").unwrap();
        writeln!(res, "    }},").unwrap();
    }
    writeln!(res, "];").unwrap();
    res
}

fn generate_char_to_bespoke_idx(output: &IndexMap<u32, KeysymInfo>) -> String {
    let mut map = HashMap::new();
    for v in output.values() {
        if let Some(o) = v.code_point {
            if v.keysym >> 24 != 0x01 {
                match map.entry(char::from_u32(o).unwrap()) {
                    Entry::Vacant(e) => {
                        e.insert(v);
                    }
                    Entry::Occupied(mut e) => {
                        if e.get().names.iter().all(|n| n.deprecated) {
                            e.insert(v);
                        }
                    }
                }
            }
        }
    }
    let keys: Vec<_> = map.keys().copied().collect();
    let mut values: Vec<_> = map.values().map(|v| v.definitive_idx).collect();
    generate_map("CHAR_TO_BESPOKE_IDX", "char", "u16", &keys, &mut values)
}

fn generate_names(output: &mut IndexMap<u32, KeysymInfo>) -> String {
    let mut res = String::new();
    res.push_str("pub(super) static NAMES: &str = \"");
    let offset = res.len();
    for v in output.values_mut() {
        for name in &mut v.names {
            name.name_start = res.len() - offset;
            name.name_len = name.name.len();
            res.push_str(&name.name);
        }
    }
    res.push_str("\";\n");
    res
}

fn generate_datas(output: &IndexMap<u32, KeysymInfo>) -> String {
    use std::fmt::Write;
    let mut syms: Vec<_> = output
        .values()
        .flat_map(|v| v.names.iter().map(move |n| (v, n)))
        .collect();
    syms.sort_by_key(|(_, n)| n.idx);
    let mut res = String::new();
    res.push_str("pub(super) static DATAS: &[KeysymData] = &[\n");
    for (v, name) in syms {
        let keysym_or_definitive_idx = if name.idx == v.definitive_idx {
            v.keysym
        } else {
            v.definitive_idx as u32
        };
        res.push_str("    KeysymData {\n");
        writeln!(
            res,
            "        keysym_or_definitive_idx: 0x{keysym_or_definitive_idx:08x},"
        )
        .unwrap();
        writeln!(res, "        name_start: {},", name.name_start).unwrap();
        writeln!(res, "        name_len: {},", name.name_len).unwrap();
        res.push_str("        flags: 0");
        if v.lower.is_some() || v.is_upper {
            res.push_str(" | IS_UPPER");
        }
        if v.upper.is_some() || v.is_lower {
            res.push_str(" | IS_LOWER");
        }
        if let Some(cp) = v.code_point {
            res.push_str(" | HAS_CHAR");
            if cp == v.keysym {
                res.push_str(" | KEYSYM_IS_CHAR");
            }
        }
        if name.idx != v.definitive_idx {
            res.push_str(" | IS_SECONDARY_IDX");
        }
        res.push_str(",\n");
        res.push_str("    },\n");
    }
    res.push_str("];\n");
    res
}

fn generate_sym_consts(output: &IndexMap<u32, KeysymInfo>) -> String {
    use std::fmt::Write;
    let mut res = String::new();
    res.push_str("pub mod syms {\n");
    res.push_str("    #![allow(non_upper_case_globals)]\n");
    res.push_str("    use super::*;\n");
    for v in output.values() {
        for name in &v.names {
            writeln!(res, "    /// {}", name.name).unwrap();
            writeln!(
                res,
                "    pub const KEY_{}: Keysym = Keysym(0x{:08x});",
                name.name, v.keysym
            )
            .unwrap();
        }
    }
    res.push_str("}\n");
    res
}
