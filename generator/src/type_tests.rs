use {
    crate::{evdev::MAP, generate_map},
    std::collections::{HashMap, HashSet, hash_map::Entry},
};

pub fn main() {
    let output = generate_rust();
    std::fs::write("type-tests/src/generated.rs", output).unwrap();
    let output = generate_xkb();
    std::fs::write("type-tests/include/keycodes/generated", output).unwrap();
}

fn generate_rust() -> String {
    use std::fmt::Write;
    let name_to_code = generate_name_to_code();
    let code_to_name = generate_code_to_name();
    let mut res = String::new();
    writeln!(res, "use super::*;\n").unwrap();
    writeln!(res, "{name_to_code}").unwrap();
    writeln!(res, "{code_to_name}").unwrap();
    res
}

fn generate_xkb() -> String {
    use std::fmt::Write;
    let mut res = String::new();
    let mut seen_codes = HashMap::new();
    writeln!(res, "default xkb_keycodes {{").unwrap();
    for &(name, code) in MAP {
        match seen_codes.entry(code) {
            Entry::Occupied(e) => writeln!(res, "    alias <{name}> = <{}>;", e.get()).unwrap(),
            Entry::Vacant(e) => {
                writeln!(res, "    <{name}> = {};", code + 8).unwrap();
                e.insert(name);
            }
        }
    }
    writeln!(res, "}};").unwrap();
    res
}

fn generate_name_to_code() -> String {
    let mut keys = vec![];
    let mut values = vec![];
    for v in MAP {
        keys.push(v.0.as_bytes());
        values.push((v.0, v.1 + 8));
    }
    generate_map(
        "NAME_TO_CODE",
        "str",
        "(&'static str, u32)",
        &keys,
        &mut values,
    )
}

fn generate_code_to_name() -> String {
    let mut seen_codes = HashSet::new();
    let mut keys = vec![];
    let mut values = vec![];
    for v in MAP {
        if seen_codes.insert(v.1) {
            keys.push(v.1 + 8);
            values.push((v.0, v.1 + 8));
        }
    }
    generate_map(
        "CODE_TO_NAME",
        "u32",
        "(&'static str, u32)",
        &keys,
        &mut values,
    )
}
