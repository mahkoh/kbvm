use {
    crate::{
        keysym::{
            generated::{LEN, LONGEST_NAME},
            keysyms, Keysym,
        },
        syms,
    },
    bstr::ByteSlice,
    libxkbcommon_test_linker::{
        xkb_keysym_from_name, xkb_keysym_get_name, xkb_keysym_to_lower, xkb_keysym_to_upper,
        xkb_keysym_to_utf32, xkb_utf32_to_keysym,
    },
};

#[test]
fn keysyms_len() {
    assert_eq!(keysyms().count(), LEN);
}

#[test]
fn from_char() {
    for cp in 0..=0x10ffff {
        let Some(c) = char::from_u32(cp) else {
            continue;
        };
        let l = Keysym::from_char(c).0;
        let r = xkb_utf32_to_keysym(c as _);
        if matches!(c, '─' | '│' | '┌' | '○') {
            // xkbcommon returns deprecated keysyms for these characters
            if l == r {
                panic!("Expected different output for {c:?} but output is 0x{l:x}");
            }
        } else {
            if l != r {
                panic!("for {c:?}: self=0x{l:x}, xkbcommon=0x{r:x}");
            }
        }
    }
}

#[test]
fn to_char() {
    for sym in keysyms() {
        assert_eq!(
            sym.char().unwrap_or(0 as char) as u32,
            xkb_keysym_to_utf32(sym.0),
            "{:?} = 0x{:x}",
            sym,
            sym.0,
        );
    }
}

#[test]
fn name() {
    let mut buf = [0; LONGEST_NAME + 1];
    for sym in keysyms() {
        let res = unsafe { xkb_keysym_get_name(sym.0, buf.as_mut_ptr(), buf.len()) };
        if res < 0 {
            panic!("xkb_keysym_get_name doesn't know about {:?}", sym);
        }
        let xkb = &buf[..res as usize];
        assert_eq!(
            sym.name().unwrap(),
            xkb.as_bstr(),
            "{:?} = 0x{:x}",
            sym,
            sym.0,
        );
    }
}

#[test]
fn from_name() {
    let mut buf = [0; LONGEST_NAME + 1];
    for sym in keysyms() {
        assert_eq!(
            Keysym::from_str(sym.name().unwrap()).unwrap(),
            sym,
            "{:?} = 0x{:x}",
            sym,
            sym.0,
        );
        let name = sym.name().unwrap();
        buf[..name.len()].copy_from_slice(name.as_bytes());
        buf[name.len()] = 0;
        let ks = unsafe { xkb_keysym_from_name(buf.as_ptr(), 0) };
        assert_eq!(ks, sym.0, "{:?} = 0x{:x}", sym, sym.0,);
    }
}

#[test]
fn from_name_xf86() {
    let mut buf = [0; LONGEST_NAME + 2];
    for sym in keysyms() {
        let Some(suffix) = sym.name().unwrap().strip_prefix("XF86") else {
            continue;
        };
        let name = format!("XF86_{suffix}");
        assert_eq!(
            Keysym::from_str(&name).unwrap(),
            sym,
            "{:?} = 0x{:x}",
            sym,
            sym.0,
        );
        buf[..name.len()].copy_from_slice(name.as_bytes());
        buf[name.len()] = 0;
        let ks = unsafe { xkb_keysym_from_name(buf.as_ptr(), 0) };
        assert_eq!(ks, sym.0, "{:?} = 0x{:x}", sym, sym.0,);
    }
}

#[test]
fn from_name_insensitive() {
    let mut buf = [0; LONGEST_NAME + 1];
    for sym in keysyms() {
        let name = sym.name().unwrap().to_ascii_uppercase();
        let l = Keysym::from_str_insensitive(&name).expect(&name);
        buf[..name.len()].copy_from_slice(name.as_bytes());
        buf[name.len()] = 0;
        let r = unsafe { xkb_keysym_from_name(buf.as_ptr(), 1) };
        assert_eq!(l.0, r, "{:?} = 0x{:x} ({name})", sym, sym.0,);
    }
}

#[test]
fn from_name_numbers() {
    let mut buf = [0; 10];
    for i in 0..0x20000 {
        let n1 = format!("U{:x}", i);
        let n2 = format!("u{:x}", i);
        let n3 = format!("0x{:x}", i);
        let n4 = format!("0X{:x}", i);
        for (n, case_insensitive) in [(n1, false), (n2, true), (n3, false), (n4, true)] {
            let res = match case_insensitive {
                true => Keysym::from_str_insensitive(&n),
                false => Keysym::from_str(&n),
            };
            let l = res.map(|v| v.0).unwrap_or_default();
            buf[..n.len()].copy_from_slice(n.as_bytes());
            buf[n.len()] = 0;
            let r = unsafe { xkb_keysym_from_name(buf.as_ptr(), case_insensitive as _) };
            assert_eq!(l, r, "0x{:x} ({n})", l);
        }
    }
}

#[test]
fn to_upper() {
    for sym in keysyms() {
        assert_eq!(
            sym.to_uppercase().0,
            xkb_keysym_to_upper(sym.0),
            "{:?} = 0x{:x}",
            sym,
            sym.0,
        );
    }
}

#[test]
fn to_lower() {
    for sym in keysyms() {
        assert_eq!(
            sym.to_lowercase().0,
            xkb_keysym_to_lower(sym.0),
            "{:?} = 0x{:x}",
            sym,
            sym.0,
        );
    }
}

#[test]
fn ssharp_is_lower() {
    assert!(syms::ssharp.is_lowercase());
}
