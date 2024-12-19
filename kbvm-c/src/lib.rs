use {
    kbvm::keysym::{Keysym, Keysyms},
    std::{ffi::CStr, io::Write, slice},
};

#[no_mangle]
pub extern "C" fn xkb_keysym_is_assigned(ks: Keysym) -> bool {
    ks.is_in_unicode_range() || ks.name().is_some()
}

#[no_mangle]
pub unsafe extern "C" fn xkb_keysym_from_name(name: *const u8, flags: i32) -> Keysym {
    let s = CStr::from_ptr(name as _).to_bytes();
    let res = if flags & 1 != 0 {
        Keysym::from_str_insensitive(s)
    } else {
        Keysym::from_str(s)
    }
    .unwrap_or_default();
    if res.0 > 0x1fffffff {
        Keysym(0)
    } else {
        res
    }
}

#[no_mangle]
pub extern "C" fn xkb_keysym_is_lower(ks: Keysym) -> bool {
    ks.is_lowercase()
}

#[no_mangle]
pub extern "C" fn xkb_keysym_is_upper(ks: Keysym) -> bool {
    ks.is_uppercase()
}

#[no_mangle]
pub extern "C" fn xkb_keysym_is_upper_or_title(ks: Keysym) -> bool {
    ks.is_uppercase()
}

#[no_mangle]
pub unsafe extern "C" fn xkb_keysym_get_name(ks: Keysym, buffer: *mut u8, size: usize) -> i32 {
    let buffer = slice::from_raw_parts_mut(buffer, size);
    let mut buffer2 = &mut *buffer;
    if ks.0 > 0x1f_ff_ff_ff {
        let _ = write!(buffer2, "Invalid");
    } else if let Some(name) = ks.name() {
        let _ = write!(buffer2, "{}", name);
    } else if ks.0 >= 0x01_00_01_00 && ks.0 <= 0x01_10_ff_ff {
        let ks = ks.0 & 0xff_ff_ff;
        let width = if ks > 0xff_ff { 8 } else { 4 };
        let _ = write!(buffer2, "U{:0width$X}", ks);
    } else {
        let _ = write!(buffer2, "0x{:08x}", ks.0);
    }
    if buffer2.is_empty() {
        if let Some(last) = buffer.last_mut() {
            *last = b'\0';
        }
        buffer.len().saturating_sub(1) as _
    } else {
        buffer2[0] = b'\0';
        let rem = buffer2.len();
        (buffer.len() - rem) as _
    }
}

#[no_mangle]
pub extern "C" fn xkb_keysym_is_deprecated(
    _ks: Keysym,
    name: *const u8,
    reference_name: *mut *const u8,
) -> bool {
    false
}

#[no_mangle]
pub extern "C" fn xkb_keysym_to_upper(ks: Keysym) -> Keysym {
    ks.to_uppercase()
}

#[no_mangle]
pub extern "C" fn xkb_keysym_to_lower(ks: Keysym) -> Keysym {
    ks.to_lowercase()
}

#[no_mangle]
pub extern "C" fn xkb_utf32_to_keysym(c: u32) -> Keysym {
    char::from_u32(c).map(Keysym::from_char).unwrap_or_default()
}

#[no_mangle]
pub extern "C" fn xkb_keysym_is_keypad(ks: Keysym) -> bool {
    ks.is_keypad()
}

#[no_mangle]
pub unsafe extern "C" fn xkb_keysym_to_utf8(ks: Keysym, buffer: *mut u8, size: usize) -> i32 {
    if size < 5 {
        return -1;
    }
    let Some(c) = ks.char() else {
        return 0;
    };
    let mut buffer = slice::from_raw_parts_mut(buffer, size);
    let _ = write!(buffer, "{}", c);
    buffer[0] = b'\0';
    (size - buffer.len() + 1) as i32
}

#[no_mangle]
pub extern "C" fn xkb_keysym_is_modifier(ks: Keysym) -> bool {
    ks.is_modifier()
}

pub struct Iter {
    iter: Keysyms,
    current: Option<Keysym>,
    explicit: bool,
}

#[no_mangle]
pub extern "C" fn xkb_keysym_iterator_new(explicit: bool) -> *mut Iter {
    Box::into_raw(Box::new(Iter {
        iter: Keysym::all(),
        current: None,
        explicit,
    }))
}

#[no_mangle]
pub unsafe extern "C" fn xkb_keysym_iterator_unref(keysyms: *mut Iter) {
    drop(Box::from_raw(keysyms));
}

#[no_mangle]
pub unsafe extern "C" fn xkb_keysym_iterator_get_keysym(keysyms: *mut Iter) -> Keysym {
    let iter = &mut *keysyms;
    iter.current.unwrap_or_default()
}

#[no_mangle]
pub unsafe extern "C" fn xkb_keysym_iterator_is_explicitly_named(_keysyms: *mut Iter) -> bool {
    true
}

#[no_mangle]
pub unsafe extern "C" fn xkb_keysym_iterator_get_name(
    keysyms: *mut Iter,
    buffer: *mut u8,
    size: usize,
) -> i32 {
    let iter = &mut *keysyms;
    let Some(sym) = iter.current else { return -1 };
    xkb_keysym_get_name(sym, buffer, size)
}

#[no_mangle]
pub unsafe extern "C" fn xkb_keysym_iterator_next(keysyms: *mut Iter) -> bool {
    let iter = &mut *keysyms;
    if !iter.explicit {
        if let Some(ks) = iter.current {
            const MIN: u32 = 0x01_00_01_00;
            const MAX: u32 = 0x01_10_ff_ff;
            if ks.0 >= MIN && ks.0 <= MAX {
                if ks.0 == MAX {
                    while let Some(n) = iter.iter.next() {
                        if n.0 > MAX {
                            iter.current = Some(n);
                            return true;
                        }
                    }
                } else {
                    iter.current = Some(Keysym(ks.0 + 1));
                    return true;
                }
            }
        }
    }
    iter.current = iter.iter.next();
    iter.current.is_some()
}
