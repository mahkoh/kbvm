use std::{ffi::c_int, ptr};

unsafe extern "C" {
    pub safe fn xkb_keysym_to_lower(sym: u32) -> u32;
    pub safe fn xkb_keysym_to_upper(sym: u32) -> u32;
    pub safe fn xkb_utf32_to_keysym(ucs: u32) -> u32;
    pub safe fn xkb_keysym_to_utf32(ucs: u32) -> u32;
    pub unsafe fn xkb_keysym_get_name(sym: u32, buffer: *mut u8, size: usize) -> c_int;
    pub unsafe fn xkb_keysym_from_name(name: *const u8, flags: c_int) -> u32;
    pub safe fn xkb_context_new(flags: c_int) -> *mut u8;
    pub unsafe fn xkb_context_unref(ctx: *mut u8);
    pub unsafe fn xkb_keymap_new_from_buffer(
        context: *mut u8,
        buffer: *const u8,
        length: usize,
        format: c_int,
        flags: c_int,
    ) -> *mut u8;
    pub unsafe fn xkb_keymap_unref(keymap: *mut u8);
    pub unsafe fn xkb_state_new(map: *mut u8) -> *mut u8;
    pub unsafe fn xkb_state_unref(stat: *mut u8);
    pub unsafe fn xkb_state_update_key(state: *mut u8, key: u32, direction: c_int) -> c_int;
}

pub fn keymap_from_str(s: &str) -> *mut u8 {
    let ctx = xkb_context_new(0);
    assert_ne!(ctx, ptr::null_mut());
    let map = unsafe { xkb_keymap_new_from_buffer(ctx, s.as_ptr(), s.len(), 1, 0) };
    assert_ne!(map, ptr::null_mut());
    unsafe {
        xkb_context_unref(ctx);
    }
    map
}

pub fn state_from_str(s: &str) -> *mut u8 {
    let map = keymap_from_str(s);
    let state = unsafe { xkb_state_new(map) };
    assert_ne!(state, ptr::null_mut());
    unsafe {
        xkb_keymap_unref(map);
    }
    state
}

pub struct XState {
    state: *mut u8,
}

impl Drop for XState {
    fn drop(&mut self) {
        unsafe {
            xkb_state_unref(self.state);
        }
    }
}

impl XState {
    pub fn new(map: &str) -> Self {
        Self {
            state: state_from_str(map),
        }
    }

    pub fn handle_key(&mut self, key: u32, down: bool) {
        unsafe {
            xkb_state_update_key(self.state, key, down as c_int);
        }
    }
}
