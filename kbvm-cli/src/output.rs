use kbvm::{xkb::Keymap, GroupDelta, GroupIndex, Keycode, Keysym, ModifierMask};

pub mod ansi;
pub mod json;

pub trait Output {
    fn keymap(&mut self, keymap: &Keymap);
    fn key_down(&mut self, keycode: Keycode);
    fn key_up(&mut self, keycode: Keycode);
    fn keysym(&mut self, keysym: Keysym, char: Option<char>);
    fn mods_pressed(&mut self, mods: ModifierMask);
    fn mods_latched(&mut self, mods: ModifierMask);
    fn mods_locked(&mut self, mods: ModifierMask);
    fn mods(&mut self, mods: ModifierMask);
    fn group_pressed(&mut self, group: GroupDelta);
    fn group_latched(&mut self, group: GroupDelta);
    fn group_locked(&mut self, group: GroupIndex);
    fn group(&mut self, group: GroupIndex);
    fn compose_pending(&mut self, keysym: Keysym);
    fn compose_aborted(&mut self, keysym: Keysym);
    fn composed(&mut self, keysym: Option<Keysym>, string: Option<&str>, original_keysym: Keysym);
    fn state_reset(&mut self);
}
