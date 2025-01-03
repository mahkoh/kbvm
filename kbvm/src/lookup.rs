use {
    crate::{
        group_type::GroupType, keysym::Keysym, modifier::ModifierMask, state_machine::Keycode,
    },
    hashbrown::HashMap,
    smallvec::SmallVec,
    std::fmt::{Debug, Formatter},
};

#[derive(Debug)]
pub struct LookupTable {
    pub(crate) ctrl: Option<ModifierMask>,
    pub(crate) caps: Option<ModifierMask>,
    pub(crate) keys: HashMap<Keycode, KeyGroups>,
}

#[derive(Default, Debug)]
pub(crate) struct KeyGroups {
    pub(crate) groups: Box<[Option<KeyGroup>]>,
}

#[derive(Debug)]
pub(crate) struct KeyGroup {
    pub(crate) ty: GroupType,
    pub(crate) layers: Box<[KeyLayer]>,
}

#[derive(Default, Debug)]
pub(crate) struct KeyLayer {
    pub(crate) symbols: SmallVec<[Keysym; 1]>,
}

#[derive(Copy, Clone)]
pub struct Lookup<'a> {
    original_mods: ModifierMask,
    consumed_mods: ModifierMask,
    remaining_mods: ModifierMask,
    use_ctrl_fallback: bool,
    do_ctrl_transform: bool,
    do_caps_transform: bool,
    lookup: &'a LookupTable,
    groups: &'a [Option<KeyGroup>],
    syms: &'a [Keysym],
}

#[derive(Clone, Debug)]
pub struct KeysymsIter<'a> {
    did_ctrl_fallback: bool,
    do_ctrl_transform: bool,
    do_caps_transform: bool,
    syms: &'a [Keysym],
}

#[derive(Copy, Clone, Debug)]
pub struct KeysymProps {
    pub keysym: Keysym,
    pub char: Option<char>,
    pub did_ctrl_fallback: bool,
    pub did_ctrl_transform: bool,
    pub did_caps_transform: bool,
}

impl Debug for Lookup<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Keysyms")
            .field("original_mods", &self.original_mods)
            .field("consumed_mods", &self.consumed_mods)
            .field("remaining_mods", &self.remaining_mods)
            .field("use_ctrl_fallback", &self.use_ctrl_fallback)
            .field("do_ctrl_transform", &self.do_ctrl_transform)
            .field("do_caps_transform", &self.do_caps_transform)
            .finish_non_exhaustive()
    }
}

impl LookupTable {
    pub fn lookup(&self, group: u32, mods: ModifierMask, keycode: Keycode) -> Lookup<'_> {
        let mut consumed = ModifierMask::default();
        let mut groups = &[][..];
        let mut syms = &[][..];
        if let Some(key) = self.keys.get(&keycode) {
            if let Some(Some(group)) = key.groups.get(group as usize) {
                let mapping = group.ty.map(mods);
                if let Some(layer) = group.layers.get(mapping.layer) {
                    consumed = mapping.consumed;
                    groups = &key.groups;
                    syms = &layer.symbols;
                }
            }
        }
        Lookup {
            original_mods: mods,
            consumed_mods: consumed,
            remaining_mods: mods & !consumed,
            use_ctrl_fallback: true,
            do_ctrl_transform: true,
            do_caps_transform: true,
            lookup: self,
            groups,
            syms,
        }
    }
}

impl<'a> IntoIterator for Lookup<'a> {
    type Item = KeysymProps;
    type IntoIter = KeysymsIter<'a>;

    fn into_iter(self) -> Self::IntoIter {
        let mut do_ctrl_transform = false;
        if let Some(mask) = self.lookup.ctrl {
            if self.do_ctrl_transform && self.remaining_mods.contains(mask) {
                do_ctrl_transform = true;
            }
        }
        let mut do_caps_transform = false;
        if let Some(mask) = self.lookup.caps {
            if self.do_caps_transform && self.remaining_mods.contains(mask) {
                do_caps_transform = true;
            }
        }
        let mut syms = self.syms;
        let mut did_ctrl_fallback = false;
        if self.use_ctrl_fallback && do_ctrl_transform && syms.len() == 1 && syms[0].0 > 127 {
            for group in self.groups.iter().flatten() {
                let layer = group.ty.map(self.original_mods).layer;
                if let Some(layer) = group.layers.get(layer) {
                    if layer.symbols.len() == 1 && layer.symbols[0].0 <= 127 {
                        syms = &layer.symbols;
                        did_ctrl_fallback = true;
                        break;
                    }
                }
            }
        }
        KeysymsIter {
            did_ctrl_fallback,
            do_ctrl_transform,
            do_caps_transform,
            syms,
        }
    }
}

impl Iterator for KeysymsIter<'_> {
    type Item = KeysymProps;

    fn next(&mut self) -> Option<Self::Item> {
        let mut sym = self.syms.first().copied()?;
        self.syms = &self.syms[1..];
        let mut did_caps_transform = false;
        if self.do_caps_transform {
            let prev = sym;
            let sym = sym.to_uppercase();
            did_caps_transform = prev != sym;
        }
        let mut char = None;
        let mut did_ctrl_transform = false;
        if self.do_ctrl_transform && sym.0 <= 127 {
            let s = sym.0 as u8;
            'transform: {
                let c = match s {
                    b'@'..=b'~' | b' ' => s & 0x1f,
                    b'2' => 0,
                    b'3'..=b'7' => s - 0x1b,
                    b'8' => 0x7f,
                    b'/' => 0x1f,
                    _ => break 'transform,
                };
                char = Some(c as char);
                did_ctrl_transform = true;
            }
        }
        if char.is_none() {
            char = sym.char();
        }
        Some(KeysymProps {
            keysym: sym,
            char,
            did_ctrl_fallback: self.did_ctrl_fallback,
            did_ctrl_transform,
            did_caps_transform,
        })
    }
}
