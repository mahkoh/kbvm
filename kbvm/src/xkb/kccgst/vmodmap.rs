use arrayvec::ArrayVec;
use {
    crate::{
        modifier::{ModifierIndex, ModifierMask},
        xkb::{interner::Interned, span::Spanned},
    },
    kbvm_proc::CloneWithDelta,
};

const MAX_VMODS: usize = 24;

#[derive(Default, Debug, CloneWithDelta)]
pub struct Vmodmap {
    mods: ArrayVec<Vmod, MAX_VMODS>,
}

#[derive(Debug, CloneWithDelta)]
pub struct Vmod {
    pub name: Interned,
    pub idx: ModifierIndex,
    pub def: Option<Spanned<ModifierMask>>,
}

impl Vmodmap {
    pub fn insert(&mut self, name: Interned) -> Option<(&mut Vmod, bool)> {
        let slf = match self.get_mut_(name) {
            Ok(vmod) => return Some((vmod, false)),
            Err(slf) => slf,
        };
        if slf.mods.len() >= MAX_VMODS {
            return None;
        }
        let idx = slf.mods.len() as u32 + 8;
        slf.mods.push(Vmod {
            name,
            idx: ModifierIndex(idx),
            def: None,
        });
        Some((slf.mods.last_mut().unwrap(), true))
    }

    pub fn get(&self, name: Interned) -> Option<&Vmod> {
        self.mods.iter().find(|m| m.name == name)
    }

    pub fn get_mut(&mut self, name: Interned) -> Option<&mut Vmod> {
        self.get_mut_(name).ok()
    }

    fn get_mut_(&mut self, name: Interned) -> Result<&mut Vmod, &mut Self> {
        match self.mods.iter_mut().position(|m| m.name == name) {
            Some(m) => Ok(&mut self.mods[m]),
            _ => Err(self),
        }
    }
}
