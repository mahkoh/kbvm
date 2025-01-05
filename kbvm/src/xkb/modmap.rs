use {
    crate::{
        modifier::{ModifierIndex, ModifierMask},
        xkb::{interner::Interned, span::Spanned},
    },
    arrayvec::ArrayVec,
    std::ops::Deref,
};

const MAX_VMODS: usize = 24;

#[derive(Default, Debug)]
pub(crate) struct Vmodmap {
    mods: ArrayVec<Vmod, MAX_VMODS>,
}

#[derive(Debug)]
pub(crate) struct Vmod {
    pub(crate) name: Spanned<Interned>,
    pub(crate) idx: ModifierIndex,
    pub(crate) def: Option<Spanned<ModifierMask>>,
}

impl Vmodmap {
    pub(crate) fn insert(&mut self, name: Spanned<Interned>) -> Option<&mut Vmod> {
        let slf = match self.get_mut_(name.val) {
            Ok(vmod) => return Some(vmod),
            Err(slf) => slf,
        };
        if slf.mods.len() >= MAX_VMODS {
            return None;
        }
        let idx = slf.mods.len() as u32 + 8;
        slf.mods.push(Vmod {
            name,
            idx: ModifierIndex::new(idx)?,
            def: None,
        });
        Some(slf.mods.last_mut().unwrap())
    }

    pub(crate) fn get(&self, name: Interned) -> Option<&Vmod> {
        self.mods.iter().find(|m| m.name.val == name)
    }

    // pub(crate) fn get_mut(&mut self, name: Interned) -> Option<&mut Vmod> {
    //     self.get_mut_(name).ok()
    // }

    fn get_mut_(&mut self, name: Interned) -> Result<&mut Vmod, &mut Self> {
        match self.mods.iter_mut().position(|m| m.name.val == name) {
            Some(m) => Ok(&mut self.mods[m]),
            _ => Err(self),
        }
    }

    pub(crate) fn get_effective(&self, mask: ModifierMask) -> ModifierMask {
        self.try_get_effective(mask).unwrap_or_else(|v| v)
    }

    pub(crate) fn try_get_effective(
        &self,
        mask: ModifierMask,
    ) -> Result<ModifierMask, ModifierMask> {
        let mut ok = true;
        let mut res = mask;
        res.0 &= 0xff;
        for m in &self.mods {
            if mask.contains(m.idx.to_mask()) {
                let def = m.def.map(|d| d.val).unwrap_or_default();
                res |= def;
                if def == ModifierMask::NONE {
                    ok = false;
                }
            }
        }
        ok.then_some(res).ok_or(res)
    }
}

impl Deref for Vmodmap {
    type Target = [Vmod];

    fn deref(&self) -> &Self::Target {
        &self.mods
    }
}

impl IntoIterator for Vmodmap {
    type Item = Vmod;
    type IntoIter = <ArrayVec<Vmod, MAX_VMODS> as IntoIterator>::IntoIter;

    fn into_iter(self) -> Self::IntoIter {
        self.mods.into_iter()
    }
}

impl<'a> IntoIterator for &'a Vmodmap {
    type Item = &'a Vmod;
    type IntoIter = <&'a ArrayVec<Vmod, MAX_VMODS> as IntoIterator>::IntoIter;

    fn into_iter(self) -> Self::IntoIter {
        (&self.mods).into_iter()
    }
}

impl<'a> IntoIterator for &'a mut Vmodmap {
    type Item = &'a mut Vmod;
    type IntoIter = <&'a mut ArrayVec<Vmod, MAX_VMODS> as IntoIterator>::IntoIter;

    fn into_iter(self) -> Self::IntoIter {
        (&mut self.mods).into_iter()
    }
}
