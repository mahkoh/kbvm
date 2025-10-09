use {
    crate::{
        ModifierIndex, ModifierMask,
        xkb::{interner::Interned, span::Spanned},
    },
    arrayvec::ArrayVec,
    std::{cell::Cell, ops::Deref, rc::Rc},
};

const MAX_VMODS: usize = 24;

#[derive(Default, Debug)]
pub(crate) struct Vmodmap {
    mods: ArrayVec<Rc<Vmod>, MAX_VMODS>,
}

#[derive(Debug)]
pub(crate) struct Vmod {
    pub(crate) name: Spanned<Interned>,
    pub(crate) idx: ModifierIndex,
    pub(crate) def: Cell<Option<Spanned<ModifierMask>>>,
}

impl Vmodmap {
    pub(crate) fn insert(&mut self, name: Spanned<Interned>) -> Option<Rc<Vmod>> {
        if let Some(vmod) = self.get(name.val) {
            return Some(vmod.clone());
        }
        if self.mods.len() >= MAX_VMODS {
            return None;
        }
        let idx = self.mods.len() as u32 + 8;
        let vmod = Rc::new(Vmod {
            name,
            idx: ModifierIndex::new(idx)?,
            def: Default::default(),
        });
        self.mods.push(vmod.clone());
        Some(vmod)
    }

    pub(crate) fn get(&self, name: Interned) -> Option<&Rc<Vmod>> {
        self.mods.iter().find(|m| m.name.val == name)
    }
}

impl Deref for Vmodmap {
    type Target = [Rc<Vmod>];

    fn deref(&self) -> &Self::Target {
        &self.mods
    }
}

impl IntoIterator for Vmodmap {
    type Item = Rc<Vmod>;
    type IntoIter = <ArrayVec<Rc<Vmod>, MAX_VMODS> as IntoIterator>::IntoIter;

    fn into_iter(self) -> Self::IntoIter {
        self.mods.into_iter()
    }
}

impl<'a> IntoIterator for &'a Vmodmap {
    type Item = &'a Rc<Vmod>;
    type IntoIter = <&'a ArrayVec<Rc<Vmod>, MAX_VMODS> as IntoIterator>::IntoIter;

    fn into_iter(self) -> Self::IntoIter {
        (&self.mods).into_iter()
    }
}

#[derive(Clone, Debug)]
pub(crate) enum ModifierTree {
    Num(ModifierMask),
    VMod(Rc<Vmod>),
    Not(Box<Spanned<ModifierTree>>),
    Add(Box<Spanned<ModifierTree>>, Box<Spanned<ModifierTree>>),
    Sub(Box<Spanned<ModifierTree>>, Box<Spanned<ModifierTree>>),
}

impl ModifierTree {
    pub(crate) fn get_effective(&self) -> ModifierMask {
        self.eval_real().0
    }

    pub(crate) fn try_get_effective(&self) -> Result<ModifierMask, ModifierMask> {
        let (res, ok) = self.eval_real();
        ok.then_some(res).ok_or(res)
    }

    pub(crate) fn eval_real(&self) -> (ModifierMask, bool) {
        self.eval(&|v| v.def.get().map(|v| v.val).unwrap_or_default())
    }

    pub(crate) fn eval_virtual(&self) -> (ModifierMask, bool) {
        self.eval(&|v| v.idx.to_mask())
    }

    fn eval(&self, f: &impl Fn(&Vmod) -> ModifierMask) -> (ModifierMask, bool) {
        let mut ok = true;
        let mut res;
        match self {
            ModifierTree::Num(n) => res = *n,
            ModifierTree::VMod(v) => {
                res = f(v);
                ok = res.0 != 0;
            }
            ModifierTree::Not(n) => {
                (res, ok) = n.val.eval(f);
                let real_only = res.0 & !0xff == 0;
                res.0 = !res.0;
                if real_only {
                    res.0 &= 0xff;
                }
            }
            ModifierTree::Add(l, r) => {
                let (l, o1) = l.val.eval(f);
                let (r, o2) = r.val.eval(f);
                res = l | r;
                ok = o1 && o2;
            }
            ModifierTree::Sub(l, r) => {
                let (l, o1) = l.val.eval(f);
                let (r, o2) = r.val.eval(f);
                res = l & !r;
                ok = o1 && o2;
            }
        }
        (res, ok)
    }
}
