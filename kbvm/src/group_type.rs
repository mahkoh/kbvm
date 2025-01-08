use {crate::modifier::ModifierMask, hashbrown::HashMap, std::sync::Arc};

#[derive(Clone, Debug)]
pub struct GroupType {
    pub(crate) data: Arc<Data>,
}

#[derive(Debug)]
pub(crate) struct Data {
    pub(crate) mask: ModifierMask,
    pub(crate) cases: Vec<Case>,
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct Case {
    pub(crate) mods: ModifierMask,
    pub(crate) layer: usize,
    pub(crate) consumed: ModifierMask,
}

#[derive(Debug)]
pub(crate) struct KeyTypeMapping {
    pub(crate) layer: usize,
    pub(crate) consumed: ModifierMask,
}

impl GroupType {
    pub fn builder(mask: ModifierMask) -> GroupTypeBuilder {
        GroupTypeBuilder {
            mask,
            cases: Default::default(),
        }
    }

    pub(crate) fn map(&self, mut mods: ModifierMask) -> KeyTypeMapping {
        mods &= self.data.mask;
        for case in &self.data.cases {
            if mods == case.mods {
                let consumed = mods & case.consumed;
                return KeyTypeMapping {
                    layer: case.layer,
                    consumed,
                };
            }
        }
        KeyTypeMapping {
            layer: 0,
            consumed: mods,
        }
    }
}

pub struct GroupTypeBuilder {
    mask: ModifierMask,
    cases: HashMap<ModifierMask, Case>,
}

impl GroupTypeBuilder {
    pub fn build(&self) -> GroupType {
        GroupType {
            data: Arc::new(Data {
                mask: self.mask,
                cases: self.cases.values().copied().collect(),
            }),
        }
    }

    pub fn map(&mut self, mods: ModifierMask, layer: usize) -> &mut Self {
        self.map_preserve(mods, ModifierMask(0), layer)
    }

    pub fn map_preserve(
        &mut self,
        mods: ModifierMask,
        preserve: ModifierMask,
        layer: usize,
    ) -> &mut Self {
        self.cases.insert(
            mods,
            Case {
                mods,
                layer,
                consumed: !preserve,
            },
        );
        self
    }
}
