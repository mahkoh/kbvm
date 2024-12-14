use {crate::modifier::ModifierMask, hashbrown::HashMap, std::sync::Arc};

#[derive(Clone)]
pub struct GroupType {
    data: Arc<Data>,
}

struct Data {
    mask: ModifierMask,
    cases: Vec<Case>,
}

#[derive(Copy, Clone)]
struct Case {
    mods: ModifierMask,
    layer: usize,
    consumed: ModifierMask,
}

pub(crate) struct KeyTypeMapping {
    pub(crate) layer: usize,
    pub(crate) _consumed: ModifierMask,
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
                    _consumed: consumed,
                };
            }
        }
        KeyTypeMapping {
            layer: 0,
            _consumed: ModifierMask(0),
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
