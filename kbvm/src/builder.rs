use {
    crate::{
        group::GroupIndex,
        group_type::GroupType,
        keysym::Keysym,
        lookup::{self, LookupTable},
        modifier::{ModifierIndex, ModifierMask},
        routine::Routine,
        state_machine::{self, Keycode, StateMachine},
    },
    hashbrown::HashMap,
    isnt::std_1::primitive::IsntSliceExt,
    smallvec::SmallVec,
};

#[derive(Default, Debug)]
pub struct Builder {
    ctrl: Option<ModifierMask>,
    caps: Option<ModifierMask>,
    keys: HashMap<Keycode, BuilderKey>,
}

#[derive(Copy, Clone, Default, Debug)]
pub enum Redirect {
    #[default]
    Wrap,
    Clamp,
    Fixed(usize),
}

impl Redirect {
    #[inline]
    pub(crate) fn constrain(mut self, len: usize) -> Self {
        if let Redirect::Fixed(n) = &mut self {
            if *n >= len {
                *n = 0;
            }
        }
        self
    }

    #[inline]
    pub(crate) fn apply(self, group: GroupIndex, len: usize) -> usize {
        let n = group.0 as usize;
        if n >= len {
            match self {
                Redirect::Wrap => n % len,
                Redirect::Clamp => len - 1,
                Redirect::Fixed(f) => f,
            }
        } else {
            n
        }
    }
}

#[derive(Default, Debug)]
struct BuilderKey {
    repeats: bool,
    groups: Vec<Option<BuilderGroup>>,
    redirect: Redirect,
}

#[derive(Debug)]
struct BuilderGroup {
    ty: GroupType,
    layers: Vec<Option<BuilderLayer>>,
}

#[derive(Default, Debug)]
struct BuilderLayer {
    keysyms: SmallVec<[Keysym; 1]>,
    routine: Option<Routine>,
}

pub struct KeyBuilder<'a> {
    groups: &'a mut BuilderKey,
}

pub struct GroupBuilder<'a> {
    group: &'a mut BuilderGroup,
}

pub struct LayerBuilder<'a> {
    layer: &'a mut BuilderLayer,
}

impl Builder {
    pub fn set_ctrl(&mut self, ctrl: Option<ModifierIndex>) {
        self.ctrl = ctrl.map(|c| c.to_mask());
    }

    pub fn set_caps(&mut self, caps: Option<ModifierIndex>) {
        self.caps = caps.map(|c| c.to_mask());
    }

    pub fn add_key(&mut self, key: Keycode) -> KeyBuilder<'_> {
        KeyBuilder {
            groups: self.keys.entry(key).or_default(),
        }
    }

    pub fn build_state_machine(&self) -> StateMachine {
        let mut map = HashMap::with_capacity(self.keys.len());
        let mut num_groups = 0;
        for (keycode, key) in &self.keys {
            let mut any_groups = false;
            let mut groups = Vec::with_capacity(key.groups.len());
            num_groups = num_groups.max(key.groups.len());
            for group in &key.groups {
                match group {
                    None => groups.push(None),
                    Some(g) => {
                        let mut any_layers = false;
                        let mut layers = Vec::with_capacity(g.layers.len());
                        for layer in &g.layers {
                            match layer {
                                None => layers.push(state_machine::KeyLayer::default()),
                                Some(l) => {
                                    any_layers |= l.routine.is_some();
                                    layers.push(state_machine::KeyLayer {
                                        routine: l.routine.clone(),
                                    })
                                }
                            }
                        }
                        any_groups |= any_layers;
                        groups.push(Some(state_machine::KeyGroup {
                            ty: g.ty.clone(),
                            layers: layers.into_boxed_slice(),
                        }));
                    }
                }
            }
            if any_groups {
                map.insert(
                    *keycode,
                    state_machine::KeyGroups {
                        redirect: key.redirect.constrain(groups.len()),
                        groups: groups.into_boxed_slice(),
                    },
                );
            }
        }
        StateMachine {
            num_groups: (num_groups as u32).max(1),
            keys: map,
        }
    }

    pub fn build_lookup_table(&self) -> LookupTable {
        let mut map = HashMap::with_capacity(self.keys.len());
        for (keycode, key) in &self.keys {
            let mut any_groups = false;
            let mut groups = Vec::with_capacity(key.groups.len());
            for group in &key.groups {
                match group {
                    None => groups.push(None),
                    Some(g) => {
                        let mut any_layers = false;
                        let mut layers = Vec::with_capacity(g.layers.len());
                        for layer in &g.layers {
                            match layer {
                                None => layers.push(lookup::KeyLayer::default()),
                                Some(l) => {
                                    any_layers |= l.keysyms.is_not_empty();
                                    layers.push(lookup::KeyLayer {
                                        symbols: l.keysyms.clone(),
                                    })
                                }
                            }
                        }
                        any_groups |= any_layers;
                        groups.push(Some(lookup::KeyGroup {
                            ty: g.ty.clone(),
                            layers: layers.into_boxed_slice(),
                        }));
                    }
                }
            }
            if any_groups || !key.repeats {
                map.insert(
                    *keycode,
                    lookup::KeyGroups {
                        repeats: key.repeats,
                        redirect: key.redirect.constrain(groups.len()),
                        groups: groups.into_boxed_slice(),
                    },
                );
            }
        }
        LookupTable {
            ctrl: self.ctrl,
            caps: self.caps,
            keys: map,
        }
    }
}

impl KeyBuilder<'_> {
    pub fn repeats(&mut self, repeats: bool) {
        self.groups.repeats = repeats;
    }

    pub fn redirect(&mut self, redirect: Redirect) {
        self.groups.redirect = redirect;
    }

    pub fn add_group(&mut self, group: usize, ty: &GroupType) -> GroupBuilder<'_> {
        if self.groups.groups.len() <= group {
            self.groups.groups.resize_with(group + 1, Default::default);
        }
        let group = self.groups.groups[group].get_or_insert_with(|| BuilderGroup {
            ty: ty.clone(),
            layers: vec![],
        });
        group.ty = ty.clone();
        GroupBuilder { group }
    }
}

impl GroupBuilder<'_> {
    pub fn add_layer(&mut self, layer: usize) -> LayerBuilder<'_> {
        if self.group.layers.len() <= layer {
            self.group.layers.resize_with(layer + 1, Default::default);
        }
        let layer = self.group.layers[layer].get_or_insert_default();
        LayerBuilder { layer }
    }
}

impl LayerBuilder<'_> {
    pub fn routine(&mut self, routine: &Routine) -> &mut Self {
        self.layer.routine = Some(routine.clone());
        self
    }

    pub fn keysyms(&mut self, keysyms: &[Keysym]) -> &mut Self {
        self.layer.keysyms.clear();
        self.layer.keysyms.extend_from_slice(keysyms);
        self
    }
}
