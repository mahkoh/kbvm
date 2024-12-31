use {
    crate::{
        group_type::GroupType,
        routine::Routine,
        state_machine::{KeyGroup, KeyGroups, KeyLayer, Keycode, StateMachine},
    },
    hashbrown::HashMap,
};

#[derive(Default)]
pub struct Builder {
    keys: HashMap<Keycode, BuilderKey>,
}

#[derive(Default)]
struct BuilderKey {
    groups: Vec<Option<BuilderGroup>>,
}

struct BuilderGroup {
    ty: GroupType,
    layers: Vec<Option<BuilderLayer>>,
}

#[derive(Default)]
struct BuilderLayer {
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
    pub fn add_key(&mut self, key: Keycode) -> KeyBuilder<'_> {
        KeyBuilder {
            groups: self.keys.entry(key).or_default(),
        }
    }

    pub fn build_state_machine(&self) -> StateMachine {
        let mut map = HashMap::with_capacity(self.keys.len());
        for (keycode, key) in &self.keys {
            let mut groups = Vec::with_capacity(key.groups.len());
            for group in &key.groups {
                match group {
                    None => groups.push(None),
                    Some(g) => {
                        let mut layers = Vec::with_capacity(g.layers.len());
                        for layer in &g.layers {
                            match layer {
                                None => layers.push(KeyLayer::default()),
                                Some(l) => layers.push(KeyLayer {
                                    routine: l.routine.clone(),
                                }),
                            }
                        }
                        groups.push(Some(KeyGroup {
                            ty: g.ty.clone(),
                            layers: layers.into_boxed_slice(),
                        }));
                    }
                }
            }
            map.insert(
                *keycode,
                KeyGroups {
                    groups: groups.into_boxed_slice(),
                },
            );
        }
        // let mut map: Vec<_> = map.into_iter().map(|(k, g)| (k, g)).collect();
        // map.sort_by_key(|k| k.0);
        // let max_keycode = map.keys().map(|k| k.0 as usize).max().unwrap_or_default();
        // let mut vec = vec!();
        // vec.resize_with(max_keycode + 1, Default::default);
        // for (k, v) in map {
        //     let idx = k.0 as usize;
        //     vec[idx] = v;
        // }
        StateMachine { keys: map }
    }
}

impl KeyBuilder<'_> {
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
}
