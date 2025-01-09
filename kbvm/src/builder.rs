#[expect(unused_imports)]
use crate::xkb::Keymap;
use {
    crate::{
        group::GroupIndex,
        group_type::GroupType,
        keysym::Keysym,
        lookup::{self, LookupTable},
        modifier::{ModifierIndex, ModifierMask},
        routine::{Global, Routine},
        state_machine::{self, Keycode, StateMachine},
    },
    hashbrown::HashMap,
    isnt::std_1::primitive::IsntSliceExt,
    smallvec::SmallVec,
};

/// A builder for compositor-side [`StateMachine`] and client-side [`LookupTable`].
///
/// This type is usually created using [`Keymap::to_builder`] but can also be created
/// manually.
///
/// This type allows you to fully specify the behavior of a keyboard, which keys produce
/// which modifiers, which groups, and which keysyms.
///
/// # Example
///
/// ```
/// # use {
/// #     kbvm::{
/// #         builder::{Builder, GroupBuilder, KeyBuilder, LayerBuilder},
/// #         group_type::GroupType,
/// #         syms,
/// #         modifier::ModifierMask,
/// #         routine::Routine,
/// #         state_machine::{Direction::{Up, Down}, Keycode, LogicalEvent},
/// #         Components,
/// #     },
/// # };
/// // From /usr/include/linux/input-event-codes.h
/// const A: Keycode = Keycode::from_evdev(30);
/// const LEFT_SHIFT: Keycode = Keycode::from_evdev(42);
///
/// let mut builder = Builder::default();
///
/// // Define types for shift and alphabetic keys. These types determine how modifiers
/// // are mapped to layers. If no mapping is specified, the modifiers automatically
/// // map to the 0th layer.
/// let shift_key_type = GroupType::builder(ModifierMask::NONE).build();
/// let alphabetic_key_type = GroupType::builder(ModifierMask::SHIFT | ModifierMask::LOCK)
///     .map(ModifierMask::SHIFT, 1)
///     .map(ModifierMask::LOCK, 1)
///     .build();
///
/// // Define the A key.
/// {
///     // Lowercase letter on the first layer.
///     let mut first_layer = LayerBuilder::new(0);
///     first_layer.keysyms(&[syms::a]);
///     // Uppercase letter on the second layer.
///     let mut second_layer = LayerBuilder::new(1);
///     second_layer.keysyms(&[syms::A]);
///     // We only use a single group.
///     let mut group = GroupBuilder::new(0, &alphabetic_key_type);
///     group.add_layer(first_layer);
///     group.add_layer(second_layer);
///     let mut key = KeyBuilder::new(A);
///     key.add_group(group);
///     builder.add_key(key);
/// }
///
/// // Define the LEFT_SHIFT key.
/// {
///     // Use a custom routine to define how key-press and key-release affect the
///     // modifiers.
///     let routine = {
///         let mut routine = Routine::builder();
///         let [mods, key] = routine.allocate_vars();
///         routine
///             .load_lit(key, LEFT_SHIFT.to_raw())
///             .key_down(key)
///             .load_lit(mods, ModifierMask::SHIFT.0)
///             .pressed_mods_inc(mods)
///             .on_release()
///             .key_up(key)
///             .pressed_mods_dec(mods);
///         routine.build()
///     };
///     let mut layer = LayerBuilder::new(0);
///     layer.keysyms(&[syms::Shift_L]);
///     // Attach the routine to the first layer. The first part of the routine is
///     // executed when this layer is pressed. The second part, after `on_release`,
///     // is executed when the layer is released.
///     layer.routine(&routine);
///     let mut group = GroupBuilder::new(0, &shift_key_type);
///     group.add_layer(layer);
///     let mut key = KeyBuilder::new(LEFT_SHIFT);
///     key.add_group(group);
///     builder.add_key(key);
/// }
///
/// // Build the state machine and lookup table.
/// let state_machine = builder.build_state_machine();
/// let lookup_table = builder.build_lookup_table();
/// let mut state = state_machine.create_state();
/// let mut events = vec![];
///
/// // Simulate key press and release events.
/// state_machine.handle_key(&mut state, &mut events, A, Down);
/// state_machine.handle_key(&mut state, &mut events, A, Up);
/// state_machine.handle_key(&mut state, &mut events, LEFT_SHIFT, Down);
/// state_machine.handle_key(&mut state, &mut events, A, Down);
/// state_machine.handle_key(&mut state, &mut events, A, Up);
/// state_machine.handle_key(&mut state, &mut events, LEFT_SHIFT, Up);
/// state_machine.handle_key(&mut state, &mut events, A, Down);
/// state_machine.handle_key(&mut state, &mut events, A, Up);
///
/// // The components contain information about the currently effective group and
/// // modifiers.
/// let mut components = Components::default();
/// for event in events {
///     // Applying the event updates the group and modifiers.
///     components.apply_event(event);
///     if let LogicalEvent::KeyDown(kc) = event {
///         // Print the keysyms produced by this key press.
///         let syms = lookup_table.lookup(components.group, components.mods, kc);
///         for sym in syms {
///             println!("{}", sym.keysym().name().unwrap());
///         }
///     }
/// }
///
/// // Output:
/// // a
/// // Shift_L
/// // A
/// // a
/// ```
#[derive(Default, Debug)]
pub struct Builder {
    next_global: u32,
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

pub struct KeyBuilder {
    code: Keycode,
    key: BuilderKey,
}

pub struct GroupBuilder {
    idx: usize,
    group: BuilderGroup,
}

pub struct LayerBuilder {
    idx: usize,
    layer: BuilderLayer,
}

impl Builder {
    pub fn add_global(&mut self) -> Global {
        let g = Global(self.next_global);
        self.next_global = self.next_global.checked_add(1).unwrap();
        g
    }

    pub fn set_ctrl(&mut self, ctrl: Option<ModifierIndex>) {
        self.ctrl = ctrl.map(|c| c.to_mask());
    }

    pub fn set_caps(&mut self, caps: Option<ModifierIndex>) {
        self.caps = caps.map(|c| c.to_mask());
    }

    pub fn add_key(&mut self, key: KeyBuilder) {
        self.keys.insert(key.code, key.key);
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
            num_globals: self.next_global as usize,
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

impl KeyBuilder {
    pub fn new(key: Keycode) -> Self {
        Self {
            code: key,
            key: Default::default(),
        }
    }

    pub fn repeats(&mut self, repeats: bool) {
        self.key.repeats = repeats;
    }

    pub fn redirect(&mut self, redirect: Redirect) {
        self.key.redirect = redirect;
    }

    pub fn add_group(&mut self, group: GroupBuilder) {
        if self.key.groups.len() <= group.idx {
            self.key.groups.resize_with(group.idx + 1, Default::default);
        }
        self.key.groups[group.idx] = Some(group.group);
    }
}

impl GroupBuilder {
    pub fn new(group: usize, ty: &GroupType) -> Self {
        Self {
            idx: group,
            group: BuilderGroup {
                ty: ty.clone(),
                layers: vec![],
            },
        }
    }

    pub fn add_layer(&mut self, layer: LayerBuilder) {
        if self.group.layers.len() <= layer.idx {
            self.group
                .layers
                .resize_with(layer.idx + 1, Default::default);
        }
        self.group.layers[layer.idx] = Some(layer.layer);
    }
}

impl LayerBuilder {
    pub fn new(layer: usize) -> Self {
        Self {
            idx: layer,
            layer: Default::default(),
        }
    }

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
