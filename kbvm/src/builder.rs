//! A [`StateMachine`] and [`LookupTable`] builder.
//!
//! See the documentation of [`Builder`] for examples showing how to use this module.

#[cfg(test)]
mod tests;

#[expect(unused_imports)]
use crate::{routine::RoutineBuilder, state_machine::State, xkb::Keymap};
use {
    crate::{
        group::GroupIndex,
        lookup::{self, LookupTable},
        routine::{Global, Routine},
        state_machine::{self, StateMachine},
        GroupType, Keycode, Keysym, ModifierIndex, ModifierMask,
    },
    hashbrown::HashMap,
    isnt::std_1::primitive::IsntSliceExt,
    smallvec::SmallVec,
};

/// A builder for compositor-side [`StateMachine`]s and client-side [`LookupTable`]s.
///
/// This type is usually created using [`Keymap::to_builder`] but can also be created
/// manually.
///
/// This type allows you to fully specify the behavior of a keyboard, which keys produce
/// which modifiers, which groups, and which keysyms.
///
/// # Creating a Builder from an XKB map
///
/// ```
/// # use kbvm::{evdev, Components};
/// # use kbvm::state_machine::Direction::{Down, Up};
/// # use kbvm::state_machine::Event;
/// # use kbvm::xkb::Context;
/// # use kbvm::xkb::diagnostic::WriteToLog;
/// #
/// const KEYMAP: &str = r#"
///     xkb_keymap {
///         xkb_keycodes {
///             <a> = 38;
///             <leftshift> = 50;
///         };
///         xkb_types {
///             type "SHIFT_KEY_TYPE" {
///                 modifiers = None;
///             };
///             type "ALPHABETIC_KEY_TYPE" {
///                 modifiers = Shift + Lock;
///                 map[Shift] = Level2;
///                 map[Lock] = Level2;
///             };
///         };
///         xkb_symbols {
///             key <leftshift> {
///                 type = "SHIFT_KEY_TYPE",
///                 [ Shift_L ],
///                 [ SetMods(mods = Shift) ],
///             };
///             key <a> {
///                 type = "ALPHABETIC_KEY_TYPE",
///                 [ a, A ],
///             };
///         };
///     };
/// "#;
///
/// // Create an XKB context.
/// let context = Context::default();
/// // Parse the keymap.
/// let keymap = context
///     .keymap_from_bytes(WriteToLog, None, KEYMAP.as_bytes())
///     .unwrap();
/// // Convert the keymap to a builder.
/// let builder = keymap.to_builder();
///
/// // Build the state machine and lookup table.
/// let state_machine = builder.build_state_machine();
/// let lookup_table = builder.build_lookup_table();
/// let mut state = state_machine.create_state();
/// let mut events = vec![];
///
/// // Simulate key press and release events.
/// state_machine.handle_key(&mut state, &mut events, evdev::A, Down);
/// state_machine.handle_key(&mut state, &mut events, evdev::A, Up);
/// state_machine.handle_key(&mut state, &mut events, evdev::LEFTSHIFT, Down);
/// state_machine.handle_key(&mut state, &mut events, evdev::A, Down);
/// state_machine.handle_key(&mut state, &mut events, evdev::A, Up);
/// state_machine.handle_key(&mut state, &mut events, evdev::LEFTSHIFT, Up);
/// state_machine.handle_key(&mut state, &mut events, evdev::A, Down);
/// state_machine.handle_key(&mut state, &mut events, evdev::A, Up);
///
/// // The components contain information about the currently effective group and
/// // modifiers.
/// let mut components = Components::default();
/// for event in events {
///     // Applying the event updates the group and modifiers.
///     components.apply_event(event);
///     if let Event::KeyDown(kc) = event {
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
///
/// # Creating a Builder manually
///
/// ```
/// # use {
/// #     kbvm::{
/// #         builder::{Builder, GroupBuilder, KeyBuilder, LevelBuilder},
/// #         GroupType,
/// #         syms,
/// #         evdev,
/// #         ModifierMask,
/// #         routine::Routine,
/// #         state_machine::{Direction::{Up, Down}, Event},
/// #         Components,
/// #         Keycode,
/// #     },
/// # };
/// let mut builder = Builder::default();
///
/// // Define types for shift and alphabetic keys. These types determine how modifiers
/// // are mapped to levels. If no mapping is specified, the modifiers automatically
/// // map to the 0th level.
/// let shift_key_type = GroupType::builder(ModifierMask::NONE).build();
/// let alphabetic_key_type = GroupType::builder(ModifierMask::SHIFT | ModifierMask::LOCK)
///     .map(ModifierMask::SHIFT, 1)
///     .map(ModifierMask::LOCK, 1)
///     .build();
///
/// // Define the A key.
/// {
///     // Lowercase letter on the first level.
///     let mut first_level = LevelBuilder::new(0);
///     first_level.keysyms(&[syms::a]);
///     // Uppercase letter on the second level.
///     let mut second_level = LevelBuilder::new(1);
///     second_level.keysyms(&[syms::A]);
///     // We only use a single group.
///     let mut group = GroupBuilder::new(0, &alphabetic_key_type);
///     group.add_level(first_level);
///     group.add_level(second_level);
///     let mut key = KeyBuilder::new(evdev::A);
///     key.add_group(group);
///     builder.add_key(key);
/// }
///
/// // Define the LEFTSHIFT key.
/// {
///     // Use a custom routine to define how key-press and key-release affect the
///     // modifiers.
///     let routine = {
///         let mut routine = Routine::builder();
///         let [mods, key] = routine.allocate_vars();
///         routine
///             .load_lit(key, evdev::LEFTSHIFT.raw())
///             .key_down(key)
///             .load_lit(mods, ModifierMask::SHIFT.0)
///             .mods_pressed_inc(mods)
///             .on_release()
///             .key_up(key)
///             .mods_pressed_dec(mods);
///         routine.build()
///     };
///     let mut level = LevelBuilder::new(0);
///     level.keysyms(&[syms::Shift_L]);
///     // Attach the routine to the first level. The first part of the routine is
///     // executed when this level is pressed. The second part, after `on_release`,
///     // is executed when the level is released.
///     level.routine(&routine);
///     let mut group = GroupBuilder::new(0, &shift_key_type);
///     group.add_level(level);
///     let mut key = KeyBuilder::new(evdev::LEFTSHIFT);
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
/// state_machine.handle_key(&mut state, &mut events, evdev::A, Down);
/// state_machine.handle_key(&mut state, &mut events, evdev::A, Up);
/// state_machine.handle_key(&mut state, &mut events, evdev::LEFTSHIFT, Down);
/// state_machine.handle_key(&mut state, &mut events, evdev::A, Down);
/// state_machine.handle_key(&mut state, &mut events, evdev::A, Up);
/// state_machine.handle_key(&mut state, &mut events, evdev::LEFTSHIFT, Up);
/// state_machine.handle_key(&mut state, &mut events, evdev::A, Down);
/// state_machine.handle_key(&mut state, &mut events, evdev::A, Up);
///
/// // The components contain information about the currently effective group and
/// // modifiers.
/// let mut components = Components::default();
/// for event in events {
///     // Applying the event updates the group and modifiers.
///     components.apply_event(event);
///     if let Event::KeyDown(kc) = event {
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
#[derive(Clone, Default, Debug)]
pub struct Builder {
    next_global: u32,
    ctrl: Option<ModifierMask>,
    caps: Option<ModifierMask>,
    keys: HashMap<Keycode, BuilderKey>,
}

/// A group-redirect setting.
///
/// When a group is out-of-bounds, it will be redirected according to this setting.
#[derive(Copy, Clone, Default, Debug, Eq, PartialEq)]
pub enum Redirect {
    /// The group is wrapped modulo the number of available groups.
    #[default]
    Wrap,
    /// The group is clamped to the first or last group.
    Clamp,
    /// The group is set to a fixed group.
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
                Redirect::Clamp => {
                    if (group.0 as i32) < 0 {
                        0
                    } else {
                        len - 1
                    }
                }
                Redirect::Fixed(f) => f,
            }
        } else {
            n
        }
    }
}

#[derive(Clone, Default, Debug)]
struct BuilderKey {
    repeats: bool,
    groups: Vec<Option<BuilderGroup>>,
    redirect: Redirect,
    routine: Option<Routine>,
}

#[derive(Clone, Debug)]
struct BuilderGroup {
    ty: GroupType,
    levels: Vec<Option<BuilderLevel>>,
}

#[derive(Clone, Default, Debug)]
struct BuilderLevel {
    keysyms: SmallVec<[Keysym; 1]>,
    routine: Option<Routine>,
}

/// A builder for a key.
///
/// # Example
///
/// ```
/// # use kbvm::builder::{GroupBuilder, KeyBuilder, LevelBuilder};
/// # use kbvm::{evdev, syms};
/// # use kbvm::GroupType;
/// # use kbvm::ModifierMask;
/// let group_type = GroupType::builder(ModifierMask::NONE).build();
/// let mut level = LevelBuilder::new(0);
/// level.keysyms(&[syms::A]);
/// let mut group = GroupBuilder::new(0, &group_type);
/// group.add_level(level);
/// let mut builder = KeyBuilder::new(evdev::A);
/// builder.add_group(group);
/// ```
#[derive(Clone, Debug)]
pub struct KeyBuilder {
    code: Keycode,
    key: BuilderKey,
}

/// A builder for a key group.
///
/// # Example
///
/// ```
/// # use kbvm::builder::{GroupBuilder, LevelBuilder};
/// # use kbvm::{syms};
/// # use kbvm::GroupType;
/// # use kbvm::ModifierMask;
/// let group_type = GroupType::builder(ModifierMask::NONE).build();
/// let mut level = LevelBuilder::new(0);
/// level.keysyms(&[syms::A]);
/// let mut group = GroupBuilder::new(0, &group_type);
/// group.add_level(level);
/// ```
#[derive(Clone, Debug)]
pub struct GroupBuilder {
    idx: usize,
    group: BuilderGroup,
}

/// A builder for a key level.
///
/// # Example
///
/// ```
/// # use kbvm::builder::{LevelBuilder};
/// # use kbvm::{syms};
/// let mut level = LevelBuilder::new(0);
/// level.keysyms(&[syms::A]);
/// ```
#[derive(Clone, Debug)]
pub struct LevelBuilder {
    idx: usize,
    level: BuilderLevel,
}

impl Builder {
    /// Adds a global to this builder.
    ///
    /// The global can be used via [`RoutineBuilder::store_global`] and
    /// [`RoutineBuilder::load_global`] in the routines attached to this builder.
    ///
    /// Each global consumes 4 bytes of space in [`State`] objects created from this
    /// builder.
    pub fn add_global(&mut self) -> Global {
        let g = Global(self.next_global);
        self.next_global = self.next_global.checked_add(1).unwrap();
        g
    }

    /// Sets the ctrl modifier index for this builder.
    ///
    /// This is used to determine when to perform ctrl transformation as described in the
    /// documentation of [`LookupTable`].
    ///
    /// Unless you have a good reason to do something else, you should set this to
    /// [`ModifierIndex::CONTROL`].
    pub fn set_ctrl(&mut self, ctrl: Option<ModifierIndex>) {
        self.ctrl = ctrl.map(|c| c.to_mask());
    }

    /// Sets the caps modifier index for this builder.
    ///
    /// This is used to determine when to perform caps transformation as described in the
    /// documentation of [`LookupTable`].
    ///
    /// Unless you have a good reason to do something else, you should set this to
    /// [`ModifierIndex::LOCK`].
    pub fn set_caps(&mut self, caps: Option<ModifierIndex>) {
        self.caps = caps.map(|c| c.to_mask());
    }

    /// Adds a key to this builder.
    ///
    /// If a key with the same keycode already exists, it is overwritten.
    pub fn add_key(&mut self, key: KeyBuilder) {
        self.keys.insert(key.code, key.key);
    }

    /// Builds a compositor-side [`StateMachine`].
    pub fn build_state_machine(&self) -> StateMachine {
        let mut map = HashMap::with_capacity(self.keys.len());
        let mut has_layer1 = false;
        let mut num_groups = 0;
        for (keycode, key) in &self.keys {
            let mut groups = Vec::with_capacity(key.groups.len());
            num_groups = num_groups.max(key.groups.len());
            for group in &key.groups {
                match group {
                    None => groups.push(None),
                    Some(g) => {
                        let mut levels = Vec::with_capacity(g.levels.len());
                        for level in &g.levels {
                            match level {
                                None => levels.push(state_machine::KeyLevel::default()),
                                Some(l) => levels.push(state_machine::KeyLevel {
                                    routine: l.routine.clone(),
                                }),
                            }
                        }
                        while let Some(level) = levels.last() {
                            if level.routine.is_none() {
                                levels.pop();
                            } else {
                                break;
                            }
                        }
                        if levels.is_empty() {
                            groups.push(None);
                        } else {
                            groups.push(Some(state_machine::KeyGroup {
                                ty: g.ty.clone(),
                                levels: levels.into_boxed_slice(),
                            }));
                        }
                    }
                }
            }
            if groups.iter().all(|g| g.is_none()) {
                groups.clear();
            }
            if groups.is_not_empty() || key.routine.is_some() {
                has_layer1 |= key.routine.is_some();
                map.insert(
                    *keycode,
                    state_machine::KeyGroups {
                        redirect: key.redirect.constrain(groups.len()),
                        groups: groups.into_boxed_slice(),
                        routine: key.routine.clone(),
                    },
                );
            }
        }
        StateMachine {
            num_groups: (num_groups as u32).max(1),
            num_globals: self.next_global as usize,
            keys: map_to_vec(map),
            has_layer1,
        }
    }

    /// Builds a client-side [`LookupTable`].
    pub fn build_lookup_table(&self) -> LookupTable {
        let mut map = HashMap::with_capacity(self.keys.len());
        for (keycode, key) in &self.keys {
            let mut any_groups = false;
            let mut groups = Vec::with_capacity(key.groups.len());
            for group in &key.groups {
                match group {
                    None => groups.push(None),
                    Some(g) => {
                        let mut any_levels = false;
                        let mut levels = Vec::with_capacity(g.levels.len());
                        for level in &g.levels {
                            match level {
                                None => levels.push(lookup::KeyLevel::default()),
                                Some(l) => {
                                    any_levels |= l.keysyms.is_not_empty();
                                    levels.push(lookup::KeyLevel {
                                        symbols: l.keysyms.clone(),
                                    })
                                }
                            }
                        }
                        any_groups |= any_levels;
                        groups.push(Some(lookup::KeyGroup {
                            ty: g.ty.clone(),
                            levels: levels.into_boxed_slice(),
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
            keys: map_to_vec(map),
        }
    }
}

fn map_to_vec<T>(map: HashMap<Keycode, T>) -> Vec<Option<T>> {
    let len = map
        .keys()
        .map(|k| k.0 as usize + 1)
        .max()
        .unwrap_or_default();
    let mut keys: Vec<_> = std::iter::repeat_with(|| None).take(len).collect();
    for (k, v) in map {
        keys[k.0 as usize] = Some(v);
    }
    keys
}

impl KeyBuilder {
    /// Creates a new builder for the given keycode.
    pub fn new(key: Keycode) -> Self {
        Self {
            code: key,
            key: Default::default(),
        }
    }

    /// Sets the repeat setting of this key.
    ///
    /// The default setting is `false`.
    ///
    /// This should usually be set to `true` except for keys affecting modifiers and such.
    pub fn repeats(&mut self, repeats: bool) {
        self.key.repeats = repeats;
    }

    /// Sets the redirect setting of this key.
    ///
    /// The default setting is [`Redirect::Wrap`].
    ///
    /// If the effective group is out-of-bounds, it will be redirected according to this
    /// setting.
    pub fn redirect(&mut self, redirect: Redirect) {
        self.key.redirect = redirect;
    }

    /// Adds a group to this key.
    ///
    /// If a group with the same index already exists, it is overwritten.
    pub fn add_group(&mut self, group: GroupBuilder) {
        if self.key.groups.len() <= group.idx {
            self.key.groups.resize_with(group.idx + 1, Default::default);
        }
        self.key.groups[group.idx] = Some(group.group);
    }

    /// Sets the routine of this key.
    ///
    /// This is a more advanced feature than [`LevelBuilder::routine`] and usually not
    /// required.
    ///
    /// If no routine is set, then the default routine executes the following steps:
    ///
    /// - On press:
    ///   - `key_down` for the keycode.
    /// - On release:
    ///   - `key_up` for the keycode.
    ///
    /// This routine can be used to implement key locking, radio groups, overlays, etc. It
    /// is executed before the state machine determines which groups to use. The keycode
    /// used in `key_down` instructions determines how to retrieve the groups.
    ///
    /// The routine can use `key_down` and `key_up` as many times as it likes.
    ///
    /// The routine can neither access nor manipulate the components (modifiers, groups)
    /// of the state machine. Trying to access them always returns 0, trying to modify
    /// them has no effect. The same applies to the `later_key_actuated` flag.
    ///
    /// If the routine uses globals, these globals should have been allocated via
    /// [`Builder::add_global`] of the builder that this key is ultimately attached to.
    pub fn routine(&mut self, routine: &Routine) -> &mut Self {
        self.key.routine = Some(routine.clone());
        self
    }
}

impl GroupBuilder {
    /// Creates a new builder for the given index and group type.
    pub fn new(idx: usize, ty: &GroupType) -> Self {
        Self {
            idx,
            group: BuilderGroup {
                ty: ty.clone(),
                levels: vec![],
            },
        }
    }

    /// Adds a level to the group.
    ///
    /// If a level with the same index already exists, it is overwritten.
    pub fn add_level(&mut self, level: LevelBuilder) {
        if self.group.levels.len() <= level.idx {
            self.group
                .levels
                .resize_with(level.idx + 1, Default::default);
        }
        self.group.levels[level.idx] = Some(level.level);
    }
}

impl LevelBuilder {
    /// Creates a new builder for the given index.
    pub fn new(idx: usize) -> Self {
        Self {
            idx,
            level: Default::default(),
        }
    }

    /// Sets the routine of this level.
    ///
    /// If no routine is set, then the default routine executes the following steps:
    ///
    /// - On press:
    ///   - Emit a KeyDown event for the keycode of this level.
    ///   - Set the latched modifiers to 0.
    ///   - Set the latched group to 0.
    /// - On release:
    ///   - Emit a KeyUp event for the keycode of this level.
    ///
    /// If the routine uses globals, these globals should have been allocated via
    /// [`Builder::add_global`] of the builder that this level is ultimately attached to.
    pub fn routine(&mut self, routine: &Routine) -> &mut Self {
        self.level.routine = Some(routine.clone());
        self
    }

    /// Sets the keysyms of this level.
    pub fn keysyms(&mut self, keysyms: &[Keysym]) -> &mut Self {
        self.level.keysyms.clear();
        self.level.keysyms.extend_from_slice(keysyms);
        self
    }
}
