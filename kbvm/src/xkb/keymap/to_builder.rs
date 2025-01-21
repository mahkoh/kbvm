#[expect(unused_imports)]
use {crate::lookup::LookupTable, crate::state_machine::StateMachine};
use {
    crate::{
        builder::{Builder, GroupBuilder, KeyBuilder, LevelBuilder},
        routine::{Routine, RoutineBuilder, Var},
        xkb::{
            group::GroupChange,
            keymap::{Action, KeyBehavior, KeyType},
            Keymap,
        },
        GroupType, Keycode, ModifierIndex,
    },
    hashbrown::HashMap,
    isnt::std_1::primitive::IsntSliceExt,
};

impl Keymap {
    /// Creates a [`Builder`] which can in turn create a (client-side) [`LookupTable`] or
    /// a (compositor-side) [`StateMachine`].
    ///
    /// The builder will represent the XKB logic required by this keymap.
    pub fn to_builder(&self) -> Builder {
        let mut builder = Builder::default();
        builder.set_ctrl(Some(ModifierIndex::CONTROL));
        builder.set_caps(Some(ModifierIndex::LOCK));
        let mut types = HashMap::new();
        for ty in &self.types {
            // println!("{:?}", ty);
            let mut builder = GroupType::builder(ty.modifiers);
            for mapping in &ty.mappings {
                builder.map_preserve(
                    mapping.modifiers,
                    mapping.preserved,
                    mapping.level.to_offset(),
                );
            }
            types.insert(&**ty as *const KeyType, builder.build());
        }
        for key in self.keys.values() {
            let mut kb = KeyBuilder::new(key.key_code);
            kb.repeats(key.repeat);
            kb.redirect(key.redirect.to_redirect());
            for (idx, group) in key.groups.iter().enumerate() {
                let Some(group) = group else {
                    continue;
                };
                let ty = &types[&(&*group.key_type as *const KeyType)];
                let mut gb = GroupBuilder::new(idx, ty);
                for (idx, level) in group.levels.iter().enumerate() {
                    if level.actions.is_empty() && level.symbols.is_empty() {
                        continue;
                    }
                    let mut lb = LevelBuilder::new(idx);
                    lb.keysyms(&level.symbols);
                    if level.actions.is_not_empty() {
                        lb.routine(&actions_to_routine(key.key_code, &level.actions));
                    }
                    gb.add_level(lb);
                }
                kb.add_group(gb);
            }
            if let Some(behavior) = &key.behavior {
                kb.routine(&behavior_to_routine(&mut builder, key.key_code, behavior));
            }
            builder.add_key(kb);
        }
        builder
    }
}

fn behavior_to_routine(b: &mut Builder, key: Keycode, behavior: &KeyBehavior) -> Routine {
    let mut builder = RoutineBuilder::default();
    match behavior {
        KeyBehavior::Lock => {
            let global = b.add_global();
            let [kc, state_old, state_new] = builder.allocate_vars();
            let anchor = builder
                .load_global(state_old, global)
                .prepare_skip_if(state_old);
            let anchor = builder
                .load_lit(state_new, 1)
                .store_global(global, state_new)
                .load_lit(kc, key.raw())
                .key_down(kc)
                .finish_skip(anchor)
                .on_release()
                .prepare_skip_if_not(state_old);
            builder
                .load_lit(state_new, 0)
                .store_global(global, state_new)
                .load_lit(kc, key.raw())
                .key_up(kc)
                .finish_skip(anchor);
        }
    }
    builder.build()
}

fn actions_to_routine(key: Keycode, actions: &[Action]) -> Routine {
    let mut builder = Routine::builder();
    let preserves_latch = actions.iter().any(|a| match a {
        Action::ModsSet(_)
        | Action::ModsLatch(_)
        | Action::ModsLock(_)
        | Action::GroupSet(_)
        | Action::GroupLatch(_)
        | Action::GroupLock(_) => true,
        Action::ControlsSet(_) | Action::ControlsLock(_) | Action::RedirectKey(_) => false,
    });
    let key = if actions.iter().any(|a| matches!(a, Action::RedirectKey(_))) {
        None
    } else {
        let kc = builder.allocate_var();
        builder.load_lit(kc, key.0).key_down(kc);
        if !preserves_latch {
            let zero = builder.allocate_var();
            builder
                .load_lit(zero, 0)
                .group_latched_store(zero)
                .mods_latched_store(zero);
        }
        Some(kc)
    };
    encode_actions(&mut builder, actions, key, preserves_latch);
    // let routine = builder.build();
    // println!("{:#?}", routine);
    // routine
    builder.build()
}

fn encode_actions(
    builder: &mut RoutineBuilder,
    actions: &[Action],
    key: Option<Var>,
    preserves_latch: bool,
) {
    let Some(action) = actions.first() else {
        builder.on_release();
        if let Some(key) = key {
            builder.key_up(key);
        }
        return;
    };
    macro_rules! encode_rest {
        () => {
            encode_actions(builder, &actions[1..], key, preserves_latch);
        };
    }
    match action {
        Action::ModsSet(ms) => {
            let action_mods = builder.allocate_var();
            builder
                .load_lit(action_mods, ms.modifiers.0)
                .mods_pressed_inc(action_mods);
            encode_rest!();
            builder.mods_pressed_dec(action_mods);
            if ms.clear_locks {
                let locked_mods = builder.allocate_var();
                let later_keys_activated = builder.allocate_var();
                builder.later_key_actuated_load(later_keys_activated);
                let anchor = builder.prepare_skip_if(later_keys_activated);
                builder
                    .mods_locked_load(locked_mods)
                    .bit_nand(locked_mods, locked_mods, action_mods)
                    .mods_locked_store(locked_mods)
                    .finish_skip(anchor);
            }
        }
        Action::ModsLatch(ml) => {
            let action_mods = builder.allocate_var();
            builder
                .load_lit(action_mods, ml.modifiers.0)
                .mods_pressed_inc(action_mods);
            encode_rest!();
            builder.mods_pressed_dec(action_mods);
            let latched_mods = builder.allocate_var();
            let locked_mods = builder.allocate_var();
            let later_keys_activated = builder.allocate_var();
            builder.later_key_actuated_load(later_keys_activated);
            let anchor = builder.prepare_skip_if(later_keys_activated);
            builder.mods_locked_load(locked_mods);
            if ml.clear_locks {
                let already_locked = builder.allocate_var();
                builder
                    .bit_and(already_locked, locked_mods, action_mods)
                    .bit_nand(locked_mods, locked_mods, already_locked)
                    .bit_nand(action_mods, action_mods, already_locked);
            }
            builder.mods_latched_load(latched_mods);
            if ml.latch_to_lock {
                let already_latched = builder.allocate_var();
                builder
                    .bit_and(already_latched, latched_mods, action_mods)
                    .bit_or(locked_mods, locked_mods, already_latched)
                    .bit_nand(latched_mods, latched_mods, already_latched)
                    .bit_nand(action_mods, action_mods, already_latched);
            }
            builder
                .bit_or(latched_mods, latched_mods, action_mods)
                .mods_latched_store(latched_mods)
                .mods_locked_store(locked_mods)
                .finish_skip(anchor);
        }
        Action::ModsLock(ml) => {
            let action_mods = builder.allocate_var();
            let originally_locked_mods = builder.allocate_var();
            builder
                .load_lit(action_mods, ml.modifiers.0)
                .mods_pressed_inc(action_mods);
            if ml.lock || ml.unlock {
                builder.mods_locked_load(originally_locked_mods);
            }
            if ml.lock {
                let locked_mods = builder.allocate_var();
                builder
                    .bit_or(locked_mods, originally_locked_mods, action_mods)
                    .mods_locked_store(locked_mods);
            }
            encode_rest!();
            builder.mods_pressed_dec(action_mods);
            if ml.unlock {
                let locked_mods = builder.allocate_var();
                let already_locked_mods = builder.allocate_var();
                builder
                    .mods_locked_load(locked_mods)
                    .bit_and(already_locked_mods, originally_locked_mods, action_mods)
                    .bit_nand(locked_mods, locked_mods, already_locked_mods)
                    .mods_locked_store(locked_mods);
            }
        }
        Action::GroupSet(gs) => {
            let group_delta = builder.allocate_var();
            let group = builder.allocate_var();
            builder.group_pressed_load(group);
            match gs.group {
                GroupChange::Absolute(g) => {
                    let new_group = builder.allocate_var();
                    builder.load_lit(new_group, g.to_offset() as u32).sub(
                        group_delta,
                        new_group,
                        group,
                    );
                }
                GroupChange::Rel(r) => {
                    builder.load_lit(group_delta, r as u32);
                }
            }
            builder
                .add(group, group, group_delta)
                .group_pressed_store(group);
            encode_rest!();
            builder
                .group_pressed_load(group)
                .sub(group, group, group_delta)
                .group_pressed_store(group);
            if gs.clear_locks {
                let other_key_actuated = builder.allocate_var();
                let group_one = builder.allocate_var();
                builder.later_key_actuated_load(other_key_actuated);
                let anchor = builder.prepare_skip_if(other_key_actuated);
                builder
                    .load_lit(group_one, 0)
                    .group_locked_store(group_one)
                    .finish_skip(anchor);
            }
        }
        Action::GroupLatch(gl) => {
            let group_delta = builder.allocate_var();
            let group = builder.allocate_var();
            builder.group_pressed_load(group);
            match gl.group {
                GroupChange::Absolute(g) => {
                    let new_group = builder.allocate_var();
                    builder.load_lit(new_group, g.to_offset() as u32).sub(
                        group_delta,
                        new_group,
                        group,
                    );
                }
                GroupChange::Rel(r) => {
                    builder.load_lit(group_delta, r as u32);
                }
            }
            builder
                .add(group, group, group_delta)
                .group_pressed_store(group);
            encode_rest!();
            builder
                .group_pressed_load(group)
                .sub(group, group, group_delta)
                .group_pressed_store(group);
            let mut clear_locks_anchor = None;
            let mut latch_to_lock_anchor = None;
            let other_key_actuated = builder.allocate_var();
            builder.later_key_actuated_load(other_key_actuated);
            let other_key_actuated_anchor = builder.prepare_skip_if(other_key_actuated);
            if gl.clear_locks {
                let locked_group = builder.allocate_var();
                let group_one = builder.allocate_var();
                let group_changed = builder.allocate_var();
                builder
                    .group_locked_load(locked_group)
                    .load_lit(group_one, 0)
                    .group_locked_store(group_one)
                    .ne(group_changed, locked_group, group_one);
                clear_locks_anchor = Some(builder.prepare_skip_if(group_changed));
            }
            let latched_group = builder.allocate_var();
            builder.group_latched_load(latched_group);
            if gl.latch_to_lock {
                let locked_group = builder.allocate_var();
                let anchor = builder.prepare_skip_if_not(latched_group);
                builder
                    .sub(latched_group, latched_group, group_delta)
                    .group_latched_store(latched_group)
                    .group_locked_load(locked_group)
                    .add(locked_group, locked_group, group_delta)
                    .group_locked_store(locked_group);
                latch_to_lock_anchor = Some(builder.prepare_skip());
                builder.finish_skip(anchor);
            }
            builder
                .add(latched_group, latched_group, group_delta)
                .group_latched_store(latched_group);
            builder.finish_skip(other_key_actuated_anchor);
            if let Some(clear_locks_anchor) = clear_locks_anchor {
                builder.finish_skip(clear_locks_anchor);
            }
            if let Some(latch_to_lock_anchor) = latch_to_lock_anchor {
                builder.finish_skip(latch_to_lock_anchor);
            }
        }
        Action::GroupLock(gl) => {
            let group = builder.allocate_var();
            match gl.group {
                GroupChange::Absolute(g) => {
                    builder
                        .load_lit(group, g.to_offset() as u32)
                        .group_locked_store(group);
                }
                GroupChange::Rel(r) => {
                    let group_delta = builder.allocate_var();
                    builder
                        .load_lit(group_delta, r as u32)
                        .group_locked_load(group)
                        .add(group, group, group_delta)
                        .group_locked_store(group);
                }
            }
            encode_rest!();
        }
        Action::RedirectKey(rk) => {
            if rk.modifier_mask().0 == 0 {
                // simple case where modifiers are not changed
                let key = builder.allocate_var();
                builder.load_lit(key, rk.key_code().raw()).key_down(key);
                if !preserves_latch {
                    let zero = builder.allocate_var();
                    builder
                        .load_lit(zero, 0)
                        .mods_latched_store(zero)
                        .group_latched_store(zero);
                }
                encode_rest!();
                builder.key_up(key);
                return;
            }
            let [zero, key, mods, pressed, latched, locked, effective] = builder.allocate_vars();
            let encode_transform = |builder: &mut RoutineBuilder| {
                builder
                    .mods_pressed_load(pressed)
                    .mods_latched_load(latched)
                    .mods_locked_load(locked)
                    .bit_or(effective, pressed, latched)
                    .bit_or(effective, effective, locked);
                if rk.mods_to_clear().0 != 0 {
                    builder
                        .load_lit(mods, rk.mods_to_clear().0)
                        .bit_nand(effective, effective, mods);
                }
                if rk.mods_to_set().0 != 0 {
                    builder
                        .load_lit(mods, rk.mods_to_set().0)
                        .bit_or(effective, effective, mods);
                }
                builder
                    .mods_pressed_store(effective)
                    .load_lit(zero, 0)
                    .mods_latched_store(zero)
                    .mods_locked_store(zero);
            };
            encode_transform(builder);
            builder
                .load_lit(key, rk.key_code().raw())
                .key_down(key)
                .mods_pressed_store(pressed)
                .mods_locked_store(locked);
            if preserves_latch {
                builder.mods_latched_store(latched);
            } else {
                builder.group_latched_store(zero);
            }
            encode_rest!();
            encode_transform(builder);
            builder
                .key_up(key)
                .mods_pressed_store(pressed)
                .mods_latched_store(latched)
                .mods_locked_store(locked);
        }
        Action::ControlsSet(cs) => {
            let [controls, change] = builder.allocate_vars();
            builder
                .load_lit(change, cs.controls.0 as u32)
                .controls_load(controls)
                .bit_or(controls, controls, change)
                .controls_store(controls);
            encode_rest!();
            builder
                .controls_load(controls)
                .bit_nand(controls, controls, change)
                .controls_store(controls);
        }
        Action::ControlsLock(cl) => {
            let [controls, common, change] = builder.allocate_vars();
            if cl.lock || cl.unlock {
                builder
                    .load_lit(change, cl.controls.0 as u32)
                    .controls_load(controls)
                    .bit_and(common, controls, change);
            }
            if cl.lock {
                builder
                    .bit_or(controls, controls, change)
                    .controls_store(controls);
            }
            encode_rest!();
            if cl.unlock {
                builder
                    .controls_load(controls)
                    .bit_nand(controls, controls, common)
                    .controls_store(controls);
            }
        }
    }
}
