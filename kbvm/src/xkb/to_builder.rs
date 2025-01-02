use {
    crate::{
        builder::Builder,
        group_type::GroupType,
        routine::{Routine, RoutineBuilder, SkipAnchor},
        xkb::{
            keymap::{Action, KeyType},
            Keymap,
        },
    },
    hashbrown::HashMap,
};

impl Keymap {
    pub fn to_builder(&self) -> Builder {
        let mut builder = Builder::default();
        let mut types = HashMap::new();
        for ty in &self.types {
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
            let mut builder = builder.add_key(key.key_code);
            for (idx, group) in key.groups.iter().enumerate() {
                let Some(group) = group else {
                    continue;
                };
                let ty = &types[&(&*group.key_type as *const KeyType)];
                let mut builder = builder.add_group(idx, ty);
                for (idx, layer) in group.levels.iter().enumerate() {
                    if layer.actions.is_empty() {
                        continue;
                    }
                    let mut builder = builder.add_layer(idx);
                    builder.routine(&actions_to_routine(&layer.actions));
                }
            }
        }
        builder
    }
}

fn actions_to_routine(actions: &[Action]) -> Routine {
    let mut builder = Routine::builder();
    encode_actions(&mut builder, actions);
    builder.build()
}

fn encode_actions(builder: &mut RoutineBuilder, actions: &[Action]) {
    let Some(action) = actions.first() else {
        builder.on_release();
        return;
    };
    macro_rules! encode_rest {
        () => {
            encode_actions(builder, &actions[1..]);
        };
    }
    match action {
        Action::ModsSet(ms) => {
            let action_mods = builder.allocate_var();
            builder
                .load_lit(action_mods, ms.modifiers.0)
                .pressed_mods_inc(action_mods);
            encode_rest!();
            builder.pressed_mods_dec(action_mods);
            if ms.clear_locks {
                let mut anchor = SkipAnchor::default();
                let locked_mods = builder.allocate_var();
                let later_keys_activated = builder.allocate_var();
                builder
                    .later_key_activated_load(later_keys_activated)
                    .prepare_conditional_skip(later_keys_activated, false, &mut anchor)
                    .locked_mods_load(locked_mods)
                    .bit_nand(locked_mods, locked_mods, action_mods)
                    .locked_mods_store(locked_mods)
                    .finish_skip(&mut anchor);
            }
        }
        Action::ModsLatch(ml) => {
            let action_mods = builder.allocate_var();
            builder
                .load_lit(action_mods, ml.modifiers.0)
                .pressed_mods_inc(action_mods);
            encode_rest!();
            builder.pressed_mods_dec(action_mods);
            let mut anchor = SkipAnchor::default();
            let latched_mods = builder.allocate_var();
            let locked_mods = builder.allocate_var();
            let later_keys_activated = builder.allocate_var();
            builder
                .later_key_activated_load(later_keys_activated)
                .prepare_conditional_skip(later_keys_activated, false, &mut anchor)
                .locked_mods_load(locked_mods);
            if ml.clear_locks {
                let already_locked = builder.allocate_var();
                builder
                    .bit_and(already_locked, locked_mods, action_mods)
                    .bit_nand(locked_mods, locked_mods, already_locked)
                    .bit_nand(action_mods, action_mods, already_locked);
            }
            builder.latched_mods_load(latched_mods);
            if ml.latch_to_lock {
                let already_latched = builder.allocate_var();
                builder
                    .bit_and(already_latched, locked_mods, action_mods)
                    .bit_or(locked_mods, locked_mods, already_latched)
                    .bit_nand(action_mods, action_mods, already_latched);
            }
            builder
                .bit_or(latched_mods, latched_mods, action_mods)
                .latched_mods_store(latched_mods)
                .locked_mods_store(locked_mods)
                .finish_skip(&mut anchor);
        }
        Action::ModsLock(ml) => {
            let action_mods = builder.allocate_var();
            let originally_locked_mods = builder.allocate_var();
            builder
                .load_lit(action_mods, ml.modifiers.0)
                .pressed_mods_inc(action_mods);
            if ml.lock || ml.unlock {
                builder.locked_mods_load(originally_locked_mods);
            }
            if ml.lock {
                let locked_mods = builder.allocate_var();
                builder
                    .bit_or(locked_mods, originally_locked_mods, action_mods)
                    .locked_mods_store(locked_mods);
            }
            encode_rest!();
            builder.pressed_mods_dec(action_mods);
            if ml.unlock {
                let locked_mods = builder.allocate_var();
                let already_locked_mods = builder.allocate_var();
                builder
                    .locked_mods_load(locked_mods)
                    .bit_and(already_locked_mods, originally_locked_mods, action_mods)
                    .bit_nand(locked_mods, locked_mods, already_locked_mods)
                    .locked_mods_store(locked_mods);
            }
        }
        Action::GroupSet(_) => {}
        Action::GroupLatch(_) => {}
        Action::GroupLock(_) => {}
    }
}
