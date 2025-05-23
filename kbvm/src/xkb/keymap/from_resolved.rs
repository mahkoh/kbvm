use {
    crate::{
        xkb::{
            controls::ControlMask,
            group::GroupIdx,
            interner::{Interned, Interner},
            keymap::{
                actions::{ControlsLockAction, ControlsSetAction, RedirectKeyAction},
                Action, GroupLatchAction, GroupLockAction, GroupSetAction, Indicator, Key,
                KeyBehavior, KeyGroup, KeyLevel, KeyType, KeyTypeMapping, Keycode, ModMapValue,
                ModsLatchAction, ModsLockAction, ModsSetAction, OverlayBehavior,
                RadioGroupBehavior, VirtualModifier,
            },
            level::Level,
            mod_component::ModComponentMask,
            resolved::{
                BuiltInKeytype, GroupsRedirect, KeyTypeRef, ModMapField, Resolved, ResolvedAction,
                ResolvedActionMods, ResolvedKeyKind, SymbolsKeyBehavior, SymbolsKeyGroup,
            },
            span::{Despan, Spanned},
            Keymap,
        },
        ModifierMask,
    },
    arrayvec::ArrayVec,
    bstr::ByteSlice,
    hashbrown::{hash_map::Entry, HashMap, HashSet},
    linearize::{static_map, Linearize, StaticMap},
    std::sync::Arc,
};

impl Keymap {
    pub(crate) fn from_resolved(interner: &Interner, resolved: &Resolved) -> Self {
        let mut string_cache = HashMap::<_, Arc<String>>::new();
        let mut get_string = |i: Interned| match string_cache.entry(i) {
            Entry::Occupied(o) => o.get().clone(),
            Entry::Vacant(v) => {
                let s = Arc::new(interner.get(i).as_bstr().to_string());
                v.insert(s.clone());
                s
            }
        };
        let mut virtual_modifiers = Vec::with_capacity(resolved.mods.len());
        for m in &resolved.mods {
            virtual_modifiers.push(VirtualModifier {
                name: get_string(m.name.val),
                values: m.def.get().despan().unwrap_or_default(),
            });
        }
        virtual_modifiers.sort_unstable_by(|l, r| l.name.cmp(&r.name));
        if virtual_modifiers.is_empty() {
            // https://gitlab.freedesktop.org/xorg/xserver/-/issues/1774
            virtual_modifiers.push(VirtualModifier::dummy());
        }
        let mut types_by_ident = HashMap::new();
        let mut types = Vec::with_capacity(resolved.types.key_types.len() + 8);
        let mut used_types = HashSet::new();
        for ty in resolved.types.key_types.values() {
            let mut mappings = Vec::with_capacity(ty.ty.map.len());
            for (n, v, p) in &ty.ty.map_preserve {
                mappings.push(KeyTypeMapping {
                    modifiers: *n,
                    preserved: p.despan().unwrap_or_default(),
                    level: v.val,
                });
            }
            mappings.sort_unstable_by_key(|k| (k.level, k.modifiers.0));
            let mut level_names = Vec::with_capacity(ty.ty.names.len());
            for (level, name) in &ty.ty.names {
                level_names.push((*level, get_string(name.val)));
            }
            level_names.sort_unstable_by_key(|l| l.0);
            let t = KeyType {
                name: get_string(ty.name.val),
                modifiers: ty
                    .ty
                    .modifiers
                    .as_ref()
                    .map(|v| v.val.get_effective())
                    .unwrap_or_default(),
                mappings,
                level_names,
            };
            let t = Arc::new(t);
            types_by_ident.insert(ty.name.val, t.clone());
            types.push(t);
        }
        let default_key_types = create_used_default_key_types(
            interner,
            &virtual_modifiers,
            resolved,
            &mut types,
            &mut types_by_ident,
        );
        types.sort_unstable_by(|l, r| l.name.cmp(&r.name));
        let mut indicators = Vec::with_capacity(resolved.compat.indicator_maps.len());
        for i in resolved.compat.indicator_maps.values() {
            let map = &i.indicator_map;
            let modifier_mask = map
                .modifiers
                .as_ref()
                .map(|v| v.val.get_effective())
                .unwrap_or_default();
            let group_mask = map.groups.despan().unwrap_or_default();
            let mut mod_components = map.which_modifier_state.despan().unwrap_or_default();
            if modifier_mask.0 == 0 {
                mod_components = ModComponentMask::NONE;
            } else if mod_components == ModComponentMask::NONE {
                mod_components = ModComponentMask::EFFECTIVE;
            }
            let group_components = map.which_group_state.despan().unwrap_or_default();
            let i = Indicator {
                virt: map.virt,
                index: match map.idx {
                    Some(i) => i,
                    None => continue,
                },
                name: get_string(i.name.val),
                modifier_mask,
                group_mask,
                controls: map.controls.despan().unwrap_or_default(),
                mod_components,
                group_component: group_components,
            };
            indicators.push(i);
        }
        for i in &resolved.keycodes.indicators {
            if resolved.compat.indicator_maps.contains_key(&i.name.val) {
                continue;
            }
            let i = Indicator {
                virt: i.virt.is_some(),
                index: i.idx.val,
                name: get_string(i.name.val),
                modifier_mask: Default::default(),
                group_mask: Default::default(),
                controls: Default::default(),
                mod_components: Default::default(),
                group_component: Default::default(),
            };
            indicators.push(i);
        }
        indicators.sort_unstable_by_key(|i| i.index.raw());
        if indicators.is_empty() {
            // https://gitlab.freedesktop.org/xorg/xserver/-/issues/1775
            indicators.push(Indicator::dummy());
        }
        let mut mod_maps = HashSet::with_capacity(resolved.symbols.mod_map_entries.len());
        for m in resolved.symbols.mod_map_entries.values() {
            if let Some(idx) = m.modifier {
                let (kc, ks) = match m.key.val {
                    ModMapField::Keysym(s, Some(k)) => (k, Some(s)),
                    ModMapField::Keycode(k) => (k, None),
                    _ => continue,
                };
                if !resolved.symbols.keys.contains_key(&kc) {
                    continue;
                }
                if let Some(name) = resolved.keycodes.keycode_to_name.get(&kc) {
                    mod_maps.insert((
                        idx.val,
                        ModMapValue {
                            key_name: get_string(*name),
                            key_sym: ks,
                        },
                    ));
                }
            }
        }
        let mut mod_maps = mod_maps.into_iter().collect::<Vec<_>>();
        mod_maps.sort_unstable();
        let mut group_names = Vec::with_capacity(resolved.symbols.group_names.len());
        for (idx, m) in resolved.symbols.group_names.iter().flatten() {
            group_names.push((*idx, get_string(m.val)));
        }
        let mut used_key_names = HashSet::new();
        let mut keys = Vec::with_capacity(resolved.symbols.keys.len());
        for key in resolved.symbols.keys.values() {
            let mut groups: Vec<_> = key
                .key
                .groups
                .iter()
                .map(|g| {
                    map_symbol_groups(
                        &mut used_types,
                        &mut used_key_names,
                        &mut get_string,
                        &default_key_types,
                        &types_by_ident,
                        key.key.modmap,
                        g,
                    )
                })
                .collect();
            while let Some(None) = groups.last() {
                groups.pop();
            }
            used_key_names.insert(key.name.val);
            let mut redirect = key
                .key
                .groups_redirect
                .despan()
                .unwrap_or(GroupsRedirect::Wrap);
            if let GroupsRedirect::Redirect(g) = redirect {
                if g.to_offset() >= groups.len() {
                    redirect = GroupsRedirect::Redirect(GroupIdx::ONE);
                }
            }
            let behavior = key
                .key
                .behavior
                .as_ref()
                .and_then(|b| map_behavior(&mut used_key_names, &mut get_string, &b.val));
            let k = Key {
                key_name: get_string(key.name.val),
                keycode: key.code,
                repeat: key.key.repeating.despan().flatten().unwrap_or(false),
                behavior,
                redirect,
                groups,
            };
            keys.push(k);
        }
        keys.sort_unstable_by(|l, r| l.key_name.cmp(&r.key_name));
        let keys = keys.into_iter().map(|k| (k.keycode, k)).collect();
        let mut keycodes = Vec::with_capacity(resolved.keycodes.name_to_key.len());
        // https://gitlab.freedesktop.org/xorg/xserver/-/issues/1780
        let mut max_keycode = 255;
        for key in resolved.keycodes.name_to_key.values() {
            if !used_key_names.contains(&key.name.val) {
                continue;
            }
            if let ResolvedKeyKind::Real(r) = &key.kind {
                let k = Keycode {
                    name: get_string(key.name.val),
                    keycode: r.val,
                };
                max_keycode = max_keycode.max(r.val.0);
                keycodes.push(k);
            }
        }
        keycodes.sort_unstable_by(|l, r| l.name.cmp(&r.name));
        types.retain(|kt| used_types.contains(&(&**kt as *const KeyType)));
        Self {
            name: resolved.name.despan().map(get_string),
            max_keycode,
            indicators,
            keycodes,
            types,
            virtual_modifiers,
            mod_maps,
            group_names,
            keys,
        }
    }
}

fn map_behavior(
    used_key_names: &mut HashSet<Interned>,
    get_string: &mut impl FnMut(Interned) -> Arc<String>,
    b: &SymbolsKeyBehavior,
) -> Option<KeyBehavior> {
    let behavior = match b {
        SymbolsKeyBehavior::Locks(b) => {
            if !*b {
                return None;
            }
            KeyBehavior::Lock
        }
        SymbolsKeyBehavior::Overlay((overlay, name, keycode)) => {
            used_key_names.insert(*name);
            let behavior = OverlayBehavior {
                overlay: *overlay,
                key_name: get_string(*name),
                keycode: *keycode,
            };
            KeyBehavior::Overlay(behavior)
        }
        SymbolsKeyBehavior::RadioGroup(allow_none, group) => {
            KeyBehavior::RadioGroup(RadioGroupBehavior {
                allow_none: *allow_none,
                radio_group: *group,
            })
        }
        SymbolsKeyBehavior::RepeatLastKey(b) => {
            if !*b {
                return None;
            }
            KeyBehavior::RepeatLastKey
        }
    };
    Some(behavior)
}

fn map_symbol_groups(
    used_types: &mut HashSet<*const KeyType>,
    used_key_names: &mut HashSet<Interned>,
    get_string: &mut impl FnMut(Interned) -> Arc<String>,
    default_key_types: &StaticMap<BuiltInKeytype, Option<Arc<KeyType>>>,
    types_by_ident: &HashMap<Interned, Arc<KeyType>>,
    modmap: ModifierMask,
    m: &SymbolsKeyGroup,
) -> Option<KeyGroup> {
    let Some(ktr) = &m.key_type else {
        return None;
    };
    let kt = match ktr {
        KeyTypeRef::BuiltIn(bi) => default_key_types[bi].as_ref(),
        KeyTypeRef::Named(n) => types_by_ident.get(&n.val),
    };
    let kt = kt?;
    let mut levels: Vec<_> = m
        .levels
        .iter()
        .map(|m| {
            let symbols = m.symbols.iter().map(|s| s.val).collect();
            let actions = m
                .actions
                .iter()
                .flat_map(|a| map_action(modmap, used_key_names, get_string, &a.val))
                .collect();
            KeyLevel { symbols, actions }
        })
        .collect();
    while let Some(last) = levels.last() {
        if last.symbols.is_empty() && last.actions.is_empty() {
            levels.pop();
        } else {
            break;
        }
    }
    if levels.is_empty() {
        return None;
    }
    used_types.insert(&**kt);
    Some(KeyGroup {
        key_type: kt.clone(),
        levels,
    })
}

fn map_action(
    modmap: ModifierMask,
    used_key_names: &mut HashSet<Interned>,
    get_string: &mut impl FnMut(Interned) -> Arc<String>,
    a: &ResolvedAction,
) -> Option<Action> {
    let map_action_mods = |mods: &Option<Spanned<ResolvedActionMods>>| {
        let mods = mods
            .as_ref()
            .map(|m| match &m.val {
                ResolvedActionMods::ModMap => modmap,
                ResolvedActionMods::Explicit(e) => e.get_effective(),
            })
            .unwrap_or_default();
        (mods.0 != 0).then_some(mods)
    };
    let action = match a {
        ResolvedAction::ResolvedNoAction(_) => return None,
        ResolvedAction::ResolvedVoidAction(_) => Action::ControlsLock(ControlsLockAction {
            controls: ControlMask::NONE,
            lock: false,
            unlock: false,
        }),
        ResolvedAction::ResolvedModsSet(m) => Action::ModsSet(ModsSetAction {
            clear_locks: m.clear_locks.despan().unwrap_or_default(),
            modifiers: map_action_mods(&m.modifiers)?,
        }),
        ResolvedAction::ResolvedModsLatch(m) => Action::ModsLatch(ModsLatchAction {
            clear_locks: m.clear_locks.despan().unwrap_or_default(),
            latch_to_lock: m.latch_to_lock.despan().unwrap_or_default(),
            modifiers: map_action_mods(&m.modifiers)?,
        }),
        ResolvedAction::ResolvedModsLock(m) => {
            let (lock, unlock) = match m.affect.despan() {
                None => (true, true),
                Some(a) => (a.lock, a.unlock),
            };
            Action::ModsLock(ModsLockAction {
                modifiers: map_action_mods(&m.modifiers)?,
                lock,
                unlock,
            })
        }
        ResolvedAction::ResolvedGroupSet(m) => Action::GroupSet(GroupSetAction {
            group: m.group.despan().unwrap_or_default(),
            clear_locks: m.clear_locks.despan().unwrap_or_default(),
        }),
        ResolvedAction::ResolvedGroupLatch(m) => Action::GroupLatch(GroupLatchAction {
            group: m.group.despan().unwrap_or_default(),
            clear_locks: m.clear_locks.despan().unwrap_or_default(),
            latch_to_lock: m.latch_to_lock.despan().unwrap_or_default(),
        }),
        ResolvedAction::ResolvedGroupLock(m) => Action::GroupLock(GroupLockAction {
            group: m.group.despan().unwrap_or_default(),
        }),
        ResolvedAction::ResolvedRedirectKey(m) => {
            let (name, kc) = m.keycode?.val;
            used_key_names.insert(name);
            Action::RedirectKey(RedirectKeyAction {
                key_name: get_string(name),
                keycode: kc,
                mods_to_set: map_action_mods(&m.mods_to_set).unwrap_or_default(),
                mods_to_clear: map_action_mods(&m.mods_to_clear).unwrap_or_default(),
            })
        }
        ResolvedAction::ResolvedSetControls(m) => Action::ControlsSet(ControlsSetAction {
            controls: m.controls?.val,
        }),
        ResolvedAction::ResolvedLockControls(m) => {
            let (lock, unlock) = match m.affect.despan() {
                None => (true, true),
                Some(a) => (a.lock, a.unlock),
            };
            Action::ControlsLock(ControlsLockAction {
                controls: m.controls?.val,
                lock,
                unlock,
            })
        }
    };
    Some(action)
}

fn create_used_default_key_types(
    interner: &Interner,
    mods: &[VirtualModifier],
    resolved: &Resolved,
    types: &mut Vec<Arc<KeyType>>,
    types_by_ident: &mut HashMap<Interned, Arc<KeyType>>,
) -> StaticMap<BuiltInKeytype, Option<Arc<KeyType>>> {
    let mut need_to_create = static_map! {
        _ => true,
    };
    let mut used = static_map! {
        _ => false,
    };
    let mut res = static_map! {
        _ => None,
    };
    let bi_to_interned: StaticMap<BuiltInKeytype, _> = static_map! {
        bi => interner.get_existing(bi.name().as_bytes()),
    };
    for (bi, interned) in &bi_to_interned {
        if let Some(interned) = interned {
            if let Some(ty) = types_by_ident.get(interned) {
                res[bi] = Some(ty.clone());
                need_to_create[bi] = false;
            }
        }
    }
    const N: usize = BuiltInKeytype::LENGTH;
    let mut missing_ident: ArrayVec<_, N> = bi_to_interned
        .clone()
        .into_iter()
        .flat_map(|(bi, i)| i.map(|i| (i, bi)))
        .collect();
    for key in resolved.symbols.keys.values() {
        for group in &key.key.groups {
            if let Some(kt) = &group.key_type {
                match kt {
                    KeyTypeRef::BuiltIn(bi) => used[*bi] = true,
                    KeyTypeRef::Named(n) => {
                        for (idx, (i, bi)) in missing_ident.iter().enumerate() {
                            if *i == n.val {
                                used[*bi] = true;
                                missing_ident.swap_remove(idx);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    need_to_create = need_to_create.map(|bi, v| v && used[bi]);
    create_used_default_key_types_(mods, &mut res, need_to_create, types);
    for (bi, interned) in &bi_to_interned {
        if let Some(interned) = interned {
            if let Some(ty) = &res[bi] {
                types_by_ident.insert(*interned, ty.clone());
            }
        }
    }
    res
}

fn create_used_default_key_types_(
    mods: &[VirtualModifier],
    res: &mut StaticMap<BuiltInKeytype, Option<Arc<KeyType>>>,
    need_to_create: StaticMap<BuiltInKeytype, bool>,
    types: &mut Vec<Arc<KeyType>>,
) {
    macro_rules! vmod {
        ($name:expr) => {{
            let m = mods
                .iter()
                .find(|m| *m.name == $name)
                .map(|m| m.values)
                .unwrap_or_default();
            (m.0 != 0).then_some(m)
        }};
    }
    macro_rules! ty {
        ($name:ident {
            modifiers = $($modifier:ident)|+ $(+ $($other:ident)|*)?,
            $(
                map[$($map_modifier:ident)|+ $(+ $($map_other:ident)|*)?] = $map_level:literal preserve $($keep_modifier:ident)|*,
            )*
            $(
                name[$name_level:expr] = $level_name:expr,
            )*
        }) => {{
            let mut used = HashSet::new();
            let mut mappings = vec![];
            $(
                #[allow(unused_labels)]
                'add_mapping: {
                    #[allow(unused_mut, unused_assignments)]
                    let mut other = ModifierMask::NONE;
                    $(
                        if let ($(Some($map_other),)*) = ($($map_other,)*) {
                            other = $($map_other)|*;
                        } else {
                            break 'add_mapping;
                        }
                    )?
                    let modifiers = $(ModifierMask::$map_modifier)|* | other;
                    if used.insert(modifiers) {
                        mappings.push(
                            KeyTypeMapping {
                                modifiers,
                                preserved: $(ModifierMask::$keep_modifier)|*,
                                level: Level::new($map_level).unwrap(),
                            },
                        )
                    }
                }
            )*
            mappings.shrink_to_fit();
            mappings.sort_unstable_by_key(|k| (k.level, k.modifiers.0));
            let mut level_names = vec![
                $(
                    (Level::new($name_level).unwrap(), Arc::new($level_name.to_string())),
                )*
            ];
            level_names.sort_unstable_by_key(|l| l.0);
            Arc::new(KeyType {
                name: Arc::new(BuiltInKeytype::$name.name().to_string()),
                modifiers: $(ModifierMask::$modifier)|* $(| $($other.unwrap_or_default())|+)?,
                mappings,
                level_names,
            })
        }};
    }
    macro_rules! ty_cached {
        ($($tt:tt)*) => {{
            static T: std::sync::LazyLock<Arc<KeyType>> = std::sync::LazyLock::new(|| {
                ty!($($tt)*)
            });
            T.clone()
        }};
    }
    let num_lock = vmod!("NumLock");
    let level_three = vmod!("LevelThree");
    for (bi, need_to_create) in need_to_create {
        if need_to_create {
            let kt = match bi {
                BuiltInKeytype::OneLevel => ty_cached! {
                    OneLevel {
                        modifiers = NONE,
                        map[NONE] = 1 preserve NONE,
                        name[1] = "Any",
                    }
                },
                BuiltInKeytype::Alphabetic => ty_cached! {
                    Alphabetic {
                        modifiers = SHIFT|LOCK,
                        map[SHIFT] = 2 preserve NONE,
                        map[LOCK] = 2 preserve NONE,
                        name[1] = "Base",
                        name[2] = "Caps",
                    }
                },
                BuiltInKeytype::Keypad => ty! {
                    Keypad {
                        modifiers = SHIFT + num_lock,
                        map[NONE] = 1 preserve NONE,
                        map[NONE + num_lock] = 2 preserve NONE,
                        map[SHIFT + num_lock] = 1 preserve NONE,
                        name[1] = "Base",
                        name[2] = "Number",
                    }
                },
                BuiltInKeytype::TwoLevel => ty_cached! {
                    TwoLevel {
                        modifiers = SHIFT,
                        map[SHIFT] = 2 preserve NONE,
                        name[1] = "Base",
                        name[2] = "Shift",
                    }
                },
                BuiltInKeytype::FourLevelAlphabetic => ty! {
                    FourLevelAlphabetic {
                        modifiers = SHIFT|LOCK + level_three,
                        map[NONE] = 1 preserve NONE,
                        map[SHIFT] = 2 preserve NONE,
                        map[LOCK] = 2 preserve NONE,
                        map[SHIFT|LOCK] = 1 preserve NONE,
                        map[NONE + level_three] = 3 preserve NONE,
                        map[SHIFT + level_three] = 4 preserve NONE,
                        map[LOCK + level_three] = 4 preserve NONE,
                        map[SHIFT|LOCK + level_three] = 3 preserve NONE,
                        name[1] = "Base",
                        name[2] = "Shift",
                        name[3] = "AltGr",
                        name[4] = "Shift AltGr",
                    }
                },
                BuiltInKeytype::FourLevelSemialphabetic => ty! {
                    FourLevelSemialphabetic {
                        modifiers = SHIFT|LOCK + level_three,
                        map[NONE] = 1 preserve NONE,
                        map[SHIFT] = 2 preserve NONE,
                        map[LOCK] = 2 preserve NONE,
                        map[SHIFT|LOCK] = 1 preserve NONE,
                        map[NONE + level_three] = 3 preserve NONE,
                        map[SHIFT + level_three] = 4 preserve NONE,
                        map[LOCK + level_three] = 3 preserve LOCK,
                        map[SHIFT|LOCK + level_three] = 4 preserve LOCK,
                        name[1] = "Base",
                        name[2] = "Shift",
                        name[3] = "AltGr",
                        name[4] = "Shift AltGr",
                    }
                },
                BuiltInKeytype::FourLevelKeypad => ty! {
                    FourLevelKeypad {
                        modifiers = SHIFT + num_lock|level_three,
                        map[NONE] = 1 preserve NONE,
                        map[SHIFT] = 2 preserve NONE,
                        map[NONE + num_lock] = 2 preserve NONE,
                        map[SHIFT + num_lock] = 1 preserve NONE,
                        map[NONE + level_three] = 3 preserve NONE,
                        map[SHIFT + level_three] = 4 preserve NONE,
                        map[NONE + num_lock|level_three] = 4 preserve NONE,
                        map[SHIFT + num_lock|level_three] = 3 preserve NONE,
                        name[1] = "Base",
                        name[2] = "Shift/Numlock",
                        name[3] = "AltGr",
                        name[4] = "Shift/Numlock AltGr",
                    }
                },
                BuiltInKeytype::FourLevel => ty! {
                    FourLevel {
                        modifiers = SHIFT + level_three,
                        map[NONE] = 1 preserve NONE,
                        map[SHIFT] = 2 preserve NONE,
                        map[NONE + level_three] = 3 preserve NONE,
                        map[SHIFT + level_three] = 4 preserve NONE,
                        name[1] = "Base",
                        name[2] = "Shift",
                        name[3] = "AltGr",
                        name[4] = "Shift AltGr",
                    }
                },
            };
            types.push(kt.clone());
            res[bi] = Some(kt);
        }
    }
}
