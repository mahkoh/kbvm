mod format;

use {
    crate::{
        keysym::Keysym,
        modifier::{ModifierIndex, ModifierMask},
        state_machine,
        xkb::{
            controls::ControlMask,
            group::{GroupChange, GroupIdx, GroupMask},
            group_component::GroupComponent,
            indicator::IndicatorIdx,
            interner::{Interned, Interner},
            level::Level,
            mod_component::ModComponentMask,
            resolved::{
                BuiltInKeytype, GroupsRedirect, KeyTypeRef, ModMapField, Resolved, ResolvedAction,
                ResolvedActionMods, ResolvedKeyKind, SymbolsKeyGroup,
            },
            span::{Despan, Spanned},
        },
    },
    arrayvec::ArrayVec,
    bstr::ByteSlice,
    hashbrown::{hash_map::Entry, DefaultHashBuilder, HashMap, HashSet},
    indexmap::IndexMap,
    linearize::{static_map, Linearize, StaticMap},
    smallvec::SmallVec,
    std::{fmt::Write, sync::Arc},
};

#[derive(Debug, PartialEq)]
pub struct Keymap {
    pub(crate) name: Option<Arc<String>>,
    pub(crate) max_keycode: u32,
    pub(crate) indicators: Vec<Indicator>,
    pub(crate) keycodes: Vec<Keycode>,
    pub(crate) types: Vec<Arc<KeyType>>,
    pub(crate) virtual_modifiers: Vec<VirtualModifier>,
    pub(crate) mod_maps: Vec<(ModifierIndex, ModMapValue)>,
    pub(crate) group_names: Vec<(GroupIdx, Arc<String>)>,
    pub(crate) keys: IndexMap<state_machine::Keycode, Symbol, DefaultHashBuilder>,
}

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) struct ModMapValue {
    pub(crate) key_name: Arc<String>,
    pub(crate) key_sym: Option<Keysym>,
}

#[derive(Debug, PartialEq)]
pub(crate) struct Indicator {
    pub(crate) virt: bool,
    pub(crate) index: IndicatorIdx,
    pub(crate) name: Arc<String>,
    pub(crate) modifier_mask: ModifierMask,
    pub(crate) group_mask: GroupMask,
    pub(crate) controls: ControlMask,
    pub(crate) mod_components: ModComponentMask,
    pub(crate) group_components: GroupComponent,
}

#[derive(Debug, PartialEq)]
pub(crate) struct Keycode {
    pub(crate) name: Arc<String>,
    pub(crate) keycode: state_machine::Keycode,
}

#[derive(Debug, PartialEq)]
pub(crate) struct VirtualModifier {
    pub(crate) name: Arc<String>,
    pub(crate) values: ModifierMask,
}

#[derive(Debug, PartialEq)]
pub(crate) struct KeyType {
    pub(crate) name: Arc<String>,
    pub(crate) modifiers: ModifierMask,
    pub(crate) mappings: Vec<KeyTypeMapping>,
    pub(crate) level_names: Vec<(Level, Arc<String>)>,
}

#[derive(Debug, PartialEq)]
pub(crate) struct KeyTypeMapping {
    pub(crate) modifiers: ModifierMask,
    pub(crate) preserved: ModifierMask,
    pub(crate) level: Level,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Action {
    ModsSet(ModsSet),
    ModsLatch(ModsLatch),
    ModsLock(ModsLock),
    GroupSet(GroupSet),
    GroupLatch(GroupLatch),
    GroupLock(GroupLock),
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ModsSet {
    pub(crate) clear_locks: bool,
    pub(crate) modifiers: ModifierMask,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ModsLatch {
    pub(crate) clear_locks: bool,
    pub(crate) latch_to_lock: bool,
    pub(crate) modifiers: ModifierMask,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ModsLock {
    pub(crate) modifiers: ModifierMask,
    pub(crate) lock: bool,
    pub(crate) unlock: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct GroupSet {
    pub(crate) group: GroupChange,
    pub(crate) clear_locks: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct GroupLatch {
    pub(crate) group: GroupChange,
    pub(crate) clear_locks: bool,
    pub(crate) latch_to_lock: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct GroupLock {
    pub(crate) group: GroupChange,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Symbol {
    pub(crate) key_name: Arc<String>,
    pub(crate) key_code: state_machine::Keycode,
    pub(crate) groups: Vec<Option<SymbolGroup>>,
    pub(crate) repeat: bool,
    pub(crate) redirect: GroupsRedirect,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct SymbolGroup {
    pub(crate) key_type: Arc<KeyType>,
    pub(crate) levels: Vec<SymbolLevel>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct SymbolLevel {
    pub(crate) symbols: SmallVec<[Keysym; 1]>,
    pub(crate) actions: SmallVec<[Action; 1]>,
}

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
                values: m.def.despan().unwrap_or_default(),
            });
        }
        virtual_modifiers.sort_unstable_by(|l, r| l.name.cmp(&r.name));
        if virtual_modifiers.is_empty() {
            // https://gitlab.freedesktop.org/xorg/xserver/-/issues/1774
            virtual_modifiers.push(VirtualModifier {
                name: Arc::new("Dummy".to_string()),
                values: Default::default(),
            });
        }
        let mut types_by_ident = HashMap::new();
        let mut types = Vec::with_capacity(resolved.types.key_types.len() + 8);
        let mut used_types = HashSet::new();
        for ty in resolved.types.key_types.values() {
            let mut mappings = Vec::with_capacity(ty.ty.map.len());
            for (n, v) in &ty.ty.map {
                mappings.push(KeyTypeMapping {
                    modifiers: *n,
                    preserved: ty.ty.preserved.get(n).copied().despan().unwrap_or_default(),
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
                modifiers: ty.ty.modifiers.despan().unwrap_or_default(),
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
            let modifier_mask = map.modifiers.despan().unwrap_or_default();
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
                group_components,
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
                group_components: Default::default(),
            };
            indicators.push(i);
        }
        indicators.sort_unstable_by_key(|i| i.index.raw());
        if indicators.is_empty() {
            // https://gitlab.freedesktop.org/xorg/xserver/-/issues/1775
            indicators.push(Indicator {
                virt: false,
                index: IndicatorIdx::ONE,
                name: Arc::new("DUMMY".to_string()),
                modifier_mask: Default::default(),
                group_mask: Default::default(),
                controls: Default::default(),
                mod_components: Default::default(),
                group_components: Default::default(),
            });
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
            let k = Symbol {
                key_name: get_string(key.name.val),
                key_code: key.code,
                repeat: key.key.repeating.despan().flatten().unwrap_or(false),
                redirect,
                groups,
            };
            keys.push(k);
        }
        keys.sort_unstable_by(|l, r| l.key_name.cmp(&r.key_name));
        let keys = keys.into_iter().map(|k| (k.key_code, k)).collect();
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

fn map_symbol_groups(
    used_types: &mut HashSet<*const KeyType>,
    default_key_types: &StaticMap<BuiltInKeytype, Option<Arc<KeyType>>>,
    types_by_ident: &HashMap<Interned, Arc<KeyType>>,
    modmap: ModifierMask,
    m: &SymbolsKeyGroup,
) -> Option<SymbolGroup> {
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
                .flat_map(|a| map_action(modmap, &a.val))
                .collect();
            SymbolLevel { symbols, actions }
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
    Some(SymbolGroup {
        key_type: kt.clone(),
        levels,
    })
}

fn map_action(modmap: ModifierMask, a: &ResolvedAction) -> Option<Action> {
    let map_action_mods = |mods: &Option<Spanned<ResolvedActionMods>>| {
        let mods = mods
            .despan()
            .map(|m| match m {
                ResolvedActionMods::ModMap => modmap,
                ResolvedActionMods::Explicit(e) => e,
            })
            .unwrap_or_default();
        (mods.0 != 0).then_some(mods)
    };
    let action = match a {
        ResolvedAction::ResolvedNoAction(_) => return None,
        ResolvedAction::ResolvedModsSet(m) => Action::ModsSet(ModsSet {
            clear_locks: m.clear_locks.despan().unwrap_or_default(),
            modifiers: map_action_mods(&m.modifiers)?,
        }),
        ResolvedAction::ResolvedModsLatch(m) => Action::ModsLatch(ModsLatch {
            clear_locks: m.clear_locks.despan().unwrap_or_default(),
            latch_to_lock: m.latch_to_lock.despan().unwrap_or_default(),
            modifiers: map_action_mods(&m.modifiers)?,
        }),
        ResolvedAction::ResolvedModsLock(m) => {
            let (lock, unlock) = match m.affect.despan() {
                None => (true, true),
                Some(a) => (a.lock, a.unlock),
            };
            Action::ModsLock(ModsLock {
                modifiers: map_action_mods(&m.modifiers)?,
                lock,
                unlock,
            })
        }
        ResolvedAction::ResolvedGroupSet(m) => Action::GroupSet(GroupSet {
            group: m.group.despan().unwrap_or_default(),
            clear_locks: m.clear_locks.despan().unwrap_or_default(),
        }),
        ResolvedAction::ResolvedGroupLatch(m) => Action::GroupLatch(GroupLatch {
            group: m.group.despan().unwrap_or_default(),
            clear_locks: m.clear_locks.despan().unwrap_or_default(),
            latch_to_lock: m.latch_to_lock.despan().unwrap_or_default(),
        }),
        ResolvedAction::ResolvedGroupLock(m) => Action::GroupLock(GroupLock {
            group: m.group.despan().unwrap_or_default(),
        }),
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
