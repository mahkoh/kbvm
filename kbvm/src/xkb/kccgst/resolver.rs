#[cfg(test)]
mod tests;

use {
    crate::{
        syms,
        xkb::{
            code_map::CodeMap,
            diagnostic::{DiagnosticKind, DiagnosticSink},
            group::GroupIdx,
            indicator::IndicatorIdx,
            interner::{Interned, Interner},
            kccgst::{
                ast::{
                    CompatmapDecl, ConfigItemType, Decls, DirectOrIncluded, Expr, InterpretSym,
                    Item, ItemType, KeycodeDecl, MergeModeExt, Path, SymbolsDecl, TypesDecl,
                    VModDecl, VarDecl, VarOrExpr,
                },
                expr::{
                    eval_action_default, eval_filter, eval_group, eval_indicator_map_field,
                    eval_interp_field, eval_keysyms, eval_mod_map_field, eval_real_mods,
                    eval_string, eval_symbols_field, eval_type_field, ident_to_real_mod_index,
                    EvalError, GroupList, IndicatorMapField, InterpField, SymbolsField, TypeField,
                },
                MergeMode,
            },
            level::Level,
            meaning::{Meaning, MeaningCache},
            modmap::Vmodmap,
            resolved::{
                BuiltInKeytype, Filter, GroupsRedirect, Indicator, IndicatorMap,
                IndicatorMapWithKey, Interp, InterpWithKey, KeyTypeRef, ModMapEntryWithKey,
                ModMapField, Predicate, Resolved, ResolvedAction, ResolvedActionMods,
                ResolvedCompat, ResolvedKey, ResolvedKeyKind, ResolvedKeyType,
                ResolvedKeyTypeWithName, ResolvedKeycodes, ResolvedSymbols, ResolvedTypes,
                SymbolsKey, SymbolsKeyBehavior, SymbolsKeyGroup, SymbolsKeyLevel,
                SymbolsKeyWithKey,
            },
            span::{Span, SpanExt, Spanned},
            string_cooker::StringCooker,
        },
        Keycode, Keysym, ModifierIndex, ModifierMask,
    },
    hashbrown::{hash_map::Entry, DefaultHashBuilder, HashSet},
    indexmap::IndexMap,
    isnt::std_1::primitive::IsntSliceExt,
    kbvm_proc::ad_hoc_display,
    smallvec::SmallVec,
    std::{fmt::Display, mem},
};

pub(crate) fn resolve(
    map: &mut CodeMap,
    diagnostics: &mut DiagnosticSink<'_, '_>,
    interner: &mut Interner,
    meaning_cache: &mut MeaningCache,
    cooker: &mut StringCooker,
    item: &Item,
) -> Resolved {
    let mut resolver = Resolver {
        map,
        interner,
        meaning_cache,
        diagnostics,
        cooker,
    };

    let mut mods = VmodResolver {
        r: &mut resolver,
        data: Default::default(),
    }
    .resolve(item);

    let mut keycodes = KeycodesResolver {
        r: &mut resolver,
        data: Default::default(),
    }
    .resolve(item);
    fix_resolved_keycodes(&mut resolver, &mut keycodes);

    let mut types = TypesResolver {
        r: &mut resolver,
        mods: &mods,
        data: Default::default(),
    }
    .resolve(item);
    fix_resolved_types(&mut types);

    let mut compat = CompatResolver {
        r: &mut resolver,
        keycodes: &keycodes,
        mods: &mods,
        data: Default::default(),
    }
    .resolve(item);
    fix_resolved_compat(&mut resolver, &mut keycodes, &mut compat);

    let mut symbols = SymbolsResolver {
        r: &mut resolver,
        mods: &mods,
        keycodes: &keycodes,
        types: &types,
        data: Default::default(),
        mod_map_fields: Default::default(),
    }
    .resolve(item);
    fix_resolved_symbols(&mut resolver, &types, &mut symbols);

    fix_combined_properties(&mut mods, &mut types, &mut compat, &mut symbols);

    let mut name = None;
    if let ItemType::Composite(c) = &item.ty {
        name = c.name.map(|n| resolver.cook(n));
    }

    Resolved {
        name,
        mods,
        keycodes,
        types,
        compat,
        symbols,
    }
}

struct Resolver<'a, 'b, 'c> {
    map: &'a mut CodeMap,
    interner: &'a mut Interner,
    meaning_cache: &'a mut MeaningCache,
    diagnostics: &'a mut DiagnosticSink<'b, 'c>,
    cooker: &'a mut StringCooker,
}

struct VmodResolver<'a, 'b, 'c, 'd> {
    r: &'a mut Resolver<'b, 'c, 'd>,
    data: Vmodmap,
}

struct KeycodesResolver<'a, 'b, 'c, 'd> {
    r: &'a mut Resolver<'b, 'c, 'd>,
    data: ResolvedKeycodes,
}

struct TypesResolver<'a, 'b, 'c, 'd> {
    r: &'a mut Resolver<'b, 'c, 'd>,
    mods: &'a Vmodmap,
    data: ResolvedTypes,
}

struct CompatResolver<'a, 'b, 'c, 'd> {
    r: &'a mut Resolver<'b, 'c, 'd>,
    keycodes: &'a ResolvedKeycodes,
    mods: &'a Vmodmap,
    data: ResolvedCompat,
}

struct SymbolsResolver<'a, 'b, 'c, 'd> {
    r: &'a mut Resolver<'b, 'c, 'd>,
    mods: &'a Vmodmap,
    keycodes: &'a ResolvedKeycodes,
    types: &'a ResolvedTypes,
    data: ResolvedSymbols,
    mod_map_fields: Vec<Spanned<ModMapField>>,
}

fn fix_resolved_keycodes(r: &mut Resolver<'_, '_, '_>, keycodes: &mut ResolvedKeycodes) {
    let mut res = ResolvedKeycodes::default();
    res.keycode_to_name = mem::take(&mut keycodes.keycode_to_name);
    res.indicators = mem::take(&mut keycodes.indicators);
    let mut cache = vec![];
    loop {
        let iter = keycodes.name_to_key.extract_if(|_, v| match v.kind {
            ResolvedKeyKind::Real(_) => true,
            ResolvedKeyKind::Alias(a, _, _) => res.name_to_key.contains_key(&a.val),
        });
        cache.extend(iter);
        if cache.is_empty() {
            break;
        }
        for (name, mut key) in cache.drain(..) {
            if let ResolvedKeyKind::Alias(a, _, _) = key.kind {
                let Some(real) = res.name_to_key.get(&a.val) else {
                    continue;
                };
                let (real_name, kc) = match real.kind {
                    ResolvedKeyKind::Real(kc) => (None, kc.val),
                    ResolvedKeyKind::Alias(t, kc, r) => (Some(r.unwrap_or(t)), kc),
                };
                key.kind = ResolvedKeyKind::Alias(a, kc, real_name);
            }
            res.name_to_key.insert(name, key);
        }
    }
    for key in keycodes.name_to_key.values() {
        let span = match &key.kind {
            ResolvedKeyKind::Real(_) => continue,
            ResolvedKeyKind::Alias(a, _, _) => a.span,
        };
        r.diag(
            DiagnosticKind::UnknownKeyAlias,
            ad_hoc_display!("ignoring unknown key alias").spanned2(span),
        );
    }
    *keycodes = res;
}

fn fix_resolved_types(types: &mut ResolvedTypes) {
    for ty in types.key_types.values_mut() {
        ty.ty.num_levels = ty
            .ty
            .map
            .values()
            .map(|v| v.val.to_offset() + 1)
            .max()
            .unwrap_or(0);
    }
}

fn fix_resolved_compat(
    resolver: &mut Resolver<'_, '_, '_>,
    keycodes: &mut ResolvedKeycodes,
    compat: &mut ResolvedCompat,
) {
    compat
        .interps_sorted
        .extend(compat.interps.drain(..).map(|(_, v)| v));
    // NOTE: this sort is stable, so we keep the source-code order for equal elements
    compat.interps_sorted.sort_by_key(|i| {
        let k1 = match i.keysym {
            Some(_) => 0,
            None => 1,
        };
        let k2 = match i.filter {
            None => 4,
            Some(f) => match f.val.predicate {
                Predicate::Exactly => 0,
                Predicate::AllOf => 1,
                Predicate::NoneOf => 2,
                Predicate::AnyOf => 3,
                Predicate::AnyOfOrNone => 4,
            },
        };
        (k1, k2)
    });
    let mut used_indicators = [false; u32::BITS as usize];
    for indicator in &keycodes.indicators {
        if let Some(i) = compat.indicator_maps.get_mut(&indicator.name.val) {
            i.indicator_map.idx = Some(indicator.idx.val);
            i.indicator_map.virt = indicator.virt.is_some();
        }
        used_indicators[indicator.idx.val.to_offset()] = true;
    }
    compat.indicator_maps.retain(|_, indicator| {
        if indicator.indicator_map.idx.is_some() {
            return true;
        }
        let unused = used_indicators.iter_mut().enumerate().find(|(_, v)| !**v);
        let idx = match unused {
            Some((idx, ptr)) => {
                *ptr = true;
                idx
            }
            _ => {
                resolver.diag(
                    DiagnosticKind::TooManyIndicators,
                    ad_hoc_display!("ignoring more than 32 indicators")
                        .spanned2(indicator.name.span),
                );
                return false;
            }
        };
        indicator.indicator_map.virt = true;
        indicator.indicator_map.idx = IndicatorIdx::new(idx as u32 + 1);
        true
    });
}

fn fix_resolved_symbols(
    resolver: &mut Resolver,
    types: &ResolvedTypes,
    symbols: &mut ResolvedSymbols,
) {
    for key in symbols.keys.values_mut() {
        for group in &mut key.key.groups {
            if group.key_type.is_none() {
                if let Some(def) = key.key.default_key_type {
                    group.key_type = Some(def.val);
                }
            }
            let key_type = match group.key_type {
                Some(t) => t,
                _ => {
                    let t = KeyTypeRef::BuiltIn(infer_key_type(group));
                    group.key_type = Some(t);
                    t
                }
            };
            let num_levels = match key_type {
                KeyTypeRef::BuiltIn(bi) => {
                    let mut num_levels = match bi {
                        BuiltInKeytype::OneLevel => 1,
                        BuiltInKeytype::Alphabetic
                        | BuiltInKeytype::Keypad
                        | BuiltInKeytype::TwoLevel => 2,
                        BuiltInKeytype::FourLevelAlphabetic
                        | BuiltInKeytype::FourLevelSemialphabetic
                        | BuiltInKeytype::FourLevelKeypad
                        | BuiltInKeytype::FourLevel => 4,
                    };
                    if let Some(name) = resolver.interner.get_existing(bi.name().as_bytes()) {
                        if let Some(kt) = types.key_types.get(&name) {
                            num_levels = num_levels.max(kt.ty.num_levels);
                        }
                    }
                    num_levels
                }
                KeyTypeRef::Named(n) => types
                    .key_types
                    .get(&n.val)
                    .map(|t| t.ty.num_levels)
                    .unwrap_or_default(),
            };
            group.reachable_levels = num_levels;
        }
    }
    let mut mod_map_keys = HashSet::new();
    for entry in symbols.mod_map_entries.values() {
        if entry.modifier.is_some() {
            if let ModMapField::Keycode(keycode) = entry.key.val {
                mod_map_keys.insert(keycode);
            }
        }
    }
    for entry in symbols.mod_map_entries.values_mut() {
        let Some(modifier) = entry.modifier else {
            continue;
        };
        let keycode = match entry.key.val {
            ModMapField::Keysym(ks, _) => {
                let mut max_group = usize::MAX;
                let mut max_level = usize::MAX;
                let mut keycode = None;
                for key in symbols.keys.values() {
                    for (group_idx, group) in key.key.groups.iter().enumerate() {
                        if group_idx > max_group {
                            break;
                        }
                        for (level_idx, level) in group.levels.iter().enumerate() {
                            if level_idx >= group.reachable_levels {
                                break;
                            }
                            if (group_idx, level_idx) > (max_group, max_level) {
                                break;
                            }
                            for s in &level.symbols {
                                if s.val == ks {
                                    if group_idx < max_group
                                        || level_idx < max_level
                                        || keycode.is_none()
                                        || matches!(keycode, Some(k) if k > key.code)
                                    {
                                        max_group = group_idx;
                                        max_level = level_idx;
                                        keycode = Some(key.code);
                                    }
                                    break;
                                }
                            }
                        }
                    }
                }
                entry.key.val = ModMapField::Keysym(ks, keycode);
                if let Some(keycode) = keycode {
                    if mod_map_keys.insert(keycode) {
                        entry.key.val = ModMapField::Keycode(keycode);
                    }
                }
                keycode
            }
            ModMapField::Keycode(kc) => Some(kc),
        };
        if let Some(kc) = keycode {
            if let Some(key) = symbols.keys.get_mut(&kc) {
                key.key.modmap |= modifier.val.to_mask();
            }
        }
    }
}

fn infer_key_type(group: &SymbolsKeyGroup) -> BuiltInKeytype {
    let get_symbol_of_level = |level: usize| {
        if let Some(l) = group.levels.get(level) {
            if let Some(sym) = l.symbols.first() {
                return Some(sym.val);
            }
            return Some(syms::NoSymbol);
        }
        None
    };
    let Some(s0) = get_symbol_of_level(0) else {
        return BuiltInKeytype::OneLevel;
    };
    let Some(s1) = get_symbol_of_level(1) else {
        return BuiltInKeytype::OneLevel;
    };
    if group.levels.len() > 4 {
        if group.levels[4..].iter().any(|l| l.symbols.is_not_empty()) {
            return BuiltInKeytype::OneLevel;
        }
    }
    match get_symbol_of_level(2) {
        None => {
            if s0.is_lowercase() && s1.is_uppercase() {
                return BuiltInKeytype::Alphabetic;
            }
            if s0.is_keypad() || s1.is_keypad() {
                return BuiltInKeytype::Keypad;
            }
            BuiltInKeytype::TwoLevel
        }
        Some(s2) => {
            if s0.is_lowercase() && s1.is_uppercase() {
                if let Some(s3) = get_symbol_of_level(3) {
                    if s2.is_lowercase() && s3.is_uppercase() {
                        return BuiltInKeytype::FourLevelAlphabetic;
                    }
                }
                return BuiltInKeytype::FourLevelSemialphabetic;
            }
            if s0.is_keypad() || s1.is_keypad() {
                return BuiltInKeytype::FourLevelKeypad;
            }
            BuiltInKeytype::FourLevel
        }
    }
}

fn fix_combined_properties(
    mods: &mut Vmodmap,
    types: &mut ResolvedTypes,
    compat: &mut ResolvedCompat,
    symbols: &mut ResolvedSymbols,
) {
    let mut version = 0;
    for key_with_key in symbols.keys.values_mut() {
        let key = &mut key_with_key.key;
        let mut virtual_modifiers = None::<ModifierMask>;
        for (group_idx, group) in key.groups.iter_mut().enumerate() {
            if group.has_explicit_actions {
                continue;
            }
            for (level_idx, level) in group.levels.iter_mut().enumerate() {
                let max_version = version;
                version += 1;
                for sym in &level.symbols {
                    let interp = compat.interps_sorted.iter_mut().find(|interp| {
                        if interp.version > max_version {
                            return false;
                        }
                        if let Some(ks) = interp.keysym {
                            if ks.val != sym.val {
                                return false;
                            }
                        }
                        if let Some(ks) = &interp.filter {
                            let km = if interp.interp.level_one_only() && level_idx != 0 {
                                0
                            } else {
                                key.modmap.0
                            };
                            let fm = ks.val.mask.val.0;
                            let ok = match ks.val.predicate {
                                Predicate::NoneOf => fm & km == 0,
                                Predicate::AnyOfOrNone => km == 0 || fm & km != 0,
                                Predicate::AnyOf => fm & km != 0,
                                Predicate::AllOf => fm & km == fm,
                                Predicate::Exactly => fm == km,
                            };
                            if !ok {
                                return false;
                            }
                        }
                        true
                    });
                    let mut handle_repeat = |repeat: Spanned<bool>| {
                        if (group_idx, level_idx) == (0, 0) && key.repeating.is_none() {
                            key.repeating = Some(repeat.map(Some));
                        }
                    };
                    let Some(interp) = interp else {
                        handle_repeat(true.spanned2(key_with_key.name.span));
                        continue;
                    };
                    interp.version = version;
                    if let Some(a) = &interp.interp.actions {
                        level.actions.extend(a.val.iter().cloned());
                    }
                    if (group_idx, level_idx) == (0, 0) || !interp.interp.level_one_only() {
                        if let Some(fm) = interp.interp.virtual_modifier {
                            *virtual_modifiers.get_or_insert_default() |= fm.val.to_mask();
                        }
                    }
                    if let Some(repeat) = interp.interp.repeat {
                        handle_repeat(repeat);
                    }
                    if let Some(locks) = interp.interp.locking {
                        if (group_idx, level_idx) == (0, 0) && key.behavior.is_none() {
                            key.behavior = Some(locks.map(SymbolsKeyBehavior::Locks));
                        }
                    }
                }
            }
        }
        if key.virtualmodifiers.is_none() {
            key.virtualmodifiers = virtual_modifiers.map(|m| m.spanned2(key_with_key.name.span));
        }
        if let Some(mm) = key.virtualmodifiers {
            for m in &mut *mods {
                if mm.val.contains(m.idx.to_mask()) {
                    m.def.get_or_insert(key.modmap.spanned2(m.name.span)).val |= key.modmap;
                }
            }
        }
    }

    for ty in types.key_types.values_mut() {
        let ty = &mut ty.ty;
        if let Some(mask) = &mut ty.modifiers {
            mask.val = mods.get_effective(mask.val);
        }
        let mut new_map = IndexMap::with_hasher(DefaultHashBuilder::default());
        let mut new_preserved = IndexMap::with_hasher(DefaultHashBuilder::default());
        for (mask, val) in &ty.preserved {
            if let indexmap::map::Entry::Vacant(v) = ty.map.entry(*mask) {
                v.insert(Level::ONE.spanned2(val.span));
            }
        }
        for (mask, val) in &ty.map {
            let Ok(effective) = mods.try_get_effective(*mask) else {
                continue;
            };
            if let indexmap::map::Entry::Vacant(v) = new_map.entry(effective) {
                v.insert(*val);
                if let Some(p) = ty.preserved.get(mask) {
                    let val = mods.get_effective(p.val);
                    new_preserved.insert(effective, val.spanned2(p.span));
                }
            }
        }
        ty.map = new_map;
        ty.preserved = new_preserved;
    }

    let fix_action_mods = |action_mods: &mut Option<Spanned<ResolvedActionMods>>| {
        if let Some(action_mods) = action_mods {
            if let ResolvedActionMods::Explicit(e) = &mut action_mods.val {
                *e = mods.get_effective(*e);
            }
        }
    };
    for key in symbols.keys.values_mut() {
        for group in &mut key.key.groups {
            for level in &mut group.levels {
                for action in &mut level.actions {
                    match &mut action.val {
                        ResolvedAction::ResolvedModsSet(e) => fix_action_mods(&mut e.modifiers),
                        ResolvedAction::ResolvedModsLatch(e) => fix_action_mods(&mut e.modifiers),
                        ResolvedAction::ResolvedModsLock(e) => fix_action_mods(&mut e.modifiers),
                        ResolvedAction::ResolvedRedirectKey(e) => {
                            fix_action_mods(&mut e.mods_to_set);
                            fix_action_mods(&mut e.mods_to_clear);
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    for indicator in compat.indicator_maps.values_mut() {
        if let Some(mask) = &mut indicator.indicator_map.modifiers {
            mask.val = mods.get_effective(mask.val);
        }
    }
}

impl Resolver<'_, '_, '_> {
    fn diag(
        &mut self,
        kind: DiagnosticKind,
        message: Spanned<impl Display + Send + Sync + 'static>,
    ) {
        self.diagnostics.push(self.map, kind, message)
    }

    fn cook(&mut self, interned: Spanned<Interned>) -> Spanned<Interned> {
        self.cooker
            .cook(self.map, self.diagnostics, self.interner, interned)
            .spanned2(interned.span)
    }
}

impl VmodResolver<'_, '_, '_, '_> {
    fn resolve(mut self, e: &Item) -> Vmodmap {
        match &e.ty {
            ItemType::Composite(e) => {
                for item in &e.config_items {
                    self.resolve_config_item(&item.val.item.specific);
                }
            }
            ItemType::Config(c) => self.resolve_config_item(c),
        }
        self.data
    }

    fn resolve_config_item(&mut self, e: &ConfigItemType) {
        match e {
            ConfigItemType::Types(t) => {
                self.resolve_decls(&t.decls, &|d| match d {
                    TypesDecl::VMod(v) => Some(v),
                    _ => None,
                });
            }
            ConfigItemType::Compat(t) => {
                self.resolve_decls(&t.decls, &|d| match d {
                    CompatmapDecl::VMod(v) => Some(v),
                    _ => None,
                });
            }
            ConfigItemType::Symbols(t) => {
                self.resolve_decls(&t.decls, &|d| match d {
                    SymbolsDecl::VMod(v) => Some(v),
                    _ => None,
                });
            }
            ConfigItemType::Keycodes(_) | ConfigItemType::Geometry(_) => {}
        }
    }

    fn resolve_decls<T>(&mut self, e: &Decls<T>, f: &impl Fn(&T) -> Option<&VModDecl>) {
        for decl in &e.decls {
            let mm = decl.val.merge_mode.map(|m| m.val);
            match &decl.val.ty {
                DirectOrIncluded::Direct(d) => {
                    self.handle_decl(mm, d, f);
                }
                DirectOrIncluded::Included(i) => self.handle_child(mm, |slf| {
                    for c in &i.components {
                        let mm = c.mm.map(|m| m.val);
                        slf.handle_child(mm, |slf| slf.resolve_decls(&c.decls, f));
                    }
                }),
            }
        }
    }

    fn handle_child(&mut self, mm: Option<MergeMode>, f: impl FnOnce(&mut Self)) {
        let parent = mem::take(&mut self.data);
        f(self);
        let child = mem::replace(&mut self.data, parent);
        for vmod in child {
            self.handle_vmod(mm, vmod.name, vmod.def);
        }
    }

    fn handle_decl<T>(
        &mut self,
        mm: Option<MergeMode>,
        e: &T,
        f: &impl Fn(&T) -> Option<&VModDecl>,
    ) {
        if let Some(v) = f(e) {
            self.handle_vmod_decl(mm, v);
        }
    }

    fn handle_vmod_decl(&mut self, mm: Option<MergeMode>, e: &VModDecl) {
        for e in &e.defs {
            if ident_to_real_mod_index(self.r.interner, self.r.meaning_cache, e.val.name.val)
                .is_some()
            {
                self.r.diag(
                    DiagnosticKind::VirtualModifierHasRealName,
                    ad_hoc_display!("virtual modifier has real-modifier name")
                        .spanned2(e.val.name.span),
                );
                continue;
            }
            let value = match &e.val.val {
                None => None,
                Some(e) => {
                    let res = eval_real_mods(self.r.interner, self.r.meaning_cache, e.as_ref());
                    match res {
                        Ok(m) => Some(m),
                        Err(e) => {
                            self.r.diag(e.val.diagnostic_kind(), e);
                            continue;
                        }
                    }
                }
            };
            self.handle_vmod(mm, e.val.name, value);
        }
    }

    fn handle_vmod(
        &mut self,
        mm: Option<MergeMode>,
        name: Spanned<Interned>,
        value: Option<Spanned<ModifierMask>>,
    ) {
        let Some(vmod) = self.data.insert(name) else {
            self.r.diag(
                DiagnosticKind::TooManyVirtualModifiers,
                ad_hoc_display!("too many virtual modifiers").spanned2(name.span),
            );
            return;
        };
        if vmod.def.is_none() || vmod.def == value || mm.is_not_augment() {
            if value.is_some() {
                vmod.def = value;
            }
        } else {
            self.r.diag(
                DiagnosticKind::IgnoringVmodRedefinition,
                ad_hoc_display!("ignoring redefinition of virtual modifier").spanned2(name.span),
            );
        }
    }
}

trait ConfigWalker: Sized {
    type Data: Default;

    type Decl;

    fn accept<'a>(&mut self, ct: &'a ConfigItemType) -> Option<&'a Decls<Self::Decl>>;

    fn data(&mut self) -> &mut Self::Data;

    fn merge_child(&mut self, mm: Option<MergeMode>, group: Option<GroupIdx>, data: Self::Data);

    fn handle_decl(&mut self, mm: Option<MergeMode>, decl: &Self::Decl, span: Span);

    fn resolve(mut self, e: &Item) -> Self::Data {
        match &e.ty {
            ItemType::Composite(e) => {
                for item in &e.config_items {
                    self.resolve_config_item(&item.val.item.specific);
                }
            }
            ItemType::Config(c) => {
                self.resolve_config_item(c);
            }
        }
        mem::take(self.data())
    }

    fn resolve_config_item(&mut self, e: &ConfigItemType) {
        if let Some(e) = self.accept(e) {
            self.resolve_decls(e);
        }
    }

    fn handle_child(
        &mut self,
        mm: Option<MergeMode>,
        group: Option<GroupIdx>,
        cont: impl FnOnce(&mut Self),
    ) {
        let parent = mem::take(self.data());
        cont(self);
        let child = mem::replace(self.data(), parent);
        self.merge_child(mm, group, child);
    }

    fn resolve_decls(&mut self, decls: &Decls<Self::Decl>) {
        for decl in &decls.decls {
            let mm = decl.val.merge_mode.map(|m| m.val);
            match &decl.val.ty {
                DirectOrIncluded::Direct(d) => {
                    self.handle_decl(mm, d, decl.span);
                }
                DirectOrIncluded::Included(i) => self.handle_child(mm, None, |slf| {
                    for c in &i.components {
                        let mm = c.mm.map(|m| m.val);
                        slf.handle_child(mm, c.group.map(|g| g.group), |slf| {
                            slf.resolve_decls(&c.decls)
                        });
                    }
                }),
            }
        }
    }
}

impl KeycodesResolver<'_, '_, '_, '_> {
    fn handle_key(&mut self, mm: Option<MergeMode>, name: Spanned<Interned>, ty: ResolvedKeyKind) {
        let key = ResolvedKey { name, kind: ty };
        let e1 = match ty {
            ResolvedKeyKind::Real(r) => Some(self.data.keycode_to_name.entry(r.val)),
            _ => None,
        };
        let e2 = self.data.name_to_key.entry(name.val);
        if matches!(e1, Some(Entry::Occupied(_))) || matches!(e2, Entry::Occupied(_)) {
            if mm.is_augment() {
                return;
            }
            let mut code_to_remove = None;
            if let Entry::Occupied(e2) = e2 {
                if let ResolvedKeyKind::Real(n) = e2.remove().kind {
                    code_to_remove = Some(n.val);
                }
            }
            let mut name_to_remove = None;
            if let Some(Entry::Occupied(e1)) = e1 {
                name_to_remove = Some(e1.remove());
            }
            if let Some(code) = code_to_remove {
                self.data.keycode_to_name.remove(&code);
            }
            if let Some(name) = name_to_remove {
                self.data.name_to_key.remove(&name);
            }
        }
        if let ResolvedKeyKind::Real(r) = ty {
            self.data.keycode_to_name.insert(r.val, name.val);
        }
        self.data.name_to_key.insert(name.val, key);
    }

    fn handle_indicator_name(
        &mut self,
        mm: Option<MergeMode>,
        name: Spanned<Interned>,
        idx: Spanned<IndicatorIdx>,
        virt: Option<Span>,
    ) {
        let augment = mm.is_augment();
        let indi = Indicator { idx, name, virt };
        let mut i = 0;
        while i < self.data.indicators.len() {
            let old = &self.data.indicators[i];
            if old.idx == idx || old.name == name {
                if !augment {
                    self.data.indicators[i] = indi;
                    i += 1;
                    while i < self.data.indicators.len() {
                        let old = &self.data.indicators[i];
                        if old.idx == idx || old.name == name {
                            self.data.indicators.swap_remove(i);
                        } else {
                            i += 1;
                        }
                    }
                }
                return;
            }
            i += 1;
        }
        self.data.indicators.push(indi);
    }
}

impl ConfigWalker for KeycodesResolver<'_, '_, '_, '_> {
    type Data = ResolvedKeycodes;

    type Decl = KeycodeDecl;

    fn accept<'a>(&mut self, ct: &'a ConfigItemType) -> Option<&'a Decls<Self::Decl>> {
        match ct {
            ConfigItemType::Keycodes(c) => Some(&c.decls),
            _ => None,
        }
    }

    fn data(&mut self) -> &mut Self::Data {
        &mut self.data
    }

    fn merge_child(&mut self, mm: Option<MergeMode>, _group: Option<GroupIdx>, data: Self::Data) {
        for indicator in data.indicators {
            self.handle_indicator_name(mm, indicator.name, indicator.idx, indicator.virt);
        }
        for key in data.name_to_key.into_values() {
            self.handle_key(mm, key.name, key.kind);
        }
    }

    fn handle_decl(&mut self, mm: Option<MergeMode>, decl: &Self::Decl, span: Span) {
        match decl {
            KeycodeDecl::KeyName(e) => {
                self.handle_key(
                    mm,
                    e.name,
                    ResolvedKeyKind::Real(e.code.spanned2(e.code_str.span)),
                );
            }
            KeycodeDecl::KeyAlias(e) => {
                self.handle_key(
                    mm,
                    e.name,
                    ResolvedKeyKind::Alias(e.alias_for, Keycode(0), None),
                );
            }
            KeycodeDecl::Var(e) => {
                if let Some(i) = e.var.path.val.unique_ident() {
                    if e.var.expr.is_some() {
                        let meaning = self
                            .r
                            .meaning_cache
                            .get_case_insensitive(self.r.interner, i);
                        if matches!(meaning, Meaning::Minimum | Meaning::Maximum) {
                            return;
                        }
                    }
                }
                self.r.diag(
                    DiagnosticKind::UnknownKeycodesVariable,
                    ad_hoc_display!("unknown keycodes variable").spanned2(span),
                );
            }
            KeycodeDecl::IndicatorName(e) => {
                let idx = match IndicatorIdx::new(e.idx) {
                    Some(i) => i,
                    _ => {
                        self.r.diag(
                            DiagnosticKind::InvalidIndicatorIndex,
                            ad_hoc_display!("invalid indicator index").spanned2(e.idx_name.span),
                        );
                        return;
                    }
                };
                let res = eval_string(
                    self.r.map,
                    self.r.diagnostics,
                    self.r.interner,
                    self.r.cooker,
                    e.val.as_ref(),
                );
                let name = match res {
                    Ok(r) => r,
                    Err(e) => {
                        self.r.diag(e.val.diagnostic_kind(), e);
                        return;
                    }
                };
                self.handle_indicator_name(mm, name, idx.spanned2(e.idx_name.span), e.virt)
            }
            KeycodeDecl::Include(_) => {}
        }
    }
}

impl ResolvedKeyType {
    fn apply_field(&mut self, field: TypeField) {
        match field {
            TypeField::Modifiers(m) => {
                self.modifiers = Some(m);
            }
            TypeField::Map(k, v) => {
                self.map.insert(k.val, v);
            }
            TypeField::Preserve(k, v) => {
                self.preserved.insert(k.val, v);
            }
            TypeField::LevelName(n, v) => {
                self.names.insert(n.val, v);
            }
        }
    }
}

impl TypesResolver<'_, '_, '_, '_> {
    fn parse_field(&mut self, decl: &VarDecl, span: Span, skip_first: bool) -> Option<TypeField> {
        let res = eval_type_field(
            self.r.map,
            self.r.diagnostics,
            self.r.cooker,
            self.r.interner,
            self.r.meaning_cache,
            self.mods,
            &decl.var,
            span,
            skip_first,
        );
        match res {
            Ok(v) => Some(v.val),
            Err(e) => {
                self.r.diag(e.val.diagnostic_kind(), e);
                None
            }
        }
    }

    fn handle_type(&mut self, mm: Option<MergeMode>, name: Spanned<Interned>, ty: ResolvedKeyType) {
        let entry = self.data.key_types.entry(name.val);
        if mm.is_augment() && matches!(entry, Entry::Occupied(_)) {
            self.r.diag(
                DiagnosticKind::DuplicateKeyTypeDefinition,
                ad_hoc_display!("ignoring duplicate key type name").spanned2(name.span),
            );
            return;
        }
        entry.insert(ResolvedKeyTypeWithName { name, ty });
    }
}

impl ConfigWalker for TypesResolver<'_, '_, '_, '_> {
    type Data = ResolvedTypes;

    type Decl = TypesDecl;

    fn accept<'a>(&mut self, ct: &'a ConfigItemType) -> Option<&'a Decls<Self::Decl>> {
        match ct {
            ConfigItemType::Types(c) => Some(&c.decls),
            _ => None,
        }
    }

    fn data(&mut self) -> &mut Self::Data {
        &mut self.data
    }

    fn merge_child(&mut self, mm: Option<MergeMode>, _group: Option<GroupIdx>, data: Self::Data) {
        for key in data.key_types.into_values() {
            self.handle_type(mm, key.name, key.ty);
        }
    }

    fn handle_decl(&mut self, mm: Option<MergeMode>, decl: &Self::Decl, span: Span) {
        match decl {
            TypesDecl::KeyType(e) => {
                let name = self.r.cook(e.name);
                let mut key = self.data.default.clone();
                for decl in &e.decls {
                    if let Some(field) = self.parse_field(&decl.val, decl.span, false) {
                        key.apply_field(field);
                    }
                }
                self.handle_type(mm, name, key);
            }
            TypesDecl::Var(e) => {
                let unknown_types_variable =
                    ad_hoc_display!("unknown types variable").spanned2(span);
                let Some(cs) = e.var.path.val.components() else {
                    self.r
                        .diag(DiagnosticKind::UnknownTypesVariable, unknown_types_variable);
                    return;
                };
                let c = &cs[0];
                let meaning = self
                    .r
                    .meaning_cache
                    .get_case_insensitive(self.r.interner, c.ident);
                if c.index.is_some() || meaning != Meaning::Type {
                    self.r
                        .diag(DiagnosticKind::UnknownTypesVariable, unknown_types_variable);
                    return;
                }
                if let Some(field) = self.parse_field(e, span, true) {
                    self.data.default.apply_field(field);
                }
            }
            TypesDecl::VMod(_) | TypesDecl::Include(_) => {}
        }
    }
}

impl Interp {
    fn apply_field(&mut self, field: Spanned<InterpField>) {
        match field.val {
            InterpField::Actions(a) => {
                self.actions = Some(a.spanned2(field.span));
            }
            InterpField::VirtualModifier(m) => {
                self.virtual_modifier = Some(m.spanned2(field.span));
            }
            InterpField::Repeat(r) => {
                self.repeat = Some(r.spanned2(field.span));
            }
            InterpField::LevelOneOnly(u) => {
                self.level_one_only = Some(u.spanned2(field.span));
            }
            InterpField::Locking(e) => {
                self.locking = Some(e.spanned2(field.span));
            }
        }
    }
}

impl IndicatorMap {
    fn apply_field(&mut self, field: Spanned<IndicatorMapField>) {
        match field.val {
            IndicatorMapField::Modifiers(e) => {
                self.modifiers = Some(e);
            }
            IndicatorMapField::Groups(e) => {
                self.groups = Some(e);
            }
            IndicatorMapField::Controls(e) => {
                self.controls = Some(e);
            }
            IndicatorMapField::WhichModifierState(e) => {
                self.which_modifier_state = Some(e);
            }
            IndicatorMapField::WhichGroupState(e) => {
                self.which_group_state = Some(e);
            }
        }
    }
}

impl CompatResolver<'_, '_, '_, '_> {
    fn parse_interp_field(
        &mut self,
        decl: &VarDecl,
        span: Span,
        skip_first: bool,
    ) -> Option<Spanned<InterpField>> {
        let res = eval_interp_field(
            self.r.map,
            self.r.diagnostics,
            self.r.interner,
            self.r.meaning_cache,
            self.keycodes,
            self.mods,
            &self.data.action_defaults,
            &decl.var,
            span,
            skip_first,
        );
        match res {
            Ok(v) => Some(v),
            Err(e) => {
                self.r.diag(e.val.diagnostic_kind(), e);
                None
            }
        }
    }

    fn parse_indicator_field(
        &mut self,
        decl: &VarDecl,
        span: Span,
        skip_first: bool,
    ) -> Option<Spanned<IndicatorMapField>> {
        let res = eval_indicator_map_field(
            self.r.interner,
            self.r.meaning_cache,
            self.mods,
            &decl.var,
            span,
            skip_first,
        );
        match res {
            Ok(v) => Some(v),
            Err(e) => {
                self.r.diag(e.val.diagnostic_kind(), e);
                None
            }
        }
    }

    fn handle_interp(
        &mut self,
        mm: Option<MergeMode>,
        keysym: Option<Spanned<Keysym>>,
        filter: Option<Spanned<Filter>>,
        interp: Interp,
    ) {
        let mm = mm.unwrap_or(MergeMode::Override);
        let key = (keysym.map(|k| k.val), filter.map(|f| f.val));
        let entry = self.data.interps.entry(key);
        if mm == MergeMode::Replace {
            entry.insert_entry(InterpWithKey {
                keysym,
                filter,
                interp,
                version: 0,
            });
            return;
        }
        let augment = mm == MergeMode::Augment;
        if let indexmap::map::Entry::Occupied(e) = entry {
            let old = e.into_mut();
            macro_rules! opt {
                ($field:ident) => {
                    if let Some(x) = interp.$field {
                        if augment && old.interp.$field.is_some() {
                            self.r.diag(
                                DiagnosticKind::IgnoredInterpretField,
                                ad_hoc_display!(concat!(
                                    "ignoring redefinition of interp field `",
                                    stringify!($field),
                                    "`",
                                ))
                                .spanned2(x.span),
                            );
                        } else {
                            old.interp.$field = Some(x);
                        }
                    }
                };
            }
            opt!(actions);
            opt!(virtual_modifier);
            opt!(repeat);
            opt!(level_one_only);
        } else {
            entry.insert_entry(InterpWithKey {
                keysym,
                filter,
                interp,
                version: 0,
            });
        }
    }

    fn handle_indicator_map(
        &mut self,
        mm: Option<MergeMode>,
        name: Spanned<Interned>,
        indicator_map: IndicatorMap,
    ) {
        let mm = mm.unwrap_or(MergeMode::Override);
        let entry = self.data.indicator_maps.entry(name.val);
        if mm == MergeMode::Replace {
            entry.insert_entry(IndicatorMapWithKey {
                name,
                indicator_map,
            });
            return;
        }
        let augment = mm == MergeMode::Augment;
        if let indexmap::map::Entry::Occupied(e) = entry {
            let old = e.into_mut();
            macro_rules! opt {
                ($($field:ident)|*) => {
                    let have_any = $(old.indicator_map.$field.is_some())||*;
                    $(
                        if let Some(x) = indicator_map.$field {
                            if augment && have_any {
                                self.r.diag(
                                    DiagnosticKind::IgnoredIndicatorField,
                                    ad_hoc_display!("ignoring redefinition").spanned2(x.span),
                                );
                            } else {
                                old.indicator_map.$field = Some(x);
                            }
                        }
                    )*
                };
            }
            opt!(which_modifier_state | modifiers);
            opt!(which_group_state | groups);
            opt!(controls);
        } else {
            entry.insert_entry(IndicatorMapWithKey {
                name,
                indicator_map,
            });
        }
    }
}

impl ConfigWalker for CompatResolver<'_, '_, '_, '_> {
    type Data = ResolvedCompat;
    type Decl = CompatmapDecl;

    fn accept<'a>(&mut self, ct: &'a ConfigItemType) -> Option<&'a Decls<Self::Decl>> {
        match ct {
            ConfigItemType::Compat(c) => Some(&c.decls),
            _ => None,
        }
    }

    fn data(&mut self) -> &mut Self::Data {
        &mut self.data
    }

    fn merge_child(&mut self, mm: Option<MergeMode>, _group: Option<GroupIdx>, data: Self::Data) {
        for interp in data.interps.into_values() {
            self.handle_interp(mm, interp.keysym, interp.filter, interp.interp);
        }
        for indicator_map in data.indicator_maps.into_values() {
            self.handle_indicator_map(mm, indicator_map.name, indicator_map.indicator_map);
        }
    }

    fn handle_decl(&mut self, mm: Option<MergeMode>, decl: &Self::Decl, span: Span) {
        match decl {
            CompatmapDecl::Interpret(e) => {
                let expr = match e.match_.val.sym.val {
                    InterpretSym::Ident(i) => Expr::Path(Path::One(i)),
                    InterpretSym::String(s) => Expr::String(s),
                };
                let mut keysym = None;
                let res = eval_keysyms(
                    self.r.map,
                    self.r.diagnostics,
                    self.r.interner,
                    self.r.cooker,
                    self.r.meaning_cache,
                    (&expr).spanned2(e.match_.val.sym.span),
                    |sym| {
                        if sym != syms::NoSymbol {
                            if keysym.is_some() {
                                return Err(EvalError::MultipleKeysymsInInterpret);
                            }
                            keysym = Some(sym.spanned2(e.match_.val.sym.span));
                        }
                        Ok(())
                    },
                );
                if let Err(e) = res {
                    self.r.diag(e.val.diagnostic_kind(), e);
                    return;
                }
                let filter = match &e.match_.val.filter {
                    None => None,
                    Some(e) => {
                        let res = eval_filter(self.r.interner, self.r.meaning_cache, e.as_ref());
                        match res {
                            Ok(f) => Some(f),
                            Err(e) => {
                                self.r.diag(e.val.diagnostic_kind(), e);
                                return;
                            }
                        }
                    }
                };
                let mut interp = self.data.interp_default.clone();
                for decl in &e.vars {
                    if let Some(f) = self.parse_interp_field(&decl.val, decl.span, false) {
                        interp.apply_field(f);
                    }
                }
                self.handle_interp(mm, keysym, filter, interp);
            }
            CompatmapDecl::IndicatorMap(l) => {
                let name = self.r.cook(l.name);
                let mut indicator_map = self.data.indicator_map_default.clone();
                for decl in &l.decls {
                    if let Some(f) = self.parse_indicator_field(&decl.val, decl.span, false) {
                        indicator_map.apply_field(f);
                    }
                }
                self.handle_indicator_map(mm, name, indicator_map);
            }
            CompatmapDecl::Var(e) => {
                let unknown_compat_variable =
                    ad_hoc_display!("unknown compat variable").spanned2(span);
                let Some(cs) = e.var.path.val.components() else {
                    self.r.diag(
                        DiagnosticKind::UnknownCompatVariable,
                        unknown_compat_variable,
                    );
                    return;
                };
                let c = &cs[0];
                let meaning = self
                    .r
                    .meaning_cache
                    .get_case_insensitive(self.r.interner, c.ident);
                if c.index.is_some() {
                    self.r.diag(
                        DiagnosticKind::UnknownCompatVariable,
                        unknown_compat_variable,
                    );
                    return;
                }
                if meaning == Meaning::Interpret {
                    if let Some(f) = self.parse_interp_field(e, span, true) {
                        self.data.interp_default.apply_field(f);
                    }
                } else if meaning == Meaning::Indicator {
                    if let Some(f) = self.parse_indicator_field(e, span, true) {
                        self.data.indicator_map_default.apply_field(f);
                    }
                } else {
                    let res = eval_action_default(
                        self.r.interner,
                        self.r.meaning_cache,
                        self.keycodes,
                        self.mods,
                        &e.var,
                        span,
                        &mut self.data.action_defaults,
                    );
                    if let Err(e) = res {
                        self.r.diag(e.val.diagnostic_kind(), e);
                    }
                }
            }
            CompatmapDecl::GroupCompat(_) | CompatmapDecl::VMod(_) | CompatmapDecl::Include(_) => {}
        }
    }
}

impl SymbolsKey {
    fn apply_offset(&mut self, map: &mut CodeMap, diagnostics: &mut DiagnosticSink, offset: usize) {
        if self.groups.is_empty() {
            return;
        }
        'outer: for g in &self.groups[1..] {
            for l in &g.levels {
                if let Some(span) = l.span() {
                    diagnostics.push(
                        map,
                        DiagnosticKind::DiscardingGroup,
                        ad_hoc_display!("discarding groups other than 1 due to explicit group specifier in inclued").spanned2(span),
                    );
                    break 'outer;
                }
            }
        }
        self.groups.truncate(1);
        if offset == 0 {
            return;
        }
        let mut res = vec![SymbolsKeyGroup::default(); offset + 1];
        res[offset] = self.groups.pop().unwrap();
        self.groups = res;
    }
}

impl SymbolsKeyLevel {
    fn is_empty(&self) -> bool {
        self.symbols.is_empty() && self.actions.is_empty()
    }

    fn span(&self) -> Option<Span> {
        self.symbols
            .iter()
            .map(|s| s.span)
            .chain(self.actions.iter().map(|s| s.span))
            .next()
    }
}

impl SymbolsKeyGroup {
    fn set_symbols(&mut self, symbols: GroupList<Keysym>) {
        self.has_explicit_symbols = true;
        self.set_field(symbols, |l| &mut l.symbols);
    }

    fn set_actions(&mut self, actions: GroupList<ResolvedAction>) {
        self.has_explicit_actions = true;
        self.set_field(actions, |l| &mut l.actions);
    }

    fn set_field<T>(
        &mut self,
        t: GroupList<T>,
        mut f: impl FnMut(&mut SymbolsKeyLevel) -> &mut SmallVec<[Spanned<T>; 1]>,
    ) {
        for l in &mut self.levels {
            f(l).clear();
        }
        for (idx, t) in t.into_iter().enumerate() {
            if idx >= self.levels.len() {
                self.levels.push(SymbolsKeyLevel::default());
            }
            *f(&mut self.levels[idx]) = t;
        }
        self.trim();
    }

    fn trim(&mut self) {
        while let Some(last) = self.levels.last() {
            if last.is_empty() {
                self.levels.pop();
            } else {
                break;
            }
        }
    }
}

impl SymbolsKey {
    fn apply_field(&mut self, f: Spanned<SymbolsField>) {
        fn get_group(
            groups: &mut Vec<SymbolsKeyGroup>,
            group_idx: Option<GroupIdx>,
            place_here: impl Fn(&SymbolsKeyGroup) -> bool,
        ) -> &mut SymbolsKeyGroup {
            let idx = match group_idx {
                Some(idx) => idx.to_offset(),
                _ => groups.iter().position(place_here).unwrap_or(groups.len()),
            };
            if idx >= groups.len() {
                groups.resize(idx + 1, SymbolsKeyGroup::default());
            }
            &mut groups[idx]
        }

        match f.val {
            SymbolsField::GroupKeyType(g, e) => {
                get_group(&mut self.groups, Some(g), |_| false).key_type = Some(e);
            }
            SymbolsField::DefaultKeyType(e) => {
                self.default_key_type = Some(e.spanned2(f.span));
            }
            SymbolsField::Symbols((group, e)) => {
                get_group(&mut self.groups, group, |g| !g.has_explicit_symbols).set_symbols(e);
            }
            SymbolsField::Actions((group, e)) => {
                get_group(&mut self.groups, group, |g| !g.has_explicit_actions).set_actions(e);
            }
            SymbolsField::Virtualmodifiers(e) => {
                self.virtualmodifiers = Some(e.spanned2(f.span));
            }
            SymbolsField::Repeating(e) => {
                self.repeating = Some(e.spanned2(f.span));
            }
            SymbolsField::Locks(e) => {
                self.behavior = Some(SymbolsKeyBehavior::Locks(e).spanned2(f.span));
            }
            SymbolsField::Groupswrap => {
                self.groups_redirect = Some(GroupsRedirect::Wrap.spanned2(f.span))
            }
            SymbolsField::Groupsclamp => {
                self.groups_redirect = Some(GroupsRedirect::Clamp.spanned2(f.span))
            }
            SymbolsField::Groupsredirect(e) => {
                self.groups_redirect = Some(GroupsRedirect::Redirect(e).spanned2(f.span))
            }
            SymbolsField::Overlay(e) => {
                self.behavior = Some(SymbolsKeyBehavior::Overlay(e).spanned2(f.span))
            }
            SymbolsField::AllowNone(allow, mask) => match allow {
                true => self.allow_none |= mask,
                false => self.allow_none &= !mask,
            },
            SymbolsField::RadioGroup(rg) => {
                self.behavior = rg.map(|rg| {
                    let allow_none = self.allow_none & rg.to_mask() != 0;
                    SymbolsKeyBehavior::RadioGroup(allow_none, rg).spanned2(f.span)
                });
            }
        }
    }
}

impl SymbolsResolver<'_, '_, '_, '_> {
    fn parse_symbols_field(
        &mut self,
        not: Option<Span>,
        path: Option<Spanned<&Path>>,
        expr: Option<Spanned<&Expr>>,
        span: Span,
        skip_first: bool,
    ) -> Option<Spanned<SymbolsField>> {
        let res = eval_symbols_field(
            self.r.map,
            self.r.diagnostics,
            self.r.interner,
            self.r.cooker,
            self.r.meaning_cache,
            &self.data.action_defaults,
            self.keycodes,
            self.mods,
            self.types,
            not,
            path,
            expr,
            span,
            skip_first,
        );
        match res {
            Ok(v) => Some(v),
            Err(e) => {
                self.r.diag(e.val.diagnostic_kind(), e);
                None
            }
        }
    }

    fn handle_mod_map_entry(
        &mut self,
        mm: Option<MergeMode>,
        key: Spanned<ModMapField>,
        modifier: Option<Spanned<ModifierIndex>>,
    ) {
        let mm = mm.unwrap_or(MergeMode::Override);
        let augment = mm == MergeMode::Augment;
        let entry = self.data.mod_map_entries.entry(key.val);
        if augment && matches!(entry, indexmap::map::Entry::Occupied(_)) {
            self.r.diag(
                DiagnosticKind::IgnoredModMapEntry,
                ad_hoc_display!("ignoring duplicate mod map entry").spanned2(key.span),
            );
            return;
        }
        entry.insert_entry(ModMapEntryWithKey { key, modifier });
    }

    fn handle_key(&mut self, mm: Option<MergeMode>, name: Spanned<Interned>, key: SymbolsKey) {
        let Some(kd) = self.keycodes.name_to_key.get(&name.val) else {
            self.r.diag(
                DiagnosticKind::UnknownKey,
                ad_hoc_display!("unknown key").spanned2(name.span),
            );
            return;
        };
        let (kc, real_name) = match kd.kind {
            ResolvedKeyKind::Real(c) => (c.val, name),
            ResolvedKeyKind::Alias(n, c, name) => (c, name.unwrap_or(n)),
        };
        let mm = mm.unwrap_or(MergeMode::Override);
        let entry = self.data.keys.entry(kc);
        if mm == MergeMode::Replace {
            entry.insert(SymbolsKeyWithKey {
                name: real_name,
                code: kc,
                key,
            });
            return;
        }
        let augment = mm == MergeMode::Augment;
        let ignoring_redefinition = ad_hoc_display!("ignoring redefinition");
        if let Entry::Occupied(e) = entry {
            let old = e.into_mut();
            for (idx, group) in key.groups.into_iter().enumerate() {
                if let Some(old) = old.key.groups.get_mut(idx) {
                    if let Some(x) = group.key_type {
                        if augment && old.key_type.is_some() {
                            if let Some(KeyTypeRef::Named(n)) = &old.key_type {
                                self.r.diag(
                                    DiagnosticKind::IgnoredKeyField,
                                    ignoring_redefinition.spanned2(n.span),
                                );
                            }
                        } else {
                            old.key_type = Some(x);
                        }
                    }
                    for (idx, level) in group.levels.into_iter().enumerate() {
                        if idx >= old.levels.len() {
                            old.levels.resize(idx + 1, SymbolsKeyLevel::default());
                        }
                        let span = level.span();
                        macro_rules! merge {
                            ($field:ident, $explicit:ident) => {
                                if level.$field.is_not_empty() {
                                    if augment && old.levels[idx].$field.is_not_empty() {
                                        if let Some(span) = span {
                                            self.r.diag(
                                                DiagnosticKind::IgnoredKeyField,
                                                ignoring_redefinition.spanned2(span),
                                            );
                                        }
                                    } else {
                                        old.levels[idx].$field = level.$field;
                                        old.$explicit |= group.$explicit;
                                    }
                                }
                            };
                        }
                        merge!(symbols, has_explicit_symbols);
                        merge!(actions, has_explicit_actions);
                    }
                } else {
                    old.key.groups.push(group);
                }
            }
            macro_rules! opt {
                ($field:ident) => {
                    if let Some(x) = key.$field {
                        if augment && old.key.$field.is_some() {
                            self.r.diag(
                                DiagnosticKind::IgnoredKeyField,
                                ignoring_redefinition.spanned2(x.span),
                            );
                        } else {
                            old.key.$field = Some(x);
                        }
                    }
                };
            }
            opt!(default_key_type);
            opt!(virtualmodifiers);
            opt!(repeating);
            opt!(groups_redirect);
            opt!(behavior);
        } else {
            entry.insert(SymbolsKeyWithKey {
                name: real_name,
                code: kc,
                key,
            });
        }
    }

    fn handle_group_name(&mut self, mm: Option<MergeMode>, idx: GroupIdx, name: Spanned<Interned>) {
        let offset = idx.to_offset();
        if offset >= self.data.group_names.len() {
            self.data.group_names.resize(offset + 1, None);
        }
        let target = &mut self.data.group_names[offset];
        if target.is_some() && mm.is_augment() {
            self.r.diag(
                DiagnosticKind::IgnoredKeyField,
                ad_hoc_display!("ignoring group name").spanned2(name.span),
            );
        } else {
            *target = Some((idx, name));
        }
    }
}

impl ConfigWalker for SymbolsResolver<'_, '_, '_, '_> {
    type Data = ResolvedSymbols;
    type Decl = SymbolsDecl;

    fn accept<'a>(&mut self, ct: &'a ConfigItemType) -> Option<&'a Decls<Self::Decl>> {
        match ct {
            ConfigItemType::Symbols(c) => Some(&c.decls),
            _ => None,
        }
    }

    fn data(&mut self) -> &mut Self::Data {
        &mut self.data
    }

    fn merge_child(&mut self, mm: Option<MergeMode>, group: Option<GroupIdx>, data: Self::Data) {
        for mut key in data.keys.into_values() {
            if let Some(group) = group {
                key.key
                    .apply_offset(self.r.map, self.r.diagnostics, group.to_offset());
            }
            self.handle_key(mm, key.name, key.key);
        }
        for entry in data.mod_map_entries.into_values() {
            self.handle_mod_map_entry(mm, entry.key, entry.modifier);
        }
        if data.group_names.is_not_empty() {
            if let Some(group) = group {
                if data.group_names.len() > 1 {
                    let span = data.group_names.last().unwrap().unwrap().1.span;
                    self.r.diag(
                        DiagnosticKind::DiscardingGroup,
                        ad_hoc_display!("discarding groups other than 1 due to explicit group specifier in include").spanned2(span),
                    );
                }
                if let Some((_, name)) = data.group_names[0] {
                    self.handle_group_name(mm, group, name);
                }
            } else {
                for (idx, name) in data.group_names.into_iter().flatten() {
                    self.handle_group_name(mm, idx, name);
                }
            }
        }
    }

    fn handle_decl(&mut self, mm: Option<MergeMode>, decl: &Self::Decl, span: Span) {
        match decl {
            SymbolsDecl::Symbols(s) => {
                let mut key = self.data.key_default.clone();
                for f in &s.vars {
                    let f = match &f.val {
                        VarOrExpr::Var(v) => self.parse_symbols_field(
                            v.not,
                            Some(v.path.as_ref()),
                            v.expr.as_ref().map(|e| e.as_ref()),
                            f.span,
                            false,
                        ),
                        VarOrExpr::Expr(e) => self.parse_symbols_field(
                            None,
                            None,
                            Some(e.spanned2(f.span)),
                            f.span,
                            false,
                        ),
                    };
                    if let Some(f) = f {
                        key.apply_field(f);
                    }
                }
                self.handle_key(mm, s.key, key);
            }
            SymbolsDecl::ModMap(m) => {
                let name = self
                    .r
                    .meaning_cache
                    .get_case_insensitive(self.r.interner, m.modifier.val);
                let modifier = if name == Meaning::None {
                    None
                } else {
                    let res = ident_to_real_mod_index(
                        self.r.interner,
                        self.r.meaning_cache,
                        m.modifier.val,
                    );
                    let modifier = match res {
                        Some(m) => m,
                        None => {
                            self.r.diag(
                                DiagnosticKind::UnknownModifier,
                                ad_hoc_display!("unknown modifier").spanned2(m.modifier.span),
                            );
                            return;
                        }
                    };
                    Some(modifier.spanned2(m.modifier.span))
                };
                for key in &m.keys {
                    let res = eval_mod_map_field(
                        self.r.map,
                        self.r.diagnostics,
                        self.r.interner,
                        self.r.cooker,
                        self.r.meaning_cache,
                        self.keycodes,
                        &key.val,
                        key.span,
                        |key| {
                            self.mod_map_fields.push(key);
                        },
                    );
                    if let Err(e) = res {
                        self.r.diag(e.val.diagnostic_kind(), e);
                    }
                }
                let mut mod_map_fields = mem::take(&mut self.mod_map_fields);
                for key in mod_map_fields.drain(..) {
                    self.handle_mod_map_entry(mm, key, modifier);
                }
                self.mod_map_fields = mod_map_fields;
            }
            SymbolsDecl::Var(v) => {
                let unknown_field = ad_hoc_display!("unknown field").spanned2(v.var.path.span);
                let Some(cs) = v.var.path.val.components() else {
                    self.r
                        .diag(DiagnosticKind::UnknownSymbolsVariable, unknown_field);
                    return;
                };
                let meaning = self
                    .r
                    .meaning_cache
                    .get_case_insensitive(self.r.interner, cs[0].ident);
                match meaning {
                    Meaning::Key => {
                        if cs[0].index.is_some() {
                            self.r
                                .diag(DiagnosticKind::UnknownSymbolsVariable, unknown_field);
                            return;
                        }
                        let field = self.parse_symbols_field(
                            v.var.not,
                            Some(v.var.path.as_ref()),
                            v.var.expr.as_ref().map(|e| e.as_ref()),
                            span,
                            true,
                        );
                        if let Some(f) = field {
                            self.data.key_default.apply_field(f);
                        }
                    }
                    Meaning::Allownone => {
                        let field = self.parse_symbols_field(
                            v.var.not,
                            Some(v.var.path.as_ref()),
                            v.var.expr.as_ref().map(|e| e.as_ref()),
                            span,
                            false,
                        );
                        if let Some(f) = field {
                            self.data.key_default.apply_field(f);
                        }
                    }
                    Meaning::Name | Meaning::Groupname => {
                        if cs.len() > 1 || cs[0].index.is_none() {
                            self.r
                                .diag(DiagnosticKind::UnknownSymbolsVariable, unknown_field);
                            return;
                        }
                        let expr = match v.var.expr.as_ref() {
                            Some(e) => e.as_ref(),
                            _ => {
                                self.r.diag(
                                    DiagnosticKind::MissingGroupName,
                                    ad_hoc_display!("missing group name").spanned2(span),
                                );
                                return;
                            }
                        };
                        let name = eval_string(
                            self.r.map,
                            self.r.diagnostics,
                            self.r.interner,
                            self.r.cooker,
                            expr,
                        );
                        let name = match name {
                            Ok(n) => n,
                            Err(e) => {
                                self.r.diag(e.val.diagnostic_kind(), e);
                                return;
                            }
                        };
                        let idx = cs[0].index.as_ref().unwrap();
                        let group_idx = eval_group(self.r.interner, idx.index.as_ref());
                        match group_idx {
                            Ok(idx) => {
                                self.handle_group_name(mm, idx.val, name);
                            }
                            Err(e) => {
                                self.r.diag(e.val.diagnostic_kind(), e);
                            }
                        }
                    }
                    _ => {
                        let res = eval_action_default(
                            self.r.interner,
                            self.r.meaning_cache,
                            self.keycodes,
                            self.mods,
                            &v.var,
                            span,
                            &mut self.data.action_defaults,
                        );
                        if let Err(e) = res {
                            self.r.diag(e.val.diagnostic_kind(), e);
                        }
                    }
                }
            }
            SymbolsDecl::Include(_) | SymbolsDecl::VMod(_) => {}
        }
    }
}
