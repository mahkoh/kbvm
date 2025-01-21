use {
    crate::{
        from_bytes::FromBytes,
        syms,
        xkb::{
            code_map::CodeMap,
            controls::ControlMask,
            diagnostic::{
                DiagnosticKind::{self},
                DiagnosticSink,
            },
            group::{GroupChange, GroupIdx, GroupMask},
            group_component::GroupComponent,
            interner::{Interned, Interner},
            kccgst::ast::{CallArg, Expr, Path, Var},
            level::Level,
            meaning::{Meaning, MeaningCache},
            mod_component::ModComponentMask,
            modmap::Vmodmap,
            resolved::{
                ActionDefaults, Filter, ModMapField, Predicate, ResolvedAction,
                ResolvedActionAffect, ResolvedActionMods, ResolvedGroupLatch, ResolvedGroupLock,
                ResolvedGroupSet, ResolvedKeyKind, ResolvedKeycodes, ResolvedLockControls,
                ResolvedModsLatch, ResolvedModsLock, ResolvedModsSet, ResolvedNoAction,
                ResolvedRedirectKey, ResolvedSetControls, ResolvedTypes,
            },
            span::{Span, SpanExt, SpanResult1, SpanResult2, Spanned},
            string_cooker::StringCooker,
        },
        Keycode, Keysym, ModifierIndex, ModifierMask,
    },
    isnt::std_1::primitive::IsntU8SliceExt,
    smallvec::SmallVec,
    std::ops::{BitAnd, BitOr, Deref, Not},
    thiserror::Error,
    EvalError::*,
};

#[derive(Copy, Clone, Debug, Error)]
pub(crate) enum EvalError {
    #[error("modifier calculation overflowed")]
    ModsCalculationOverflow,
    #[error("keysym calculation overflowed")]
    KeysymCalculationOverflow,
    #[error("level calculation overflowed")]
    LevelCalculationOverflow,
    #[error("unknown group")]
    UnknownGroup,
    #[error("unsupported expression for group")]
    UnsupportedExpressionForGroup,
    #[error("unsupported expression for group change")]
    UnsupportedExpressionForGroupChange,
    #[error("group calculation overflowed")]
    GroupCalculationOverflow,
    #[error("unknown level")]
    UnknownLevel,
    #[error("unsupported expression for level")]
    UnsupportedExpressionForLevel,
    #[error("unsupported expression for string")]
    UnsupportedExpressionForString,
    #[error("unsupported expression for boolean")]
    UnsupportedExpressionForBoolean,
    #[error("unsupported expression for keysym")]
    UnsupportedExpressionForKeysym,
    #[error("unsupported expression for keycode")]
    UnsupportedExpressionForKeycode,
    #[error("unsupported expression for mod mask")]
    UnsupportedExpressionForModMask,
    #[error("unsupported expression for key repeat")]
    UnsupportedExpressionForKeyRepeats,
    #[error("unsupported expression for LockMods(affect)")]
    UnsupportedExpressionForLockModsAffect,
    #[error("unsupported expression for action")]
    UnsupportedExpressionForAction,
    #[error("unsupported expression for symbols/actions")]
    UnsupportedExpressionForSymbolsOrActions,
    #[error("unknown boolean value")]
    UnknownBooleanValue,
    #[error("unknown keysym")]
    UnknownKeysym,
    #[error("unknown action")]
    UnknownAction,
    #[error("unimplemented action")]
    UnimplementedAction,
    #[error("unknown NoAction parameter")]
    UnknownParameterForNoAction,
    #[error("unknown SetMods parameter")]
    UnknownParameterForSetMods,
    #[error("unknown LatchMods parameter")]
    UnknownParameterForLatchMods,
    #[error("unknown LockMods parameter")]
    UnknownParameterForLockMods,
    #[error("unknown SetGroup parameter")]
    UnknownParameterForSetGroup,
    #[error("unknown LatchGroup parameter")]
    UnknownParameterForLatchGroup,
    #[error("unknown LockGroup parameter")]
    UnknownParameterForLockGroup,
    #[error("missing value for SetMods(mods)")]
    MissingValueForSetModsMods,
    #[error("missing value for LatchMods(mods)")]
    MissingValueForLatchModsMods,
    #[error("missing value for LockMods(mods)")]
    MissingValueForLockModsMods,
    #[error("missing value for LockMods(affect)")]
    MissingValueForLockModsAffect,
    #[error("missing value for SetGroup(group)")]
    MissingValueForSetGroupGroup,
    #[error("missing value for LatchGroup(group)")]
    MissingValueForLatchGroupGroup,
    #[error("missing value for LockGroup(group)")]
    MissingValueForLockGroupGroup,
    #[error("missing value for RedirectKey(key)")]
    MissingValueForRedirectKeyKey,
    #[error("missing value for RedirectKey(clearMods)")]
    MissingValueForRedirectKeyClearmods,
    #[error("missing value for RedirectKey(mods)")]
    MissingValueForRedirectKeyMods,
    #[error("missing value for SetControls(controls)")]
    MissingValueForSetControlsControls,
    #[error("missing value for LockControls(controls)")]
    MissingValueForLockControlsControls,
    #[error("missing value for LockControls(affect)")]
    MissingValueForLockControlsAffect,
    #[error("unknown modifier")]
    UnknownModifier,
    #[error("unknown LockMods(affect) value")]
    UnknownLockModsAffect,
    #[error("interpret filter must have exactly one argument")]
    NotOneInterpretFilterArgument,
    #[error("interpret filter argument must not have a name")]
    NamedFilterArgArgument,
    #[error("unknown interpret filter predicate")]
    UnknownFilterPredicate,
    #[error("unknown `interpret` field")]
    UnknownInterpretField,
    #[error("interpret `action` argument must have a value")]
    MissingInterpretActionValue,
    #[error("interpret `virtualmodifier` argument must have a value")]
    MissingInterpretVirtualmodValue,
    #[error("unknown virtual modifier")]
    UnknownInterpretVirtualModifier,
    #[error("unknown `type` field")]
    UnknownTypeField,
    #[error("type `modifiers` argument must have a value")]
    MissingTypeModifiersValue,
    #[error("type `map` argument must have a value")]
    MissingTypeMapValue,
    #[error("type `preserve` argument must have a value")]
    MissingTypePreserveValue,
    #[error("type `level_name` argument must have a value")]
    MissingTypeLevelNameValue,
    #[error("interpret `useModMap` argument must have a value")]
    MissingInterpretUseModMapModValue,
    #[error("invalid interpret `useModMap` argument")]
    InvalidInterpretUseModMapModValue,
    #[error("unknown `indicator` field")]
    UnknownIndicatorField,
    #[error("unimplemented `indicator` field")]
    UnimplementedIndicatorField,
    #[error("indicator `modifiers` argument must have a value")]
    MissingIndicatorModifiersValue,
    #[error("out of bounds group value")]
    GroupOutOfBounds,
    #[error("out of bounds level value")]
    LevelOutOfBounds,
    #[error("indicator `groups` argument must have a value")]
    MissingIndicatorGroupsValue,
    #[error("unknown control")]
    UnknownControl,
    #[error("indicator `control` argument must have a value")]
    MissingIndicatorControlValue,
    #[error("unknown mod component")]
    UnknownModComponent,
    #[error("unsupported expression for mod component")]
    UnsupportedExpressionForModComponent,
    #[error("unsupported expression for control")]
    UnsupportedExpressionForControl,
    #[error("indicator `whichModState` argument must have a value")]
    MissingIndicatorWhichModStateValue,
    #[error("unknown group component")]
    UnknownGroupComponent,
    #[error("unsupported expression for group component")]
    UnsupportedExpressionForGroupComponent,
    #[error("indicator `whichGroupState` argument must have a value")]
    MissingIndicatorWhichGroupStateValue,
    #[error("unknown keycode")]
    UnknownKeycode,
    #[error("unknown `key` field")]
    UnknownKeyField,
    #[error("key `groupsRedirect` argument must have a value")]
    MissingKeyGroupsRedirectValue,
    #[error("key `symbols` argument must have a value")]
    MissingKeySymbolsValue,
    #[error("key `actions` argument must have a value")]
    MissingKeyActionsValue,
    #[error("key `virtual_modifiers` argument must have a value")]
    MissingKeyVirtualModifiersValue,
    #[error("unknown key `repeating` value")]
    UnknownKeyRepeatingValue,
    #[error("key `type` argument must have a value")]
    MissingKeyTypeValue,
    #[error("unknown key type")]
    UnknownKeyType,
    #[error("unknown RedirectKey parameter")]
    UnknownParameterForRedirectKey,
    #[error("unknown SetControls parameter")]
    UnknownParameterForSetControls,
    #[error("unknown LockControls parameter")]
    UnknownParameterForLockControls,
}

impl EvalError {
    pub(crate) fn diagnostic_kind(&self) -> DiagnosticKind {
        macro_rules! map {
            ($($ident:ident,)*) => {
                match self {
                    $(
                        $ident => DiagnosticKind::$ident,
                    )*
                }
            };
        }
        map! {
            ModsCalculationOverflow,
            KeysymCalculationOverflow,
            LevelCalculationOverflow,
            UnknownGroup,
            UnsupportedExpressionForGroup,
            UnsupportedExpressionForGroupChange,
            GroupCalculationOverflow,
            UnknownLevel,
            UnsupportedExpressionForLevel,
            UnsupportedExpressionForString,
            UnsupportedExpressionForBoolean,
            UnsupportedExpressionForKeysym,
            UnsupportedExpressionForModMask,
            UnsupportedExpressionForKeyRepeats,
            UnsupportedExpressionForLockModsAffect,
            UnsupportedExpressionForAction,
            UnsupportedExpressionForSymbolsOrActions,
            UnsupportedExpressionForKeycode,
            UnknownBooleanValue,
            UnknownKeysym,
            UnknownAction,
            UnimplementedAction,
            UnknownParameterForNoAction,
            UnknownParameterForSetMods,
            UnknownParameterForLatchMods,
            UnknownParameterForLockMods,
            UnknownParameterForSetGroup,
            UnknownParameterForLatchGroup,
            UnknownParameterForLockGroup,
            UnknownParameterForRedirectKey,
            UnknownParameterForSetControls,
            UnknownParameterForLockControls,
            MissingValueForSetModsMods,
            MissingValueForLatchModsMods,
            MissingValueForLockModsMods,
            MissingValueForLockModsAffect,
            MissingValueForSetGroupGroup,
            MissingValueForLatchGroupGroup,
            MissingValueForLockGroupGroup,
            MissingValueForRedirectKeyKey,
            MissingValueForRedirectKeyClearmods,
            MissingValueForRedirectKeyMods,
            MissingValueForSetControlsControls,
            MissingValueForLockControlsControls,
            MissingValueForLockControlsAffect,
            UnknownModifier,
            UnknownLockModsAffect,
            NotOneInterpretFilterArgument,
            NamedFilterArgArgument,
            UnknownFilterPredicate,
            UnknownInterpretField,
            MissingInterpretActionValue,
            MissingInterpretVirtualmodValue,
            UnknownInterpretVirtualModifier,
            UnknownTypeField,
            MissingTypeModifiersValue,
            MissingTypeMapValue,
            MissingTypePreserveValue,
            MissingTypeLevelNameValue,
            MissingInterpretUseModMapModValue,
            InvalidInterpretUseModMapModValue,
            UnknownIndicatorField,
            UnimplementedIndicatorField,
            MissingIndicatorModifiersValue,
            GroupOutOfBounds,
            LevelOutOfBounds,
            MissingIndicatorGroupsValue,
            UnknownControl,
            MissingIndicatorControlValue,
            UnknownModComponent,
            UnsupportedExpressionForModComponent,
            UnsupportedExpressionForControl,
            MissingIndicatorWhichModStateValue,
            UnknownGroupComponent,
            UnsupportedExpressionForGroupComponent,
            MissingIndicatorWhichGroupStateValue,
            UnknownKeycode,
            UnknownKeyField,
            MissingKeyGroupsRedirectValue,
            MissingKeySymbolsValue,
            MissingKeyActionsValue,
            MissingKeyVirtualModifiersValue,
            UnknownKeyRepeatingValue,
            MissingKeyTypeValue,
            UnknownKeyType,
        }
    }
}

pub(crate) fn eval_group(
    interner: &Interner,
    expr: Spanned<&Expr>,
) -> Result<Spanned<GroupIdx>, Spanned<EvalError>> {
    let res = eval_u32_str_prefix(
        interner,
        expr,
        "group",
        UnknownGroup,
        UnsupportedExpressionForGroup,
        GroupCalculationOverflow,
    )?;
    GroupIdx::new(res.val)
        .ok_or(GroupOutOfBounds)
        .span_either(res.span)
}

pub(crate) fn eval_group_mask(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    expr: Spanned<&Expr>,
) -> Result<Spanned<GroupMask>, Spanned<EvalError>> {
    eval_u32(
        expr,
        UnknownGroup,
        UnsupportedExpressionForGroup,
        GroupCalculationOverflow,
        |i| {
            match meaning_cache.get_case_insensitive(interner, i) {
                Meaning::All => return Ok(u32::MAX as _),
                Meaning::None => return Ok(0),
                _ => {}
            };
            let num = parse_u32_1_to_32_str_prefix(interner, i, "group").ok_or(UnknownGroup)?;
            Ok(1 << (num - 1))
        },
    )
    .span_map(GroupMask)
}

pub(crate) fn eval_control_mask(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    expr: Spanned<&Expr>,
) -> Result<Spanned<ControlMask>, Spanned<EvalError>> {
    eval_keyed_mask(
        interner,
        meaning_cache,
        expr,
        UnknownControl,
        UnsupportedExpressionForControl,
        &mut |meaning| ControlMask::from_meaning(meaning).ok_or(UnknownControl),
    )
}

pub(crate) fn eval_mod_component_mask(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    expr: Spanned<&Expr>,
) -> Result<Spanned<ModComponentMask>, Spanned<EvalError>> {
    eval_keyed_mask(
        interner,
        meaning_cache,
        expr,
        UnknownModComponent,
        UnsupportedExpressionForModComponent,
        &mut |meaning| ModComponentMask::from_meaning(meaning).ok_or(UnknownModComponent),
    )
}

pub(crate) fn eval_group_component(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    expr: Spanned<&Expr>,
) -> Result<Spanned<GroupComponent>, Spanned<EvalError>> {
    let interned = eval_interned(expr, UnsupportedExpressionForGroupComponent)?;
    let meaning = meaning_cache.get_case_insensitive(interner, interned.val);
    let component = match meaning {
        Meaning::None => GroupComponent::None,
        Meaning::Base => GroupComponent::Base,
        Meaning::Latched => GroupComponent::Latched,
        Meaning::Locked => GroupComponent::Locked,
        Meaning::Effective => GroupComponent::Effective,
        _ => return Err(UnknownGroupComponent.spanned2(expr.span)),
    };
    Ok(component.spanned2(expr.span))
}

pub(crate) fn eval_level(
    interner: &Interner,
    expr: Spanned<&Expr>,
) -> Result<Spanned<Level>, Spanned<EvalError>> {
    let res = eval_u32_str_prefix(
        interner,
        expr,
        "level",
        UnknownLevel,
        UnsupportedExpressionForLevel,
        LevelCalculationOverflow,
    )?;
    Level::new(res.val)
        .ok_or(LevelOutOfBounds)
        .span_either(res.span)
}

fn parse_u32_1_to_32_str_prefix(interner: &Interner, i: Interned, prefix: &str) -> Option<u32> {
    let n = interner.get(i);
    let len = prefix.len();
    if n.len() < len {
        return None;
    }
    if n[..len].not_eq_ignore_ascii_case(prefix.as_bytes()) {
        return None;
    }
    let num = &n[len..];
    let num = u32::from_bytes_dec(num)?;
    if num == 0 || num > u32::BITS {
        return None;
    };
    Some(num)
}

fn eval_u32_str_prefix(
    interner: &Interner,
    expr: Spanned<&Expr>,
    prefix: &str,
    unknown_value: EvalError,
    unsupported_expression_type: EvalError,
    overflow: EvalError,
) -> Result<Spanned<u32>, Spanned<EvalError>> {
    eval_u32(
        expr,
        unknown_value,
        unsupported_expression_type,
        overflow,
        |i| {
            parse_u32_1_to_32_str_prefix(interner, i, prefix)
                .map(|num| num as _)
                .ok_or(unknown_value)
        },
    )
}

fn eval_i32_str_prefix(
    interner: &Interner,
    expr: Spanned<&Expr>,
    prefix: &str,
    unknown_value: EvalError,
    unsupported_expression_type: EvalError,
    overflow: EvalError,
) -> Result<Spanned<i32>, Spanned<EvalError>> {
    let res = eval_i64(
        expr,
        unknown_value,
        unsupported_expression_type,
        overflow,
        &mut |i| {
            parse_u32_1_to_32_str_prefix(interner, i, prefix)
                .map(|num| num as _)
                .ok_or(unknown_value)
        },
    )?;
    i32::try_from(res.val)
        .map_err(|_| overflow)
        .span_either(res.span)
}

pub(crate) fn eval_u32(
    expr: Spanned<&Expr>,
    unknown_value: EvalError,
    unsupported_expression_type: EvalError,
    overflow: EvalError,
    mut f: impl FnMut(Interned) -> Result<i64, EvalError>,
) -> Result<Spanned<u32>, Spanned<EvalError>> {
    u32::try_from(
        eval_i64(
            expr,
            unknown_value,
            unsupported_expression_type,
            overflow,
            &mut f,
        )?
        .val,
    )
    .map_err(|_| overflow)
    .span_either(expr.span)
}

fn eval_keycode(
    keycodes: &ResolvedKeycodes,
    expr: Spanned<&Expr>,
) -> Result<Spanned<(Interned, Keycode)>, Spanned<EvalError>> {
    let Expr::KeyName(name) = &expr.val else {
        return Err(UnsupportedExpressionForKeycode.spanned2(expr.span));
    };
    let Some(kc) = keycodes.name_to_key.get(name) else {
        return Err(UnknownKeycode.spanned2(expr.span));
    };
    let (name, kc) = match kc.kind {
        ResolvedKeyKind::Real(kc) => (*name, kc.val),
        ResolvedKeyKind::Alias(n1, kc, n2) => (n2.unwrap_or(n1).val, kc),
    };
    Ok((name, kc).spanned2(expr.span))
}

pub(crate) fn eval_group_change(
    interner: &Interner,
    expr: Spanned<&Expr>,
) -> Result<Spanned<GroupChange>, Spanned<EvalError>> {
    let (expr, neg) = match &expr.val {
        Expr::UnPlus(e) => (e, false),
        Expr::UnMinus(e) => (e, true),
        _ => return eval_group(interner, expr).span_map(GroupChange::Absolute),
    };
    let mut res = eval_i32_str_prefix(
        interner,
        expr.deref().as_ref(),
        "group",
        UnknownGroup,
        UnsupportedExpressionForGroupChange,
        GroupCalculationOverflow,
    )?;
    if neg {
        res.val = res
            .val
            .checked_neg()
            .ok_or(GroupCalculationOverflow.spanned2(expr.span))?;
    }
    Ok(res.map(GroupChange::Rel))
}

fn eval_i64(
    expr: Spanned<&Expr>,
    unknown_value: EvalError,
    unsupported_expression_type: EvalError,
    overflow: EvalError,
    f: &mut impl FnMut(Interned) -> Result<i64, EvalError>,
) -> Result<Spanned<i64>, Spanned<EvalError>> {
    macro_rules! fwd {
        ($val:expr) => {
            eval_i64(
                $val.deref().as_ref(),
                unknown_value,
                unsupported_expression_type,
                overflow,
                f,
            )?
            .val
        };
    }
    macro_rules! bi {
        ($op:ident, $l:expr, $r:expr) => {
            fwd!($l).$op(fwd!($r)).ok_or(overflow)
        };
    }
    let res = match expr.val {
        Expr::UnMinus(v) => fwd!(v).checked_neg().ok_or(overflow),
        Expr::UnPlus(v) => Ok(fwd!(v)),
        Expr::UnNot(v) => Ok((fwd!(v) == 0) as i64),
        Expr::UnInverse(v) => Ok(!fwd!(v) & u32::MAX as i64),
        Expr::Path(p) => p.unique_ident().ok_or(unknown_value).and_then(f),
        Expr::Integer(_, i) => Ok(*i),
        Expr::Parenthesized(p) => Ok(fwd!(p)),
        Expr::Mul(l, r) => bi!(checked_mul, l, r),
        Expr::Div(l, r) => bi!(checked_div, l, r),
        Expr::Add(l, r) => bi!(checked_add, l, r),
        Expr::Sub(l, r) => bi!(checked_sub, l, r),
        _ => Err(unsupported_expression_type),
    };
    res.span_either(expr.span)
}

fn eval_keyed_mask<T>(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    expr: Spanned<&Expr>,
    unknown_value: EvalError,
    unsupported_expression_type: EvalError,
    f: &mut impl FnMut(Meaning) -> Result<T, EvalError>,
) -> Result<Spanned<T>, Spanned<EvalError>>
where
    T: BitAnd<Output = T> + BitOr<Output = T> + Not<Output = T>,
{
    macro_rules! fwd {
        ($val:expr) => {
            eval_keyed_mask::<T>(
                interner,
                meaning_cache,
                $val.deref().as_ref(),
                unknown_value,
                unsupported_expression_type,
                f,
            )?
            .val
        };
    }
    let res = match expr.val {
        Expr::UnNot(v) | Expr::UnInverse(v) => Ok(!fwd!(v)),
        Expr::Path(p) => p.unique_ident().ok_or(unknown_value).and_then(|i| {
            let meaning = meaning_cache.get_case_insensitive(interner, i);
            f(meaning)
        }),
        Expr::Parenthesized(p) => Ok(fwd!(p)),
        Expr::Add(l, r) => Ok(fwd!(l) | fwd!(r)),
        Expr::Sub(l, r) => Ok(fwd!(l) & !fwd!(r)),
        _ => Err(unsupported_expression_type),
    };
    res.span_either(expr.span)
}

pub(crate) fn eval_string(
    map: &mut CodeMap,
    diagnostics: &mut DiagnosticSink,
    interner: &mut Interner,
    cooker: &mut StringCooker,
    expr: Spanned<&Expr>,
) -> Result<Spanned<Interned>, Spanned<EvalError>> {
    match &expr.val {
        Expr::String(e) => Ok(cooker
            .cook(map, diagnostics, interner, (*e).spanned2(expr.span))
            .spanned2(expr.span)),
        _ => Err(UnsupportedExpressionForString.spanned2(expr.span)),
    }
}

fn eval_boolean(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    expr: Spanned<&Expr>,
) -> Result<bool, Spanned<EvalError>> {
    match &expr.val {
        Expr::UnNot(e) | Expr::UnInverse(e) => {
            eval_boolean(interner, meaning_cache, e.deref().as_ref()).map(|b| !b)
        }
        Expr::Path(p) => p
            .unique_ident()
            .and_then(|i| {
                let meaning = meaning_cache.get_case_insensitive(interner, i);
                match meaning {
                    Meaning::True | Meaning::Yes | Meaning::On => Some(true),
                    Meaning::False | Meaning::No | Meaning::Off => Some(false),
                    _ => None,
                }
            })
            .ok_or(UnknownBooleanValue.spanned2(expr.span)),
        _ => Err(UnsupportedExpressionForBoolean.spanned2(expr.span)),
    }
}

pub(crate) fn keysym_from_ident(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    ident: Spanned<Interned>,
) -> Result<Spanned<Keysym>, Spanned<EvalError>> {
    let meaning = meaning_cache.get_case_insensitive(interner, ident.val);
    let sym = match meaning {
        Meaning::Any | Meaning::Nosymbol => syms::NoSymbol,
        Meaning::None | Meaning::Voidsymbol => syms::VoidSymbol,
        _ => {
            return Keysym::from_str(interner.get(ident.val))
                .ok_or(UnknownKeysym)
                .span_either(ident.span);
        }
    };
    Ok(sym.spanned2(ident.span))
}

pub(crate) fn eval_keysym(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    expr: Spanned<&Expr>,
) -> Result<Spanned<Keysym>, Spanned<EvalError>> {
    let res = match &expr.val {
        Expr::Path(p) => match p.unique_ident() {
            Some(id) => return keysym_from_ident(interner, meaning_cache, id.spanned2(expr.span)),
            _ => Err(UnknownKeysym),
        },
        Expr::Integer(_, i) => match *i {
            0..=9 => Ok(Keysym(syms::_0.0 + *i as u32)),
            _ => u32::try_from(*i)
                .map(Keysym)
                .map_err(|_| KeysymCalculationOverflow),
        },
        _ => Err(UnsupportedExpressionForKeysym),
    };
    res.span_either(expr.span)
}

fn meaning_to_real_mod_index(meaning: Meaning) -> Option<ModifierIndex> {
    let v = match meaning {
        Meaning::Shift => 0,
        Meaning::Lock => 1,
        Meaning::Control => 2,
        Meaning::Mod1 => 3,
        Meaning::Mod2 => 4,
        Meaning::Mod3 => 5,
        Meaning::Mod4 => 6,
        Meaning::Mod5 => 7,
        _ => return None,
    };
    ModifierIndex::new(v)
}

pub(crate) fn ident_to_real_mod_index(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    ident: Interned,
) -> Option<ModifierIndex> {
    let meaning = meaning_cache.get_case_insensitive(interner, ident);
    meaning_to_real_mod_index(meaning)
}

fn ident_to_real_mod_mask(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    ident: Interned,
) -> Option<u32> {
    let meaning = meaning_cache.get_case_insensitive(interner, ident);
    let v = match meaning {
        Meaning::None => 0x00,
        Meaning::All => 0xff,
        _ => return meaning_to_real_mod_index(meaning).map(|v| v.to_mask().0),
    };
    Some(v)
}

pub(crate) fn eval_real_mods(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    expr: Spanned<&Expr>,
) -> Result<Spanned<ModifierMask>, Spanned<EvalError>> {
    eval_mods_(expr, &mut |ident| {
        ident_to_real_mod_mask(interner, meaning_cache, ident)
    })
}

pub(crate) fn eval_mods(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    vmods: &Vmodmap,
    expr: Spanned<&Expr>,
) -> Result<Spanned<ModifierMask>, Spanned<EvalError>> {
    eval_mods_(expr, &mut |ident| {
        ident_to_real_mod_mask(interner, meaning_cache, ident)
            .or_else(|| vmods.get(ident).map(|m| m.idx.to_mask().0))
    })
}

pub(crate) fn eval_virtual_mods(
    vmods: &Vmodmap,
    expr: Spanned<&Expr>,
) -> Result<Spanned<ModifierMask>, Spanned<EvalError>> {
    eval_mods_(expr, &mut |ident| {
        vmods.get(ident).map(|m| m.idx.to_mask().0)
    })
}

fn eval_mods_(
    expr: Spanned<&Expr>,
    name_to_mod: &mut impl FnMut(Interned) -> Option<u32>,
) -> Result<Spanned<ModifierMask>, Spanned<EvalError>> {
    let res = match &expr.val {
        Expr::UnInverse(v) => {
            return eval_mods_(v.deref().as_ref(), name_to_mod).map(|mut v| {
                let only_real = v.val.0 & !0xff == 0;
                v.val.0 = !v.val.0;
                if only_real {
                    // assume that only real modifiers are desired here
                    v.val.0 &= 0xff;
                }
                v.val.spanned2(expr.span)
            });
        }
        Expr::Path(p) => p
            .unique_ident()
            .and_then(name_to_mod)
            .ok_or(UnknownModifier),
        Expr::Integer(_, v) => u32::try_from(*v).map_err(|_| ModsCalculationOverflow),
        Expr::Parenthesized(v) => return eval_mods_(v.deref().as_ref(), name_to_mod),
        Expr::Add(l, r) => {
            let l = eval_mods_(l.deref().as_ref(), name_to_mod)?;
            let r = eval_mods_(r.deref().as_ref(), name_to_mod)?;
            Ok(l.val.0 | r.val.0)
        }
        Expr::Sub(l, r) => {
            let l = eval_mods_(l.deref().as_ref(), name_to_mod)?;
            let r = eval_mods_(r.deref().as_ref(), name_to_mod)?;
            Ok(l.val.0 & !r.val.0)
        }
        _ => Err(UnsupportedExpressionForModMask),
    };
    res.map(ModifierMask).span_either(expr.span)
}

pub(crate) fn eval_interned(
    expr: Spanned<&Expr>,
    unsupported_expression_type: EvalError,
) -> Result<Spanned<Interned>, Spanned<EvalError>> {
    if let Expr::Path(p) = expr.val {
        if let Some(u) = p.unique_ident() {
            return Ok(u.spanned2(expr.span));
        }
    }
    Err(unsupported_expression_type.spanned2(expr.span))
}

pub(crate) fn eval_filter(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    mut expr: Spanned<&Expr>,
) -> Result<Spanned<Filter>, Spanned<EvalError>> {
    if let Expr::Path(p) = &expr.val {
        if let Some(u) = p.unique_ident() {
            let meaning = meaning_cache.get_case_insensitive(interner, u);
            if meaning == Meaning::Any {
                return Ok(Filter {
                    predicate: Predicate::AnyOf,
                    mask: ModifierMask(!0).spanned2(expr.span),
                }
                .spanned2(expr.span));
            }
        }
    }
    let mut predicate = Predicate::Exactly;
    if let Expr::Call(e) = expr.val {
        let cs = &e.path.val.components;
        let c = &cs[0];
        if cs.len() != 1 || c.index.is_some() {
            return Err(UnknownFilterPredicate.spanned2(e.path.span));
        }
        let meaning = meaning_cache.get_case_insensitive(interner, c.ident.val);
        predicate = match meaning {
            Meaning::NoneOf => Predicate::NoneOf,
            Meaning::AnyOfOrNone => Predicate::AnyOfOrNone,
            Meaning::AnyOf => Predicate::AnyOf,
            Meaning::AllOf => Predicate::AllOf,
            Meaning::Exactly => Predicate::Exactly,
            _ => return Err(UnknownFilterPredicate.spanned2(e.path.span)),
        };
        if e.args.len() != 1 {
            return Err(NotOneInterpretFilterArgument.spanned2(expr.span));
        }
        let c = &e.args[0];
        expr = match &c.val {
            CallArg::Expr(e) => e.spanned2(c.span),
            CallArg::NamedParam(e) => {
                return Err(NamedFilterArgArgument.spanned2(e.name.span));
            }
        };
    }
    Ok(Filter {
        predicate,
        mask: eval_real_mods(interner, meaning_cache, expr)?,
    }
    .spanned2(expr.span))
}

enum ActionParameterValue<'a> {
    Boolean(bool),
    Expr(&'a Expr),
}

trait ActionParameters: Clone {
    const UNKNOWN_PARAMETER: EvalError;

    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
        keycodes: &ResolvedKeycodes,
        vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), Spanned<EvalError>>;
}

macro_rules! boolean {
    ($interner:expr, $meaning_cache:expr, $value:expr) => {
        match $value.val {
            ActionParameterValue::Boolean(b) => Ok(b.spanned2($value.span)),
            ActionParameterValue::Expr(e) => {
                eval_boolean($interner, $meaning_cache, e.spanned2($value.span))
                    .map(|v| v.spanned2($value.span))
            }
        }
    };
}

macro_rules! value {
    ($value:expr, $err:expr) => {
        match $value.val {
            ActionParameterValue::Boolean(_) => {
                return Err($err.spanned2($value.span));
            }
            ActionParameterValue::Expr(e) => e.spanned2($value.span),
        }
    };
}

fn eval_action_mods(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    vmods: &Vmodmap,
    expr: Spanned<&Expr>,
) -> Result<Spanned<ResolvedActionMods>, Spanned<EvalError>> {
    if let Expr::Path(p) = &expr.val {
        if let Some(id) = p.unique_ident() {
            let meaning = meaning_cache.get_case_insensitive(interner, id);
            if matches!(meaning, Meaning::Usemodmapmods | Meaning::Modmapmods) {
                return Ok(ResolvedActionMods::ModMap.spanned2(expr.span));
            }
        }
    }
    let mods = eval_mods(interner, meaning_cache, vmods, expr)?;
    Ok(mods.map(ResolvedActionMods::Explicit))
}

fn eval_action_affect(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    expr: Spanned<&Expr>,
) -> Result<ResolvedActionAffect, Spanned<EvalError>> {
    if let Expr::Path(p) = &expr.val {
        if let Some(id) = p.unique_ident() {
            let meaning = meaning_cache.get_case_insensitive(interner, id);
            let (lock, unlock) = match meaning {
                Meaning::Lock => (true, false),
                Meaning::Unlock => (false, true),
                Meaning::Both => (true, true),
                Meaning::Neither => (false, false),
                _ => return Err(UnknownLockModsAffect.spanned2(expr.span)),
            };
            return Ok(ResolvedActionAffect { lock, unlock });
        }
    }
    Err(UnsupportedExpressionForLockModsAffect.spanned2(expr.span))
}

impl ActionParameters for ResolvedNoAction {
    const UNKNOWN_PARAMETER: EvalError = UnknownParameterForNoAction;

    fn handle_field(
        &mut self,
        _interner: &Interner,
        _meaning_cache: &mut MeaningCache,
        _keycodes: &ResolvedKeycodes,
        _vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        _value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), Spanned<EvalError>> {
        Err(UnknownParameterForNoAction.spanned2(meaning.span))
    }
}

impl ActionParameters for ResolvedModsSet {
    const UNKNOWN_PARAMETER: EvalError = UnknownParameterForSetMods;

    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
        _keycodes: &ResolvedKeycodes,
        vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), Spanned<EvalError>> {
        match meaning.val {
            Meaning::ClearLocks => {
                self.clear_locks = Some(boolean!(interner, meaning_cache, value)?)
            }
            Meaning::Mods | Meaning::Modifiers => {
                self.modifiers = Some(eval_action_mods(
                    interner,
                    meaning_cache,
                    vmods,
                    value!(value, MissingValueForSetModsMods),
                )?);
            }
            _ => {
                return Err(UnknownParameterForSetMods.spanned2(meaning.span));
            }
        }
        Ok(())
    }
}

impl ActionParameters for ResolvedModsLatch {
    const UNKNOWN_PARAMETER: EvalError = UnknownParameterForLatchMods;

    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
        _keycodes: &ResolvedKeycodes,
        vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), Spanned<EvalError>> {
        match meaning.val {
            Meaning::ClearLocks => {
                self.clear_locks = Some(boolean!(interner, meaning_cache, value)?)
            }
            Meaning::LatchToLock => {
                self.latch_to_lock = Some(boolean!(interner, meaning_cache, value)?)
            }
            Meaning::Mods | Meaning::Modifiers => {
                self.modifiers = Some(eval_action_mods(
                    interner,
                    meaning_cache,
                    vmods,
                    value!(value, MissingValueForLatchModsMods),
                )?);
            }
            _ => {
                return Err(UnknownParameterForLatchMods.spanned2(meaning.span));
            }
        }
        Ok(())
    }
}

impl ActionParameters for ResolvedModsLock {
    const UNKNOWN_PARAMETER: EvalError = UnknownParameterForLockMods;

    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
        _keycodes: &ResolvedKeycodes,
        vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), Spanned<EvalError>> {
        match meaning.val {
            Meaning::Mods | Meaning::Modifiers => {
                self.modifiers = Some(eval_action_mods(
                    interner,
                    meaning_cache,
                    vmods,
                    value!(value, MissingValueForLockModsMods),
                )?);
            }
            Meaning::Affect => {
                self.affect = Some(
                    eval_action_affect(
                        interner,
                        meaning_cache,
                        value!(value, MissingValueForLockModsAffect),
                    )?
                    .spanned2(value.span),
                );
            }
            _ => {
                return Err(UnknownParameterForLockMods.spanned2(meaning.span));
            }
        }
        Ok(())
    }
}

impl ActionParameters for ResolvedGroupSet {
    const UNKNOWN_PARAMETER: EvalError = UnknownParameterForSetGroup;

    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
        _keycodes: &ResolvedKeycodes,
        _vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), Spanned<EvalError>> {
        match meaning.val {
            Meaning::Group => {
                self.group = Some(eval_group_change(
                    interner,
                    value!(value, MissingValueForSetGroupGroup),
                )?);
            }
            Meaning::ClearLocks => {
                self.clear_locks = Some(boolean!(interner, meaning_cache, value)?)
            }
            _ => {
                return Err(UnknownParameterForSetGroup.spanned2(meaning.span));
            }
        }
        Ok(())
    }
}

impl ActionParameters for ResolvedGroupLatch {
    const UNKNOWN_PARAMETER: EvalError = UnknownParameterForLatchGroup;

    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
        _keycodes: &ResolvedKeycodes,
        _vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), Spanned<EvalError>> {
        match meaning.val {
            Meaning::Group => {
                self.group = Some(eval_group_change(
                    interner,
                    value!(value, MissingValueForLatchGroupGroup),
                )?);
            }
            Meaning::ClearLocks => {
                self.clear_locks = Some(boolean!(interner, meaning_cache, value)?)
            }
            Meaning::LatchToLock => {
                self.latch_to_lock = Some(boolean!(interner, meaning_cache, value)?)
            }
            _ => {
                return Err(UnknownParameterForLatchGroup.spanned2(meaning.span));
            }
        }
        Ok(())
    }
}

impl ActionParameters for ResolvedGroupLock {
    const UNKNOWN_PARAMETER: EvalError = UnknownParameterForLockGroup;

    fn handle_field(
        &mut self,
        interner: &Interner,
        _meaning_cache: &mut MeaningCache,
        _keycodes: &ResolvedKeycodes,
        _vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), Spanned<EvalError>> {
        match meaning.val {
            Meaning::Group => {
                self.group = Some(eval_group_change(
                    interner,
                    value!(value, MissingValueForLockGroupGroup),
                )?);
            }
            _ => {
                return Err(UnknownParameterForLockGroup.spanned2(meaning.span));
            }
        }
        Ok(())
    }
}

impl ActionParameters for ResolvedRedirectKey {
    const UNKNOWN_PARAMETER: EvalError = UnknownParameterForRedirectKey;

    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
        keycodes: &ResolvedKeycodes,
        vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), Spanned<EvalError>> {
        match meaning.val {
            Meaning::Kc | Meaning::Keycode | Meaning::Key => {
                self.keycode = Some(eval_keycode(
                    keycodes,
                    value!(value, MissingValueForRedirectKeyKey),
                )?);
            }
            Meaning::Clearmods | Meaning::Clearmodifiers => {
                self.mods_to_clear = Some(eval_action_mods(
                    interner,
                    meaning_cache,
                    vmods,
                    value!(value, MissingValueForRedirectKeyClearmods),
                )?);
            }
            Meaning::Mods | Meaning::Modifiers => {
                self.mods_to_set = Some(eval_action_mods(
                    interner,
                    meaning_cache,
                    vmods,
                    value!(value, MissingValueForRedirectKeyMods),
                )?);
            }
            _ => {
                return Err(UnknownParameterForRedirectKey.spanned2(meaning.span));
            }
        }
        Ok(())
    }
}

impl ActionParameters for ResolvedSetControls {
    const UNKNOWN_PARAMETER: EvalError = UnknownParameterForSetControls;

    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
        _keycodes: &ResolvedKeycodes,
        _vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), Spanned<EvalError>> {
        match meaning.val {
            Meaning::Controls | Meaning::Ctrls => {
                self.controls = Some(eval_control_mask(
                    interner,
                    meaning_cache,
                    value!(value, MissingValueForSetControlsControls),
                )?);
            }
            _ => {
                return Err(UnknownParameterForSetControls.spanned2(meaning.span));
            }
        }
        Ok(())
    }
}

impl ActionParameters for ResolvedLockControls {
    const UNKNOWN_PARAMETER: EvalError = UnknownParameterForLockControls;

    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
        _keycodes: &ResolvedKeycodes,
        _vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), Spanned<EvalError>> {
        match meaning.val {
            Meaning::Controls | Meaning::Ctrls => {
                self.controls = Some(eval_control_mask(
                    interner,
                    meaning_cache,
                    value!(value, MissingValueForLockControlsControls),
                )?);
            }
            Meaning::Affect => {
                self.affect = Some(
                    eval_action_affect(
                        interner,
                        meaning_cache,
                        value!(value, MissingValueForLockControlsAffect),
                    )?
                    .spanned2(value.span),
                );
            }
            _ => {
                return Err(UnknownParameterForLockControls.spanned2(meaning.span));
            }
        }
        Ok(())
    }
}

fn create_action<T: ActionParameters>(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    keycodes: &ResolvedKeycodes,
    vmods: &Vmodmap,
    default: &T,
    args: &[Spanned<CallArg>],
) -> Result<T, Spanned<EvalError>> {
    let mut t = default.clone();
    for arg in args {
        let (path, value) = match &arg.val {
            CallArg::Expr(e) => match e {
                Expr::Path(p) => (
                    p.spanned2(arg.span),
                    ActionParameterValue::Boolean(true).spanned2(arg.span),
                ),
                Expr::UnNot(e) | Expr::UnInverse(e) => match &e.val {
                    Expr::Path(p) => (
                        p.spanned2(e.span),
                        ActionParameterValue::Boolean(false).spanned2(arg.span),
                    ),
                    _ => return Err(T::UNKNOWN_PARAMETER.spanned2(arg.span)),
                },
                _ => return Err(T::UNKNOWN_PARAMETER.spanned2(arg.span)),
            },
            CallArg::NamedParam(n) => (
                (&n.name.val).spanned2(n.name.span),
                ActionParameterValue::Expr(&n.expr.val).spanned2(n.expr.span),
            ),
        };
        let ident = path
            .val
            .unique_ident()
            .ok_or(T::UNKNOWN_PARAMETER.spanned2(path.span))?;
        let meaning = meaning_cache.get_case_insensitive(interner, ident);
        t.handle_field(
            interner,
            meaning_cache,
            keycodes,
            vmods,
            meaning.spanned2(path.span),
            value,
        )?;
    }
    Ok(t)
}

fn handle_action_global<T: ActionParameters>(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    keycodes: &ResolvedKeycodes,
    vmods: &Vmodmap,
    t: &mut T,
    var: &Var,
    span: Span,
) -> Result<(), Spanned<EvalError>> {
    let cs = &var.path.val.components;
    if cs.len() != 2 || cs[0].index.is_some() || cs[1].index.is_some() {
        return Err(T::UNKNOWN_PARAMETER.spanned2(span));
    }
    let c = &cs[1];
    let value = if let Some(e) = &var.expr {
        ActionParameterValue::Expr(&e.val).spanned2(e.span)
    } else if var.not.is_some() {
        ActionParameterValue::Boolean(false).spanned2(span)
    } else {
        ActionParameterValue::Boolean(true).spanned2(span)
    };
    let meaning = meaning_cache.get_case_insensitive(interner, c.ident.val);
    t.handle_field(
        interner,
        meaning_cache,
        keycodes,
        vmods,
        meaning.spanned2(var.path.span),
        value,
    )
}

macro_rules! generate_action_name_meta {
    ($($($meaning:ident)|* => $big:ident | $little:ident,)* ; $($unimplemented:ident,)*) => {
        macro_rules! action_name_meta {
            ($macro_:ident, $meaning_inner:expr, $span:expr) => {
                match $meaning_inner {
                    $($(Meaning::$meaning)|* => $macro_!($big, $little),)*
                    $(Meaning::$unimplemented)|* => return Err(UnimplementedAction.spanned2($span)),
                    _ => return Err(UnknownAction.spanned2($span)),
                }
            };
        }
    };
}

generate_action_name_meta! {
    NoAction => ResolvedNoAction | no_action,
    SetMods => ResolvedModsSet | mods_set,
    LatchMods => ResolvedModsLatch | mods_latch,
    LockMods => ResolvedModsLock | mods_lock,
    SetGroup => ResolvedGroupSet | group_set,
    LatchGroup => ResolvedGroupLatch | group_latch,
    LockGroup => ResolvedGroupLock | group_lock,
    RedirectKey | Redirect => ResolvedRedirectKey | redirect_key,
    SetControls => ResolvedSetControls | set_controls,
    LockControls => ResolvedLockControls | lock_controls,
    ;
    MovePtr,
    MovePointer,
    PtrBtn,
    PointerButton,
    LockPtrBtn,
    LockPointerBtn,
    LockPtrButton,
    LockPointerButton,
    SetPtrDflt,
    SetPointerDefault,
    Terminate,
    TerminateServer,
    SwitchScreen,
    Private,
    ISOLock,
    ActionMessage,
    MessageAction,
    Message,
    DeviceBtn,
    DevBtn,
    DevButton,
    DeviceButton,
    LockDeviceBtn,
    LockDevBtn,
    LockDevButton,
    LockDeviceButton,
    DeviceValuator,
    DevVal,
    DeviceVal,
    DevValuator,
}

fn eval_action(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    keycodes: &ResolvedKeycodes,
    vmods: &Vmodmap,
    action_defaults: &ActionDefaults,
    expr: Spanned<&Expr>,
) -> Result<Spanned<ResolvedAction>, Spanned<EvalError>> {
    let call = match expr.val {
        Expr::Call(c) => c,
        _ => return Err(UnsupportedExpressionForAction.spanned2(expr.span)),
    };
    if call.path.val.components.len() > 1 || call.path.val.components[0].index.is_some() {
        return Err(UnknownAction.spanned2(call.path.span));
    }
    let name = call.path.val.components[0].ident;
    let meaning = meaning_cache.get_case_insensitive(interner, name.val);
    macro_rules! handle {
        ($action:ident, $field:ident) => {
            ResolvedAction::$action(create_action(
                interner,
                meaning_cache,
                keycodes,
                vmods,
                &action_defaults.$field,
                &call.args,
            )?)
        };
    }
    let action = action_name_meta!(handle, meaning, call.path.span);
    Ok(action.spanned2(expr.span))
}

pub(crate) fn eval_action_default(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    keycodes: &ResolvedKeycodes,
    vmods: &Vmodmap,
    decl: &Var,
    span: Span,
    defaults: &mut ActionDefaults,
) -> Result<(), Spanned<EvalError>> {
    let meaning =
        meaning_cache.get_case_insensitive(interner, decl.path.val.components[0].ident.val);
    macro_rules! handle {
        ($_:ident, $field:ident) => {
            handle_action_global(
                interner,
                meaning_cache,
                keycodes,
                vmods,
                &mut defaults.$field,
                decl,
                span,
            )?
        };
    }
    action_name_meta!(handle, meaning, span);
    Ok(())
}

pub(crate) enum InterpField {
    Action(ResolvedAction),
    VirtualModifier(ModifierIndex),
    Repeat(bool),
    Locking(bool),
    LevelOneOnly(bool),
}

#[expect(clippy::too_many_arguments)]
pub(crate) fn eval_interp_field(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    keycodes: &ResolvedKeycodes,
    vmods: &Vmodmap,
    action_defaults: &ActionDefaults,
    var: &Var,
    span: Span,
    skip_first: bool,
) -> Result<Spanned<InterpField>, Spanned<EvalError>> {
    let mut cs = &var.path.val.components[..];
    if skip_first {
        cs = &cs[1..];
    }
    if cs.len() != 1 {
        return Err(UnknownInterpretField.spanned2(var.path.span));
    }
    let c = &cs[0];
    if c.index.is_some() {
        return Err(UnknownInterpretField.spanned2(var.path.span));
    }
    let meaning = meaning_cache.get_case_insensitive(interner, c.ident.val);
    let mut boolean = || match &var.expr {
        Some(e) => eval_boolean(interner, meaning_cache, e.as_ref()),
        _ => Ok(var.not.is_none()),
    };
    let res = match meaning {
        Meaning::Action => {
            let e = match &var.expr {
                None => return Err(MissingInterpretActionValue.spanned2(span)),
                Some(e) => e,
            };
            InterpField::Action(
                eval_action(
                    interner,
                    meaning_cache,
                    keycodes,
                    vmods,
                    action_defaults,
                    e.as_ref(),
                )?
                .val,
            )
        }
        Meaning::Virtualmodifier | Meaning::Virtualmod => {
            let e = match &var.expr {
                None => return Err(MissingInterpretVirtualmodValue.spanned2(span)),
                Some(e) => e,
            };
            let mut vmod = None;
            if let Expr::Path(e) = &e.val {
                if let Some(i) = e.unique_ident() {
                    vmod = vmods.get(i);
                }
            }
            let vmod = match vmod {
                None => return Err(UnknownInterpretVirtualModifier.spanned2(e.span)),
                Some(v) => v,
            };
            InterpField::VirtualModifier(vmod.idx)
        }
        Meaning::Usemodmapmods | Meaning::Usemodmap => {
            let e = match &var.expr {
                None => return Err(MissingInterpretUseModMapModValue.spanned2(span)),
                Some(e) => e,
            };
            let val = 'ok: {
                if let Expr::Path(e) = &e.val {
                    if let Some(i) = e.unique_ident() {
                        let meaning = meaning_cache.get_case_insensitive(interner, i);
                        match meaning {
                            Meaning::Levelone | Meaning::Level1 => break 'ok true,
                            Meaning::Anylevel | Meaning::Any => break 'ok false,
                            _ => {}
                        }
                    }
                }
                return Err(InvalidInterpretUseModMapModValue.spanned2(span));
            };
            InterpField::LevelOneOnly(val)
        }
        Meaning::Repeat => InterpField::Repeat(boolean()?),
        Meaning::Locking => InterpField::Locking(boolean()?),
        _ => return Err(UnknownInterpretField.spanned2(var.path.span)),
    };
    Ok(res.spanned2(span))
}

pub(crate) enum TypeField {
    Modifiers(Spanned<ModifierMask>),
    Map(Spanned<ModifierMask>, Spanned<Level>),
    Preserve(Spanned<ModifierMask>, Spanned<ModifierMask>),
    LevelName(Spanned<Level>, Spanned<Interned>),
}

#[expect(clippy::too_many_arguments)]
pub(crate) fn eval_type_field(
    map: &mut CodeMap,
    diagnostics: &mut DiagnosticSink<'_, '_>,
    cooker: &mut StringCooker,
    interner: &mut Interner,
    meaning_cache: &mut MeaningCache,
    mods: &Vmodmap,
    var: &Var,
    span: Span,
    skip_first: bool,
) -> Result<Spanned<TypeField>, Spanned<EvalError>> {
    let mut cs = &var.path.val.components[..];
    if skip_first {
        cs = &cs[1..];
    }
    if cs.len() != 1 {
        return Err(UnknownTypeField.spanned2(var.path.span));
    }
    let c = &cs[0];
    let meaning = meaning_cache.get_case_insensitive(interner, c.ident.val);
    let field = match meaning {
        Meaning::Modifiers => {
            if c.index.is_some() {
                return Err(UnknownTypeField.spanned2(var.path.span));
            }
            let expr = match &var.expr {
                Some(e) => e,
                _ => return Err(MissingTypeModifiersValue.spanned2(var.path.span)),
            };
            let mm = eval_mods(interner, meaning_cache, mods, expr.as_ref())?;
            TypeField::Modifiers(mm)
        }
        Meaning::Map => {
            let idx = match &c.index {
                Some(i) => i.index.as_ref(),
                _ => return Err(UnknownTypeField.spanned2(var.path.span)),
            };
            let mask = eval_mods(interner, meaning_cache, mods, idx)?;
            let expr = match &var.expr {
                Some(e) => e.as_ref(),
                _ => return Err(MissingTypeMapValue.spanned2(var.path.span)),
            };
            let level = eval_level(interner, expr)?;
            TypeField::Map(mask, level)
        }
        Meaning::Preserve => {
            let idx = match &c.index {
                Some(i) => i.index.as_ref(),
                _ => return Err(UnknownTypeField.spanned2(var.path.span)),
            };
            let mask = eval_mods(interner, meaning_cache, mods, idx)?;
            let expr = match &var.expr {
                Some(e) => e.as_ref(),
                _ => return Err(MissingTypePreserveValue.spanned2(var.path.span)),
            };
            let preserved = eval_mods(interner, meaning_cache, mods, expr)?;
            TypeField::Preserve(mask, preserved)
        }
        Meaning::LevelName | Meaning::Levelname => {
            let idx = match &c.index {
                Some(i) => i.index.as_ref(),
                _ => return Err(UnknownTypeField.spanned2(var.path.span)),
            };
            let level = eval_level(interner, idx)?;
            let expr = match &var.expr {
                Some(e) => e.as_ref(),
                _ => return Err(MissingTypeLevelNameValue.spanned2(var.path.span)),
            };
            let name = eval_string(map, diagnostics, interner, cooker, expr)?;
            TypeField::LevelName(level, name)
        }
        _ => return Err(UnknownTypeField.spanned2(var.path.span)),
    };
    Ok(field.spanned2(span))
}

pub(crate) enum IndicatorMapField {
    Modifiers(Spanned<ModifierMask>),
    Groups(Spanned<GroupMask>),
    Controls(Spanned<ControlMask>),
    WhichModifierState(Spanned<ModComponentMask>),
    WhichGroupState(Spanned<GroupComponent>),
}

pub(crate) fn eval_indicator_map_field(
    interner: &mut Interner,
    meaning_cache: &mut MeaningCache,
    mods: &Vmodmap,
    var: &Var,
    span: Span,
    skip_first: bool,
) -> Result<Spanned<IndicatorMapField>, Spanned<EvalError>> {
    let mut cs = &var.path.val.components[..];
    if skip_first {
        cs = &cs[1..];
    }
    if cs.len() != 1 || cs[0].index.is_some() {
        return Err(UnknownIndicatorField.spanned2(var.path.span));
    }
    let c = &cs[0];
    let meaning = meaning_cache.get_case_insensitive(interner, c.ident.val);
    macro_rules! get_expr {
        ($err:ident) => {
            match &var.expr {
                Some(e) => <Spanned<Expr>>::as_ref(e),
                _ => return Err($err.spanned2(span)),
            }
        };
    }
    let field = match meaning {
        Meaning::Modifiers | Meaning::Mods => {
            let value = get_expr!(MissingIndicatorModifiersValue);
            let mods = eval_mods(interner, meaning_cache, mods, value)?;
            IndicatorMapField::Modifiers(mods)
        }
        Meaning::Groups => {
            let value = get_expr!(MissingIndicatorGroupsValue);
            let groups = eval_group_mask(interner, meaning_cache, value)?;
            IndicatorMapField::Groups(groups)
        }
        Meaning::Controls | Meaning::Ctrls => {
            let value = get_expr!(MissingIndicatorControlValue);
            let controls = eval_control_mask(interner, meaning_cache, value)?;
            IndicatorMapField::Controls(controls)
        }
        Meaning::Whichmodifierstate | Meaning::Whichmodstate => {
            let value = get_expr!(MissingIndicatorWhichModStateValue);
            let components = eval_mod_component_mask(interner, meaning_cache, value)?;
            IndicatorMapField::WhichModifierState(components)
        }
        Meaning::Whichgroupstate => {
            let value = get_expr!(MissingIndicatorWhichGroupStateValue);
            let components = eval_group_component(interner, meaning_cache, value)?;
            IndicatorMapField::WhichGroupState(components)
        }
        Meaning::Allowexplicit | Meaning::Indicatordriveskbd | Meaning::Indicatordriveskeyboard => {
            return Err(UnimplementedIndicatorField.spanned2(var.path.span))
        }
        _ => return Err(UnknownIndicatorField.spanned2(var.path.span)),
    };
    Ok(field.spanned2(span))
}

pub(crate) fn eval_mod_map_field(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    keycodes: &ResolvedKeycodes,
    expr: &Expr,
    span: Span,
) -> Result<Spanned<ModMapField>, Spanned<EvalError>> {
    if let Expr::KeyName(n) = expr {
        if let Some(c) = keycodes.name_to_key.get(n) {
            let code = match c.kind {
                ResolvedKeyKind::Real(kc) => kc.val,
                ResolvedKeyKind::Alias(_, kc, _) => kc,
            };
            return Ok(ModMapField::Keycode(code).spanned2(span));
        }
        return Err(UnknownKeycode.spanned2(span));
    }
    eval_keysym(interner, meaning_cache, expr.spanned2(span))
        .span_map(|s| ModMapField::Keysym(s, None))
}

pub(crate) enum SymbolsField {
    GroupKeyType(GroupIdx, Interned),
    DefaultKeyType(Interned),
    Symbols((Option<GroupIdx>, GroupList<Keysym>)),
    Actions((Option<GroupIdx>, GroupList<ResolvedAction>)),
    Virtualmodifiers(ModifierMask),
    Repeating(Option<bool>),
    Locks(bool),
    Groupswrap,
    Groupsclamp,
    Groupsredirect(GroupIdx),
}

pub(crate) type GroupList<T> = Vec<SmallVec<[Spanned<T>; 1]>>;

fn eval_symbols_list<T>(
    group_expr: Option<Spanned<&Expr>>,
    map: &mut CodeMap,
    interner: &Interner,
    diagnostic: &mut DiagnosticSink,
    expr: Spanned<&Expr>,
    mut eval: impl FnMut(Spanned<&Expr>) -> Result<Option<Spanned<T>>, Spanned<EvalError>>,
) -> Result<(Option<GroupIdx>, GroupList<T>), Spanned<EvalError>> {
    let mut group = None;
    if let Some(e) = group_expr {
        group = Some(eval_group(interner, e)?.val);
    }
    let Expr::BracketList(b) = expr.val else {
        return Err(UnsupportedExpressionForSymbolsOrActions.spanned2(expr.span));
    };
    let mut levels = Vec::with_capacity(b.len());
    for e in b {
        let mut elements = SmallVec::new();
        let mut handle = |e: Spanned<&Expr>| match eval(e) {
            Ok(v) => v,
            Err(e) => {
                diagnostic.push(map, e.val.diagnostic_kind(), e);
                None
            }
        };
        if let Expr::BraceList(b) = &e.val {
            elements.reserve_exact(b.len());
            for e in b {
                if let Some(e) = handle(e.as_ref()) {
                    elements.push(e);
                }
            }
        } else {
            if let Some(e) = handle(e.as_ref()) {
                elements.push(e);
            }
        }
        levels.push(elements);
    }
    while let Some(last) = levels.last() {
        if last.is_empty() {
            levels.pop();
        } else {
            break;
        }
    }
    Ok((group, levels))
}

#[expect(clippy::too_many_arguments)]
pub(crate) fn eval_symbols_field(
    map: &mut CodeMap,
    diagnostics: &mut DiagnosticSink,
    interner: &mut Interner,
    cooker: &mut StringCooker,
    meaning_cache: &mut MeaningCache,
    action_defaults: &ActionDefaults,
    keycodes: &ResolvedKeycodes,
    mods: &Vmodmap,
    key_types: &ResolvedTypes,
    not: Option<Span>,
    path: Option<Spanned<&Path>>,
    expr: Option<Spanned<&Expr>>,
    span: Span,
    skip_first: bool,
) -> Result<Spanned<SymbolsField>, Spanned<EvalError>> {
    let mut c = None;
    let meaning = match path {
        Some(path) => {
            let mut cs = &path.val.components[..];
            if skip_first {
                cs = &cs[1..];
            }
            if cs.len() != 1 {
                return Err(UnknownKeyField.spanned2(path.span));
            }
            c = Some(&cs[0]);
            meaning_cache.get_case_insensitive(interner, cs[0].ident.val)
        }
        _ => {
            let mut meaning = Meaning::Symbols;
            let expr = expr.unwrap();
            if let Expr::BracketList(l) = &expr.val {
                if let Some(first) = l.first() {
                    if let Expr::BraceList(l) = &first.val {
                        if let Some(first) = l.first() {
                            if let Expr::Call(_) = &first.val {
                                meaning = Meaning::Actions;
                            }
                        }
                    } else if let Expr::Call(_) = &first.val {
                        meaning = Meaning::Actions;
                    }
                }
            }
            meaning
        }
    };
    macro_rules! try_get_idx {
        () => {
            c.and_then(|c| c.index.as_ref().map(|c| c.index.as_ref()))
        };
    }
    macro_rules! deny_idx {
        () => {
            if try_get_idx!().is_some() {
                return Err(UnknownKeyField.spanned2(span));
            }
        };
    }
    macro_rules! get_expr {
        ($err:ident) => {
            match expr {
                Some(e) => e,
                _ => return Err($err.spanned2(span)),
            }
        };
    }
    let mut boolean = || match expr {
        Some(e) => eval_boolean(interner, meaning_cache, e),
        _ => Ok(not.is_none()),
    };
    let field = match meaning {
        Meaning::Type => {
            let name = eval_string(
                map,
                diagnostics,
                interner,
                cooker,
                get_expr!(MissingKeyTypeValue),
            )?;
            if !key_types.key_types.contains_key(&name.val) {
                match meaning_cache.get_case_sensitive(interner, name.val) {
                    Meaning::ONE_LEVEL
                    | Meaning::ALPHABETIC
                    | Meaning::KEYPAD
                    | Meaning::TWO_LEVEL
                    | Meaning::FOUR_LEVEL_ALPHABETIC
                    | Meaning::FOUR_LEVEL_SEMIALPHABETIC
                    | Meaning::FOUR_LEVEL_KEYPAD
                    | Meaning::FOUR_LEVEL => {}
                    _ => return Err(UnknownKeyType.spanned2(name.span)),
                }
            }
            match try_get_idx!() {
                None => SymbolsField::DefaultKeyType(name.val),
                Some(idx) => {
                    let group = eval_group(interner, idx)?;
                    SymbolsField::GroupKeyType(group.val, name.val)
                }
            }
        }
        Meaning::Symbols => eval_symbols_list(
            try_get_idx!(),
            map,
            interner,
            diagnostics,
            get_expr!(MissingKeySymbolsValue),
            |e| {
                let ks = eval_keysym(interner, meaning_cache, e)?;
                let ks = match ks.val {
                    syms::NoSymbol => None,
                    _ => Some(ks),
                };
                Ok(ks)
            },
        )
        .map(SymbolsField::Symbols)?,
        Meaning::Actions => eval_symbols_list(
            try_get_idx!(),
            map,
            interner,
            diagnostics,
            get_expr!(MissingKeyActionsValue),
            |e| {
                let action =
                    eval_action(interner, meaning_cache, keycodes, mods, action_defaults, e)?;
                let action = match action.val {
                    ResolvedAction::ResolvedNoAction(_) => None,
                    _ => Some(action),
                };
                Ok(action)
            },
        )
        .map(SymbolsField::Actions)?,
        Meaning::Vmods | Meaning::Virtualmods | Meaning::Virtualmodifiers => {
            deny_idx!();
            eval_virtual_mods(mods, get_expr!(MissingKeyVirtualModifiersValue))
                .map(|v| SymbolsField::Virtualmodifiers(v.val))?
        }
        Meaning::Repeating | Meaning::Repeats | Meaning::Repeat => {
            deny_idx!();
            let repeating = if let Some(expr) = expr {
                let e = eval_interned(expr, UnsupportedExpressionForKeyRepeats)?;
                let meaning = meaning_cache.get_case_insensitive(interner, e.val);
                match meaning {
                    Meaning::True | Meaning::Yes | Meaning::On => Some(true),
                    Meaning::False | Meaning::No | Meaning::Off => Some(false),
                    Meaning::Default => None,
                    _ => return Err(UnknownKeyRepeatingValue.spanned2(e.span)),
                }
            } else {
                Some(boolean()?)
            };
            SymbolsField::Repeating(repeating)
        }
        Meaning::Locking | Meaning::Lock | Meaning::Locks => {
            deny_idx!();
            SymbolsField::Locks(boolean()?)
        }
        Meaning::Groupswrap | Meaning::Wrapgroups => {
            deny_idx!();
            match boolean()? {
                true => SymbolsField::Groupswrap,
                false => SymbolsField::Groupsclamp,
            }
        }
        Meaning::Groupsclamp | Meaning::Clampgroups => {
            deny_idx!();
            match boolean()? {
                true => SymbolsField::Groupsclamp,
                false => SymbolsField::Groupswrap,
            }
        }
        Meaning::Groupsredirect | Meaning::Redirectgroups => {
            deny_idx!();
            let expr = get_expr!(MissingKeyGroupsRedirectValue);
            let group = eval_group(interner, expr)?;
            SymbolsField::Groupsredirect(group.val)
        }
        _ => return Err(UnknownKeyField.spanned2(span)),
    };
    Ok(field.spanned2(span))
}
