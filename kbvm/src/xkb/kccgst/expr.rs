use {
    crate::{
        from_bytes::FromBytes,
        keysym::Keysym,
        keysyms::{KEY_NoSymbol, KEY_VoidSymbol, KEY_0},
        modifier::{ModifierIndex, ModifierMask},
        xkb::{
            code_map::CodeMap,
            controls::ControlMask,
            diagnostic::{DiagnosticKind, DiagnosticSink},
            group::{GroupChange, GroupIdx, GroupMask},
            group_component::GroupComponentMask,
            interner::{Interned, Interner},
            kccgst::ast::{CallArg, Expr, Path, Var},
            level::Level,
            meaning::{Meaning, MeaningCache},
            mod_component::ModComponentMask,
            modmap::Vmodmap,
            resolved::{
                ActionDefaults, Filter, ModMapField, Predicate, ResolvedAction,
                ResolvedActionAffect, ResolvedActionMods, ResolvedGroupLatch, ResolvedGroupLock,
                ResolvedGroupSet, ResolvedKeyKind, ResolvedKeycodes, ResolvedModsLatch,
                ResolvedModsLock, ResolvedModsSet, ResolvedNoAction, ResolvedTypes,
            },
            span::{Span, SpanExt, SpanResult1, SpanResult2, Spanned},
            string_cooker::StringCooker,
        },
    },
    isnt::std_1::primitive::IsntU8SliceExt,
    smallvec::SmallVec,
    std::ops::{BitOr, Deref, Not},
    thiserror::Error,
    EvalError::*,
};

#[derive(Copy, Clone, Debug, Error)]
pub(crate) enum EvalError {
    #[error("Overflow")]
    Overflow,
    #[error("UnknownPath")]
    UnknownPath,
    #[error("UnknownGroup")]
    UnknownGroup,
    #[error("UnknownLevel")]
    UnknownLevel,
    #[error("UnsupportedExpressionType")]
    UnsupportedExpressionType,
    #[error("UnknownBooleanValue")]
    UnknownBooleanValue,
    #[error("UnknownKeysym")]
    UnknownKeysym,
    #[error("n")]
    UnknownAction,
    #[error("UnimplementedAction")]
    UnimplementedAction,
    #[error("UnsupportedParameter")]
    UnsupportedParameter,
    #[error("MissingArgument")]
    MissingArgument,
    #[error("UnknownModifier")]
    UnknownModifier,
    #[error("UnknownAffect")]
    UnknownAffect,
    #[error("NotOneFilterArgument")]
    NotOneFilterArgument,
    #[error("NamedFilterArgParam")]
    NamedFilterArgParam,
    #[error("UnknownPredicate")]
    UnknownPredicate,
    #[error("UnknownInterpField")]
    UnknownInterpField,
    #[error("MissingActionValue")]
    MissingActionValue,
    #[error("MissingVirtualmodValue")]
    MissingVirtualmodValue,
    #[error("UnknownVirtualModifier")]
    UnknownVirtualModifier,
    #[error("UnknownTypeField")]
    UnknownTypeField,
    #[error("MissingTypeModifiersValue")]
    MissingTypeModifiersValue,
    #[error("MissingTypeMapValue")]
    MissingTypeMapValue,
    #[error("MissingTypePreserveValue")]
    MissingTypePreserveValue,
    #[error("MissingTypeLevelValue")]
    MissingTypeLevelValue,
    #[error("MissingUseModMapModValue")]
    MissingUseModMapModValue,
    #[error("InvalidUseModMapModValue")]
    InvalidUseModMapModValue,
    #[error("UnknownIndicatorMapField")]
    UnknownIndicatorMapField,
    #[error("MissingIndicatorMapModifiersValue")]
    MissingIndicatorMapModifiersValue,
    #[error("GroupOutOfBounds")]
    GroupOutOfBounds,
    #[error("LevelOutOfBounds")]
    LevelOutOfBounds,
    #[error("MissingIndicatorMapGroupsValue")]
    MissingIndicatorMapGroupsValue,
    #[error("UnknownControl")]
    UnknownControl,
    #[error("MissingIndicatorMapControlValue")]
    MissingIndicatorMapControlValue,
    #[error("UnknownModComponent")]
    UnknownModComponent,
    #[error("MissingIndicatorMapWhichmodifierValue")]
    MissingIndicatorMapWhichmodifierValue,
    #[error("UnknownGroupComponent")]
    UnknownGroupComponent,
    #[error("MissingIndicatorMapWhichgroupstateValue")]
    MissingIndicatorMapWhichgroupstateValue,
    #[error("UnknownKeycode")]
    UnknownKeycode,
    #[error("InvalidSymbolsField")]
    InvalidSymbolsField,
    #[error("MissingSymbolsGroupsredirectValue")]
    MissingSymbolsGroupsredirectValue,
    #[error("MissingSymbolsSymbolsValue")]
    MissingSymbolsSymbolsValue,
    #[error("MissingSymbolsActionsValue")]
    MissingSymbolsActionsValue,
    #[error("MissingSymbolsVirtualmodifiersValue")]
    MissingSymbolsVirtualmodifiersValue,
    #[error("MissingSymbolsRepeatingValue")]
    MissingSymbolsRepeatingValue,
    #[error("UnknownSymbolsRepeatingValue")]
    UnknownSymbolsRepeatingValue,
    #[error("MissingSymbolsTypeValue")]
    MissingSymbolsTypeValue,
    #[error("UnknownKeyType")]
    UnknownKeyType,
}

pub(crate) fn eval_group(
    interner: &Interner,
    expr: Spanned<&Expr>,
) -> Result<Spanned<GroupIdx>, Spanned<EvalError>> {
    let res = eval_u32_str_prefix(interner, expr, "group", UnknownGroup)?;
    GroupIdx::new(res.val)
        .ok_or(GroupOutOfBounds)
        .span_either(res.span)
}

pub(crate) fn eval_group_mask(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    expr: Spanned<&Expr>,
) -> Result<Spanned<GroupMask>, Spanned<EvalError>> {
    eval_u32(true, expr, |i| {
        match meaning_cache.get_case_insensitive(interner, i) {
            Meaning::All => return Ok(u32::MAX as _),
            Meaning::None => return Ok(0),
            _ => {}
        };
        let num = parse_u32_str_prefix(interner, i, "group").ok_or(UnknownGroup)?;
        Ok(1 << (num - 1))
    })
    .span_map(GroupMask)
}

pub(crate) fn eval_control_mask(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    expr: Spanned<&Expr>,
) -> Result<Spanned<ControlMask>, Spanned<EvalError>> {
    eval_keyed_mask(interner, meaning_cache, expr, &mut |meaning| {
        ControlMask::from_meaning(meaning).ok_or(UnknownControl)
    })
}

pub(crate) fn eval_mod_component_mask(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    expr: Spanned<&Expr>,
) -> Result<Spanned<ModComponentMask>, Spanned<EvalError>> {
    eval_keyed_mask(interner, meaning_cache, expr, &mut |meaning| {
        ModComponentMask::from_meaning(meaning).ok_or(UnknownModComponent)
    })
}

pub(crate) fn eval_group_component_mask(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    expr: Spanned<&Expr>,
) -> Result<Spanned<GroupComponentMask>, Spanned<EvalError>> {
    eval_keyed_mask(interner, meaning_cache, expr, &mut |meaning| {
        GroupComponentMask::from_meaning(meaning).ok_or(UnknownGroupComponent)
    })
}

pub(crate) fn eval_level(
    interner: &Interner,
    expr: Spanned<&Expr>,
) -> Result<Spanned<Level>, Spanned<EvalError>> {
    let res = eval_u32_str_prefix(interner, expr, "level", UnknownLevel)?;
    Level::new(res.val)
        .ok_or(LevelOutOfBounds)
        .span_either(res.span)
}

fn parse_u32_str_prefix(interner: &Interner, i: Interned, prefix: &str) -> Option<u32> {
    let n = interner.get(i);
    let len = prefix.len();
    if n.len() < len {
        return None;
    }
    if n[..len].not_eq_ignore_ascii_case(prefix.as_bytes()) {
        return None;
    }
    let num = &n[len..];
    let Some(num) = u32::from_bytes_dec(num) else {
        return None;
    };
    if num == 0 || num > u32::BITS {
        return None;
    };
    Some(num)
}

fn eval_u32_str_prefix(
    interner: &Interner,
    expr: Spanned<&Expr>,
    prefix: &str,
    err: EvalError,
) -> Result<Spanned<u32>, Spanned<EvalError>> {
    eval_u32(true, expr, |i| {
        parse_u32_str_prefix(interner, i, prefix)
            .map(|num| num as _)
            .ok_or(err)
    })
}

fn eval_i32_str_prefix(
    interner: &Interner,
    expr: Spanned<&Expr>,
    prefix: &str,
    err: EvalError,
) -> Result<Spanned<i32>, Spanned<EvalError>> {
    let res = eval_i64(true, expr, &mut |i| {
        parse_u32_str_prefix(interner, i, prefix)
            .map(|num| num as _)
            .ok_or(err)
    })?;
    i32::try_from(res.val)
        .map(|v| v.spanned2(res.span))
        .map_err(|_| Overflow.spanned2(expr.span))
}

pub(crate) fn eval_u32(
    allow_literal: bool,
    expr: Spanned<&Expr>,
    mut f: impl FnMut(Interned) -> Result<i64, EvalError>,
) -> Result<Spanned<u32>, Spanned<EvalError>> {
    u32::try_from(eval_i64(allow_literal, expr, &mut f)?.val)
        .map_err(|_| Overflow)
        .span_either(expr.span)
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
    let mut res = eval_i32_str_prefix(interner, expr.deref().as_ref(), "group", UnknownGroup)?;
    if neg {
        res.val = res.val.checked_neg().ok_or(Overflow.spanned2(expr.span))?;
    }
    Ok(res.map(GroupChange::Rel))
}

fn eval_i64(
    allow_literal: bool,
    expr: Spanned<&Expr>,
    f: &mut impl FnMut(Interned) -> Result<i64, EvalError>,
) -> Result<Spanned<i64>, Spanned<EvalError>> {
    macro_rules! fwd {
        ($val:expr) => {
            eval_i64(allow_literal, $val.deref().as_ref(), f)?.val
        };
    }
    macro_rules! bi {
        ($op:ident, $l:expr, $r:expr) => {
            fwd!($l).$op(fwd!($r)).ok_or(EvalError::Overflow)
        };
    }
    let res = match expr.val {
        Expr::UnMinus(v) => fwd!(v).checked_neg().ok_or(Overflow),
        Expr::UnPlus(v) => Ok(fwd!(v)),
        Expr::UnNot(v) => Ok((fwd!(v) == 0) as i64),
        Expr::UnInverse(v) => Ok(!fwd!(v)),
        Expr::Path(p) => p.unique_ident().ok_or(UnknownPath).and_then(f),
        Expr::Integer(_, i) if allow_literal => Ok(*i),
        Expr::Parenthesized(p) => Ok(fwd!(p)),
        Expr::Mul(l, r) => bi!(checked_mul, l, r),
        Expr::Div(l, r) => bi!(checked_div, l, r),
        Expr::Add(l, r) => bi!(checked_add, l, r),
        Expr::Sub(l, r) => bi!(checked_sub, l, r),
        _ => Err(UnsupportedExpressionType),
    };
    res.span_either(expr.span)
}

fn eval_keyed_mask<T>(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    expr: Spanned<&Expr>,
    f: &mut impl FnMut(Meaning) -> Result<T, EvalError>,
) -> Result<Spanned<T>, Spanned<EvalError>>
where
    T: BitOr<Output = T> + Not<Output = T>,
{
    macro_rules! fwd {
        ($val:expr) => {
            eval_keyed_mask::<T>(interner, meaning_cache, $val.deref().as_ref(), f)?.val
        };
    }
    let res = match expr.val {
        Expr::UnNot(v) | Expr::UnInverse(v) => Ok(!fwd!(v)),
        Expr::Path(p) => p.unique_ident().ok_or(UnknownPath).and_then(|i| {
            let meaning = meaning_cache.get_case_insensitive(interner, i);
            f(meaning)
        }),
        Expr::Parenthesized(p) => Ok(fwd!(p)),
        Expr::Add(l, r) => Ok(fwd!(l) | fwd!(r)),
        Expr::Sub(l, r) => Ok(fwd!(l) | !fwd!(r)),
        _ => Err(UnsupportedExpressionType),
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
        _ => Err(UnsupportedExpressionType.spanned2(expr.span)),
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
        _ => Err(UnsupportedExpressionType.spanned2(expr.span)),
    }
}

pub(crate) fn keysym_from_ident(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    ident: Spanned<Interned>,
) -> Result<Spanned<Keysym>, Spanned<EvalError>> {
    let meaning = meaning_cache.get_case_insensitive(interner, ident.val);
    let sym = match meaning {
        Meaning::Any | Meaning::Nosymbol => KEY_NoSymbol,
        Meaning::None | Meaning::Voidsymbol => KEY_VoidSymbol,
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
            _ => Err(UnsupportedExpressionType),
        },
        Expr::Integer(_, i) => match *i {
            0..=9 => Ok(Keysym(KEY_0.0 + *i as u32)),
            _ => u32::try_from(*i).map(Keysym).map_err(|_| Overflow),
        },
        _ => Err(UnsupportedExpressionType),
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
                v.val.0 = !v.val.0;
                v.val.spanned2(expr.span)
            })
        }
        Expr::Path(p) => p
            .unique_ident()
            .and_then(name_to_mod)
            .ok_or(UnknownModifier),
        Expr::Integer(_, v) => u32::try_from(*v).map_err(|_| Overflow),
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
        _ => Err(UnsupportedExpressionType),
    };
    res.map(ModifierMask).span_either(expr.span)
}

pub(crate) fn eval_interned(expr: Spanned<&Expr>) -> Result<Spanned<Interned>, Spanned<EvalError>> {
    if let Expr::Path(p) = expr.val {
        if let Some(u) = p.unique_ident() {
            return Ok(u.spanned2(expr.span));
        }
    }
    Err(UnsupportedExpressionType.spanned2(expr.span))
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
            return Err(UnknownPredicate.spanned2(e.path.span));
        }
        let meaning = meaning_cache.get_case_insensitive(interner, c.ident.val);
        predicate = match meaning {
            Meaning::NoneOf => Predicate::NoneOf,
            Meaning::AnyOfOrNone => Predicate::AnyOfOrNone,
            Meaning::AnyOf => Predicate::AnyOf,
            Meaning::AllOf => Predicate::AllOf,
            Meaning::Exactly => Predicate::Exactly,
            _ => return Err(UnknownPredicate.spanned2(e.path.span)),
        };
        if e.args.len() != 1 {
            return Err(NotOneFilterArgument.spanned2(expr.span));
        }
        let c = &e.args[0];
        expr = match &c.val {
            CallArg::Expr(e) => e.spanned2(c.span),
            CallArg::NamedParam(e) => {
                return Err(NamedFilterArgParam.spanned2(e.name.span));
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
    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
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
    ($value:expr) => {
        match $value.val {
            ActionParameterValue::Boolean(_) => {
                return Err(EvalError::MissingArgument.spanned2($value.span));
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
                _ => return Err(UnknownAffect.spanned2(expr.span)),
            };
            return Ok(ResolvedActionAffect { lock, unlock });
        }
    }
    Err(UnsupportedExpressionType.spanned2(expr.span))
}

impl ActionParameters for ResolvedNoAction {
    fn handle_field(
        &mut self,
        _interner: &Interner,
        _meaning_cache: &mut MeaningCache,
        _vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        _value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), Spanned<EvalError>> {
        Err(EvalError::UnsupportedParameter.spanned2(meaning.span))
    }
}

impl ActionParameters for ResolvedModsSet {
    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
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
                    value!(value),
                )?);
            }
            _ => {
                return Err(EvalError::UnsupportedParameter.spanned2(meaning.span));
            }
        }
        Ok(())
    }
}

impl ActionParameters for ResolvedModsLatch {
    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
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
                    value!(value),
                )?);
            }
            _ => {
                return Err(EvalError::UnsupportedParameter.spanned2(meaning.span));
            }
        }
        Ok(())
    }
}

impl ActionParameters for ResolvedModsLock {
    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
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
                    value!(value),
                )?);
            }
            Meaning::Affect => {
                self.affect = Some(
                    eval_action_affect(interner, meaning_cache, value!(value))?
                        .spanned2(value.span),
                );
            }
            _ => {
                return Err(UnsupportedParameter.spanned2(meaning.span));
            }
        }
        Ok(())
    }
}

impl ActionParameters for ResolvedGroupSet {
    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
        _vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), Spanned<EvalError>> {
        match meaning.val {
            Meaning::Group => {
                self.group = Some(eval_group_change(interner, value!(value))?);
            }
            Meaning::ClearLocks => {
                self.clear_locks = Some(boolean!(interner, meaning_cache, value)?)
            }
            _ => {
                return Err(UnsupportedParameter.spanned2(meaning.span));
            }
        }
        Ok(())
    }
}

impl ActionParameters for ResolvedGroupLatch {
    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
        _vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), Spanned<EvalError>> {
        match meaning.val {
            Meaning::Group => {
                self.group = Some(eval_group_change(interner, value!(value))?);
            }
            Meaning::ClearLocks => {
                self.clear_locks = Some(boolean!(interner, meaning_cache, value)?)
            }
            Meaning::LatchToLock => {
                self.latch_to_lock = Some(boolean!(interner, meaning_cache, value)?)
            }
            _ => {
                return Err(EvalError::UnsupportedParameter.spanned2(meaning.span));
            }
        }
        Ok(())
    }
}

impl ActionParameters for ResolvedGroupLock {
    fn handle_field(
        &mut self,
        interner: &Interner,
        _meaning_cache: &mut MeaningCache,
        _vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), Spanned<EvalError>> {
        match meaning.val {
            Meaning::Group => {
                self.group = Some(eval_group_change(interner, value!(value))?);
            }
            _ => {
                return Err(EvalError::UnsupportedParameter.spanned2(meaning.span));
            }
        }
        Ok(())
    }
}

fn create_action<T: ActionParameters>(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
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
                    _ => return Err(UnsupportedParameter.spanned2(arg.span)),
                },
                _ => return Err(UnsupportedParameter.spanned2(arg.span)),
            },
            CallArg::NamedParam(n) => (
                (&n.name.val).spanned2(n.name.span),
                ActionParameterValue::Expr(&n.expr.val).spanned2(n.expr.span),
            ),
        };
        let ident = path
            .val
            .unique_ident()
            .ok_or(EvalError::UnsupportedParameter.spanned2(path.span))?;
        let meaning = meaning_cache.get_case_insensitive(interner, ident);
        t.handle_field(
            interner,
            meaning_cache,
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
    vmods: &Vmodmap,
    t: &mut T,
    var: &Var,
    span: Span,
) -> Result<(), Spanned<EvalError>> {
    let cs = &var.path.val.components;
    if cs.len() != 2 || cs[1].index.is_some() {
        return Err(EvalError::UnsupportedParameter.spanned2(span));
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
        vmods,
        meaning.spanned2(var.path.span),
        value,
    )
}

macro_rules! generate_action_name_meta {
    ($($meaning:ident => $big:ident | $little:ident,)* ; $($unimplemented:ident,)*) => {
        macro_rules! action_name_meta {
            ($macro_:ident, $meaning_inner:expr, $span:expr) => {
                match $meaning_inner {
                    $(Meaning::$meaning => $macro_!($big, $little),)*
                    $(Meaning::$unimplemented)|* => return Err(EvalError::UnimplementedAction.spanned2($span)),
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
    SetControls,
    LockControls,
    Private,
    RedirectKey,
    Redirect,
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
    vmods: &Vmodmap,
    action_defaults: &ActionDefaults,
    expr: Spanned<&Expr>,
) -> Result<Spanned<ResolvedAction>, Spanned<EvalError>> {
    let call = match expr.val {
        Expr::Call(c) => c,
        _ => return Err(UnsupportedExpressionType.spanned2(expr.span)),
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

pub(crate) fn eval_interp_field(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
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
        return Err(UnknownInterpField.spanned2(var.path.span));
    }
    let c = &cs[0];
    if c.index.is_some() {
        return Err(UnknownInterpField.spanned2(var.path.span));
    }
    let meaning = meaning_cache.get_case_insensitive(interner, c.ident.val);
    let mut boolean = || match &var.expr {
        Some(e) => eval_boolean(interner, meaning_cache, e.as_ref()),
        _ => Ok(var.not.is_none()),
    };
    let res = match meaning {
        Meaning::Action => {
            let e = match &var.expr {
                None => return Err(MissingActionValue.spanned2(span)),
                Some(e) => e,
            };
            InterpField::Action(
                eval_action(interner, meaning_cache, vmods, action_defaults, e.as_ref())?.val,
            )
        }
        Meaning::Virtualmodifier | Meaning::Virtualmod => {
            let e = match &var.expr {
                None => return Err(MissingVirtualmodValue.spanned2(span)),
                Some(e) => e,
            };
            let mut vmod = None;
            if let Expr::Path(e) = &e.val {
                if let Some(i) = e.unique_ident() {
                    vmod = vmods.get(i);
                }
            }
            let vmod = match vmod {
                None => return Err(UnknownVirtualModifier.spanned2(e.span)),
                Some(v) => v,
            };
            InterpField::VirtualModifier(vmod.idx)
        }
        Meaning::Usemodmapmods | Meaning::Usemodmap => {
            let e = match &var.expr {
                None => return Err(MissingUseModMapModValue.spanned2(span)),
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
                return Err(InvalidUseModMapModValue.spanned2(span));
            };
            InterpField::LevelOneOnly(val)
        }
        Meaning::Repeat => InterpField::Repeat(boolean()?),
        Meaning::Locking => InterpField::Locking(boolean()?),
        _ => return Err(UnknownInterpField.spanned2(var.path.span)),
    };
    Ok(res.spanned2(span))
}

pub(crate) enum TypeField {
    Modifiers(Spanned<ModifierMask>),
    Map(Spanned<ModifierMask>, Spanned<Level>),
    Preserve(Spanned<ModifierMask>, Spanned<ModifierMask>),
    LevelName(Spanned<Level>, Spanned<Interned>),
}

pub(crate) fn eval_type_field(
    map: &mut CodeMap,
    diagnostics: &mut DiagnosticSink<'_>,
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
                _ => return Err(MissingTypeLevelValue.spanned2(var.path.span)),
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
    Whichmodifierstate(Spanned<ModComponentMask>),
    Whichgroupstate(Spanned<GroupComponentMask>),
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
        return Err(UnknownIndicatorMapField.spanned2(var.path.span));
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
            let value = get_expr!(MissingIndicatorMapModifiersValue);
            let mods = eval_mods(interner, meaning_cache, mods, value)?;
            IndicatorMapField::Modifiers(mods)
        }
        Meaning::Groups => {
            let value = get_expr!(MissingIndicatorMapGroupsValue);
            let groups = eval_group_mask(interner, meaning_cache, value)?;
            IndicatorMapField::Groups(groups)
        }
        Meaning::Controls | Meaning::Ctrls => {
            let value = get_expr!(MissingIndicatorMapControlValue);
            let controls = eval_control_mask(interner, meaning_cache, value)?;
            IndicatorMapField::Controls(controls)
        }
        Meaning::Whichmodifierstate | Meaning::Whichmodstate => {
            let value = get_expr!(MissingIndicatorMapWhichmodifierValue);
            let components = eval_mod_component_mask(interner, meaning_cache, value)?;
            IndicatorMapField::Whichmodifierstate(components)
        }
        Meaning::Whichgroupstate => {
            let value = get_expr!(MissingIndicatorMapWhichgroupstateValue);
            let components = eval_group_component_mask(interner, meaning_cache, value)?;
            IndicatorMapField::Whichgroupstate(components)
        }
        _ => return Err(UnknownIndicatorMapField.spanned2(var.path.span)),
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
    diagnostic_kind: DiagnosticKind,
    expr: Spanned<&Expr>,
    mut eval: impl FnMut(Spanned<&Expr>) -> Result<Option<Spanned<T>>, Spanned<EvalError>>,
) -> Result<(Option<GroupIdx>, GroupList<T>), Spanned<EvalError>> {
    let mut group = None;
    if let Some(e) = group_expr {
        group = Some(eval_group(interner, e)?.val);
    }
    let Expr::BracketList(b) = expr.val else {
        return Err(UnsupportedExpressionType.spanned2(expr.span));
    };
    let mut levels = Vec::with_capacity(b.len());
    for e in b {
        let mut elements = SmallVec::new();
        let mut handle = |e: Spanned<&Expr>| match eval(e) {
            Ok(v) => v,
            Err(e) => {
                diagnostic.push(map, diagnostic_kind, e);
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

pub(crate) fn eval_symbols_field(
    map: &mut CodeMap,
    diagnostics: &mut DiagnosticSink,
    interner: &mut Interner,
    cooker: &mut StringCooker,
    meaning_cache: &mut MeaningCache,
    action_defaults: &ActionDefaults,
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
                return Err(InvalidSymbolsField.spanned2(path.span));
            }
            c = Some(&cs[0]);
            meaning_cache.get_case_insensitive(interner, cs[0].ident.val)
        }
        _ => {
            let mut meaning = Meaning::Symbols;
            let expr = expr.unwrap();
            if let Expr::BracketList(l) = &expr.val {
                if let Some(first) = l.get(0) {
                    if let Expr::BraceList(l) = &first.val {
                        if let Some(first) = l.get(0) {
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
                return Err(InvalidSymbolsField.spanned2(span));
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
                get_expr!(MissingSymbolsTypeValue),
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
            DiagnosticKind::InvalidKeysym,
            get_expr!(MissingSymbolsSymbolsValue),
            |e| {
                let ks = eval_keysym(interner, meaning_cache, e)?;
                let ks = match ks.val {
                    #[allow(non_upper_case_globals)]
                    KEY_NoSymbol => None,
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
            DiagnosticKind::InvalidAction,
            get_expr!(MissingSymbolsActionsValue),
            |e| {
                let action = eval_action(interner, meaning_cache, mods, action_defaults, e)?;
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
            eval_virtual_mods(mods, get_expr!(MissingSymbolsVirtualmodifiersValue))
                .map(|v| SymbolsField::Virtualmodifiers(v.val))?
        }
        Meaning::Repeating | Meaning::Repeats | Meaning::Repeat => {
            deny_idx!();
            let e = eval_interned(get_expr!(MissingSymbolsRepeatingValue))?;
            let meaning = meaning_cache.get_case_insensitive(interner, e.val);
            let repeating = match meaning {
                Meaning::True | Meaning::Yes | Meaning::On => Some(true),
                Meaning::False | Meaning::No | Meaning::Off => Some(false),
                Meaning::Default => None,
                _ => return Err(UnknownSymbolsRepeatingValue.spanned2(e.span)),
            };
            SymbolsField::Repeating(repeating)
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
            let expr = get_expr!(MissingSymbolsGroupsredirectValue);
            let group = eval_group(interner, expr)?;
            SymbolsField::Groupsredirect(group.val)
        }
        _ => return Err(InvalidSymbolsField.spanned2(span)),
    };
    Ok(field.spanned2(span))
}
