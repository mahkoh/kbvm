use {
    crate::{
        keysym::Keysym,
        keysyms::KEY_0,
        modifier::ModifierMask,
        xkb::{
            interner::{Interned, Interner},
            kccgst::{
                action::{
                    Action, ActionAffect, ActionMods, GroupLatch, GroupLock, GroupSet, ModsLatch,
                    ModsLock, ModsSet,
                },
                ast::{CallArg, Expr},
                expr::EvalErrorType::{Overflow, UnknownGroup, UnknownLevel},
                meaning::{Meaning, MeaningCache},
                vmodmap::Vmodmap,
            },
            span::{Span, SpanExt, Spanned},
        },
    },
    bstr::ByteSlice,
    isnt::std_1::primitive::IsntU8SliceExt,
    std::ops::Deref,
};

pub(crate) struct EvalError {
    span: Span,
    ty: EvalErrorType,
}

#[derive(Copy, Clone, Debug)]
pub(crate) enum EvalErrorType {
    Overflow,
    UnknownPath,
    UnknownGroup,
    UnknownLevel,
    UnsupportedExpressionType,
    UnknownBooleanValue,
    UnknownKeysym,
    UnknownAction,
    UnimplementedAction,
    UnsupportedParameter,
    MissingArgument,
    UnknownModifier,
    UnknownAffect,
}

pub fn eval_group(interner: &Interner, expr: Spanned<&Expr>) -> Result<u32, EvalError> {
    eval_str_prefix(interner, expr, "group", UnknownGroup)
}

pub fn eval_level(interner: &Interner, expr: Spanned<&Expr>) -> Result<u32, EvalError> {
    eval_str_prefix(interner, expr, "level", UnknownLevel)
}

fn eval_str_prefix(
    interner: &Interner,
    expr: Spanned<&Expr>,
    prefix: &str,
    err: EvalErrorType,
) -> Result<u32, EvalError> {
    eval_u32(expr, |i| {
        let n = interner.get(i);
        let len = prefix.len();
        if n.len() < len {
            return Err(err);
        }
        if n[..len].not_eq_ignore_ascii_case(prefix.as_bytes()) {
            return Err(err);
        }
        let num = &n[len..];
        let Ok(num) = num.to_str() else {
            return Err(err);
        };
        num.parse::<i64>().map_err(|_| err)
    })
}

pub fn eval_u32(
    expr: Spanned<&Expr>,
    mut f: impl FnMut(Interned) -> Result<i64, EvalErrorType>,
) -> Result<u32, EvalError> {
    match u32::try_from(eval_i64_(expr, &mut f)?) {
        Ok(v) => Ok(v),
        Err(_) => Err(EvalError {
            span: expr.span,
            ty: Overflow,
        }),
    }
}

pub enum Sign {
    Plus,
    Minus,
}

pub fn eval_rel_group(
    interner: &Interner,
    expr: Spanned<&Expr>,
) -> Result<(u32, Option<Sign>), EvalError> {
    let (expr, sign) = match &expr.val {
        Expr::UnPlus(e) => (e.deref().as_ref(), Some(Sign::Plus)),
        Expr::UnMinus(e) => (e.deref().as_ref(), Some(Sign::Minus)),
        _ => (expr, None),
    };
    let v = eval_group(interner, expr)?;
    Ok((v, sign))
}

fn eval_i64_(
    expr: Spanned<&Expr>,
    f: &mut impl FnMut(Interned) -> Result<i64, EvalErrorType>,
) -> Result<i64, EvalError> {
    macro_rules! bi {
        ($op:ident, $l:expr, $r:expr) => {
            eval_i64_($l.deref().as_ref(), f)?
                .$op(eval_i64_($r.deref().as_ref(), f)?)
                .ok_or(EvalErrorType::Overflow)
        };
    }
    let res = match expr.val {
        Expr::UnMinus(v) => eval_i64_(v.deref().as_ref(), f)?
            .checked_neg()
            .ok_or(Overflow),
        Expr::UnPlus(v) => return eval_i64_(v.deref().as_ref(), f),
        Expr::UnNot(v) => return Ok((eval_i64_(v.deref().as_ref(), f)? == 0) as i64),
        Expr::UnInverse(v) => return Ok(!eval_i64_(v.deref().as_ref(), f)?),
        Expr::Path(p) => p
            .unique_ident()
            .ok_or(EvalErrorType::UnknownPath)
            .and_then(f),
        Expr::Integer(_, i) => return Ok(*i),
        Expr::Parenthesized(p) => return eval_i64_(p.deref().as_ref(), f),
        Expr::Mul(l, r) => bi!(checked_mul, l, r),
        Expr::Div(l, r) => bi!(checked_div, l, r),
        Expr::Add(l, r) => bi!(checked_add, l, r),
        Expr::Sub(l, r) => bi!(checked_sub, l, r),
        _ => Err(EvalErrorType::UnsupportedExpressionType),
    };
    res.map_err(|ty| EvalError {
        span: expr.span,
        ty,
    })
}

fn eval_boolean(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    expr: Spanned<&Expr>,
) -> Result<bool, EvalError> {
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
            .ok_or(EvalError {
                span: expr.span,
                ty: EvalErrorType::UnknownBooleanValue,
            }),
        _ => Err(EvalError {
            span: expr.span,
            ty: EvalErrorType::UnsupportedExpressionType,
        }),
    }
}

fn eval_keysym(interner: &Interner, expr: &Spanned<Expr>) -> Result<Keysym, EvalError> {
    let res = match &expr.val {
        Expr::Path(p) => p
            .unique_ident()
            .ok_or(EvalErrorType::UnsupportedExpressionType)
            .and_then(|id| Keysym::from_str(interner.get(id)).ok_or(EvalErrorType::UnknownKeysym)),
        Expr::Integer(_, i) => match *i {
            0..=9 => Ok(Keysym(KEY_0.0 + *i as u32)),
            _ => u32::try_from(*i).map(Keysym).map_err(|_| Overflow),
        },
        _ => Err(EvalErrorType::UnsupportedExpressionType),
    };
    res.map_err(|ty| EvalError {
        span: expr.span,
        ty,
    })
}

fn ident_to_real_mod_mask(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    ident: Interned,
) -> Option<u32> {
    let meaning = meaning_cache.get_case_insensitive(interner, ident);
    let v = match meaning {
        Meaning::None => 0x00,
        Meaning::Shift => 0x01,
        Meaning::Lock => 0x02,
        Meaning::Control => 0x04,
        Meaning::Mod1 => 0x08,
        Meaning::Mod2 => 0x10,
        Meaning::Mod3 => 0x20,
        Meaning::Mod4 => 0x40,
        Meaning::Mod5 => 0x80,
        Meaning::All => 0xff,
        _ => return None,
    };
    Some(v)
}

pub fn eval_real_mods(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    expr: Spanned<&Expr>,
) -> Result<ModifierMask, EvalError> {
    eval_mods_(expr, &mut |ident| {
        ident_to_real_mod_mask(interner, meaning_cache, ident)
    })
}

fn eval_mods(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    vmods: &Vmodmap,
    expr: Spanned<&Expr>,
) -> Result<ModifierMask, EvalError> {
    eval_mods_(expr, &mut |ident| {
        ident_to_real_mod_mask(interner, meaning_cache, ident)
            .or_else(|| vmods.get(ident).map(|m| m.idx.to_mask().0))
    })
}

fn eval_mods_(
    expr: Spanned<&Expr>,
    name_to_mod: &mut impl FnMut(Interned) -> Option<u32>,
) -> Result<ModifierMask, EvalError> {
    let res = match &expr.val {
        Expr::UnInverse(v) => {
            return eval_mods_(v.deref().as_ref(), name_to_mod).map(|mut v| {
                v.0 = !v.0;
                v
            })
        }
        Expr::Path(p) => p
            .unique_ident()
            .and_then(name_to_mod)
            .ok_or(EvalErrorType::UnknownModifier),
        Expr::Integer(_, v) => u32::try_from(*v).map_err(|_| Overflow),
        Expr::Parenthesized(v) => return eval_mods_(v.deref().as_ref(), name_to_mod),
        Expr::Add(l, r) => {
            let l = eval_mods_(l.deref().as_ref(), name_to_mod)?;
            let r = eval_mods_(r.deref().as_ref(), name_to_mod)?;
            Ok(l.0 | r.0)
        }
        Expr::Sub(l, r) => {
            let l = eval_mods_(l.deref().as_ref(), name_to_mod)?;
            let r = eval_mods_(r.deref().as_ref(), name_to_mod)?;
            Ok(l.0 & !r.0)
        }
        _ => Err(EvalErrorType::UnsupportedExpressionType),
    };
    res.map(ModifierMask).map_err(|ty| EvalError {
        span: expr.span,
        ty,
    })
}

enum ActionParameterValue<'a> {
    Boolean(bool),
    Expr(&'a Expr),
}

trait ActionParameters: Default {
    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
        vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), EvalError>;
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
        match &$value.val {
            ActionParameterValue::Boolean(_) => {
                return Err(EvalError {
                    span: $value.span,
                    ty: EvalErrorType::MissingArgument,
                });
            }
            ActionParameterValue::Expr(e) => Spanned {
                span: $value.span,
                val: e,
            },
        }
    };
}

fn eval_action_mods(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    vmods: &Vmodmap,
    expr: Spanned<&Expr>,
) -> Result<ActionMods, EvalError> {
    if let Expr::Path(p) = &expr.val {
        if let Some(id) = p.unique_ident() {
            let meaning = meaning_cache.get_case_insensitive(interner, id);
            if matches!(meaning, Meaning::Usemodmapmods | Meaning::Modmapmods) {
                return Ok(ActionMods::ModMap);
            }
        }
    }
    let mods = eval_mods(interner, meaning_cache, vmods, expr)?;
    Ok(ActionMods::Explicit(mods))
}

fn eval_action_affect(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    expr: Spanned<&Expr>,
) -> Result<ActionAffect, EvalError> {
    if let Expr::Path(p) = &expr.val {
        if let Some(id) = p.unique_ident() {
            let meaning = meaning_cache.get_case_insensitive(interner, id);
            let (lock, unlock) = match meaning {
                Meaning::Lock => (true, false),
                Meaning::Unlock => (false, true),
                Meaning::Both => (true, true),
                Meaning::Neither => (false, false),
                _ => {
                    return Err(EvalError {
                        span: expr.span,
                        ty: EvalErrorType::UnknownAffect,
                    })
                }
            };
            return Ok(ActionAffect { lock, unlock });
        }
    }
    Err(EvalError {
        span: expr.span,
        ty: EvalErrorType::UnsupportedExpressionType,
    })
}

impl ActionParameters for ModsSet {
    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
        vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), EvalError> {
        match meaning.val {
            Meaning::ClearLocks => {
                self.clear_locks = Some(boolean!(interner, meaning_cache, value)?)
            }
            Meaning::Mods => {
                self.modifiers = Some(
                    eval_action_mods(interner, meaning_cache, vmods, value!(value))?
                        .spanned2(value.span),
                );
            }
            _ => {
                return Err(EvalError {
                    span: meaning.span,
                    ty: EvalErrorType::UnsupportedParameter,
                });
            }
        }
        Ok(())
    }
}

impl ActionParameters for ModsLatch {
    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
        vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), EvalError> {
        match meaning.val {
            Meaning::ClearLocks => {
                self.clear_locks = Some(boolean!(interner, meaning_cache, value)?)
            }
            Meaning::LatchToLock => {
                self.latch_to_lock = Some(boolean!(interner, meaning_cache, value)?)
            }
            Meaning::Mods => {
                self.modifiers = Some(
                    eval_action_mods(interner, meaning_cache, vmods, value!(value))?
                        .spanned2(value.span),
                );
            }
            _ => {
                return Err(EvalError {
                    span: meaning.span,
                    ty: EvalErrorType::UnsupportedParameter,
                });
            }
        }
        Ok(())
    }
}

impl ActionParameters for ModsLock {
    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
        vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), EvalError> {
        match meaning.val {
            Meaning::Mods => {
                self.modifiers = Some(
                    eval_action_mods(interner, meaning_cache, vmods, value!(value))?
                        .spanned2(value.span),
                );
            }
            Meaning::Affect => {
                self.affect = Some(
                    eval_action_affect(interner, meaning_cache, value!(value))?
                        .spanned2(value.span),
                );
            }
            _ => {
                return Err(EvalError {
                    span: meaning.span,
                    ty: EvalErrorType::UnsupportedParameter,
                });
            }
        }
        Ok(())
    }
}

impl ActionParameters for GroupSet {
    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
        _vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), EvalError> {
        match meaning.val {
            Meaning::Group => {
                self.group = Some(eval_rel_group(interner, value!(value))?.spanned2(value.span));
            }
            Meaning::ClearLocks => {
                self.clear_locks = Some(boolean!(interner, meaning_cache, value)?)
            }
            _ => {
                return Err(EvalError {
                    span: meaning.span,
                    ty: EvalErrorType::UnsupportedParameter,
                });
            }
        }
        Ok(())
    }
}

impl ActionParameters for GroupLatch {
    fn handle_field(
        &mut self,
        interner: &Interner,
        meaning_cache: &mut MeaningCache,
        _vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), EvalError> {
        match meaning.val {
            Meaning::Group => {
                self.group = Some(eval_rel_group(interner, value!(value))?.spanned2(value.span));
            }
            Meaning::ClearLocks => {
                self.clear_locks = Some(boolean!(interner, meaning_cache, value)?)
            }
            Meaning::LatchToLock => {
                self.latch_to_lock = Some(boolean!(interner, meaning_cache, value)?)
            }
            _ => {
                return Err(EvalError {
                    span: meaning.span,
                    ty: EvalErrorType::UnsupportedParameter,
                });
            }
        }
        Ok(())
    }
}

impl ActionParameters for GroupLock {
    fn handle_field(
        &mut self,
        interner: &Interner,
        _meaning_cache: &mut MeaningCache,
        _vmods: &Vmodmap,
        meaning: Spanned<Meaning>,
        value: Spanned<ActionParameterValue<'_>>,
    ) -> Result<(), EvalError> {
        match meaning.val {
            Meaning::Group => {
                self.group = Some(eval_rel_group(interner, value!(value))?.spanned2(value.span));
            }
            _ => {
                return Err(EvalError {
                    span: meaning.span,
                    ty: EvalErrorType::UnsupportedParameter,
                });
            }
        }
        Ok(())
    }
}

fn create_action<T: ActionParameters>(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    vmods: &Vmodmap,
    args: &[Spanned<CallArg>],
) -> Result<T, EvalError> {
    let mut t = T::default();
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
                    _ => {
                        return Err(EvalError {
                            span: arg.span,
                            ty: EvalErrorType::UnsupportedParameter,
                        })
                    }
                },
                _ => {
                    return Err(EvalError {
                        span: arg.span,
                        ty: EvalErrorType::UnsupportedParameter,
                    })
                }
            },
            CallArg::NamedParam(n) => (
                (&n.name.val).spanned2(n.name.span),
                ActionParameterValue::Expr(&n.expr.val).spanned2(n.expr.span),
            ),
        };
        let ident = path.val.unique_ident().ok_or(EvalError {
            span: path.span,
            ty: EvalErrorType::UnsupportedParameter,
        })?;
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

fn eval_action(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    vmods: &Vmodmap,
    expr: &Spanned<Expr>,
) -> Result<Action, EvalError> {
    let call = match &expr.val {
        Expr::Call(c) => c,
        _ => {
            return Err(EvalError {
                span: expr.span,
                ty: EvalErrorType::UnsupportedExpressionType,
            })
        }
    };
    if call.path.val.components.len() > 1 || call.path.val.components[0].index.is_some() {
        return Err(EvalError {
            span: call.path.span,
            ty: EvalErrorType::UnknownAction,
        });
    }
    let name = call.path.val.components[0].ident;
    let meaning = meaning_cache.get_case_insensitive(interner, name.val);
    macro_rules! handle {
        (
            $($meaning:ident => $action:ident,)* ; $($unimplemented:ident,)*
        ) => {
            match meaning {
                $(Meaning::$meaning => Action::$action(create_action(interner, meaning_cache, vmods, &call.args)?),)*
                $(Meaning::$unimplemented)|* => return Err(EvalError {
                    span: call.path.span,
                    ty: EvalErrorType::UnimplementedAction,
                }),
                _ => return Err(EvalError {
                    span: call.path.span,
                    ty: EvalErrorType::UnknownAction,
                }),
            }
        }
    }
    let action = handle! {
        SetMods => ModsSet,
        LatchMods => ModsLatch,
        LockMods => ModsLock,
        SetGroup => GroupSet,
        LatchGroup => GroupLatch,
        LockGroup => GroupLock,
        ;
        NoAction,
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
    };
    Ok(action)
}
