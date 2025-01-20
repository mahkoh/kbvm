//! Custom logic to modify the behavior of keys.
//!
//! Routines are run as part of the [`StateMachine`]'s handling of key events. They put
//! the VM in KBVM.
//!
//! Routines consist of two parts:
//!
//! 1. A part that is run when the key is pressed.
//! 2. A part that is run when the key is release.
//!
//! A routine can use [variables](Var) to store state. Variables are shared between the
//! two parts of the routine.
//!
//! If routines want to communicate with each other or if they want to preserve state
//! across key-press/release events, they can use [`Global`]s.
//!
//! Variables and globals represent `u32`s. All of the usual arithmetic operations can
//! be performed on variables and these operations use wrapping semantics.
//!
//! Routines are primarily useful to emit key-press/release events and to modify the
//! [`Components`] of the state.
//!
//! If a key does not have a routine assigned, the following default routine is used.
//!
//! ```
//! # use kbvm::Keycode;
//! # use kbvm::routine::Routine;
//! #
//! fn default_routine(keycode: Keycode) -> Routine {
//!     let mut builder = Routine::builder();
//!     let [kc, zero] = builder.allocate_vars();
//!     builder
//!         .load_lit(kc, keycode.raw())
//!         .key_down(kc)
//!         .load_lit(zero, 0)
//!         .mods_latched_store(zero)
//!         .group_latched_store(zero)
//!         .on_release() // everything below this point is executed when the key is released
//!         .key_up(kc);
//!     builder.build()
//! }
//! ```
//!
//! # Conditional execution
//!
//! Routines support conditional execution by skipping forward within the routine.
//!
//! ```
//! # use kbvm::routine::Routine;
//! let mut builder = Routine::builder();
//! let [zero] = builder.allocate_vars();
//! builder.load_lit(zero, 0);
//! let anchor = builder.prepare_skip();
//! builder
//!     .mods_pressed_store(zero)
//!     .finish_skip(anchor);
//! ```
//!
//! This example unconditionally skips over the `mods_pressed = zero` instruction. Most of
//! the time, you want to make use of [`RoutineBuilder::prepare_skip_if`] and
//! [`RoutineBuilder::prepare_skip_if_not`] to make the skip conditional.
//!
//! # Manipulating the components
//!
//! The following functions can be used to access the [`Components`]:
//!
//! - [`RoutineBuilder::mods_pressed_load`]
//! - [`RoutineBuilder::mods_latched_load`]
//! - [`RoutineBuilder::mods_locked_load`]
//! - [`RoutineBuilder::group_pressed_load`]
//! - [`RoutineBuilder::group_latched_load`]
//! - [`RoutineBuilder::group_locked_load`]
//!
//! To manipulate the components, you will likely want to use the following functions:
//!
//! - [`RoutineBuilder::mods_pressed_inc`]
//! - [`RoutineBuilder::mods_pressed_dec`]
//! - [`RoutineBuilder::mods_latched_store`]
//! - [`RoutineBuilder::mods_locked_store`]
//! - [`RoutineBuilder::group_pressed_store`]
//! - [`RoutineBuilder::group_latched_store`]
//! - [`RoutineBuilder::group_locked_store`]
//!
//! There is also a function, `mods_pressed_store`, to set the pressed modifiers directly
//! but this is almost never what you want.
//!
//! For example, if both the left and the right shift key are pressed, you don't want the
//! shift modifier to be unset when only one of the keys is released. Instead, the
//! `mods_pressed_inc` and `mods_pressed_dec` functions implement per-modifier reference
//! counting that handles this situation.
//!
//! There are still usecases for `mods_pressed_store`, for example, when you want to change
//! the pressed modifiers before emitting a `key_down` or `key_up` event. This is fine as
//! long as you reset the modifiers afterwards.
//!
//! # Key Up/Down events
//!
//! Routines are responsible for emitting [`key_down`](RoutineBuilder::key_down) and
//! [`key_up`](RoutineBuilder::key_up) events. This allows them to implement complex
//! behaviors such as sticky keys, radio groups, locked keys, or key redirects.
//!
//! # Flags
//!
//! Sometimes a routine wants to change its behavior depending on whether another key
//! was pressed or released while the key was down. This information is available via the
//! [`RoutineBuilder::later_key_actuated_load`] function.

#[cfg(test)]
mod tests;

#[allow(unused_imports)]
use {crate::builder::Builder, crate::state_machine::StateMachine, crate::Components};
use {
    crate::{Keycode, ModifierMask},
    debug_fn::debug_fn,
    hashbrown::{hash_map::Entry, HashMap, HashSet},
    isnt::std_1::primitive::IsntSliceExt,
    linearize::{Linearize, StaticMap},
    smallvec::SmallVec,
    std::{
        array,
        collections::VecDeque,
        fmt::{Debug, Formatter},
        mem::{self, ManuallyDrop},
        sync::Arc,
    },
};

/// A variable in a routine.
///
/// You can create variables by calling [`RoutineBuilder::allocate_var`] or
/// [`RoutineBuilder::allocate_vars`].
///
/// Variables are local to the routine. They cannot be shared between routines. Use
/// [`Global`]s to communicate between different routines in the same [`StateMachine`].
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct Var(u64);

#[derive(Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd, Linearize)]
pub(crate) enum Register {
    R0,
    R1,
    R2,
    R3,
    R4,
    R5,
    R6,
    R7,
}

impl Debug for Register {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "r{}", self.linearize())
    }
}

/// A global variable in a state machine.
///
/// Objects of this type are created using [`Builder::add_global`].
#[derive(Copy, Clone, Eq, PartialEq, Hash)]
pub struct Global(pub(crate) u32);

impl Debug for Global {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "g{}", self.0)
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Hash)]
pub(crate) enum BinOp {
    Add,
    Sub,
    Mul,
    Udiv,
    Idiv,
    Urem,
    Irem,
    Shl,
    Lshr,
    Ashr,
    BitNand,
    BitAnd,
    BitOr,
    BitXor,
    LogNand,
    LogAnd,
    LogOr,
    LogXor,
    Eq,
    Ne,
    Ult,
    Ilt,
    Ule,
    Ile,
}

impl Debug for BinOp {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            BinOp::Add => "add",
            BinOp::Sub => "sub",
            BinOp::Mul => "mul",
            BinOp::Udiv => "udiv",
            BinOp::Idiv => "idiv",
            BinOp::Urem => "urem",
            BinOp::Irem => "irem",
            BinOp::Shl => "shl",
            BinOp::Lshr => "lshr",
            BinOp::Ashr => "ashr",
            BinOp::BitNand => "bit_nand",
            BinOp::BitAnd => "bit_and",
            BinOp::BitOr => "bit_or",
            BinOp::BitXor => "bit_xor",
            BinOp::LogNand => "log_nand",
            BinOp::LogAnd => "log_and",
            BinOp::LogOr => "log_or",
            BinOp::LogXor => "log_xor",
            BinOp::Eq => "eq",
            BinOp::Ne => "ne",
            BinOp::Ult => "ult",
            BinOp::Ilt => "ilt",
            BinOp::Ule => "ule",
            BinOp::Ile => "ile",
        };
        f.write_str(s)
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub(crate) enum UnOp {
    Move,
    Neg,
    BitNot,
    LogNot,
}

#[derive(Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd, Linearize)]
pub(crate) enum Flag {
    LaterKeyActuated,
}

impl Debug for Flag {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            Flag::LaterKeyActuated => "later_key_actuated",
        };
        f.write_str(s)
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd, Linearize)]
enum Component {
    ModsPressed,
    ModsLatched,
    ModsLocked,
    GroupPressed,
    GroupLatched,
    GroupLocked,
}

impl Debug for Component {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            Component::ModsPressed => "mods_pressed",
            Component::ModsLatched => "mods_latched",
            Component::ModsLocked => "mods_locked",
            Component::GroupPressed => "group_pressed",
            Component::GroupLatched => "group_latched",
            Component::GroupLocked => "group_locked",
        };
        f.write_str(s)
    }
}

#[derive(Clone, Eq, PartialEq, Hash)]
enum Hi {
    // Generics ops
    Jump {
        to: usize,
        args: SmallVec<[(Var, Var); 1]>,
    },
    JumpIf {
        rs: Var,
        not: bool,
        to: usize,
        args: SmallVec<[(Var, Var); 1]>,
    },
    RegLit {
        rd: Var,
        lit: u32,
    },
    GlobalLoad {
        rd: Var,
        g: Global,
    },
    GlobalStore {
        rs: Var,
        g: Global,
    },
    BinOp {
        op: BinOp,
        rd: Var,
        rl: Var,
        rr: Var,
    },
    UnOp {
        op: UnOp,
        rd: Var,
        rs: Var,
    },
    // State machine ops
    FlagLoad {
        rd: Var,
        flag: Flag,
    },
    PressedModsInc {
        rs: Var,
    },
    PressedModsDec {
        rs: Var,
    },
    ComponentLoad {
        rd: Var,
        component: Component,
    },
    ComponentStore {
        rs: Var,
        component: Component,
    },
    KeyDown {
        rs: Var,
    },
    KeyUp {
        rs: Var,
    },
}

impl Debug for Hi {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let format_args = |f: &mut Formatter<'_>, args: &[(Var, Var)]| {
            for (idx, arg) in args.iter().enumerate() {
                if idx > 0 {
                    write!(f, ", ")?;
                }
                write!(f, "v{} = v{}", arg.0 .0, arg.1 .0)?;
            }
            Ok(())
        };
        match self {
            Hi::Jump { to, args } => {
                write!(f, "jump {to}, [{}]", debug_fn(|f| format_args(f, args)))
            }
            Hi::JumpIf { not, rs, to, args } => write!(
                f,
                "jump_if{} v{}, {to}, [{}]",
                if *not { "_not" } else { "" },
                rs.0,
                debug_fn(|f| format_args(f, args)),
            ),
            Hi::RegLit { rd, lit } => write!(f, "v{} = 0x{lit:x}", rd.0),
            Hi::GlobalLoad { rd, g } => write!(f, "v{} = {g:?}", rd.0),
            Hi::GlobalStore { rs, g } => write!(f, "{g:?} = v{}", rs.0),
            Hi::BinOp { op, rd, rl, rr } => write!(f, "v{} = {op:?} v{}, v{}", rd.0, rl.0, rr.0),
            Hi::UnOp { op, rd, rs } => write!(f, "v{} = {op:?} v{}", rd.0, rs.0),
            Hi::PressedModsInc { rs } => write!(f, "{:?} += v{}", Component::ModsPressed, rs.0),
            Hi::PressedModsDec { rs } => write!(f, "{:?} -= v{}", Component::ModsPressed, rs.0),
            Hi::ComponentLoad { rd, component } => write!(f, "v{} = {component:?}", rd.0),
            Hi::ComponentStore { rs, component } => write!(f, "{component:?} = v{}", rs.0),
            Hi::FlagLoad { rd, flag } => write!(f, "v{} = {flag:?}", rd.0),
            Hi::KeyDown { rs } => write!(f, "key_down v{}", rs.0),
            Hi::KeyUp { rs } => write!(f, "key_up v{}", rs.0),
        }
    }
}

macro_rules! lo {
    (
        binops:
            $($binop:ident,)*;
        unops:
            $($unop:ident,)*;
        components:
            $($component:ident = $load:ident, $store:ident,)*;
    ) => {
        #[derive(Copy, Clone, Eq, PartialEq, Hash)]
        pub(crate) enum Lo {
            // Generics ops
            Skip {
                n: usize,
            },
            SkipIf {
                rs: Register,
                n: usize,
            },
            SkipIfNot {
                rs: Register,
                n: usize,
            },
            SpillLoad {
                rd: Register,
                pos: usize,
            },
            SpillStore {
                rs: Register,
                pos: usize,
            },
            SpillMove {
                src: usize,
                dst: usize,
            },
            RegLit {
                rd: Register,
                lit: u32,
            },
            GlobalLoad {
                rd: Register,
                g: Global,
            },
            GlobalStore {
                rs: Register,
                g: Global,
            },
            $(
                $binop {
                    rd: Register,
                    rl: Register,
                    rr: Register,
                },
            )*
            $(
                $unop {
                    rd: Register,
                    rs: Register,
                },
            )*
            // State machine ops
            FlagLoad {
                rd: Register,
                flag: Flag,
            },
            PressedModsInc {
                rs: Register,
            },
            PressedModsDec {
                rs: Register,
            },
            $(
                $load {
                    rd: Register,
                },
                $store {
                    rs: Register,
                },
            )*
            KeyDown {
                rs: Register,
            },
            KeyUp {
                rs: Register,
            },
        }

        impl Debug for Lo {
            fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
                match self {
                    Lo::Skip { n } => {
                        write!(f, "skip {n}")
                    }
                    Lo::SkipIf { rs, n } => {
                        write!(f, "skip_if {rs:?}, {n}")
                    }
                    Lo::SkipIfNot { rs, n } => {
                        write!(f, "skip_if_not {rs:?}, {n}")
                    }
                    Lo::SpillLoad { rd, pos } => {
                        write!(f, "{rd:?} = spill[{pos}]")
                    }
                    Lo::SpillStore { rs, pos } => {
                        write!(f, "spill[{pos}] = {rs:?}")
                    }
                    Lo::SpillMove { src, dst } => {
                        write!(f, "spill[{dst}] = spill[{src}]")
                    }
                    Lo::RegLit { rd, lit } => {
                        write!(f, "{rd:?} = 0x{lit:x}")
                    }
                    Lo::GlobalLoad { rd, g } => {
                        write!(f, "{rd:?} = {g:?}")
                    }
                    Lo::GlobalStore { rs, g } => {
                        write!(f, "{g:?} = {rs:?}")
                    }
                    $(
                        Lo::$binop { rd, rl, rr } => {
                            write!(f, "{rd:?} = {:?} {rl:?}, {rr:?}", BinOp::$binop)
                        },
                    )*
                    $(
                        Lo::$unop { rd, rs } => {
                            write!(f, "{rd:?} = {:?} {rs:?}", UnOp::$unop)
                        },
                    )*
                    Lo::PressedModsInc { rs } => {
                        write!(f, "{:?} += {rs:?}", Component::ModsPressed)
                    }
                    Lo::PressedModsDec { rs } => {
                        write!(f, "{:?} -= {rs:?}", Component::ModsPressed)
                    }
                    $(
                        Lo::$load { rd } => {
                            write!(f, "{rd:?} = {:?}", Component::$component)
                        }
                        Lo::$store { rs } => {
                            write!(f, "{:?} = {rs:?}", Component::$component)
                        }
                    )*
                    Lo::FlagLoad { rd, flag } => {
                        write!(f, "{rd:?} = {flag:?}")
                    }
                    Lo::KeyDown { rs } => {
                        write!(f, "key_down {rs:?}")
                    }
                    Lo::KeyUp { rs } => {
                        write!(f, "key_up {rs:?}")
                    }
                }
            }
        }
    };
}

lo!(
    binops:
        Add, Sub, Mul, Udiv, Idiv, Urem, Irem, Shl, Lshr, Ashr, BitNand, BitAnd, BitOr,
        BitXor, LogNand, LogAnd, LogOr, LogXor, Eq, Ne, Ult, Ilt, Ule, Ile,
        ;
    unops:
        Move, Neg, BitNot, LogNot,
        ;
    components:
        ModsPressed = ModsPressedLoad, ModsPressedStore,
        ModsLatched = ModsLatchedLoad, ModsLatchedStore,
        ModsLocked = ModsLockedLoad, ModsLockedStore,
        GroupPressed = GroupPressedLoad, GroupPressedStore,
        GroupLatched = GroupLatchedLoad, GroupLatchedStore,
        GroupLocked = GroupLockedLoad, GroupLockedStore,
        ;
);

pub(crate) trait StateEventHandler {
    fn mods_pressed_inc(&mut self, mods: ModifierMask) {
        let _ = mods;
    }

    fn mods_pressed_dec(&mut self, mods: ModifierMask) {
        let _ = mods;
    }

    fn mods_pressed_load(&self) -> u32 {
        0
    }

    fn mods_pressed_store(&mut self, val: u32) {
        let _ = val;
    }

    fn mods_latched_load(&self) -> u32 {
        0
    }

    fn mods_latched_store(&mut self, val: u32) {
        let _ = val;
    }

    fn mods_locked_load(&self) -> u32 {
        0
    }

    fn mods_locked_store(&mut self, val: u32) {
        let _ = val;
    }

    fn group_pressed_load(&self) -> u32 {
        0
    }

    fn group_pressed_store(&mut self, val: u32) {
        let _ = val;
    }

    fn group_latched_load(&self) -> u32 {
        0
    }

    fn group_latched_store(&mut self, val: u32) {
        let _ = val;
    }

    fn group_locked_load(&self) -> u32 {
        0
    }

    fn group_locked_store(&mut self, val: u32) {
        let _ = val;
    }

    fn key_down(&mut self, keycode: Keycode) {
        let _ = keycode;
    }

    fn key_up(&mut self, keycode: Keycode) {
        let _ = keycode;
    }
}

/// A routine for use in a [`StateMachine`].
///
/// You can create routines with the [`RoutineBuilder`].
///
/// See the documentation of the [module](self) for more information about routines.
#[derive(Clone)]
pub struct Routine {
    pub(crate) on_press: Arc<[Lo]>,
    pub(crate) on_release: Arc<[Lo]>,
    pub(crate) spill: usize,
}

impl Debug for Routine {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        if f.alternate() {
            for (name, ops) in [
                ("on_press", &self.on_press),
                ("on_release", &self.on_release),
            ] {
                writeln!(f, "{name}:")?;
                for op in &**ops {
                    writeln!(f, "    {op:?}")?;
                }
            }
            Ok(())
        } else {
            f.debug_struct("Routine")
                .field("on_press", &self.on_press)
                .field("on_release", &self.on_release)
                .field("spill", &self.spill)
                .finish()
        }
    }
}

pub(crate) fn run<H>(
    h: &mut H,
    ops: &[Lo],
    registers: &mut StaticMap<Register, u32>,
    globals: &mut [u32],
    flags: &mut StaticMap<Flag, u32>,
    spill: &mut [u32],
) where
    H: StateEventHandler,
{
    let mut i = 0;
    while i < ops.len() {
        let op = ops[i];
        macro_rules! binop {
            ($rd:ident, $rl:ident, $rr:ident, $v:expr) => {{
                let $rl = registers[$rl];
                let $rr = registers[$rr];
                registers[$rd] = $v;
            }};
        }
        macro_rules! unop {
            ($rd:ident, $rs:ident, $v:expr) => {{
                let $rs = registers[$rs];
                registers[$rd] = $v;
            }};
        }
        match op {
            Lo::Skip { n } => {
                i = i.saturating_add(n);
            }
            Lo::SkipIf { rs, n } => {
                let s = registers[rs];
                if s != 0 {
                    i = i.saturating_add(n);
                }
            }
            Lo::SkipIfNot { rs, n } => {
                let s = registers[rs];
                if s == 0 {
                    i = i.saturating_add(n);
                }
            }
            Lo::SpillMove { src, dst } => {
                let src = spill.get(src).copied().unwrap_or_default();
                if let Some(dst) = spill.get_mut(dst) {
                    *dst = src;
                }
            }
            Lo::SpillLoad { rd, pos } => {
                if let Some(src) = spill.get(pos) {
                    registers[rd] = *src;
                }
            }
            Lo::SpillStore { rs, pos } => {
                if let Some(dst) = spill.get_mut(pos) {
                    *dst = registers[rs];
                }
            }
            Lo::RegLit { rd, lit } => {
                registers[rd] = lit;
            }
            Lo::GlobalLoad { rd, g } => {
                registers[rd] = match globals.get(g.0 as usize) {
                    None => 0,
                    Some(v) => *v,
                };
            }
            Lo::GlobalStore { rs, g } => {
                if let Some(v) = globals.get_mut(g.0 as usize) {
                    *v = registers[rs];
                }
            }
            Lo::Add { rd, rl, rr } => binop!(rd, rl, rr, rl.wrapping_add(rr)),
            Lo::Sub { rd, rl, rr } => binop!(rd, rl, rr, rl.wrapping_sub(rr)),
            Lo::Mul { rd, rl, rr } => binop!(rd, rl, rr, rl.wrapping_mul(rr)),
            Lo::Udiv { rd, rl, rr } => binop!(rd, rl, rr, {
                if rr == 0 {
                    0
                } else {
                    rl / rr
                }
            }),
            Lo::Idiv { rd, rl, rr } => binop!(rd, rl, rr, {
                let rl = rl as i32;
                let rr = rr as i32;
                if rr == 0 {
                    0
                } else if rr == -1 {
                    rl.wrapping_neg() as u32
                } else {
                    (rl / rr) as u32
                }
            }),
            Lo::Urem { rd, rl, rr } => binop!(rd, rl, rr, {
                if rr == 0 {
                    0
                } else {
                    rl % rr
                }
            }),
            Lo::Irem { rd, rl, rr } => binop!(rd, rl, rr, {
                let rl = rl as i32;
                let rr = rr as i32;
                if rr == 0 || rr == -1 {
                    0
                } else {
                    (rl % rr) as u32
                }
            }),
            Lo::Shl { rd, rl, rr } => binop!(rd, rl, rr, rl.wrapping_shl(rr)),
            Lo::Lshr { rd, rl, rr } => binop!(rd, rl, rr, rl.wrapping_shr(rr)),
            Lo::Ashr { rd, rl, rr } => binop!(rd, rl, rr, (rl as i32).wrapping_shr(rr) as u32),
            Lo::BitNand { rd, rl, rr } => binop!(rd, rl, rr, rl & !rr),
            Lo::BitAnd { rd, rl, rr } => binop!(rd, rl, rr, rl & rr),
            Lo::BitOr { rd, rl, rr } => binop!(rd, rl, rr, rl | rr),
            Lo::BitXor { rd, rl, rr } => binop!(rd, rl, rr, rl ^ rr),
            Lo::LogNand { rd, rl, rr } => binop!(rd, rl, rr, ((rl != 0) && (rr == 0)) as u32),
            Lo::LogAnd { rd, rl, rr } => binop!(rd, rl, rr, ((rl != 0) && (rr != 0)) as u32),
            Lo::LogOr { rd, rl, rr } => binop!(rd, rl, rr, ((rl != 0) || (rr != 0)) as u32),
            Lo::LogXor { rd, rl, rr } => binop!(rd, rl, rr, ((rl != 0) ^ (rr != 0)) as u32),
            Lo::Eq { rd, rl, rr } => binop!(rd, rl, rr, (rl == rr) as u32),
            Lo::Ne { rd, rl, rr } => binop!(rd, rl, rr, (rl != rr) as u32),
            Lo::Ult { rd, rl, rr } => binop!(rd, rl, rr, (rl < rr) as u32),
            Lo::Ilt { rd, rl, rr } => binop!(rd, rl, rr, ((rl as i32) < (rr as i32)) as u32),
            Lo::Ule { rd, rl, rr } => binop!(rd, rl, rr, (rl <= rr) as u32),
            Lo::Ile { rd, rl, rr } => binop!(rd, rl, rr, ((rl as i32) <= (rr as i32)) as u32),
            Lo::Move { rd, rs } => unop!(rd, rs, rs),
            Lo::Neg { rd, rs } => unop!(rd, rs, rs.wrapping_neg()),
            Lo::BitNot { rd, rs } => unop!(rd, rs, !rs),
            Lo::LogNot { rd, rs } => unop!(rd, rs, (rs == 0) as u32),
            Lo::PressedModsInc { rs } => {
                let s = registers[rs];
                h.mods_pressed_inc(ModifierMask(s));
            }
            Lo::PressedModsDec { rs } => {
                let s = registers[rs];
                h.mods_pressed_dec(ModifierMask(s));
            }
            Lo::FlagLoad { rd, flag } => {
                registers[rd] = flags[flag];
            }
            Lo::KeyDown { rs } => {
                let s = registers[rs];
                h.key_down(Keycode(s));
            }
            Lo::KeyUp { rs } => {
                let s = registers[rs];
                h.key_up(Keycode(s));
            }
            Lo::ModsPressedLoad { rd } => registers[rd] = h.mods_pressed_load(),
            Lo::ModsPressedStore { rs } => h.mods_pressed_store(registers[rs]),
            Lo::ModsLatchedLoad { rd } => registers[rd] = h.mods_latched_load(),
            Lo::ModsLatchedStore { rs } => h.mods_latched_store(registers[rs]),
            Lo::ModsLockedLoad { rd } => registers[rd] = h.mods_locked_load(),
            Lo::ModsLockedStore { rs } => h.mods_locked_store(registers[rs]),
            Lo::GroupPressedLoad { rd } => registers[rd] = h.group_pressed_load(),
            Lo::GroupPressedStore { rs } => h.group_pressed_store(registers[rs]),
            Lo::GroupLatchedLoad { rd } => registers[rd] = h.group_latched_load(),
            Lo::GroupLatchedStore { rs } => h.group_latched_store(registers[rs]),
            Lo::GroupLockedLoad { rd } => registers[rd] = h.group_locked_load(),
            Lo::GroupLockedStore { rs } => h.group_locked_store(registers[rs]),
        }
        i = i.saturating_add(1);
    }
}

/// An anchor for skipping forward in a [`Routine`].
///
/// This object can be used to skip forward in a routine using
/// [`RoutineBuilder::prepare_skip`], [`RoutineBuilder::prepare_skip_if`], and
/// [`RoutineBuilder::prepare_skip_if_not`].
///
/// # Panic
///
/// This type panics when dropped. You must pass it back into
/// [`RoutineBuilder::finish_skip`] to prevent this.
#[derive(Debug)]
#[must_use = "dropping this object panics"]
pub struct SkipAnchor {
    cond: Option<Var>,
    not: bool,
    block: usize,
    offset: usize,
}

impl Drop for SkipAnchor {
    fn drop(&mut self) {
        panic!("SkipAnchor is dropped");
    }
}

/// A builder for a [`Routine`].
#[derive(Default)]
pub struct RoutineBuilder {
    blocks: Vec<Vec<Hi>>,
    ops: Vec<Hi>,
    next_var: u64,
    on_release: Option<usize>,
}

impl Routine {
    /// Creates a [`RoutineBuilder`].
    pub fn builder() -> RoutineBuilder {
        RoutineBuilder::default()
    }
}

impl RoutineBuilder {
    /// Builds the [`Routine`].
    pub fn build(mut self) -> Routine {
        if self.ops.is_not_empty() {
            self.blocks.push(mem::take(&mut self.ops));
        }
        convert_to_ssa(self.next_var, &mut self.blocks);
        // println!("{:#?}", self.blocks);
        let mut allocator = RegisterAllocator::default();
        allocator.allocate_registers(&self.blocks);
        let mut on_press: Vec<_> = allocator.out.into();
        let mut snip = on_press.len();
        if let Some(on_release) = self.on_release {
            snip -= allocator
                .block_offsets
                .get(on_release)
                .copied()
                .unwrap_or_default();
        }
        let on_release = on_press[snip..].to_owned();
        on_press.truncate(snip);
        Routine {
            on_press: on_press.into(),
            on_release: on_release.into(),
            spill: allocator.spill.len(),
        }
    }

    /// Starts the part of the routine that runs when the key is released.
    ///
    /// The values of variables are preserved.
    ///
    /// # Panics
    ///
    /// Panics if `on_release` was already called on this builder.
    pub fn on_release(&mut self) -> &mut Self {
        assert!(self.on_release.is_none());
        self.ops.push(Hi::Jump {
            to: self.blocks.len() + 1,
            args: Default::default(),
        });
        self.blocks.push(mem::take(&mut self.ops));
        self.on_release = Some(self.blocks.len());
        self
    }

    /// Allocates a variable for use with this routine.
    ///
    /// The returned variable can only be used with this builder.
    ///
    /// All variables are implicitly initialized to `0` but initializing them explicitly
    /// will produce more efficient code.
    pub fn allocate_var(&mut self) -> Var {
        let res = Var(self.next_var);
        self.next_var += 1;
        res
    }

    /// Allocates `N` variables for use with this routine.
    ///
    /// This is a convenience function that forwards to [`Self::allocate_var`].
    pub fn allocate_vars<const N: usize>(&mut self) -> [Var; N] {
        array::from_fn(|_| self.allocate_var())
    }

    /// Prepares an unconditional skip from this point in the routine to a later point.
    ///
    /// [`SkipAnchor`] panics when dropped. You must pass it back into
    /// [`Self::finish_skip`] to prevent this.
    pub fn prepare_skip(&mut self) -> SkipAnchor {
        let offset = self.ops.len();
        self.ops.push(Hi::Jump {
            to: self.blocks.len() + 1,
            args: Default::default(),
        });
        SkipAnchor {
            cond: None,
            not: false,
            block: self.blocks.len(),
            offset,
        }
    }

    /// Prepares a conditional skip from this point in the routine to a later point.
    ///
    /// The skip is performed if `var != 0`.
    ///
    /// [`SkipAnchor`] panics when dropped. You must pass it back into
    /// [`Self::finish_skip`] to prevent this.
    pub fn prepare_skip_if(&mut self, var: Var) -> SkipAnchor {
        self.prepare_conditional_skip(var, false)
    }

    /// Prepares a conditional skip from this point in the routine to a later point.
    ///
    /// The skip is performed if `var == 0`.
    ///
    /// [`SkipAnchor`] panics when dropped. You must pass it back into
    /// [`Self::finish_skip`] to prevent this.
    pub fn prepare_skip_if_not(&mut self, var: Var) -> SkipAnchor {
        self.prepare_conditional_skip(var, true)
    }

    fn prepare_conditional_skip(&mut self, var: Var, inverse: bool) -> SkipAnchor {
        let offset = self.ops.len();
        self.ops.push(Hi::JumpIf {
            rs: var,
            not: inverse,
            to: self.blocks.len() + 1,
            args: Default::default(),
        });
        SkipAnchor {
            cond: Some(var),
            not: inverse,
            block: self.blocks.len(),
            offset,
        }
    }

    /// Sets the destination of a skip.
    ///
    /// The [`SkipAnchor`] should have been created via [`Self::prepare_skip`],
    /// [`Self::prepare_skip_if`], or [`Self::prepare_skip_if_not`].
    ///
    /// If this anchor was created from a different [`RoutineBuilder`], the behavior is
    /// unspecified.
    ///
    /// If [`Self::build`] is called before all skip destinations have been set, the
    /// behavior is unspecified.
    ///
    /// If you try to jump from before [`Self::on_release`] to after [`Self::on_release`],
    /// the behavior is unspecified.
    pub fn finish_skip(&mut self, anchor: SkipAnchor) -> &mut Self {
        let anchor = ManuallyDrop::new(anchor);
        if let Some(last) = self.ops.last() {
            if !matches!(last, Hi::Jump { .. }) {
                self.ops.push(Hi::Jump {
                    to: self.blocks.len() + 1,
                    args: Default::default(),
                });
            }
            self.blocks.push(mem::take(&mut self.ops));
        }
        let op = match anchor.cond {
            None => Hi::Jump {
                to: self.blocks.len(),
                args: Default::default(),
            },
            Some(rs) => Hi::JumpIf {
                rs,
                not: anchor.not,
                to: self.blocks.len(),
                args: Default::default(),
            },
        };
        self.blocks[anchor.block][anchor.offset] = op;
        self
    }

    /// `rd = lit`
    pub fn load_lit(&mut self, rd: Var, lit: u32) -> &mut Self {
        self.ops.push(Hi::RegLit { rd, lit });
        self
    }

    /// `rd = g`
    pub fn load_global(&mut self, rd: Var, g: Global) -> &mut Self {
        self.ops.push(Hi::GlobalLoad { rd, g });
        self
    }

    /// `g = rs`
    pub fn store_global(&mut self, g: Global, rs: Var) -> &mut Self {
        self.ops.push(Hi::GlobalStore { rs, g });
        self
    }

    fn bin_op(&mut self, op: BinOp, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.ops.push(Hi::BinOp { op, rd, rl, rr });
        self
    }

    /// `rd = rl + rr`
    pub fn add(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Add, rd, rl, rr)
    }

    /// `rd = rl - rr`
    pub fn sub(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Sub, rd, rl, rr)
    }

    /// `rd = rl * rr`
    pub fn mul(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Mul, rd, rl, rr)
    }

    /// `rd = rl / rr`
    ///
    /// If `rr` is 0, the value of `rd` is unspecified.
    pub fn udiv(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Udiv, rd, rl, rr)
    }

    /// `rd = ((rl as i32) / (rr as i32)) as u32`
    ///
    /// If `rr` is 0, the value of `rd` is unspecified.
    pub fn idiv(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Idiv, rd, rl, rr)
    }

    /// `rd = rl % rr`
    ///
    /// If `rr` is 0, the value of `rd` is unspecified.
    pub fn urem(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Urem, rd, rl, rr)
    }

    /// `rd = ((rl as i32) % (rr as i32)) as u32`
    ///
    /// If `rr` is 0, the value of `rd` is unspecified.
    pub fn irem(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Irem, rd, rl, rr)
    }

    /// `rd = rl << rr`
    pub fn shl(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Shl, rd, rl, rr)
    }

    /// `rd = rl >> rr`
    pub fn lshr(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Lshr, rd, rl, rr)
    }

    /// `rd = ((rl as i32) >> rr) as u32`
    pub fn ashr(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Ashr, rd, rl, rr)
    }

    /// `rd = rl & !rr`
    pub fn bit_nand(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::BitNand, rd, rl, rr)
    }

    /// `rd = rl & rr`
    pub fn bit_and(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::BitAnd, rd, rl, rr)
    }

    /// `rd = rl | rr`
    pub fn bit_or(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::BitOr, rd, rl, rr)
    }

    /// `rd = rl ^ rr`
    pub fn bit_xor(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::BitXor, rd, rl, rr)
    }

    /// `rd = ((rl != 0) && (rr == 0)) as u32`
    pub fn log_nand(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::LogNand, rd, rl, rr)
    }

    /// `rd = ((rl != 0) && (rr != 0)) as u32`
    pub fn log_and(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::LogAnd, rd, rl, rr)
    }

    /// `rd = ((rl != 0) || (rr != 0)) as u32`
    pub fn log_or(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::LogOr, rd, rl, rr)
    }

    /// `rd = ((rl != 0) ^ (rr != 0)) as u32`
    pub fn log_xor(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::LogXor, rd, rl, rr)
    }

    /// `rd = (rl == rr) as u32`
    pub fn eq(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Eq, rd, rl, rr)
    }

    /// `rd = (rl != rr) as u32`
    pub fn ne(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Ne, rd, rl, rr)
    }

    /// `rd = (rl <= rr) as u32`
    pub fn ule(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Ule, rd, rl, rr)
    }

    /// `rd = ((rl as i32) <= (rr as i32)) as u32`
    pub fn ile(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Ile, rd, rl, rr)
    }

    /// `rd = (rl < rr) as u32`
    pub fn ult(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Ult, rd, rl, rr)
    }

    /// `rd = ((rl as i32) < (rr as i32)) as u32`
    pub fn ilt(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Ilt, rd, rl, rr)
    }

    /// `rd = (rl >= rr) as u32`
    pub fn uge(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Ule, rd, rr, rl)
    }

    /// `rd = ((rl as i32) >= (rr as i32)) as u32`
    pub fn ige(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Ile, rd, rr, rl)
    }

    /// `rd = (rl > rr) as u32`
    pub fn ugt(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Ult, rd, rr, rl)
    }

    /// `rd = ((rl as i32) > (rr as i32)) as u32`
    pub fn igt(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Ilt, rd, rr, rl)
    }

    fn un_op(&mut self, op: UnOp, rd: Var, rs: Var) -> &mut Self {
        self.ops.push(Hi::UnOp { op, rd, rs });
        self
    }

    /// `rd = rs`
    pub fn move_(&mut self, rd: Var, rs: Var) -> &mut Self {
        self.un_op(UnOp::Move, rd, rs)
    }

    /// `rd = -rs`
    pub fn neg(&mut self, rd: Var, rs: Var) -> &mut Self {
        self.un_op(UnOp::Neg, rd, rs)
    }

    /// `rd = !rs`
    pub fn bit_not(&mut self, rd: Var, rs: Var) -> &mut Self {
        self.un_op(UnOp::BitNot, rd, rs)
    }

    /// `rd = (rs == 0) as u32`
    pub fn log_not(&mut self, rd: Var, rs: Var) -> &mut Self {
        self.un_op(UnOp::LogNot, rd, rs)
    }

    /// `mods_pressed += rs`
    ///
    /// This increments the per-modifier reference count for all bits set in `rs`.
    ///
    /// If the reference count of a modifier changes from 0 to 1, the bit will be set in
    /// the `mods_pressed`.
    pub fn mods_pressed_inc(&mut self, rs: Var) -> &mut Self {
        self.ops.push(Hi::PressedModsInc { rs });
        self
    }

    /// `mods_pressed -= rs`
    ///
    /// This decrements the per-modifier reference count for all bits set in `rs`.
    ///
    /// If the reference count of a modifier changes from 1 to 0, the bit will be cleared
    /// in the `mods_pressed`.
    pub fn mods_pressed_dec(&mut self, rs: Var) -> &mut Self {
        self.ops.push(Hi::PressedModsDec { rs });
        self
    }

    fn component_load(&mut self, rd: Var, component: Component) -> &mut Self {
        self.ops.push(Hi::ComponentLoad { rd, component });
        self
    }

    fn component_store(&mut self, rs: Var, component: Component) -> &mut Self {
        self.ops.push(Hi::ComponentStore { rs, component });
        self
    }

    /// `rd = mods_pressed`
    pub fn mods_pressed_load(&mut self, rd: Var) -> &mut Self {
        self.component_load(rd, Component::ModsPressed)
    }

    /// `mods_pressed = rs`
    pub fn mods_pressed_store(&mut self, rs: Var) -> &mut Self {
        self.component_store(rs, Component::ModsPressed)
    }

    /// `rd = mods_latched`
    pub fn mods_latched_load(&mut self, rd: Var) -> &mut Self {
        self.component_load(rd, Component::ModsLatched)
    }

    /// `mods_latched = rs`
    pub fn mods_latched_store(&mut self, rs: Var) -> &mut Self {
        self.component_store(rs, Component::ModsLatched)
    }

    /// `rd = mods_locked`
    pub fn mods_locked_load(&mut self, rd: Var) -> &mut Self {
        self.component_load(rd, Component::ModsLocked)
    }

    /// `mods_locked = rs`
    pub fn mods_locked_store(&mut self, rs: Var) -> &mut Self {
        self.component_store(rs, Component::ModsLocked)
    }

    /// `rd = group_pressed`
    pub fn group_pressed_load(&mut self, rd: Var) -> &mut Self {
        self.component_load(rd, Component::GroupPressed)
    }

    /// `group_pressed = rs`
    pub fn group_pressed_store(&mut self, rs: Var) -> &mut Self {
        self.component_store(rs, Component::GroupPressed)
    }

    /// `rd = group_latched`
    pub fn group_latched_load(&mut self, rd: Var) -> &mut Self {
        self.component_load(rd, Component::GroupLatched)
    }

    /// `group_latched = rs`
    pub fn group_latched_store(&mut self, rs: Var) -> &mut Self {
        self.component_store(rs, Component::GroupLatched)
    }

    /// `rd = group_locked`
    pub fn group_locked_load(&mut self, rd: Var) -> &mut Self {
        self.component_load(rd, Component::GroupLocked)
    }

    /// `group_locked = rs`
    pub fn group_locked_store(&mut self, rs: Var) -> &mut Self {
        self.component_store(rs, Component::GroupLocked)
    }

    fn flag_load(&mut self, rd: Var, flag: Flag) -> &mut Self {
        self.ops.push(Hi::FlagLoad { rd, flag });
        self
    }

    /// `rd = later_key_actuated`
    pub fn later_key_actuated_load(&mut self, rd: Var) -> &mut Self {
        self.flag_load(rd, Flag::LaterKeyActuated)
    }

    /// `key_down(rs)`
    pub fn key_down(&mut self, rs: Var) -> &mut Self {
        self.ops.push(Hi::KeyDown { rs });
        self
    }

    /// `key_up(rs)`
    pub fn key_up(&mut self, rs: Var) -> &mut Self {
        self.ops.push(Hi::KeyUp { rs });
        self
    }
}

fn convert_to_ssa(mut next_var: u64, blocks: &mut [Vec<Hi>]) {
    let mut allocate_var = || {
        let var = Var(next_var);
        next_var += 1;
        var
    };
    let mut current_block_arguments = HashSet::new();
    let mut block_arguments = vec![Vec::<(Var, Var)>::new(); blocks.len()];
    for (idx, ops) in blocks.iter().enumerate().rev() {
        for op in ops.iter().rev() {
            match op {
                Hi::Jump { to, .. } => {
                    if let Some(ba) = block_arguments.get(*to) {
                        current_block_arguments.extend(ba.iter().map(|b| b.1));
                    }
                }
                Hi::JumpIf { rs, to, .. } => {
                    if let Some(ba) = block_arguments.get(*to) {
                        current_block_arguments.extend(ba.iter().map(|b| b.1));
                    }
                    current_block_arguments.insert(*rs);
                }
                Hi::RegLit { rd, .. } => {
                    current_block_arguments.remove(rd);
                }
                Hi::GlobalLoad { rd, .. } => {
                    current_block_arguments.remove(rd);
                }
                Hi::GlobalStore { rs, .. } => {
                    current_block_arguments.insert(*rs);
                }
                Hi::BinOp { rd, rl, rr, .. } => {
                    current_block_arguments.remove(rd);
                    current_block_arguments.insert(*rl);
                    current_block_arguments.insert(*rr);
                }
                Hi::UnOp { rd, rs, .. } => {
                    current_block_arguments.remove(rd);
                    current_block_arguments.insert(*rs);
                }
                Hi::PressedModsInc { rs } => {
                    current_block_arguments.insert(*rs);
                }
                Hi::PressedModsDec { rs } => {
                    current_block_arguments.insert(*rs);
                }
                Hi::ComponentLoad { rd, .. } => {
                    current_block_arguments.remove(rd);
                }
                Hi::ComponentStore { rs, .. } => {
                    current_block_arguments.insert(*rs);
                }
                Hi::FlagLoad { rd, .. } => {
                    current_block_arguments.remove(rd);
                }
                Hi::KeyDown { rs } | Hi::KeyUp { rs } => {
                    current_block_arguments.insert(*rs);
                }
            }
        }
        for arg in current_block_arguments.drain() {
            block_arguments[idx].push((Var(0), arg));
        }
        block_arguments[idx].sort_by_key(|v| v.1 .0);
        for (var, _) in &mut block_arguments[idx] {
            *var = allocate_var();
        }
    }
    let mut names = HashMap::new();
    for (idx, block) in blocks.iter_mut().enumerate() {
        names.clear();
        for (dst, src) in &block_arguments[idx] {
            names.insert(*src, *dst);
        }
        for op in block {
            let add_jump_source = |to: usize, args: &mut SmallVec<_>| {
                if let Some(ba) = block_arguments.get(to) {
                    for (dst, src) in ba {
                        args.push((*dst, names[src]));
                    }
                }
            };
            let rename_read = |rs: &mut Var, names: &HashMap<_, _>| *rs = names[rs];
            let mut rename_write = |rd: &mut Var, names: &mut HashMap<_, _>| {
                let var = allocate_var();
                names.insert(*rd, var);
                *rd = var;
            };
            match op {
                Hi::Jump { to, args } => {
                    add_jump_source(*to, args);
                }
                Hi::JumpIf { rs, to, args, .. } => {
                    add_jump_source(*to, args);
                    rename_read(rs, &names);
                }
                Hi::RegLit { rd, .. } => {
                    rename_write(rd, &mut names);
                }
                Hi::GlobalLoad { rd, .. } => {
                    rename_write(rd, &mut names);
                }
                Hi::GlobalStore { rs, .. } => {
                    rename_read(rs, &names);
                }
                Hi::BinOp { rd, rl, rr, .. } => {
                    rename_read(rl, &names);
                    rename_read(rr, &names);
                    rename_write(rd, &mut names);
                }
                Hi::UnOp { rd, rs, .. } => {
                    rename_read(rs, &names);
                    rename_write(rd, &mut names);
                }
                Hi::PressedModsInc { rs } => {
                    rename_read(rs, &names);
                }
                Hi::PressedModsDec { rs } => {
                    rename_read(rs, &names);
                }
                Hi::ComponentLoad { rd, .. } => {
                    rename_write(rd, &mut names);
                }
                Hi::ComponentStore { rs, .. } => {
                    rename_read(rs, &names);
                }
                Hi::FlagLoad { rd, .. } => {
                    rename_write(rd, &mut names);
                }
                Hi::KeyDown { rs } | Hi::KeyUp { rs } => {
                    rename_read(rs, &names);
                }
            }
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug)]
enum VariableLocation {
    Register(Register),
    Spilled(usize),
}

#[derive(Copy, Clone)]
struct RegisterOwner {
    // the variable currently stored in the register
    var: Var,
    // the index of the next use of the variable (in revere order). this is only a hint
    // used to decide which register to spill.
    earlier_use: usize,
    // the next time this register is used. this is used to prevent the same register
    // being allocated multiple times for a single instruction.
    later_use: usize,
}

#[derive(Default)]
struct RegisterAllocator {
    registers: StaticMap<Register, Option<RegisterOwner>>,
    spill: Vec<bool>,
    variable_uses: HashMap<Var, SmallVec<[usize; 2]>>,
    prefix: VecDeque<Lo>,
    out: VecDeque<Lo>,
    variable_locations: HashMap<Var, VariableLocation>,
    block_offsets: Vec<usize>,
}

impl RegisterAllocator {
    fn allocate_register(&mut self, idx: usize) -> Register {
        let mut earliest_register = None;
        let mut earliest_point = usize::MAX;
        for (rs, next_use) in &mut self.registers {
            match next_use {
                None => return rs,
                Some(ro) => {
                    if ro.later_use > idx && ro.earlier_use < earliest_point {
                        earliest_point = ro.earlier_use;
                        earliest_register = Some(rs);
                    }
                }
            }
        }
        let r = earliest_register.unwrap();
        let var = self.registers[r].unwrap().var;
        let spill = self.spill.iter().position(|v| !*v).unwrap_or_else(|| {
            self.spill.push(false);
            self.spill.len() - 1
        });
        self.spill[spill] = true;
        if self.out.front() == Some(&Lo::SpillStore { rs: r, pos: spill }) {
            self.out.pop_front();
        }
        self.out.push_front(Lo::SpillLoad { rd: r, pos: spill });
        self.variable_locations
            .insert(var, VariableLocation::Spilled(spill));
        r
    }

    fn get_var_register(&mut self, idx: usize, var: &Var, read: bool) -> Register {
        let next_location = self.variable_locations.get(var).copied();
        let r = match next_location {
            Some(VariableLocation::Register(rs)) => rs,
            Some(VariableLocation::Spilled(pos)) => {
                let r = self.allocate_register(idx);
                self.spill[pos] = false;
                if read {
                    self.prefix.push_front(Lo::SpillStore { rs: r, pos });
                } else {
                    self.out.push_front(Lo::SpillStore { rs: r, pos });
                }
                r
            }
            None => self.allocate_register(idx),
        };
        self.registers[r] = match self.variable_uses.get_mut(var).unwrap().pop() {
            None => {
                if next_location.is_some() {
                    self.variable_locations.remove(var);
                }
                None
            }
            Some(next_use) => {
                self.variable_locations
                    .insert(*var, VariableLocation::Register(r));
                Some(RegisterOwner {
                    var: *var,
                    earlier_use: next_use,
                    later_use: idx,
                })
            }
        };
        r
    }

    fn get_read_register(&mut self, idx: usize, var: &Var) -> Register {
        self.get_var_register(idx, var, true)
    }

    fn get_write_register(&mut self, idx: usize, var: &Var) -> Register {
        self.get_var_register(idx, var, false)
    }

    fn find_variable_uses(&mut self, block: &[Hi]) {
        for (idx, op) in block.iter().enumerate() {
            let mut use_ = |var: &Var, read: bool| {
                let uses = match self.variable_uses.entry(*var) {
                    Entry::Occupied(e) => e.into_mut(),
                    Entry::Vacant(v) => {
                        let e = v.insert(SmallVec::default());
                        if read {
                            e.push(0);
                        }
                        e
                    }
                };
                uses.push(idx);
            };
            macro_rules! read {
                ($r:expr) => {
                    use_($r, true)
                };
            }
            macro_rules! write {
                ($r:expr) => {
                    use_($r, false)
                };
            }
            match op {
                Hi::Jump { .. } => {}
                Hi::JumpIf { rs, .. } => {
                    read!(rs);
                }
                Hi::RegLit { rd, .. } => {
                    write!(rd);
                }
                Hi::GlobalLoad { rd, .. } => {
                    write!(rd);
                }
                Hi::GlobalStore { rs, .. } => {
                    read!(rs);
                }
                Hi::BinOp { rd, rl, rr, .. } => {
                    read!(rl);
                    read!(rr);
                    write!(rd);
                }
                Hi::UnOp { rd, rs, .. } => {
                    read!(rs);
                    write!(rd);
                }
                Hi::PressedModsInc { rs, .. } => {
                    read!(rs);
                }
                Hi::PressedModsDec { rs, .. } => {
                    read!(rs);
                }
                Hi::ComponentLoad { rd, .. } => {
                    write!(rd);
                }
                Hi::ComponentStore { rs, .. } => {
                    read!(rs);
                }
                Hi::FlagLoad { rd, .. } => {
                    write!(rd);
                }
                Hi::KeyDown { rs } | Hi::KeyUp { rs } => {
                    read!(rs);
                }
            }
        }
        for uses in self.variable_uses.values_mut() {
            uses.pop();
        }
    }

    fn acquire_unused_location(&mut self, src: Var) -> VariableLocation {
        for (rs, next_use) in &mut self.registers {
            if next_use.is_none() {
                *next_use = Some(RegisterOwner {
                    var: src,
                    earlier_use: self
                        .variable_uses
                        .get(&src)
                        .and_then(|v| v.last())
                        .copied()
                        .unwrap_or_default(),
                    later_use: usize::MAX,
                });
                return VariableLocation::Register(rs);
            }
        }
        for (idx, used) in self.spill.iter_mut().enumerate() {
            if !*used {
                *used = true;
                return VariableLocation::Spilled(idx);
            }
        }
        self.spill.push(true);
        VariableLocation::Spilled(self.spill.len() - 1)
    }

    fn encode_skip_if(&mut self, rs: Register, mut not: bool, mut n: usize) {
        loop {
            if n != 1 {
                break;
            }
            let Some(Lo::Skip { n: m }) = self.out.front() else {
                break;
            };
            n = *m;
            not = !not;
            self.out.pop_front();
        }
        if n == 0 || self.out.is_empty() {
            return;
        }
        let op = match not {
            true => Lo::SkipIfNot { rs, n },
            false => Lo::SkipIf { rs, n },
        };
        self.out.push_front(op);
    }

    fn translate_skip(&mut self, to: &usize, args: &[(Var, Var)]) {
        let n = self.out.len() - self.block_offsets.get(*to).copied().unwrap_or_default();
        if n != 0 {
            self.out.push_front(Lo::Skip { n });
        }
        let mut todo = vec![];
        let mut preserve = HashSet::<VariableLocation>::new();
        for (dst, src) in args {
            let dst_loc = self.variable_locations[dst];
            match self.variable_locations.entry(*src) {
                Entry::Occupied(o) => {
                    if *o.get() != dst_loc {
                        todo.push((dst_loc, *src));
                        preserve.insert(*o.get());
                    }
                }
                Entry::Vacant(v) => match dst_loc {
                    VariableLocation::Register(r) => {
                        if self.registers[r].is_none() {
                            self.registers[r] = Some(RegisterOwner {
                                var: *src,
                                earlier_use: self
                                    .variable_uses
                                    .get(src)
                                    .and_then(|v| v.last())
                                    .copied()
                                    .unwrap_or_default(),
                                later_use: usize::MAX,
                            });
                            v.insert(dst_loc);
                        } else {
                            todo.push((dst_loc, *src));
                        }
                    }
                    VariableLocation::Spilled(pos) =>
                    {
                        #[expect(clippy::bool_comparison)]
                        if self.spill[pos] == false {
                            self.spill[pos] = true;
                            v.insert(dst_loc);
                        } else {
                            todo.push((dst_loc, *src));
                        }
                    }
                },
            }
        }
        if todo.is_empty() {
            return;
        }
        let mut moved_out = vec![];
        let mut ins = vec![];
        let mut translate_move = |dst, src| match (dst, src) {
            (VariableLocation::Spilled(dst), VariableLocation::Spilled(src)) => {
                ins.push(Lo::SpillMove { src, dst });
            }
            (VariableLocation::Spilled(pos), VariableLocation::Register(rs)) => {
                ins.push(Lo::SpillStore { rs, pos });
            }
            (VariableLocation::Register(rd), VariableLocation::Spilled(pos)) => {
                ins.push(Lo::SpillLoad { rd, pos });
            }
            (VariableLocation::Register(rd), VariableLocation::Register(rs)) => {
                ins.push(Lo::Move { rd, rs });
            }
        };
        'outer: while todo.len() > 0 {
            for (idx, &(dst, src)) in todo.iter().enumerate() {
                if preserve.contains(&dst) {
                    continue;
                }
                let src = match self.variable_locations.get(&src) {
                    Some(src) => {
                        preserve.remove(src);
                        *src
                    }
                    None => {
                        let loc = self.acquire_unused_location(src);
                        self.variable_locations.insert(src, loc);
                        loc
                    }
                };
                translate_move(dst, src);
                todo.swap_remove(idx);
                continue 'outer;
            }
            let (dst, src) = todo.pop().unwrap();
            let unused = self.acquire_unused_location(src);
            let src = self.variable_locations[&src];
            preserve.remove(&src);
            translate_move(unused, src);
            moved_out.push((dst, unused));
        }
        for (dst, src) in moved_out {
            translate_move(dst, src);
            match src {
                VariableLocation::Register(r) => self.registers[r] = None,
                VariableLocation::Spilled(pos) => self.spill[pos] = false,
            }
        }
        while let Some(ins) = ins.pop() {
            self.out.push_front(ins);
        }
    }

    fn translate_instructions(&mut self, block: &[Hi]) {
        for (idx, op) in block.iter().enumerate().rev() {
            match op {
                Hi::Jump { to, args } => {
                    self.translate_skip(to, args);
                }
                Hi::JumpIf { rs, not, to, args } => {
                    let rs = self.get_read_register(idx, rs);
                    let prev_len = self.out.len();
                    self.translate_skip(to, args);
                    if prev_len < self.out.len() {
                        self.encode_skip_if(rs, !*not, self.out.len() - prev_len);
                    }
                }
                Hi::RegLit { rd, lit } => {
                    let rd = self.get_write_register(idx, rd);
                    self.out.push_front(Lo::RegLit { rd, lit: *lit });
                }
                Hi::GlobalLoad { rd, g } => {
                    let rd = self.get_write_register(idx, rd);
                    self.out.push_front(Lo::GlobalLoad { rd, g: *g });
                }
                Hi::GlobalStore { rs, g } => {
                    let rs = self.get_read_register(idx, rs);
                    self.out.push_front(Lo::GlobalStore { rs, g: *g });
                }
                Hi::BinOp { rd, rl, rr, op } => {
                    let rd = self.get_write_register(idx, rd);
                    let rl = self.get_read_register(idx, rl);
                    let rr = self.get_read_register(idx, rr);
                    macro_rules! ops {
                        ($($name:ident,)*) => {
                            match op {
                                $(
                                    BinOp::$name => Lo::$name { rd, rl, rr },
                                )*
                            }
                        };
                    }
                    let lo = ops! {
                        Add, Sub, Mul, Udiv, Idiv, Urem, Irem, Shl, Lshr, Ashr, BitNand, BitAnd, BitOr,
                        BitXor, LogNand, LogAnd, LogOr, LogXor, Eq, Ne, Ult, Ilt, Ule, Ile,
                    };
                    self.out.push_front(lo);
                }
                Hi::UnOp { rd, rs, op } => {
                    let rd = self.get_write_register(idx, rd);
                    let rs = self.get_read_register(idx, rs);
                    macro_rules! ops {
                        ($($name:ident,)*) => {
                            match op {
                                $(
                                    UnOp::$name => Lo::$name { rd, rs },
                                )*
                            }
                        };
                    }
                    let lo = ops! {
                        Move, Neg, BitNot, LogNot,
                    };
                    self.out.push_front(lo);
                }
                Hi::PressedModsInc { rs } => {
                    let rs = self.get_read_register(idx, rs);
                    self.out.push_front(Lo::PressedModsInc { rs });
                }
                Hi::PressedModsDec { rs } => {
                    let rs = self.get_read_register(idx, rs);
                    self.out.push_front(Lo::PressedModsDec { rs });
                }
                Hi::ComponentLoad { rd, component } => {
                    let rd = self.get_write_register(idx, rd);
                    macro_rules! comp {
                        ($($name:ident => $f:ident,)*) => {
                            match component {
                                $(
                                    Component::$name => Lo::$f { rd },
                                )*
                            }
                        };
                    }
                    let lo = comp! {
                        ModsPressed => ModsPressedLoad,
                        ModsLatched => ModsLatchedLoad,
                        ModsLocked => ModsLockedLoad,
                        GroupPressed => GroupPressedLoad,
                        GroupLatched => GroupLatchedLoad,
                        GroupLocked => GroupLockedLoad,
                    };
                    self.out.push_front(lo);
                }
                Hi::ComponentStore { rs, component } => {
                    let rs = self.get_read_register(idx, rs);
                    macro_rules! comp {
                        ($($name:ident => $f:ident,)*) => {
                            match component {
                                $(
                                    Component::$name => Lo::$f { rs },
                                )*
                            }
                        };
                    }
                    let lo = comp! {
                        ModsPressed => ModsPressedStore,
                        ModsLatched => ModsLatchedStore,
                        ModsLocked => ModsLockedStore,
                        GroupPressed => GroupPressedStore,
                        GroupLatched => GroupLatchedStore,
                        GroupLocked => GroupLockedStore,
                    };
                    self.out.push_front(lo);
                }
                Hi::FlagLoad { rd, flag } => {
                    let rd = self.get_write_register(idx, rd);
                    self.out.push_front(Lo::FlagLoad { rd, flag: *flag });
                }
                Hi::KeyDown { rs } => {
                    let rs = self.get_read_register(idx, rs);
                    self.out.push_front(Lo::KeyDown { rs });
                }
                Hi::KeyUp { rs } => {
                    let rs = self.get_read_register(idx, rs);
                    self.out.push_front(Lo::KeyUp { rs });
                }
            }
            while let Some(op) = self.prefix.pop_back() {
                self.out.push_front(op);
            }
        }
    }

    fn allocate_registers(&mut self, blocks: &[Vec<Hi>]) {
        // println!("{:#?}", blocks);
        self.block_offsets = vec![0; blocks.len()];
        for (idx, block) in blocks.iter().enumerate().rev() {
            self.registers.clear();
            self.variable_uses.clear();
            self.spill.iter_mut().for_each(|v| *v = false);
            self.find_variable_uses(block);
            self.translate_instructions(block);
            self.block_offsets[idx] = self.out.len();
        }
    }
}
