#[cfg(test)]
mod tests;

use isnt::std_1::primitive::IsntSliceExt;
use {
    crate::modifier::ModifierMask,
    debug_fn::debug_fn,
    hashbrown::{hash_map::Entry, DefaultHashBuilder, HashMap, HashSet},
    linearize::{Linearize, StaticMap},
    min_max_heap::MinMaxHeap,
    smallvec::SmallVec,
    std::{
        fmt::{Debug, Formatter},
        mem,
        sync::Arc,
    },
};

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct Var(u64);

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd, Linearize)]
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

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd, Linearize)]
pub enum Global {
    G0,
    G1,
    G2,
    G3,
    G4,
    G5,
    G6,
    G7,
    G8,
    G9,
    G10,
    G11,
    G12,
    G13,
    G14,
    G15,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
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
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub(crate) enum UnOp {
    Neg,
    BitNot,
    LogNot,
}

#[derive(Clone, Eq, PartialEq, Hash)]
pub(crate) enum Hi {
    // Generics ops
    Skip {
        n: usize,
        args: SmallVec<[(Var, Var); 1]>,
    },
    SkipIf {
        rs: Var,
        n: usize,
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
    Phi {
        dst: Var,
        src: Vec<Var>,
    },
    // State machine ops
    PressedModsInc {
        rs: Var,
    },
    PressedModsDec {
        rs: Var,
    },
    LatchedModsLoad {
        rd: Var,
    },
    LatchedModsStore {
        rs: Var,
    },
    LockedModsLoad {
        rd: Var,
    },
    LockedModsStore {
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
            Hi::Skip { n, args } => write!(f, "skip {n}, [{}]", debug_fn(|f| format_args(f, args))),
            Hi::SkipIf { rs, n, args } => write!(
                f,
                "skip_if v{}, {n}, [{}]",
                rs.0,
                debug_fn(|f| format_args(f, args)),
            ),
            Hi::RegLit { rd, lit } => write!(f, "v{} = 0x{lit:x}", rd.0),
            Hi::GlobalLoad { rd, g } => write!(f, "v{} = {g:?}", rd.0),
            Hi::GlobalStore { rs, g } => write!(f, "{g:?} = v{}", rs.0),
            Hi::BinOp { op, rd, rl, rr } => write!(f, "v{} = {op:?} v{}, v{}", rd.0, rl.0, rr.0),
            Hi::UnOp { op, rd, rs } => write!(f, "v{} = {op:?} v{}", rd.0, rs.0),
            Hi::Phi { dst, src } => write!(
                f,
                "v{} = phi({})",
                dst.0,
                debug_fn(|f| {
                    for (idx, v) in src.iter().enumerate() {
                        if idx > 0 {
                            write!(f, ", ")?;
                        }
                        write!(f, "v{}", v.0)?;
                    }
                    Ok(())
                })
            ),
            Hi::PressedModsInc { rs } => write!(f, "pressed_mods += v{}", rs.0),
            Hi::PressedModsDec { rs } => write!(f, "pressed_mods -= v{}", rs.0),
            Hi::LatchedModsLoad { rd } => write!(f, "v{} = latched_mods", rd.0),
            Hi::LatchedModsStore { rs } => write!(f, "locked_mods = v{}", rs.0),
            Hi::LockedModsLoad { rd } => write!(f, "v{} = locked_mods", rd.0),
            Hi::LockedModsStore { rs } => write!(f, "locked_mods = v{}", rs.0),
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub(crate) enum Lo {
    // Generics ops
    Skip {
        n: usize,
    },
    SkipIf {
        rs: Register,
        n: usize,
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
    BinOp {
        op: BinOp,
        rd: Register,
        rl: Register,
        rr: Register,
    },
    UnOp {
        op: UnOp,
        rd: Register,
        rs: Register,
    },
    // State machine ops
    PressedModsInc {
        rs: Register,
    },
    PressedModsDec {
        rs: Register,
    },
    LatchedModsLoad {
        rd: Register,
    },
    LatchedModsStore {
        rs: Register,
    },
    LockedModsLoad {
        rd: Register,
    },
    LockedModsStore {
        rs: Register,
    },
}

impl Hi {
    fn defines_register(&self) -> bool {
        match self {
            Hi::RegLit { .. }
            | Hi::GlobalLoad { .. }
            | Hi::BinOp { .. }
            | Hi::UnOp { .. }
            | Hi::LatchedModsLoad { .. }
            | Hi::LockedModsLoad { .. } => true,
            Hi::Skip { .. }
            | Hi::SkipIf { .. }
            | Hi::GlobalStore { .. }
            | Hi::PressedModsInc { .. }
            | Hi::PressedModsDec { .. }
            | Hi::LatchedModsStore { .. }
            | Hi::LockedModsStore { .. } => false,
            Hi::Phi { .. } => {
                unreachable!()
            }
        }
    }
}

pub trait StateEventHandler {
    fn mods_pressed_inc(&mut self, mods: ModifierMask) {
        let _ = mods;
    }

    fn mods_pressed_dec(&mut self, mods: ModifierMask) {
        let _ = mods;
    }

    fn mods_latched_load(&self) -> ModifierMask {
        ModifierMask(0)
    }

    fn mods_latched_store(&mut self, mods: ModifierMask) {
        let _ = mods;
    }

    fn mods_locked_load(&self) -> ModifierMask {
        ModifierMask(0)
    }

    fn mods_locked_store(&mut self, mods: ModifierMask) {
        let _ = mods;
    }
}

#[derive(Clone)]
pub struct Routine {
    pub(crate) ops: Arc<[Lo]>,
}

#[inline]
pub(crate) fn run<H>(
    h: &mut H,
    ops: &[Lo],
    registers: &mut StaticMap<Register, u32>,
    globals: &mut StaticMap<Global, u32>,
) where
    H: StateEventHandler,
{
    let mut i = 0;
    let len = ops.len();
    while i < len {
        let op = ops[i];
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
            Lo::RegLit { rd, lit } => {
                registers[rd] = lit;
            }
            Lo::GlobalLoad { rd, g } => {
                registers[rd] = globals[g];
            }
            Lo::GlobalStore { rs, g } => {
                globals[g] = registers[rs];
            }
            Lo::BinOp { op, rd, rl, rr } => {
                let l = registers[rl];
                let r = registers[rr];
                registers[rd] = match op {
                    BinOp::Add => l.wrapping_add(r),
                    BinOp::Sub => l.wrapping_sub(r),
                    BinOp::Mul => l.wrapping_mul(r),
                    BinOp::Udiv => {
                        if r == 0 {
                            0
                        } else {
                            l / r
                        }
                    }
                    BinOp::Idiv => {
                        let l = l as i32;
                        let r = r as i32;
                        if r == 0 {
                            0
                        } else if r == -1 {
                            l.wrapping_neg() as u32
                        } else {
                            (l / r) as u32
                        }
                    }
                    BinOp::Urem => {
                        if r == 0 {
                            0
                        } else {
                            l % r
                        }
                    }
                    BinOp::Irem => {
                        let l = l as i32;
                        let r = r as i32;
                        if r == 0 || r == -1 {
                            0
                        } else {
                            (l % r) as u32
                        }
                    }
                    BinOp::Shl => l << r,
                    BinOp::Lshr => l >> r,
                    BinOp::Ashr => (l as i32 >> r) as u32,
                    BinOp::BitNand => l & !r,
                    BinOp::BitAnd => l & r,
                    BinOp::BitOr => l | r,
                    BinOp::BitXor => l ^ r,
                    BinOp::LogNand => ((l != 0) && (r == 0)) as u32,
                    BinOp::LogAnd => ((l != 0) && (r != 0)) as u32,
                    BinOp::LogOr => ((l != 0) || (r != 0)) as u32,
                    BinOp::LogXor => ((l != 0) ^ (r != 0)) as u32,
                };
            }
            Lo::UnOp { op, rd, rs } => {
                let s = registers[rs];
                registers[rd] = match op {
                    UnOp::Neg => s.wrapping_neg(),
                    UnOp::BitNot => !s,
                    UnOp::LogNot => (s == 0) as u32,
                }
            }
            Lo::PressedModsInc { rs } => {
                let s = registers[rs];
                h.mods_pressed_inc(ModifierMask(s));
            }
            Lo::PressedModsDec { rs } => {
                let s = registers[rs];
                h.mods_pressed_dec(ModifierMask(s));
            }
            Lo::LatchedModsLoad { rd } => {
                registers[rd] = h.mods_latched_load().0;
            }
            Lo::LatchedModsStore { rs } => {
                let s = registers[rs];
                h.mods_latched_store(ModifierMask(s));
            }
            Lo::LockedModsLoad { rd } => {
                registers[rd] = h.mods_locked_load().0;
            }
            Lo::LockedModsStore { rs } => {
                let s = registers[rs];
                h.mods_locked_store(ModifierMask(s));
            }
        }
        i = i.saturating_add(1);
    }
}

pub struct SkipAnchor {
    cond: Option<Var>,
    idx: usize,
}

impl Default for SkipAnchor {
    fn default() -> Self {
        Self {
            cond: None,
            idx: !0,
        }
    }
}

pub struct RoutineBuilder {
    ops: Vec<Hi>,
    next_var: u64,
}

impl Routine {
    pub fn builder() -> RoutineBuilder {
        RoutineBuilder {
            ops: Default::default(),
            next_var: 0,
        }
    }
}

impl RoutineBuilder {
    pub fn build(&self) -> Routine {
        todo!()
    }

    pub fn on_release(&mut self) -> &mut Self {
        todo!()
    }

    pub fn allocate_var(&mut self) -> Var {
        self.allocate_vars::<1>()[0]
    }

    pub fn allocate_vars<const N: usize>(&mut self) -> [Var; N] {
        let mut res = [Var(0); N];
        for i in 0..N {
            res[i] = Var(self.next_var);
            self.next_var += 1;
        }
        res
    }

    pub fn prepare_skip(&mut self, anchor: &mut SkipAnchor) -> &mut Self {
        let idx = self.ops.len();
        self.ops.push(Hi::Skip {
            n: 0,
            args: Default::default(),
        });
        *anchor = SkipAnchor { cond: None, idx };
        self
    }

    pub fn prepare_conditional_skip(&mut self, var: Var, anchor: &mut SkipAnchor) -> &mut Self {
        let idx = self.ops.len();
        self.ops.push(Hi::Skip {
            n: 0,
            args: Default::default(),
        });
        *anchor = SkipAnchor {
            cond: Some(var),
            idx,
        };
        self
    }

    pub fn finish_skip(&mut self, anchor: &mut SkipAnchor) -> &mut Self {
        let n = self.ops.len() - anchor.idx - 1;
        let op = match anchor.cond {
            None => Hi::Skip {
                n,
                args: Default::default(),
            },
            Some(rs) => Hi::SkipIf {
                rs,
                n,
                args: Default::default(),
            },
        };
        self.ops[anchor.idx] = op;
        *anchor = Default::default();
        self
    }

    pub fn load_lit(&mut self, rd: Var, lit: u32) -> &mut Self {
        self.ops.push(Hi::RegLit { rd, lit });
        self
    }

    pub fn load_global(&mut self, rd: Var, g: Global) -> &mut Self {
        self.ops.push(Hi::GlobalLoad { rd, g });
        self
    }

    pub fn store_global(&mut self, g: Global, rs: Var) -> &mut Self {
        self.ops.push(Hi::GlobalStore { rs, g });
        self
    }

    fn bin_op(&mut self, op: BinOp, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.ops.push(Hi::BinOp { op, rd, rl, rr });
        self
    }

    pub fn add(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Add, rd, rl, rr)
    }

    pub fn sub(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Sub, rd, rl, rr)
    }

    pub fn mul(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Mul, rd, rl, rr)
    }

    pub fn udiv(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Udiv, rd, rl, rr)
    }

    pub fn idiv(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Idiv, rd, rl, rr)
    }

    pub fn urem(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Urem, rd, rl, rr)
    }

    pub fn irem(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Irem, rd, rl, rr)
    }

    pub fn shl(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Shl, rd, rl, rr)
    }

    pub fn lshr(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Lshr, rd, rl, rr)
    }

    pub fn ashr(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::Ashr, rd, rl, rr)
    }

    pub fn bit_nand(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::BitNand, rd, rl, rr)
    }

    pub fn bit_and(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::BitAnd, rd, rl, rr)
    }

    pub fn bit_or(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::BitOr, rd, rl, rr)
    }

    pub fn bit_xor(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::BitXor, rd, rl, rr)
    }

    pub fn log_nand(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::LogNand, rd, rl, rr)
    }

    pub fn log_and(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::LogAnd, rd, rl, rr)
    }

    pub fn log_or(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::LogOr, rd, rl, rr)
    }

    pub fn log_xor(&mut self, rd: Var, rl: Var, rr: Var) -> &mut Self {
        self.bin_op(BinOp::LogXor, rd, rl, rr)
    }

    fn un_op(&mut self, op: UnOp, rd: Var, rs: Var) -> &mut Self {
        self.ops.push(Hi::UnOp { op, rd, rs });
        self
    }

    pub fn neg(&mut self, rd: Var, rs: Var) -> &mut Self {
        self.un_op(UnOp::Neg, rd, rs)
    }

    pub fn bit_not(&mut self, rd: Var, rs: Var) -> &mut Self {
        self.un_op(UnOp::BitNot, rd, rs)
    }

    pub fn log_not(&mut self, rd: Var, rs: Var) -> &mut Self {
        self.un_op(UnOp::LogNot, rd, rs)
    }

    pub fn pressed_mods_inc(&mut self, rs: Var) -> &mut Self {
        self.ops.push(Hi::PressedModsInc { rs });
        self
    }

    pub fn pressed_mods_dec(&mut self, rs: Var) -> &mut Self {
        self.ops.push(Hi::PressedModsDec { rs });
        self
    }

    pub fn latched_mods_load(&mut self, rd: Var) -> &mut Self {
        self.ops.push(Hi::LatchedModsLoad { rd });
        self
    }

    pub fn latched_mods_store(&mut self, rs: Var) -> &mut Self {
        self.ops.push(Hi::LatchedModsStore { rs });
        self
    }

    pub fn locked_mods_load(&mut self, rd: Var) -> &mut Self {
        self.ops.push(Hi::LockedModsLoad { rd });
        self
    }

    pub fn locked_mods_store(&mut self, rs: Var) -> &mut Self {
        self.ops.push(Hi::LockedModsStore { rs });
        self
    }
}

// struct RegisterAllocation {
//     on_press: Vec<Op<Register>>,
//     on_release: Vec<Op<Register>>,
//     spill_size: usize,
// }
//
// fn perform_register_allocation(ops: &[Op<Var>]) -> RegisterAllocation {
//     let mut live_vars = HashMap::new();
//     for (idx, op) in ops.iter().enumerate() {
//         let access = |var: &Var| {
//             live_vars.entry(*var).or_insert((idx, idx)).1 = idx + 1;
//         };
//         match op {
//             Op::Skip { .. } => {}
//             Op::SkipIf { rs, .. } => {
//                 access(rs);
//             }
//             Op::RegLit { rd, .. } => {
//                 access(rd);
//             }
//             Op::GlobalLoad { rd, .. } => {
//                 access(rd);
//             }
//             Op::GlobalStore { rs, .. } => {
//                 access(rs);
//             }
//             Op::BinOp { rd, rl, rr, .. } => {
//                 access(rl);
//                 access(rr);
//                 access(rd);
//             }
//             Op::UnOp { rd, rs, .. } => {
//                 access(rs);
//                 access(rd);
//             }
//             Op::PressedModsInc { rs, .. } => {
//                 access(rs);
//             }
//             Op::PressedModsDec { rs, .. } => {
//                 access(rs);
//             }
//             Op::LatchedModsLoad { rd, .. } => {
//                 access(rd);
//             }
//             Op::LatchedModsStore { rs, .. } => {
//                 access(rs);
//             }
//             Op::LockedModsLoad { rd, .. } => {
//                 access(rd);
//             }
//             Op::LockedModsStore { rs, .. } => {
//                 access(rs);
//             }
//         }
//     }
//     let mut live_ranges = vec![];
//     for (var, range) in live_vars {
//         live_ranges.push((var, range.0, range.1));
//     }
//     live_ranges.sort_unstable_by_key(|k| k.1);
//     enum VarLocation {
//         Register(Register),
//         Spilled(usize),
//     }
//     let mut var_locations = HashMap::new();
//     let mut jump_offset = 0usize;
//     let mut res = vec![];
//     let mut jump_sources = HashMap::<_, Vec<_>>::new();
//     let handle_jump_sources = |idx, jump_sources, res| {
//         for (idx, offset) in jump_sources.remove(&idx).unwrap_or_default() {
//             match &mut res[idx] {
//                 Op::Skip { n } => *n += jump_offset - offset,
//                 Op::SkipIf { n, .. } => *n += jump_offset - offset,
//                 _ => unreachable!(),
//             }
//         }
//     };
//     for (idx, op) in ops.iter().enumerate() {
//         handle_jump_sources(idx, &mut jump_sources, &mut res);
//         let add_jump_source = |n: usize| {
//             jump_sources
//                 .entry(idx + n + 1)
//                 .or_default()
//                 .push((res.len(), jump_offset));
//         };
//         match op {
//             Op::Skip { n } => {
//                 add_jump_source(*n);
//                 res.push(Op::Skip { n: *n });
//             }
//             Op::SkipIf { rs, n } => {
//                 match var_locations.get(rs) {
//                     None =>
//                 }
//                 add_jump_source(*n);
//                 match var_locations
//             }
//             Op::RegLit { .. } => {}
//             Op::GlobalLoad { .. } => {}
//             Op::GlobalStore { .. } => {}
//             Op::BinOp { .. } => {}
//             Op::UnOp { .. } => {}
//             Op::PressedModsInc { .. } => {}
//             Op::PressedModsDec { .. } => {}
//             Op::LatchedModsLoad { .. } => {}
//             Op::LatchedModsStore { .. } => {}
//             Op::LockedModsLoad { .. } => {}
//             Op::LockedModsStore { .. } => {}
//         }
//     }
//     handle_jump_sources(ops.len(), &mut jump_sources, &mut res);
//     struct RegisterStatus {
//         var: Var,
//         hi: usize,
//     }
//     let mut used_registers: StaticMap<Register, Option<RegisterStatus>> = StaticMap::default();
//     let mut spill_slots = vec![];
//     enum VarStatus {}
//     let mut register_ranges = vec![];
//     for (var, lo, hi) in live_ranges {
//         let register = 'get_register: {
//             let mut max_hi = 0;
//             let mut max_hi_register = Register::R0;
//             for (register, status) in &mut used_registers {
//                 match status {
//                     None => break 'get_register register,
//                     Some(s) => {
//                         if s.hi > max_hi {
//                             max_hi = s.hi;
//                             max_hi_register = register;
//                         }
//                     }
//                 }
//             }
//         };
//     }
// }

fn convert_to_ssa(mut next_var: u64, ops: &[Hi]) -> Vec<Hi> {
    let mut predecessors = vec![SmallVec::<[usize; 1]>::new(); ops.len() + 1];
    let mut last_was_skip = true;
    for (idx, op) in ops.iter().enumerate() {
        if !last_was_skip {
            predecessors[idx].push(idx - 1);
        }
        last_was_skip = false;
        match op {
            Hi::Skip { n, .. } if *n > 0 => {
                predecessors[idx + *n + 1].push(idx);
                last_was_skip = true;
            }
            Hi::SkipIf { n, .. } if *n > 0 => {
                predecessors[idx + *n + 1].push(idx);
            }
            _ => {}
        }
    }
    predecessors.pop();
    let mut idom_candidates = MinMaxHeap::new();
    let mut idom = vec![None; ops.len()];
    for (idx, predecessors) in predecessors.iter().enumerate() {
        if predecessors.is_empty() {
            continue;
        }
        if predecessors.len() == 1 {
            idom[idx] = Some(predecessors[0]);
            continue;
        }
        idom_candidates.clear();
        for &predecessor in predecessors {
            if let Some(idom) = idom[predecessor] {
                idom_candidates.push(idom);
            }
        }
        while idom_candidates.len() > 1 {
            if let Some(idom) = idom[idom_candidates.pop_max().unwrap()] {
                idom_candidates.push(idom);
            }
        }
        idom[idx] = idom_candidates.pop_max();
    }
    let mut dominance_frontiers = vec![SmallVec::<[usize; 1]>::new(); ops.len()];
    for (idx, predecessors) in predecessors.iter().enumerate() {
        if predecessors.len() < 2 {
            continue;
        }
        for &predecessor in predecessors {
            let mut runner = predecessor;
            while Some(runner) != idom[idx] {
                if ops[runner].defines_register() {
                    if dominance_frontiers[runner].contains(&idx) {
                        break;
                    }
                    dominance_frontiers[runner].push(idx);
                }
                runner = match idom[runner] {
                    Some(i) => i,
                    _ => break,
                };
            }
        }
    }
    let mut allocate_var = || {
        let var = Var(next_var);
        next_var += 1;
        var
    };
    let mut current_block_arguments = HashSet::new();
    let mut block_arguments = vec![Vec::<(Var, Var)>::new(); ops.len()];
    for (idx, op) in ops.iter().enumerate().rev() {
        match op {
            Hi::Skip { n, .. } => {
                if let Some(ba) = block_arguments.get(idx + *n) {
                    current_block_arguments.extend(ba.iter().map(|b| b.1));
                }
            }
            Hi::SkipIf { rs, n, .. } => {
                if let Some(ba) = block_arguments.get(idx + *n) {
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
            Hi::Phi { .. } => {}
            Hi::PressedModsInc { rs, .. } => {
                current_block_arguments.insert(*rs);
            }
            Hi::PressedModsDec { rs, .. } => {
                current_block_arguments.insert(*rs);
            }
            Hi::LatchedModsLoad { rd, .. } => {
                current_block_arguments.remove(rd);
            }
            Hi::LatchedModsStore { rs, .. } => {
                current_block_arguments.insert(*rs);
            }
            Hi::LockedModsLoad { rd, .. } => {
                current_block_arguments.remove(rd);
            }
            Hi::LockedModsStore { rs, .. } => {
                current_block_arguments.insert(*rs);
            }
        }
        if predecessors[idx].len() > 1 {
            for &arg in &current_block_arguments {
                block_arguments[idx].push((allocate_var(), arg));
            }
            current_block_arguments.clear();
        }
    }
    let mut jump_offset = 0usize;
    let mut res = vec![];
    let mut jump_sources = HashMap::<_, Vec<_>>::new();
    let mut names = HashMap::new();
    let mut phis = vec![HashMap::<Var, Vec<Var>>::new(); ops.len()];
    let handle_jump_sources =
        |idx, jump_sources: &mut HashMap<_, _>, res: &mut Vec<_>, jump_offset: usize| {
            for (idx, offset) in jump_sources.remove(&idx).unwrap_or_default() {
                match &mut res[idx] {
                    Hi::Skip { n, .. } => *n += jump_offset - offset,
                    Hi::SkipIf { n, .. } => *n += jump_offset - offset,
                    _ => unreachable!(),
                }
            }
        };
    let mut last_was_skip = false;
    for (idx, op) in ops.iter().enumerate() {
        if block_arguments[idx].is_not_empty() {
            if !last_was_skip {
                let mut args = SmallVec::new();
                for (dst, src) in &block_arguments[idx] {
                    if let Some(name) = names.get(dst) {
                        args.push((*dst, *name));
                    }
                }
                res.push(Hi::Skip {
                    n: 0,
                    args,
                });
            }
        }
        handle_jump_sources(idx, &mut jump_sources, &mut res, jump_offset);
        for (&orig, new) in &mut phis[idx] {
            let var = allocate_var();
            res.push(Hi::Phi {
                dst: var,
                src: mem::take(new),
            });
            jump_offset += 1;
            names.insert(orig, var);
        }
        let mut add_jump_source = |n: usize| {
            jump_sources
                .entry(idx + n + 1)
                .or_default()
                .push((res.len(), jump_offset));
        };
        let rename_read =
            |rs: &mut Var, names: &HashMap<_, _>| *rs = names.get(rs).copied().unwrap_or(*rs);
        let mut rename_write = |rd: &mut Var, names: &mut HashMap<_, _>| {
            let var = allocate_var();
            let entry = names.entry(*rd);
            for &idx in &dominance_frontiers[idx] {
                let entry = match phis[idx].entry(*rd) {
                    Entry::Occupied(e) => e.into_mut(),
                    Entry::Vacant(e) => {
                        let src = e.insert(vec![]);
                        if let Entry::Occupied(e) = &entry {
                            src.push(*e.get());
                        }
                        src
                    }
                };
                entry.push(var);
            }
            entry.insert(var);
            *rd = var;
        };
        let mut op = op.clone();
        last_was_skip = false;
        match &mut op {
            Hi::Skip { n, .. } => {
                add_jump_source(*n);
                last_was_skip = true;
            }
            Hi::SkipIf { rs, n, .. } => {
                add_jump_source(*n);
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
            Hi::LatchedModsLoad { rd } => {
                rename_write(rd, &mut names);
            }
            Hi::LatchedModsStore { rs } => {
                rename_read(rs, &names);
            }
            Hi::LockedModsLoad { rd } => {
                rename_write(rd, &mut names);
            }
            Hi::LockedModsStore { rs } => {
                rename_read(rs, &names);
            }
            Hi::Phi { .. } => {
                unreachable!();
            }
        }
        res.push(op);
    }
    handle_jump_sources(ops.len(), &mut jump_sources, &mut res, jump_offset);
    res
}
