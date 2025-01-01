#[cfg(test)]
mod tests;

use {
    crate::modifier::ModifierMask,
    debug_fn::debug_fn,
    hashbrown::{
        hash_map::{Entry, OccupiedEntry},
        DefaultHashBuilder, HashMap, HashSet,
    },
    isnt::std_1::primitive::IsntSliceExt,
    linearize::{Linearize, StaticMap},
    smallvec::SmallVec,
    std::{
        collections::VecDeque,
        fmt::{Debug, Formatter},
        mem,
        ops::Range,
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
    Eq,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub(crate) enum UnOp {
    Move,
    Neg,
    BitNot,
    LogNot,
}

#[derive(Clone, Eq, PartialEq, Hash)]
pub(crate) enum Hi {
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
            Hi::PressedModsInc { rs } => write!(f, "pressed_mods += v{}", rs.0),
            Hi::PressedModsDec { rs } => write!(f, "pressed_mods -= v{}", rs.0),
            Hi::LatchedModsLoad { rd } => write!(f, "v{} = latched_mods", rd.0),
            Hi::LatchedModsStore { rs } => write!(f, "latched_mods = v{}", rs.0),
            Hi::LockedModsLoad { rd } => write!(f, "v{} = locked_mods", rd.0),
            Hi::LockedModsStore { rs } => write!(f, "locked_mods = v{}", rs.0),
        }
    }
}

impl Debug for Lo {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Lo::Skip { n } => {
                write!(f, "skip {n}")
            }
            Lo::SkipIf { rs, not, n } => {
                write!(f, "skip_if{} {rs:?}, {n}", if *not { "_not" } else { "" })
            }
            Lo::Move { rd, rs } => {
                write!(f, "{rd:?} = {rs:?}")
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
                write!(f, "{rd:?} = {lit}")
            }
            Lo::GlobalLoad { rd, g } => {
                write!(f, "{rd:?} = {g:?}")
            }
            Lo::GlobalStore { rs, g } => {
                write!(f, "{g:?} = {rs:?}")
            }
            Lo::BinOp { op, rd, rl, rr } => {
                write!(f, "{rd:?} = {op:?} {rl:?}, {rr:?}")
            }
            Lo::UnOp { op, rd, rs } => {
                write!(f, "{rd:?} = {op:?} {rs:?}")
            }
            Lo::PressedModsInc { rs } => {
                write!(f, "pressed_mods += {rs:?}")
            }
            Lo::PressedModsDec { rs } => {
                write!(f, "pressed_mods -= {rs:?}")
            }
            Lo::LatchedModsLoad { rd } => {
                write!(f, "{rd:?} = latched_mods")
            }
            Lo::LatchedModsStore { rs } => {
                write!(f, "latched_mods = {rs:?}")
            }
            Lo::LockedModsLoad { rd } => {
                write!(f, "{rd:?} = locked_mods")
            }
            Lo::LockedModsStore { rs } => {
                write!(f, "locked_mods = {rs:?}")
            }
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Hash)]
pub(crate) enum Lo {
    // Generics ops
    Skip {
        n: usize,
    },
    SkipIf {
        rs: Register,
        not: bool,
        n: usize,
    },
    Move {
        rd: Register,
        rs: Register,
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
            Hi::Jump { .. }
            | Hi::JumpIf { .. }
            | Hi::GlobalStore { .. }
            | Hi::PressedModsInc { .. }
            | Hi::PressedModsDec { .. }
            | Hi::LatchedModsStore { .. }
            | Hi::LockedModsStore { .. } => false,
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
    pub(crate) on_press: Arc<[Lo]>,
    pub(crate) on_release: Arc<[Lo]>,
    pub(crate) spill: usize,
}

#[inline]
pub(crate) fn run<H>(
    h: &mut H,
    ops: &[Lo],
    registers: &mut StaticMap<Register, u32>,
    globals: &mut StaticMap<Global, u32>,
    spill: &mut [u32],
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
            Lo::SkipIf { rs, not, n } => {
                let s = registers[rs];
                if s != not as u32 {
                    i = i.saturating_add(n);
                }
            }
            Lo::Move { rd, rs } => {
                registers[rd] = registers[rs];
            }
            Lo::SpillMove { src, dst } => {
                spill[dst] = spill[src];
            }
            Lo::SpillLoad { rd, pos } => {
                registers[rd] = spill[pos];
            }
            Lo::SpillStore { rs, pos } => {
                spill[pos] = registers[rs];
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
                    BinOp::Eq => (l == r) as u32,
                };
            }
            Lo::UnOp { op, rd, rs } => {
                let s = registers[rs];
                registers[rd] = match op {
                    UnOp::Move => s,
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
    not: bool,
    block: usize,
    offset: usize,
}

impl Default for SkipAnchor {
    fn default() -> Self {
        Self {
            cond: None,
            not: false,
            block: !0,
            offset: !0,
        }
    }
}

pub struct RoutineBuilder {
    blocks: Vec<Vec<Hi>>,
    ops: Vec<Hi>,
    next_var: u64,
    on_release: Option<usize>,
}

impl Routine {
    pub fn builder() -> RoutineBuilder {
        RoutineBuilder {
            blocks: Default::default(),
            ops: Default::default(),
            next_var: 0,
            on_release: None,
        }
    }
}

impl RoutineBuilder {
    pub fn build(mut self) -> Routine {
        if self.ops.is_not_empty() {
            self.blocks.push(mem::take(&mut self.ops));
        }
        convert_to_ssa(self.next_var, &mut self.blocks);
        let mut allocator = RegisterAllocator::default();
        allocator.allocate_registers(&self.blocks);
        let mut on_press: Vec<_> = allocator.out.into();
        let mut snip = on_press.len();
        if let Some(on_release) = self.on_release {
            snip = on_press.len() - allocator.block_offsets[on_release];
        }
        let on_release = on_press[snip..].to_owned();
        on_press.truncate(snip);
        Routine {
            on_press: on_press.into(),
            on_release: on_release.into(),
            spill: allocator.spill.len(),
        }
    }

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
        let offset = self.ops.len();
        self.ops.push(Hi::Jump {
            to: 0,
            args: Default::default(),
        });
        *anchor = SkipAnchor {
            cond: None,
            not: false,
            block: self.blocks.len(),
            offset,
        };
        self
    }

    pub fn prepare_conditional_skip(
        &mut self,
        var: Var,
        inverse: bool,
        anchor: &mut SkipAnchor,
    ) -> &mut Self {
        let offset = self.ops.len();
        self.ops.push(Hi::Jump {
            to: 0,
            args: Default::default(),
        });
        *anchor = SkipAnchor {
            cond: Some(var),
            not: inverse,
            block: self.blocks.len(),
            offset,
        };
        self
    }

    pub fn finish_skip(&mut self, anchor: &mut SkipAnchor) -> &mut Self {
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

    pub fn move_(&mut self, rd: Var, rs: Var) -> &mut Self {
        self.un_op(UnOp::Move, rd, rs)
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
                Hi::LatchedModsLoad { rd } => {
                    current_block_arguments.remove(rd);
                }
                Hi::LatchedModsStore { rs } => {
                    current_block_arguments.insert(*rs);
                }
                Hi::LockedModsLoad { rd } => {
                    current_block_arguments.remove(rd);
                }
                Hi::LockedModsStore { rs } => {
                    current_block_arguments.insert(*rs);
                }
            }
        }
        for &arg in &current_block_arguments {
            block_arguments[idx].push((Var(0), arg));
        }
        current_block_arguments.clear();
        block_arguments[idx].sort_by_key(|v| v.1 .0);
        for (var, _) in &mut block_arguments[idx] {
            *var = allocate_var();
        }
    }
    let mut names = HashMap::new();
    for (idx, block) in blocks.iter_mut().enumerate() {
        for (dst, src) in &block_arguments[idx] {
            names.insert(*src, *dst);
        }
        for op in block {
            let mut add_jump_source = |to: usize, args: &mut SmallVec<_>| {
                if let Some(ba) = block_arguments.get(to) {
                    for (dst, src) in ba {
                        if let Some(name) = names.get(src) {
                            args.push((*dst, *name));
                        }
                    }
                }
            };
            let rename_read =
                |rs: &mut Var, names: &HashMap<_, _>| *rs = names.get(rs).copied().unwrap_or(*rs);
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
            }
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug)]
enum VariableLocation {
    Register(Register),
    Spilled(usize),
}

#[derive(Default)]
struct RegisterAllocator {
    registers: StaticMap<Register, Option<(Var, usize, usize)>>,
    spill: Vec<bool>,
    variable_uses: HashMap<Var, SmallVec<[usize; 2]>>,
    out: VecDeque<Lo>,
    variable_locations: HashMap<Var, VariableLocation>,
    block_offsets: Vec<usize>,
}

impl RegisterAllocator {
    fn allocate_register(&mut self, idx: usize) -> Register {
        let mut furthest_register = Register::R0;
        let mut furthest_point = usize::MAX;
        for (rs, next_use) in &mut self.registers {
            match next_use {
                None => return rs,
                Some((_, next_use, prev_use)) => {
                    if *prev_use > idx && *next_use < furthest_point {
                        furthest_point = *next_use;
                        furthest_register = rs;
                    }
                }
            }
        }
        let r = furthest_register;
        let var = self.registers[r].unwrap().0;
        let spill = self.spill.iter().position(|v| !*v).unwrap_or_else(|| {
            self.spill.push(false);
            self.spill.len() - 1
        });
        self.spill[spill] = true;
        self.out.push_front(Lo::SpillLoad { rd: r, pos: spill });
        self.variable_locations
            .insert(var, VariableLocation::Spilled(spill));
        r
    }

    fn get_var_register(&mut self, idx: usize, var: &Var) -> Register {
        let next_location = self.variable_locations.get(var).copied();
        let r = match next_location {
            Some(VariableLocation::Register(rs)) => rs,
            Some(VariableLocation::Spilled(pos)) => {
                let r = self.allocate_register(idx);
                self.spill[pos] = false;
                self.out.push_front(Lo::SpillStore { rs: r, pos });
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
                Some((*var, next_use, idx))
            }
        };
        r
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
            match op {
                Hi::Jump { .. } => {}
                Hi::JumpIf { rs, .. } => {
                    use_(rs, true);
                }
                Hi::RegLit { rd, .. } => {
                    use_(rd, false);
                }
                Hi::GlobalLoad { rd, .. } => {
                    use_(rd, false);
                }
                Hi::GlobalStore { rs, .. } => {
                    use_(rs, true);
                }
                Hi::BinOp { rd, rl, rr, .. } => {
                    use_(rl, true);
                    use_(rr, true);
                    use_(rd, false);
                }
                Hi::UnOp { rd, rs, .. } => {
                    use_(rs, true);
                    use_(rd, false);
                }
                Hi::PressedModsInc { rs, .. } => {
                    use_(rs, true);
                }
                Hi::PressedModsDec { rs, .. } => {
                    use_(rs, true);
                }
                Hi::LatchedModsLoad { rd, .. } => {
                    use_(rd, false);
                }
                Hi::LatchedModsStore { rs, .. } => {
                    use_(rs, true);
                }
                Hi::LockedModsLoad { rd, .. } => {
                    use_(rd, false);
                }
                Hi::LockedModsStore { rs, .. } => {
                    use_(rs, true);
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
                self.registers[rs] = Some((
                    src,
                    self.variable_uses
                        .get(&src)
                        .and_then(|v| v.last())
                        .copied()
                        .unwrap_or_default(),
                    usize::MAX,
                ));
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
        self.out.push_front(Lo::SkipIf { rs, not, n });
    }

    fn translate_skip(&mut self, to: &usize, args: &[(Var, Var)]) {
        let n = self.out.len() - self.block_offsets.get(*to).copied().unwrap_or_default();
        if n != 0 {
            self.out.push_front(Lo::Skip { n });
        }
        let mut todo = vec![];
        let mut preserve = HashSet::new();
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
                            self.registers[r] = Some((
                                *src,
                                self.variable_uses
                                    .get(src)
                                    .and_then(|v| v.last())
                                    .copied()
                                    .unwrap_or_default(),
                                usize::MAX,
                            ));
                            v.insert(dst_loc);
                        } else {
                            todo.push((dst_loc, *src));
                        }
                    }
                    VariableLocation::Spilled(pos) => {
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
                    let mut prev_len = self.out.len();
                    self.translate_skip(to, args);
                    if prev_len < self.out.len() {
                        let rs = self.get_var_register(idx, rs);
                        self.encode_skip_if(rs, !*not, self.out.len() - prev_len);
                    }
                }
                Hi::RegLit { rd, lit } => {
                    let rd = self.get_var_register(idx, rd);
                    self.out.push_front(Lo::RegLit { rd, lit: *lit });
                }
                Hi::GlobalLoad { rd, g } => {
                    let rd = self.get_var_register(idx, rd);
                    self.out.push_front(Lo::GlobalLoad { rd, g: *g });
                }
                Hi::GlobalStore { rs, g } => {
                    let rs = self.get_var_register(idx, rs);
                    self.out.push_front(Lo::GlobalStore { rs, g: *g });
                }
                Hi::BinOp { rd, rl, rr, op } => {
                    let rd = self.get_var_register(idx, rd);
                    let rl = self.get_var_register(idx, rl);
                    let rr = self.get_var_register(idx, rr);
                    self.out.push_front(Lo::BinOp {
                        rd,
                        rl,
                        rr,
                        op: *op,
                    });
                }
                Hi::UnOp { rd, rs, op } => {
                    let rd = self.get_var_register(idx, rd);
                    let rs = self.get_var_register(idx, rs);
                    if rd != rs || *op != UnOp::Move {
                        self.out.push_front(Lo::UnOp { rd, rs, op: *op });
                    }
                }
                Hi::PressedModsInc { rs } => {
                    let rs = self.get_var_register(idx, rs);
                    self.out.push_front(Lo::PressedModsInc { rs });
                }
                Hi::PressedModsDec { rs } => {
                    let rs = self.get_var_register(idx, rs);
                    self.out.push_front(Lo::PressedModsDec { rs });
                }
                Hi::LatchedModsLoad { rd } => {
                    let rd = self.get_var_register(idx, rd);
                    self.out.push_front(Lo::LatchedModsLoad { rd });
                }
                Hi::LatchedModsStore { rs } => {
                    let rs = self.get_var_register(idx, rs);
                    self.out.push_front(Lo::LatchedModsStore { rs });
                }
                Hi::LockedModsLoad { rd } => {
                    let rd = self.get_var_register(idx, rd);
                    self.out.push_front(Lo::LockedModsLoad { rd });
                }
                Hi::LockedModsStore { rs } => {
                    let rs = self.get_var_register(idx, rs);
                    self.out.push_front(Lo::LockedModsStore { rs });
                }
            }
        }
    }

    fn allocate_registers(&mut self, blocks: &[Vec<Hi>]) {
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
