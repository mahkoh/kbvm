#[cfg(test)]
mod tests;

use {
    crate::modifier::ModifierMask,
    linearize::{Linearize, StaticMap},
    std::sync::Arc,
};

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd, Linearize)]
pub enum Register {
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

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub(crate) enum Op {
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
    pub(crate) ops: Arc<[Op]>,
}

#[inline]
pub(crate) fn run<H>(
    h: &mut H,
    ops: &[Op],
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
            Op::Skip { n } => {
                i = i.saturating_add(n);
            }
            Op::SkipIf { rs, n } => {
                let s = registers[rs];
                if s != 0 {
                    i = i.saturating_add(n);
                }
            }
            Op::RegLit { rd, lit } => {
                registers[rd] = lit;
            }
            Op::GlobalLoad { rd, g } => {
                registers[rd] = globals[g];
            }
            Op::GlobalStore { rs, g } => {
                globals[g] = registers[rs];
            }
            Op::BinOp { op, rd, rl, rr } => {
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
            Op::UnOp { op, rd, rs } => {
                let s = registers[rs];
                registers[rd] = match op {
                    UnOp::Neg => s.wrapping_neg(),
                    UnOp::BitNot => !s,
                    UnOp::LogNot => (s == 0) as u32,
                }
            }
            Op::PressedModsInc { rs } => {
                let s = registers[rs];
                h.mods_pressed_inc(ModifierMask(s));
            }
            Op::PressedModsDec { rs } => {
                let s = registers[rs];
                h.mods_pressed_dec(ModifierMask(s));
            }
            Op::LatchedModsLoad { rd } => {
                registers[rd] = h.mods_latched_load().0;
            }
            Op::LatchedModsStore { rs } => {
                let s = registers[rs];
                h.mods_latched_store(ModifierMask(s));
            }
            Op::LockedModsLoad { rd } => {
                registers[rd] = h.mods_locked_load().0;
            }
            Op::LockedModsStore { rs } => {
                let s = registers[rs];
                h.mods_locked_store(ModifierMask(s));
            }
        }
        i = i.saturating_add(1);
    }
}

pub struct SkipAnchor {
    cond: Option<Register>,
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

#[derive(Default)]
pub struct RoutineBuilder {
    ops: Vec<Op>,
}

impl Routine {
    pub fn builder() -> RoutineBuilder {
        RoutineBuilder::default()
    }
}

impl RoutineBuilder {
    pub fn build(&self) -> Routine {
        Routine {
            ops: Arc::from(&*self.ops),
        }
    }

    pub fn prepare_skip(&mut self, anchor: &mut SkipAnchor) -> &mut Self {
        let idx = self.ops.len();
        self.ops.push(Op::Skip { n: 0 });
        *anchor = SkipAnchor { cond: None, idx };
        self
    }

    pub fn prepare_conditional_skip(
        &mut self,
        register: Register,
        anchor: &mut SkipAnchor,
    ) -> &mut Self {
        let idx = self.ops.len();
        self.ops.push(Op::Skip { n: 0 });
        *anchor = SkipAnchor {
            cond: Some(register),
            idx,
        };
        self
    }

    pub fn finish_skip(&mut self, anchor: &mut SkipAnchor) -> &mut Self {
        let n = self.ops.len() - anchor.idx - 1;
        let op = match anchor.cond {
            None => Op::Skip { n },
            Some(rs) => Op::SkipIf { rs, n },
        };
        self.ops[anchor.idx] = op;
        *anchor = Default::default();
        self
    }

    pub fn load_lit(&mut self, rd: Register, lit: u32) -> &mut Self {
        self.ops.push(Op::RegLit { rd, lit });
        self
    }

    pub fn load_global(&mut self, rd: Register, g: Global) -> &mut Self {
        self.ops.push(Op::GlobalLoad { rd, g });
        self
    }

    pub fn store_global(&mut self, g: Global, rs: Register) -> &mut Self {
        self.ops.push(Op::GlobalStore { rs, g });
        self
    }

    fn bin_op(&mut self, op: BinOp, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.ops.push(Op::BinOp { op, rd, rl, rr });
        self
    }

    pub fn add(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::Add, rd, rl, rr)
    }

    pub fn sub(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::Sub, rd, rl, rr)
    }

    pub fn mul(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::Mul, rd, rl, rr)
    }

    pub fn udiv(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::Udiv, rd, rl, rr)
    }

    pub fn idiv(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::Idiv, rd, rl, rr)
    }

    pub fn urem(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::Urem, rd, rl, rr)
    }

    pub fn irem(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::Irem, rd, rl, rr)
    }

    pub fn shl(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::Shl, rd, rl, rr)
    }

    pub fn lshr(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::Lshr, rd, rl, rr)
    }

    pub fn ashr(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::Ashr, rd, rl, rr)
    }

    pub fn bit_nand(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::BitNand, rd, rl, rr)
    }

    pub fn bit_and(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::BitAnd, rd, rl, rr)
    }

    pub fn bit_or(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::BitOr, rd, rl, rr)
    }

    pub fn bit_xor(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::BitXor, rd, rl, rr)
    }

    pub fn log_nand(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::LogNand, rd, rl, rr)
    }

    pub fn log_and(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::LogAnd, rd, rl, rr)
    }

    pub fn log_or(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::LogOr, rd, rl, rr)
    }

    pub fn log_xor(&mut self, rd: Register, rl: Register, rr: Register) -> &mut Self {
        self.bin_op(BinOp::LogXor, rd, rl, rr)
    }

    fn un_op(&mut self, op: UnOp, rd: Register, rs: Register) -> &mut Self {
        self.ops.push(Op::UnOp { op, rd, rs });
        self
    }

    pub fn neg(&mut self, rd: Register, rs: Register) -> &mut Self {
        self.un_op(UnOp::Neg, rd, rs)
    }

    pub fn bit_not(&mut self, rd: Register, rs: Register) -> &mut Self {
        self.un_op(UnOp::BitNot, rd, rs)
    }

    pub fn log_not(&mut self, rd: Register, rs: Register) -> &mut Self {
        self.un_op(UnOp::LogNot, rd, rs)
    }

    pub fn pressed_mods_inc(&mut self, rs: Register) -> &mut Self {
        self.ops.push(Op::PressedModsInc { rs });
        self
    }

    pub fn pressed_mods_dec(&mut self, rs: Register) -> &mut Self {
        self.ops.push(Op::PressedModsDec { rs });
        self
    }

    pub fn latched_mods_load(&mut self, rd: Register) -> &mut Self {
        self.ops.push(Op::LatchedModsLoad { rd });
        self
    }

    pub fn latched_mods_store(&mut self, rs: Register) -> &mut Self {
        self.ops.push(Op::LatchedModsStore { rs });
        self
    }

    pub fn locked_mods_load(&mut self, rd: Register) -> &mut Self {
        self.ops.push(Op::LockedModsLoad { rd });
        self
    }

    pub fn locked_mods_store(&mut self, rs: Register) -> &mut Self {
        self.ops.push(Op::LockedModsStore { rs });
        self
    }
}
