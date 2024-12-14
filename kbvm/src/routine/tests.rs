use {
    crate::routine::{
        run, Global, Register, Routine, RoutineBuilder, SkipAnchor, StateEventHandler,
    },
    linearize::{Linearize, StaticMap},
    Global::*,
    Register::*,
};

struct DummyHandler;

impl StateEventHandler for DummyHandler {}

fn test(routine: &Routine, reg: &[u32], global: &[u32]) {
    let mut registers = StaticMap::default();
    let mut globals = StaticMap::default();
    run(
        &mut DummyHandler,
        &routine.ops,
        &mut registers,
        &mut globals,
    );

    let mut r_expected = [0; Register::LENGTH];
    let mut g_expected = [0; Global::LENGTH];
    r_expected[..reg.len()].copy_from_slice(reg);
    g_expected[..global.len()].copy_from_slice(global);

    assert_eq!(registers.0, r_expected);
    assert_eq!(globals.0, g_expected);
}

#[test]
fn skip() {
    let mut anchor = SkipAnchor::default();
    let ops = RoutineBuilder::default()
        .load_lit(R0, 1)
        .prepare_skip(&mut anchor)
        .load_lit(R1, 2)
        .finish_skip(&mut anchor)
        .load_lit(R2, 3)
        .build();
    test(&ops, &[1, 0, 3], &[]);
}

#[test]
fn skip_conditional() {
    let mut anchor = SkipAnchor::default();
    let ops = RoutineBuilder::default()
        .load_lit(R0, 1)
        .prepare_conditional_skip(R1, &mut anchor)
        .load_lit(R1, 2)
        .finish_skip(&mut anchor)
        .prepare_conditional_skip(R0, &mut anchor)
        .load_lit(R2, 3)
        .finish_skip(&mut anchor)
        .build();
    test(&ops, &[1, 2, 0], &[]);
}

#[test]
fn load_global() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 3)
        .store_global(G0, R0)
        .build();
    test(&ops, &[3], &[3]);
}

#[test]
fn store_global() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 3)
        .store_global(G0, R0)
        .load_global(R1, G0)
        .build();
    test(&ops, &[3, 3], &[3]);
}

#[test]
fn add() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 1)
        .load_lit(R1, 2)
        .add(R2, R0, R1)
        .build();
    test(&ops, &[1, 2, 3], &[]);
}

#[test]
fn sub() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 1)
        .load_lit(R1, 2)
        .sub(R2, R0, R1)
        .build();
    test(&ops, &[1, 2, !0], &[]);
}

#[test]
fn mul() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 3)
        .load_lit(R1, 7)
        .mul(R2, R0, R1)
        .build();
    test(&ops, &[3, 7, 21], &[]);
}

#[test]
fn udiv() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 3)
        .load_lit(R1, 7)
        .udiv(R2, R1, R0)
        .build();
    test(&ops, &[3, 7, 2], &[]);
}

#[test]
fn udiv_0() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 0)
        .load_lit(R1, 7)
        .udiv(R2, R1, R0)
        .build();
    test(&ops, &[0, 7, 0], &[]);
}

#[test]
fn idiv() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, -2i32 as u32)
        .load_lit(R1, 7)
        .idiv(R2, R1, R0)
        .build();
    test(&ops, &[-2i32 as u32, 7, -3i32 as u32], &[]);
}

#[test]
fn idiv_0() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 0)
        .load_lit(R1, 7)
        .idiv(R2, R1, R0)
        .build();
    test(&ops, &[0, 7, 0], &[]);
}

#[test]
fn idiv_neg_1() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, !0)
        .load_lit(R1, i32::MIN as u32)
        .idiv(R2, R1, R0)
        .build();
    test(&ops, &[!0, i32::MIN as u32, i32::MIN as u32], &[]);
}

#[test]
fn urem() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 3)
        .load_lit(R1, 7)
        .urem(R2, R1, R0)
        .build();
    test(&ops, &[3, 7, 1], &[]);
}

#[test]
fn urem_0() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 0)
        .load_lit(R1, 7)
        .urem(R2, R1, R0)
        .build();
    test(&ops, &[0, 7, 0], &[]);
}

#[test]
fn irem() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 2)
        .load_lit(R1, -7i32 as u32)
        .irem(R2, R1, R0)
        .build();
    test(&ops, &[2, -7i32 as u32, -1i32 as u32], &[]);
}

#[test]
fn irem_0() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 0)
        .load_lit(R1, 7)
        .irem(R2, R1, R0)
        .build();
    test(&ops, &[0, 7, 0], &[]);
}

#[test]
fn irem_neg_1() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, !0)
        .load_lit(R1, i32::MIN as u32)
        .irem(R2, R1, R0)
        .build();
    test(&ops, &[!0, i32::MIN as u32, 0], &[]);
}

#[test]
fn shl() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 3)
        .load_lit(R1, 2)
        .shl(R2, R1, R0)
        .build();
    test(&ops, &[3, 2, 2 << 3], &[]);
}

#[test]
fn lshr() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 7)
        .load_lit(R1, i32::MIN as u32)
        .lshr(R2, R1, R0)
        .build();
    test(&ops, &[7, i32::MIN as u32, 0x01_00_00_00], &[]);
}

#[test]
fn ashr() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 7)
        .load_lit(R1, i32::MIN as u32)
        .ashr(R2, R1, R0)
        .build();
    test(&ops, &[7, i32::MIN as u32, 0xff_00_00_00], &[]);
}

#[test]
fn bit_nand() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 7)
        .load_lit(R1, !0 >> 8)
        .bit_nand(R2, R1, R0)
        .build();
    test(&ops, &[7, !0 >> 8, 0x00_ff_ff_f8], &[]);
}

#[test]
fn bit_and() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 7)
        .load_lit(R1, !0 >> 8)
        .bit_and(R2, R1, R0)
        .build();
    test(&ops, &[7, !0 >> 8, 7], &[]);
}

#[test]
fn bit_or() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 7)
        .load_lit(R1, 0xff_00)
        .bit_or(R2, R1, R0)
        .build();
    test(&ops, &[7, 0xff_00, 0xff_07], &[]);
}

#[test]
fn bit_xor() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 0xff0)
        .load_lit(R1, 0x0ff)
        .bit_xor(R2, R1, R0)
        .build();
    test(&ops, &[0xff0, 0x0ff, 0xf0f], &[]);
}

#[test]
fn log_nand() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 0xff0)
        .load_lit(R1, 0x0ff)
        .log_nand(R2, R1, R0)
        .log_nand(R3, R1, R4)
        .build();
    test(&ops, &[0xff0, 0x0ff, 0, 1], &[]);
}

#[test]
fn log_and() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 0xff0)
        .load_lit(R1, 0x0ff)
        .log_and(R2, R1, R0)
        .log_and(R3, R1, R4)
        .build();
    test(&ops, &[0xff0, 0x0ff, 1, 0], &[]);
}

#[test]
fn log_or() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 0xff0)
        .load_lit(R1, 0x0ff)
        .log_or(R2, R1, R0)
        .log_or(R3, R1, R4)
        .log_or(R4, R4, R4)
        .build();
    test(&ops, &[0xff0, 0x0ff, 1, 1, 0], &[]);
}

#[test]
fn log_xor() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 0xff0)
        .load_lit(R1, 0x0ff)
        .log_xor(R2, R1, R0)
        .log_xor(R3, R1, R4)
        .log_xor(R4, R4, R4)
        .build();
    test(&ops, &[0xff0, 0x0ff, 0, 1, 0], &[]);
}

#[test]
fn neg() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 0xff0)
        .neg(R1, R0)
        .build();
    test(&ops, &[0xff0, 0xff0u32.wrapping_neg()], &[]);
}

#[test]
fn bit_not() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 0xff0)
        .bit_not(R1, R0)
        .build();
    test(&ops, &[0xff0, !0xff0], &[]);
}

#[test]
fn log_not() {
    let ops = RoutineBuilder::default()
        .load_lit(R0, 0xff0)
        .load_lit(R1, 0)
        .log_not(R2, R0)
        .log_not(R3, R1)
        .build();
    test(&ops, &[0xff0, 0, 0, 1], &[]);
}
