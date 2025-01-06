use {
    crate::routine::{
        convert_to_ssa, run, Global, Hi, Register, Routine, RoutineBuilder, SkipAnchor,
        StateEventHandler,
    },
    isnt::std_1::vec::IsntVecExt,
    linearize::{Linearize, LinearizeExt, StaticMap},
    std::{array, mem},
    Global::*,
};

struct DummyHandler;

impl StateEventHandler for DummyHandler {}

fn test(builder: RoutineBuilder, global: &[u32]) {
    let routine = builder.build();
    println!("{:#?}", routine);
    let mut registers = StaticMap::default();
    let mut globals = StaticMap::default();
    let mut flags = StaticMap::default();
    let mut spill = vec![0; routine.spill];
    for ops in [&routine.on_press, &routine.on_release] {
        run(
            &mut DummyHandler,
            ops,
            &mut registers,
            &mut globals,
            &mut flags,
            &mut spill,
        );
    }

    let mut g_expected = [0; Global::LENGTH];
    g_expected[..global.len()].copy_from_slice(global);

    assert_eq!(globals.0, g_expected);
}

#[test]
fn skip() {
    let mut anchor = SkipAnchor::default();
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 1)
        .load_lit(r1, 4)
        .prepare_skip(&mut anchor)
        .load_lit(r1, 2)
        .finish_skip(&mut anchor)
        .load_lit(r2, 3)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[1, 4, 3]);
}

#[test]
fn skip_conditional() {
    let mut anchor = SkipAnchor::default();
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 1)
        .load_lit(r1, 0)
        .load_lit(r2, 0)
        .prepare_conditional_skip(r1, false, &mut anchor)
        .load_lit(r1, 2)
        .finish_skip(&mut anchor)
        .prepare_conditional_skip(r0, false, &mut anchor)
        .load_lit(r2, 3)
        .finish_skip(&mut anchor)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[1, 2, 0]);
}

#[test]
fn load_global() {
    let mut builder = Routine::builder();
    let [r0] = builder.allocate_vars();
    builder.load_lit(r0, 3).store_global(G0, r0);
    test(builder, &[3]);
}

#[test]
fn store_global() {
    let mut builder = Routine::builder();
    let [r0, r1] = builder.allocate_vars();
    builder
        .load_lit(r0, 3)
        .store_global(G0, r0)
        .load_global(r1, G0)
        .store_global(G1, r1);
    test(builder, &[3, 3]);
}

#[test]
fn add() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 1)
        .load_lit(r1, 2)
        .add(r2, r0, r1)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[1, 2, 3]);
}

#[test]
fn sub() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 1)
        .load_lit(r1, 2)
        .sub(r2, r0, r1)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[1, 2, !0]);
}

#[test]
fn mul() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 3)
        .load_lit(r1, 7)
        .mul(r2, r0, r1)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[3, 7, 21]);
}

#[test]
fn udiv() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 3)
        .load_lit(r1, 7)
        .udiv(r2, r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[3, 7, 2]);
}

#[test]
fn udiv_0() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 0)
        .load_lit(r1, 7)
        .udiv(r2, r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[0, 7, 0]);
}

#[test]
fn idiv() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, -2i32 as u32)
        .load_lit(r1, 7)
        .idiv(r2, r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[-2i32 as u32, 7, -3i32 as u32]);
}

#[test]
fn idiv_0() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 0)
        .load_lit(r1, 7)
        .idiv(r2, r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[0, 7, 0]);
}

#[test]
fn idiv_neg_1() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, !0)
        .load_lit(r1, i32::MIN as u32)
        .idiv(r2, r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[!0, i32::MIN as u32, i32::MIN as u32]);
}

#[test]
fn urem() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 3)
        .load_lit(r1, 7)
        .urem(r2, r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[3, 7, 1]);
}

#[test]
fn urem_0() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 0)
        .load_lit(r1, 7)
        .urem(r2, r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[0, 7, 0]);
}

#[test]
fn irem() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 2)
        .load_lit(r1, -7i32 as u32)
        .irem(r2, r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[2, -7i32 as u32, -1i32 as u32]);
}

#[test]
fn irem_0() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 0)
        .load_lit(r1, 7)
        .irem(r2, r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[0, 7, 0]);
}

#[test]
fn irem_neg_1() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, !0)
        .load_lit(r1, i32::MIN as u32)
        .irem(r2, r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[!0, i32::MIN as u32, 0]);
}

#[test]
fn shl() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 3)
        .load_lit(r1, 2)
        .shl(r2, r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[3, 2, 2 << 3]);
}

#[test]
fn lshr() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 7)
        .load_lit(r1, i32::MIN as u32)
        .lshr(r2, r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[7, i32::MIN as u32, 0x01_00_00_00]);
}

#[test]
fn ashr() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 7)
        .load_lit(r1, i32::MIN as u32)
        .ashr(r2, r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[7, i32::MIN as u32, 0xff_00_00_00]);
}

#[test]
fn bit_nand() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 7)
        .load_lit(r1, !0 >> 8)
        .bit_nand(r2, r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[7, !0 >> 8, 0x00_ff_ff_f8]);
}

#[test]
fn bit_and() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 7)
        .load_lit(r1, !0 >> 8)
        .bit_and(r2, r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[7, !0 >> 8, 7]);
}

#[test]
fn bit_or() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 7)
        .load_lit(r1, 0xff_00)
        .bit_or(r2, r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[7, 0xff_00, 0xff_07]);
}

#[test]
fn bit_xor() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 0xff0)
        .load_lit(r1, 0x0ff)
        .bit_xor(r2, r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2);
    test(builder, &[0xff0, 0x0ff, 0xf0f]);
}

#[test]
fn log_nand() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3, r4] = builder.allocate_vars();
    builder
        .load_lit(r0, 0xff0)
        .load_lit(r1, 0x0ff)
        .log_nand(r2, r1, r0)
        .log_nand(r3, r1, r4)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2)
        .store_global(G3, r3)
        .store_global(G4, r4);
    test(builder, &[0xff0, 0x0ff, 0, 1]);
}

#[test]
fn log_and() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3, r4] = builder.allocate_vars();
    builder
        .load_lit(r0, 0xff0)
        .load_lit(r1, 0x0ff)
        .log_and(r2, r1, r0)
        .log_and(r3, r1, r4)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2)
        .store_global(G3, r3)
        .store_global(G4, r4);
    test(builder, &[0xff0, 0x0ff, 1, 0]);
}

#[test]
fn log_or() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3, r4] = builder.allocate_vars();
    builder
        .load_lit(r0, 0xff0)
        .load_lit(r1, 0x0ff)
        .log_or(r2, r1, r0)
        .log_or(r3, r1, r4)
        .log_or(r4, r4, r4)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2)
        .store_global(G3, r3)
        .store_global(G4, r4);
    test(builder, &[0xff0, 0x0ff, 1, 1, 0]);
}

#[test]
fn log_xor() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3, r4] = builder.allocate_vars();
    builder
        .load_lit(r0, 0xff0)
        .load_lit(r1, 0x0ff)
        .log_xor(r2, r1, r0)
        .log_xor(r3, r1, r4)
        .log_xor(r4, r4, r4)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2)
        .store_global(G3, r3)
        .store_global(G4, r4);
    test(builder, &[0xff0, 0x0ff, 0, 1, 0]);
}

#[test]
fn neg() {
    let mut builder = Routine::builder();
    let [r0, r1] = builder.allocate_vars();
    builder
        .load_lit(r0, 0xff0)
        .neg(r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1);
    test(builder, &[0xff0, 0xff0u32.wrapping_neg()]);
}

#[test]
fn bit_not() {
    let mut builder = Routine::builder();
    let [r0, r1] = builder.allocate_vars();
    builder
        .load_lit(r0, 0xff0)
        .bit_not(r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1);
    test(builder, &[0xff0, !0xff0]);
}

#[test]
fn move2() {
    let mut builder = Routine::builder();
    let [r0, r1] = builder.allocate_vars();
    builder
        .load_lit(r0, 1)
        .move_(r1, r0)
        .store_global(G0, r0)
        .store_global(G1, r1);
    test(builder, &[1, 1]);
}

#[test]
fn log_not() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3] = builder.allocate_vars();
    builder
        .load_lit(r0, 0xff0)
        .load_lit(r1, 0)
        .log_not(r2, r0)
        .log_not(r3, r1)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2)
        .store_global(G3, r3);
    test(builder, &[0xff0, 0, 0, 1]);
}

#[test]
fn eq() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3] = builder.allocate_vars();
    builder
        .load_lit(r0, 0)
        .load_lit(r1, 1)
        .load_lit(r2, 1)
        .eq(r3, r0, r1)
        .store_global(G0, r3)
        .eq(r3, r0, r2)
        .store_global(G1, r3)
        .eq(r3, r1, r2)
        .store_global(G2, r3);
    test(builder, &[0, 0, 1]);
}

#[test]
fn ne() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3] = builder.allocate_vars();
    builder
        .load_lit(r0, 0)
        .load_lit(r1, 1)
        .load_lit(r2, 1)
        .ne(r3, r0, r1)
        .store_global(G0, r3)
        .ne(r3, r0, r2)
        .store_global(G1, r3)
        .ne(r3, r1, r2)
        .store_global(G2, r3);
    test(builder, &[1, 1, 0]);
}

#[test]
fn ult() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3] = builder.allocate_vars();
    builder
        .load_lit(r0, 0)
        .load_lit(r1, !0)
        .load_lit(r2, !0)
        .ult(r3, r0, r0)
        .store_global(G0, r3)
        .ult(r3, r0, r1)
        .store_global(G1, r3)
        .ult(r3, r0, r2)
        .store_global(G2, r3)
        .ult(r3, r1, r0)
        .store_global(G3, r3)
        .ult(r3, r1, r1)
        .store_global(G4, r3)
        .ult(r3, r1, r2)
        .store_global(G5, r3)
        .ult(r3, r2, r0)
        .store_global(G6, r3)
        .ult(r3, r2, r1)
        .store_global(G7, r3)
        .ult(r3, r2, r2)
        .store_global(G8, r3);
    test(builder, &[0, 1, 1]);
}

#[test]
fn uge() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3] = builder.allocate_vars();
    builder
        .load_lit(r0, 0)
        .load_lit(r1, !0)
        .load_lit(r2, !0)
        .uge(r3, r0, r0)
        .store_global(G0, r3)
        .uge(r3, r0, r1)
        .store_global(G1, r3)
        .uge(r3, r0, r2)
        .store_global(G2, r3)
        .uge(r3, r1, r0)
        .store_global(G3, r3)
        .uge(r3, r1, r1)
        .store_global(G4, r3)
        .uge(r3, r1, r2)
        .store_global(G5, r3)
        .uge(r3, r2, r0)
        .store_global(G6, r3)
        .uge(r3, r2, r1)
        .store_global(G7, r3)
        .uge(r3, r2, r2)
        .store_global(G8, r3);
    test(builder, &[1, 0, 0, 1, 1, 1, 1, 1, 1]);
}

#[test]
fn ule() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3] = builder.allocate_vars();
    builder
        .load_lit(r0, 0)
        .load_lit(r1, !0)
        .load_lit(r2, !0)
        .ule(r3, r0, r0)
        .store_global(G0, r3)
        .ule(r3, r0, r1)
        .store_global(G1, r3)
        .ule(r3, r0, r2)
        .store_global(G2, r3)
        .ule(r3, r1, r0)
        .store_global(G3, r3)
        .ule(r3, r1, r1)
        .store_global(G4, r3)
        .ule(r3, r1, r2)
        .store_global(G5, r3)
        .ule(r3, r2, r0)
        .store_global(G6, r3)
        .ule(r3, r2, r1)
        .store_global(G7, r3)
        .ule(r3, r2, r2)
        .store_global(G8, r3);
    test(builder, &[1, 1, 1, 0, 1, 1, 0, 1, 1]);
}

#[test]
fn ugt() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3] = builder.allocate_vars();
    builder
        .load_lit(r0, 0)
        .load_lit(r1, !0)
        .load_lit(r2, !0)
        .ugt(r3, r0, r0)
        .store_global(G0, r3)
        .ugt(r3, r0, r1)
        .store_global(G1, r3)
        .ugt(r3, r0, r2)
        .store_global(G2, r3)
        .ugt(r3, r1, r0)
        .store_global(G3, r3)
        .ugt(r3, r1, r1)
        .store_global(G4, r3)
        .ugt(r3, r1, r2)
        .store_global(G5, r3)
        .ugt(r3, r2, r0)
        .store_global(G6, r3)
        .ugt(r3, r2, r1)
        .store_global(G7, r3)
        .ugt(r3, r2, r2)
        .store_global(G8, r3);
    test(builder, &[0, 0, 0, 1, 0, 0, 1, 0, 0]);
}

#[test]
fn ilt() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3] = builder.allocate_vars();
    builder
        .load_lit(r0, 0)
        .load_lit(r1, !0)
        .load_lit(r2, !0)
        .ilt(r3, r0, r0)
        .store_global(G0, r3)
        .ilt(r3, r0, r1)
        .store_global(G1, r3)
        .ilt(r3, r0, r2)
        .store_global(G2, r3)
        .ilt(r3, r1, r0)
        .store_global(G3, r3)
        .ilt(r3, r1, r1)
        .store_global(G4, r3)
        .ilt(r3, r1, r2)
        .store_global(G5, r3)
        .ilt(r3, r2, r0)
        .store_global(G6, r3)
        .ilt(r3, r2, r1)
        .store_global(G7, r3)
        .ilt(r3, r2, r2)
        .store_global(G8, r3);
    test(builder, &[0, 0, 0, 1, 0, 0, 1]);
}

#[test]
fn ige() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3] = builder.allocate_vars();
    builder
        .load_lit(r0, 0)
        .load_lit(r1, !0)
        .load_lit(r2, !0)
        .ige(r3, r0, r0)
        .store_global(G0, r3)
        .ige(r3, r0, r1)
        .store_global(G1, r3)
        .ige(r3, r0, r2)
        .store_global(G2, r3)
        .ige(r3, r1, r0)
        .store_global(G3, r3)
        .ige(r3, r1, r1)
        .store_global(G4, r3)
        .ige(r3, r1, r2)
        .store_global(G5, r3)
        .ige(r3, r2, r0)
        .store_global(G6, r3)
        .ige(r3, r2, r1)
        .store_global(G7, r3)
        .ige(r3, r2, r2)
        .store_global(G8, r3);
    test(builder, &[1, 1, 1, 0, 1, 1, 0, 1, 1]);
}

#[test]
fn ile() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3] = builder.allocate_vars();
    builder
        .load_lit(r0, 0)
        .load_lit(r1, !0)
        .load_lit(r2, !0)
        .ile(r3, r0, r0)
        .store_global(G0, r3)
        .ile(r3, r0, r1)
        .store_global(G1, r3)
        .ile(r3, r0, r2)
        .store_global(G2, r3)
        .ile(r3, r1, r0)
        .store_global(G3, r3)
        .ile(r3, r1, r1)
        .store_global(G4, r3)
        .ile(r3, r1, r2)
        .store_global(G5, r3)
        .ile(r3, r2, r0)
        .store_global(G6, r3)
        .ile(r3, r2, r1)
        .store_global(G7, r3)
        .ile(r3, r2, r2)
        .store_global(G8, r3);
    test(builder, &[1, 0, 0, 1, 1, 1, 1, 1, 1]);
}

#[test]
fn igt() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3] = builder.allocate_vars();
    builder
        .load_lit(r0, 0)
        .load_lit(r1, !0)
        .load_lit(r2, !0)
        .igt(r3, r0, r0)
        .store_global(G0, r3)
        .igt(r3, r0, r1)
        .store_global(G1, r3)
        .igt(r3, r0, r2)
        .store_global(G2, r3)
        .igt(r3, r1, r0)
        .store_global(G3, r3)
        .igt(r3, r1, r1)
        .store_global(G4, r3)
        .igt(r3, r1, r2)
        .store_global(G5, r3)
        .igt(r3, r2, r0)
        .store_global(G6, r3)
        .igt(r3, r2, r1)
        .store_global(G7, r3)
        .igt(r3, r2, r2)
        .store_global(G8, r3);
    test(builder, &[0, 1, 1]);
}

#[test]
fn move_() {
    let mut builder = Routine::builder();
    const N: usize = Register::LENGTH + 1;
    let mut anchor1 = SkipAnchor::default();
    let mut anchor2 = SkipAnchor::default();
    let mut anchor3 = SkipAnchor::default();
    let c = builder.allocate_var();
    builder.load_lit(c, 0);
    let v = builder.allocate_vars::<N>();
    for i in 0..N {
        builder.load_lit(v[i], i as u32);
    }
    builder.prepare_conditional_skip(c, true, &mut anchor2);
    {
        for i in 0..N {
            builder.store_global(Global::from_linear(i).unwrap(), v[i]);
        }
        builder.prepare_skip(&mut anchor3);
    }
    builder.finish_skip(&mut anchor2);
    {
        for i in 0..N {
            builder.store_global(Global::from_linear((i + 1) % N).unwrap(), v[i]);
        }
        builder.prepare_skip(&mut anchor1);
    }
    builder.finish_skip(&mut anchor1);
    builder.finish_skip(&mut anchor3);
    test(builder, &[8, 0, 1, 2, 3, 4, 5, 6, 7]);
}

#[test]
fn global_load() {
    let mut builder = Routine::builder();
    let [r0, r1] = builder.allocate_vars();
    builder
        .load_lit(r0, 1)
        .store_global(G0, r0)
        .load_global(r1, G0)
        .store_global(G1, r1)
        .store_global(G2, r0);
    test(builder, &[1, 1, 1]);
}

#[test]
fn global_load2() {
    let mut builder = Routine::builder();
    let mut anchor1 = SkipAnchor::default();
    let [r0, r1] = builder.allocate_vars();
    builder
        .load_lit(r0, 1)
        .store_global(G0, r0)
        .prepare_skip(&mut anchor1)
        .finish_skip(&mut anchor1)
        .load_global(r1, G0)
        .store_global(G1, r1)
        .store_global(G2, r0);
    test(builder, &[1, 1, 1]);
}

#[test]
fn binop() {
    let mut builder = Routine::builder();
    let mut anchor1 = SkipAnchor::default();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 1)
        .load_lit(r2, 1)
        .store_global(G0, r0)
        .prepare_skip(&mut anchor1)
        .finish_skip(&mut anchor1)
        .add(r1, r0, r2)
        .store_global(G1, r1);
    test(builder, &[1, 2]);
}

#[test]
fn neg2() {
    let mut builder = Routine::builder();
    let mut anchor1 = SkipAnchor::default();
    let [r0, r1] = builder.allocate_vars();
    builder
        .load_lit(r0, 1)
        .prepare_skip(&mut anchor1)
        .finish_skip(&mut anchor1)
        .neg(r1, r0)
        .store_global(G0, r1);
    test(builder, &[!0]);
}

#[test]
fn fallthrough() {
    let mut builder = Routine::builder();
    const N: usize = Register::LENGTH + 1;
    let mut anchor1 = SkipAnchor::default();
    let c = builder.allocate_var();
    builder.load_lit(c, 0);
    let v = builder.allocate_vars::<N>();
    for i in 0..N {
        builder.load_lit(v[i], i as u32);
    }
    builder.prepare_conditional_skip(c, false, &mut anchor1);
    builder.store_global(G0, v[0]);
    builder.finish_skip(&mut anchor1);
    for i in 0..N {
        builder.store_global(Global::from_linear((i + 1) % N).unwrap(), v[i]);
    }
    test(builder, &[8, 0, 1, 2, 3, 4, 5, 6, 7]);
}

#[test]
fn double_jump() {
    let mut builder = Routine::builder();
    let mut anchor1 = SkipAnchor::default();
    let [r0, r1, r2, r3] = builder.allocate_vars();
    builder
        .load_lit(r0, 1)
        .prepare_skip(&mut anchor1)
        .load_lit(r1, 1)
        .finish_skip(&mut anchor1)
        .load_lit(r2, 1)
        .prepare_skip(&mut anchor1)
        .load_lit(r3, 1)
        .finish_skip(&mut anchor1)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2)
        .store_global(G3, r3);
    test(builder, &[1, 0, 1, 0]);
}

#[test]
fn double_cond_jump() {
    let mut builder = Routine::builder();
    let mut anchor1 = SkipAnchor::default();
    let [r0, r1, r2, r3, r4] = builder.allocate_vars();
    builder
        .load_lit(r4, 1)
        .load_lit(r0, 1)
        .prepare_conditional_skip(r4, false, &mut anchor1)
        .load_lit(r1, 1)
        .finish_skip(&mut anchor1)
        .load_lit(r2, 1)
        .prepare_conditional_skip(r4, false, &mut anchor1)
        .load_lit(r3, 1)
        .finish_skip(&mut anchor1)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2)
        .store_global(G3, r3);
    test(builder, &[1, 0, 1, 0]);
}

#[test]
fn double_cond_jump2() {
    let mut builder = Routine::builder();
    let mut anchor1 = SkipAnchor::default();
    let [r0, r1, r2, r3, r4, r5] = builder.allocate_vars();
    builder
        .load_lit(r4, 1)
        .load_lit(r5, 0)
        .load_lit(r0, 1)
        .prepare_conditional_skip(r4, false, &mut anchor1)
        .load_lit(r1, 1)
        .finish_skip(&mut anchor1)
        .load_lit(r2, 1)
        .prepare_conditional_skip(r5, false, &mut anchor1)
        .load_lit(r3, 1)
        .finish_skip(&mut anchor1)
        .store_global(G0, r0)
        .store_global(G1, r1)
        .store_global(G2, r2)
        .store_global(G3, r3);
    test(builder, &[1, 0, 1, 1]);
}

#[test]
fn many_uninit() {
    let mut builder = Routine::builder();
    let r = builder.allocate_vars::<{ Register::LENGTH + 1 }>();
    for (idx, r) in r.iter().enumerate() {
        builder.store_global(Global::from_linear(idx).unwrap(), *r);
    }
    test(builder, &[]);
}

fn get_args(f: impl FnOnce(&mut RoutineBuilder)) -> usize {
    let mut builder = Routine::builder();
    let mut anchor1 = SkipAnchor::default();
    builder.prepare_skip(&mut anchor1).finish_skip(&mut anchor1);
    f(&mut builder);
    if builder.ops.is_not_empty() {
        builder.blocks.push(mem::take(&mut builder.ops));
    }
    convert_to_ssa(builder.next_var, &mut builder.blocks);
    println!("{:#?}", builder.blocks);
    let Hi::Jump { args, .. } = &builder.blocks[0][0] else {
        unreachable!();
    };
    args.len()
}

#[test]
fn pressed_mods_inc() {
    let n = get_args(|builder| {
        let v = builder.allocate_var();
        builder.pressed_mods_inc(v);
    });
    assert_eq!(n, 1);
}

#[test]
fn pressed_mods_dec() {
    let n = get_args(|builder| {
        let v = builder.allocate_var();
        builder.pressed_mods_dec(v);
    });
    assert_eq!(n, 1);
}

#[test]
fn component_load() {
    let n = get_args(|builder| {
        let v = builder.allocate_var();
        builder.latched_mods_load(v).store_global(G0, v);
    });
    assert_eq!(n, 0);
}

#[test]
fn component_store() {
    let n = get_args(|builder| {
        let v = builder.allocate_var();
        builder.latched_mods_store(v);
    });
    assert_eq!(n, 1);
}

#[test]
fn flag_load() {
    let n = get_args(|builder| {
        let v = builder.allocate_var();
        builder.later_key_actuated_load(v).store_global(G0, v);
    });
    assert_eq!(n, 0);
}

#[test]
fn key_down() {
    let n = get_args(|builder| {
        let v = builder.allocate_var();
        builder.key_down(v);
    });
    assert_eq!(n, 1);
}

#[test]
fn key_up() {
    let n = get_args(|builder| {
        let v = builder.allocate_var();
        builder.key_up(v);
    });
    assert_eq!(n, 1);
}

#[test]
fn bin_rename() {
    let mut builder = Routine::builder();
    let [r0] = builder.allocate_vars();
    builder.load_lit(r0, 1).add(r0, r0, r0).store_global(G0, r0);
    test(builder, &[2]);
}

#[test]
fn un_rename() {
    let mut builder = Routine::builder();
    let [r0] = builder.allocate_vars();
    builder.load_lit(r0, 1).neg(r0, r0).store_global(G0, r0);
    test(builder, &[!0]);
}

#[test]
fn spill_many() {
    let mut builder = Routine::builder();
    const N: usize = Global::LENGTH;
    let v = builder.allocate_vars::<N>();
    for i in 0..N {
        builder.load_lit(v[i], i as u32);
    }
    for i in 0..N {
        builder.store_global(Global::from_linear(i).unwrap(), v[i]);
    }
    for i in 0..N {
        builder.store_global(Global::from_linear(i).unwrap(), v[i]);
    }
    test(builder, &array::from_fn::<_, N, _>(|n| n as u32));
}

#[test]
fn spill_prefix() {
    /*
       Check that spill is written before the read operation.

       r0 = 0x1
       spill[0] = r0
       r0 = Neg r0
       g1 = r0
       ...
       r0 = spill[0]
       ...
       g0 = r0
    */
    let mut builder = Routine::builder();
    let [a, b] = builder.allocate_vars();
    const N: usize = Register::LENGTH;
    let v = builder.allocate_vars::<N>();
    builder.load_lit(a, 1);
    builder.neg(b, a).store_global(G1, b);
    for i in 0..N {
        builder.load_lit(v[i], 0);
    }
    for i in 0..N {
        builder.store_global(Global::from_linear(i + 2).unwrap(), v[i]);
    }
    builder.store_global(G0, a);
    test(builder, &[1, !0]);
}

#[test]
fn load_spilled_before_skip() {
    /*
       r0 = 0x63
       spill[0] = r0
       ...
       r0 = spill[0]
       skip 9
       ...
       g0 = r0
    */
    let mut builder = Routine::builder();
    let [b] = builder.allocate_vars();
    let mut anchor1 = SkipAnchor::default();
    let mut anchor2 = SkipAnchor::default();
    const N: usize = Register::LENGTH;
    let mut u = builder.allocate_vars::<N>();
    builder.load_lit(b, 99);
    for i in 0..N {
        builder.load_lit(u[i], i as u32);
    }
    builder.prepare_skip(&mut anchor1);
    for i in 0..N {
        builder.store_global(G1, u[i]);
    }
    builder.prepare_skip(&mut anchor2);
    builder.finish_skip(&mut anchor1);
    builder.store_global(G0, b);
    builder.finish_skip(&mut anchor2);
    test(builder, &[99]);
}

#[test]
fn load_spill_to_spill() {
    /*
        r0 = 0x0
        spill[1] = r0
        r0 = 0x1
        spill[9] = r0
        r0 = 0x2
        spill[8] = r0
        r0 = 0x3
        spill[7] = r0
        r0 = 0x4
        spill[6] = r0
        r0 = 0x5
        spill[5] = r0
        r0 = 0x6
        spill[4] = r0
        r0 = 0x7
        spill[3] = r0
        r0 = 0x8
        spill[2] = r0
        r0 = spill[1]
        spill[0] = spill[2]
        r1 = spill[3]
        r2 = spill[4]
        r3 = spill[5]
        r4 = spill[6]
        r5 = spill[7]
        r6 = spill[8]
        r7 = spill[9]
        skip 11
        ...
        g0 = r0
        r0 = spill[0]
        g0 = r7
        g0 = r6
        g0 = r5
        g0 = r4
        g0 = r3
        g0 = r2
        g0 = r1
        g0 = r0
    */
    let mut builder = Routine::builder();
    let mut anchor1 = SkipAnchor::default();
    let mut anchor2 = SkipAnchor::default();
    const N: usize = Register::LENGTH + 1;
    let mut v = builder.allocate_vars::<N>();
    let mut u = builder.allocate_vars::<N>();
    for i in 0..N {
        builder.load_lit(v[i], i as u32);
    }
    for i in 0..N {
        builder.load_lit(u[i], i as u32);
    }
    builder.prepare_skip(&mut anchor1);
    for i in 0..N {
        builder.store_global(G1, u[i]);
    }
    builder.prepare_skip(&mut anchor2);
    builder.finish_skip(&mut anchor1);
    for i in 0..N {
        builder.store_global(G0, v[i]);
    }
    builder.finish_skip(&mut anchor2);
    test(builder, &[N as u32 - 1]);
}
