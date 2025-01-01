use {
    crate::{
        modifier::ModifierMask,
        routine::{
            convert_to_ssa, run, BinOp, Global, Register, Routine, SkipAnchor, StateEventHandler,
        },
    },
    isnt::std_1::primitive::IsntSliceExt,
    linearize::{Linearize, StaticMap},
    std::mem,
    Global::*,
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
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 1)
        .prepare_skip(&mut anchor)
        .load_lit(r1, 2)
        .finish_skip(&mut anchor)
        .load_lit(r2, 3)
        .build();
    test(&ops, &[1, 0, 3], &[]);
}

#[test]
fn skip_conditional() {
    let mut anchor = SkipAnchor::default();
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 1)
        .prepare_conditional_skip(r1, false, &mut anchor)
        .load_lit(r1, 2)
        .finish_skip(&mut anchor)
        .prepare_conditional_skip(r0, false, &mut anchor)
        .load_lit(r2, 3)
        .finish_skip(&mut anchor)
        .build();
    test(&ops, &[1, 2, 0], &[]);
}

#[test]
fn load_global() {
    let mut builder = Routine::builder();
    let [r0] = builder.allocate_vars();
    let ops = builder.load_lit(r0, 3).store_global(G0, r0).build();
    test(&ops, &[3], &[3]);
}

#[test]
fn store_global() {
    let mut builder = Routine::builder();
    let [r0, r1] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 3)
        .store_global(G0, r0)
        .load_global(r1, G0)
        .build();
    test(&ops, &[3, 3], &[3]);
}

#[test]
fn add() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 1)
        .load_lit(r1, 2)
        .add(r2, r0, r1)
        .build();
    test(&ops, &[1, 2, 3], &[]);
}

#[test]
fn sub() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 1)
        .load_lit(r1, 2)
        .sub(r2, r0, r1)
        .build();
    test(&ops, &[1, 2, !0], &[]);
}

#[test]
fn mul() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 3)
        .load_lit(r1, 7)
        .mul(r2, r0, r1)
        .build();
    test(&ops, &[3, 7, 21], &[]);
}

#[test]
fn udiv() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 3)
        .load_lit(r1, 7)
        .udiv(r2, r1, r0)
        .build();
    test(&ops, &[3, 7, 2], &[]);
}

#[test]
fn udiv_0() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 0)
        .load_lit(r1, 7)
        .udiv(r2, r1, r0)
        .build();
    test(&ops, &[0, 7, 0], &[]);
}

#[test]
fn idiv() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, -2i32 as u32)
        .load_lit(r1, 7)
        .idiv(r2, r1, r0)
        .build();
    test(&ops, &[-2i32 as u32, 7, -3i32 as u32], &[]);
}

#[test]
fn idiv_0() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 0)
        .load_lit(r1, 7)
        .idiv(r2, r1, r0)
        .build();
    test(&ops, &[0, 7, 0], &[]);
}

#[test]
fn idiv_neg_1() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, !0)
        .load_lit(r1, i32::MIN as u32)
        .idiv(r2, r1, r0)
        .build();
    test(&ops, &[!0, i32::MIN as u32, i32::MIN as u32], &[]);
}

#[test]
fn urem() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 3)
        .load_lit(r1, 7)
        .urem(r2, r1, r0)
        .build();
    test(&ops, &[3, 7, 1], &[]);
}

#[test]
fn urem_0() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 0)
        .load_lit(r1, 7)
        .urem(r2, r1, r0)
        .build();
    test(&ops, &[0, 7, 0], &[]);
}

#[test]
fn irem() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 2)
        .load_lit(r1, -7i32 as u32)
        .irem(r2, r1, r0)
        .build();
    test(&ops, &[2, -7i32 as u32, -1i32 as u32], &[]);
}

#[test]
fn irem_0() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 0)
        .load_lit(r1, 7)
        .irem(r2, r1, r0)
        .build();
    test(&ops, &[0, 7, 0], &[]);
}

#[test]
fn irem_neg_1() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, !0)
        .load_lit(r1, i32::MIN as u32)
        .irem(r2, r1, r0)
        .build();
    test(&ops, &[!0, i32::MIN as u32, 0], &[]);
}

#[test]
fn shl() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 3)
        .load_lit(r1, 2)
        .shl(r2, r1, r0)
        .build();
    test(&ops, &[3, 2, 2 << 3], &[]);
}

#[test]
fn lshr() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 7)
        .load_lit(r1, i32::MIN as u32)
        .lshr(r2, r1, r0)
        .build();
    test(&ops, &[7, i32::MIN as u32, 0x01_00_00_00], &[]);
}

#[test]
fn ashr() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 7)
        .load_lit(r1, i32::MIN as u32)
        .ashr(r2, r1, r0)
        .build();
    test(&ops, &[7, i32::MIN as u32, 0xff_00_00_00], &[]);
}

#[test]
fn bit_nand() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 7)
        .load_lit(r1, !0 >> 8)
        .bit_nand(r2, r1, r0)
        .build();
    test(&ops, &[7, !0 >> 8, 0x00_ff_ff_f8], &[]);
}

#[test]
fn bit_and() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 7)
        .load_lit(r1, !0 >> 8)
        .bit_and(r2, r1, r0)
        .build();
    test(&ops, &[7, !0 >> 8, 7], &[]);
}

#[test]
fn bit_or() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 7)
        .load_lit(r1, 0xff_00)
        .bit_or(r2, r1, r0)
        .build();
    test(&ops, &[7, 0xff_00, 0xff_07], &[]);
}

#[test]
fn bit_xor() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 0xff0)
        .load_lit(r1, 0x0ff)
        .bit_xor(r2, r1, r0)
        .build();
    test(&ops, &[0xff0, 0x0ff, 0xf0f], &[]);
}

#[test]
fn log_nand() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3, r4] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 0xff0)
        .load_lit(r1, 0x0ff)
        .log_nand(r2, r1, r0)
        .log_nand(r3, r1, r4)
        .build();
    test(&ops, &[0xff0, 0x0ff, 0, 1], &[]);
}

#[test]
fn log_and() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3, r4] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 0xff0)
        .load_lit(r1, 0x0ff)
        .log_and(r2, r1, r0)
        .log_and(r3, r1, r4)
        .build();
    test(&ops, &[0xff0, 0x0ff, 1, 0], &[]);
}

#[test]
fn log_or() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3, r4] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 0xff0)
        .load_lit(r1, 0x0ff)
        .log_or(r2, r1, r0)
        .log_or(r3, r1, r4)
        .log_or(r4, r4, r4)
        .build();
    test(&ops, &[0xff0, 0x0ff, 1, 1, 0], &[]);
}

#[test]
fn log_xor() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3, r4] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 0xff0)
        .load_lit(r1, 0x0ff)
        .log_xor(r2, r1, r0)
        .log_xor(r3, r1, r4)
        .log_xor(r4, r4, r4)
        .build();
    test(&ops, &[0xff0, 0x0ff, 0, 1, 0], &[]);
}

#[test]
fn neg() {
    let mut builder = Routine::builder();
    let [r0, r1] = builder.allocate_vars();
    let ops = builder.load_lit(r0, 0xff0).neg(r1, r0).build();
    test(&ops, &[0xff0, 0xff0u32.wrapping_neg()], &[]);
}

#[test]
fn bit_not() {
    let mut builder = Routine::builder();
    let [r0, r1] = builder.allocate_vars();
    let ops = builder.load_lit(r0, 0xff0).bit_not(r1, r0).build();
    test(&ops, &[0xff0, !0xff0], &[]);
}

#[test]
fn log_not() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3] = builder.allocate_vars();
    let ops = builder
        .load_lit(r0, 0xff0)
        .load_lit(r1, 0)
        .log_not(r2, r0)
        .log_not(r3, r1)
        .build();
    test(&ops, &[0xff0, 0, 0, 1], &[]);
}

#[test]
fn to_ssa() {
    let mut anchor1 = SkipAnchor::default();
    let mut anchor2 = SkipAnchor::default();
    let mut anchor3 = SkipAnchor::default();
    let mut builder = Routine::builder();
    let action_mods = builder.allocate_var();
    let locked_mods = builder.allocate_var();
    let previously_locked = builder.allocate_var();
    let latched_mods = builder.allocate_var();
    let latch_to_lock = builder.allocate_var();
    let key_version_before = builder.allocate_var();
    let key_version_after = builder.allocate_var();
    let key_version_equal = builder.allocate_var();
    builder
        .load_lit(action_mods, 0b11)
        .prepare_conditional_skip(action_mods, false, &mut anchor2)
        .prepare_skip(&mut anchor3)
        .finish_skip(&mut anchor2)
        .load_lit(action_mods, 0b11)
        .finish_skip(&mut anchor3)
        .pressed_mods_inc(action_mods)
        .load_global(key_version_before, G0)
        .on_release()
        .pressed_mods_dec(action_mods)
        .load_global(key_version_after, G0)
        .bin_op(
            BinOp::Eq,
            key_version_equal,
            key_version_before,
            key_version_after,
        )
        .prepare_conditional_skip(key_version_after, true, &mut anchor1)
        .locked_mods_load(locked_mods)
        .bit_and(previously_locked, locked_mods, action_mods)
        .bit_nand(locked_mods, locked_mods, previously_locked)
        .bit_nand(action_mods, action_mods, previously_locked)
        .latched_mods_load(latched_mods)
        .bit_and(latch_to_lock, latched_mods, action_mods)
        .bit_nand(latched_mods, latched_mods, latch_to_lock)
        .bit_nand(action_mods, action_mods, latch_to_lock)
        .bit_or(locked_mods, locked_mods, latch_to_lock)
        .bit_or(latched_mods, latched_mods, action_mods)
        .locked_mods_store(locked_mods)
        .latched_mods_store(latched_mods)
        .finish_skip(&mut anchor1);
    if builder.ops.is_not_empty() {
        builder.blocks.push(mem::take(&mut builder.ops));
    }
    println!("{:#?}", builder.blocks);
    let ops = convert_to_ssa(builder.next_var, &builder.blocks);
    println!("{:#?}", ops);
}
