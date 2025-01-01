use {
    crate::routine::{
        convert_to_ssa, run, Global, Register, RegisterAllocator, Routine, RoutineBuilder,
        SkipAnchor, StateEventHandler,
    },
    isnt::std_1::primitive::IsntSliceExt,
    linearize::{Linearize, StaticMap},
    std::mem,
    Global::*,
};

struct DummyHandler;

impl StateEventHandler for DummyHandler {}

fn test(builder: RoutineBuilder, reg: &[u32], global: &[u32]) {
    let routine = builder.build();
    println!("{:#?}", routine.on_press);
    let mut registers = StaticMap::default();
    let mut globals = StaticMap::default();
    let mut spill = vec![0; routine.spill];
    for ops in [&routine.on_press, &routine.on_release] {
        run(
            &mut DummyHandler,
            ops,
            &mut registers,
            &mut globals,
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
        .prepare_skip(&mut anchor)
        .load_lit(r1, 2)
        .finish_skip(&mut anchor)
        .load_lit(r2, 3);
    test(builder, &[1, 0, 3], &[]);
}

#[test]
fn skip_conditional() {
    let mut anchor = SkipAnchor::default();
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 1)
        .prepare_conditional_skip(r1, false, &mut anchor)
        .load_lit(r1, 2)
        .finish_skip(&mut anchor)
        .prepare_conditional_skip(r0, false, &mut anchor)
        .load_lit(r2, 3)
        .finish_skip(&mut anchor);
    test(builder, &[1, 2, 0], &[]);
}

#[test]
fn load_global() {
    let mut builder = Routine::builder();
    let [r0] = builder.allocate_vars();
    builder.load_lit(r0, 3).store_global(G0, r0);
    test(builder, &[3], &[3]);
}

#[test]
fn store_global() {
    let mut builder = Routine::builder();
    let [r0, r1] = builder.allocate_vars();
    builder
        .load_lit(r0, 3)
        .store_global(G0, r0)
        .load_global(r1, G0);
    test(builder, &[3, 3], &[3]);
}

#[test]
fn add() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder.load_lit(r0, 1).load_lit(r1, 2).add(r2, r0, r1);
    test(builder, &[1, 2, 3], &[]);
}

#[test]
fn sub() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder.load_lit(r0, 1).load_lit(r1, 2).sub(r2, r0, r1);
    test(builder, &[1, 2, !0], &[]);
}

#[test]
fn mul() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder.load_lit(r0, 3).load_lit(r1, 7).mul(r2, r0, r1);
    test(builder, &[3, 7, 21], &[]);
}

#[test]
fn udiv() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder.load_lit(r0, 3).load_lit(r1, 7).udiv(r2, r1, r0);
    test(builder, &[3, 7, 2], &[]);
}

#[test]
fn udiv_0() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder.load_lit(r0, 0).load_lit(r1, 7).udiv(r2, r1, r0);
    test(builder, &[0, 7, 0], &[]);
}

#[test]
fn idiv() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, -2i32 as u32)
        .load_lit(r1, 7)
        .idiv(r2, r1, r0);
    test(builder, &[-2i32 as u32, 7, -3i32 as u32], &[]);
}

#[test]
fn idiv_0() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder.load_lit(r0, 0).load_lit(r1, 7).idiv(r2, r1, r0);
    test(builder, &[0, 7, 0], &[]);
}

#[test]
fn idiv_neg_1() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, !0)
        .load_lit(r1, i32::MIN as u32)
        .idiv(r2, r1, r0);
    test(builder, &[!0, i32::MIN as u32, i32::MIN as u32], &[]);
}

#[test]
fn urem() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder.load_lit(r0, 3).load_lit(r1, 7).urem(r2, r1, r0);
    test(builder, &[3, 7, 1], &[]);
}

#[test]
fn urem_0() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder.load_lit(r0, 0).load_lit(r1, 7).urem(r2, r1, r0);
    test(builder, &[0, 7, 0], &[]);
}

#[test]
fn irem() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 2)
        .load_lit(r1, -7i32 as u32)
        .irem(r2, r1, r0);
    test(builder, &[2, -7i32 as u32, -1i32 as u32], &[]);
}

#[test]
fn irem_0() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder.load_lit(r0, 0).load_lit(r1, 7).irem(r2, r1, r0);
    test(builder, &[0, 7, 0], &[]);
}

#[test]
fn irem_neg_1() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, !0)
        .load_lit(r1, i32::MIN as u32)
        .irem(r2, r1, r0);
    test(builder, &[!0, i32::MIN as u32, 0], &[]);
}

#[test]
fn shl() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder.load_lit(r0, 3).load_lit(r1, 2).shl(r2, r1, r0);
    test(builder, &[3, 2, 2 << 3], &[]);
}

#[test]
fn lshr() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 7)
        .load_lit(r1, i32::MIN as u32)
        .lshr(r2, r1, r0);
    test(builder, &[7, i32::MIN as u32, 0x01_00_00_00], &[]);
}

#[test]
fn ashr() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 7)
        .load_lit(r1, i32::MIN as u32)
        .ashr(r2, r1, r0);
    test(builder, &[7, i32::MIN as u32, 0xff_00_00_00], &[]);
}

#[test]
fn bit_nand() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 7)
        .load_lit(r1, !0 >> 8)
        .bit_nand(r2, r1, r0);
    test(builder, &[7, !0 >> 8, 0x00_ff_ff_f8], &[]);
}

#[test]
fn bit_and() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 7)
        .load_lit(r1, !0 >> 8)
        .bit_and(r2, r1, r0);
    test(builder, &[7, !0 >> 8, 7], &[]);
}

#[test]
fn bit_or() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 7)
        .load_lit(r1, 0xff_00)
        .bit_or(r2, r1, r0);
    test(builder, &[7, 0xff_00, 0xff_07], &[]);
}

#[test]
fn bit_xor() {
    let mut builder = Routine::builder();
    let [r0, r1, r2] = builder.allocate_vars();
    builder
        .load_lit(r0, 0xff0)
        .load_lit(r1, 0x0ff)
        .bit_xor(r2, r1, r0);
    test(builder, &[0xff0, 0x0ff, 0xf0f], &[]);
}

#[test]
fn log_nand() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3, r4] = builder.allocate_vars();
    builder
        .load_lit(r0, 0xff0)
        .load_lit(r1, 0x0ff)
        .log_nand(r2, r1, r0)
        .log_nand(r3, r1, r4);
    test(builder, &[0xff0, 0x0ff, 0, 1], &[]);
}

#[test]
fn log_and() {
    let mut builder = Routine::builder();
    let [r0, r1, r2, r3, r4] = builder.allocate_vars();
    builder
        .load_lit(r0, 0xff0)
        .load_lit(r1, 0x0ff)
        .log_and(r2, r1, r0)
        .log_and(r3, r1, r4);
    test(builder, &[0xff0, 0x0ff, 1, 0], &[]);
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
        .log_or(r4, r4, r4);
    test(builder, &[0xff0, 0x0ff, 1, 1, 0], &[]);
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
        .log_xor(r4, r4, r4);
    test(builder, &[0xff0, 0x0ff, 0, 1, 0], &[]);
}

#[test]
fn neg() {
    let mut builder = Routine::builder();
    let [r0, r1] = builder.allocate_vars();
    builder.load_lit(r0, 0xff0).neg(r1, r0);
    test(builder, &[0xff0, 0xff0u32.wrapping_neg()], &[]);
}

#[test]
fn bit_not() {
    let mut builder = Routine::builder();
    let [r0, r1] = builder.allocate_vars();
    builder.load_lit(r0, 0xff0).bit_not(r1, r0);
    test(builder, &[0xff0, !0xff0], &[]);
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
        .store_global(G0, r2)
        .store_global(G1, r3)
    ;
    test(builder, &[], &[0, 1]);
}

#[test]
fn to_ssa() {
    let mut anchor1 = SkipAnchor::default();
    let mut anchor2 = SkipAnchor::default();
    let mut anchor3 = SkipAnchor::default();
    let mut builder = Routine::builder();
    let undef = builder.allocate_var();
    let action_mods = builder.allocate_var();
    let locked_mods = builder.allocate_var();
    let previously_locked = builder.allocate_var();
    let latched_mods = builder.allocate_var();
    let latch_to_lock = builder.allocate_var();
    let key_version_before = builder.allocate_var();
    let key_version_after = builder.allocate_var();
    let key_version_equal = builder.allocate_var();
    // const N: usize = 8;
    // let v = builder.allocate_vars::<N>();
    // builder.load_lit(v[0], 0);
    // builder.load_lit(v[1], 1);
    // builder.prepare_conditional_skip(v[2], false, &mut anchor1);
    // builder.move_(v[2], v[0]);
    // builder.move_(v[0], v[1]);
    // builder.move_(v[1], v[2]);
    // builder.finish_skip(&mut anchor1);
    // builder.on_release();
    // builder.store_global(G0, v[0]);
    // builder.store_global(G1, v[1]);
    builder
        .load_lit(action_mods, 0b11)
        .pressed_mods_inc(action_mods)
        .on_release()
        .pressed_mods_dec(action_mods)
        .load_global(key_version_after, G0)
        .prepare_conditional_skip(key_version_after, false, &mut anchor1)
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
    convert_to_ssa(builder.next_var, &mut builder.blocks);
    println!("{:#?}", builder.blocks);
    let mut allocator = RegisterAllocator::default();
    allocator.allocate_registers(&builder.blocks);
    let mut on_press: Vec<_> = allocator.out.into();
    let snip = on_press.len() - allocator.block_offsets[builder.on_release.unwrap()];
    let on_release = on_press[snip..].to_owned();
    on_press.truncate(snip);
    println!("{:#?}", on_press);
    println!("{:#?}", on_release);
}
