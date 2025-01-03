#![expect(clippy::collapsible_else_if, clippy::single_char_add_str)]

use {crate::phf::PhfHash, permutation::Permutation, std::fmt::Debug};

mod keysyms;
#[path = "../../kbvm/src/phf.rs"]
mod phf;
mod phf_generator;
mod type_tests;
mod xkb;

fn main() {
    keysyms::main();
    xkb::main();
    type_tests::main();
}

fn generate_map(
    name: &str,
    key_type: &str,
    value_type: &str,
    keys: &[impl PhfHash],
    values: &mut [impl Debug],
) -> String {
    use std::fmt::Write;
    let state = phf_generator::generate_hash(keys);
    Permutation::oneline(state.map).apply_inv_slice_in_place(values);
    let mut res = String::new();
    writeln!(
        res,
        "pub(super) static {name}: PhfMap<{key_type}, {value_type}> = PhfMap {{"
    )
    .unwrap();
    writeln!(res, "    key: {},", state.key).unwrap();
    writeln!(res, "    disps: &{:?},", state.disps).unwrap();
    writeln!(res, "    map: &{:?},", values).unwrap();
    writeln!(res, "    _phantom: core::marker::PhantomData,").unwrap();
    writeln!(res, "}};").unwrap();
    res
}
