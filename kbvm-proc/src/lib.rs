mod ad_hoc_display;
mod clone_with_delta;
mod diagnostic_kind;

extern crate proc_macro;

#[proc_macro_derive(CloneWithDelta)]
pub fn clone_with_delta(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    clone_with_delta::derive(input)
}

#[proc_macro]
pub fn ad_hoc_display(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    ad_hoc_display::expand(input)
}

#[proc_macro_attribute]
pub fn diagnostic_kind(
    _args: proc_macro::TokenStream,
    input: proc_macro::TokenStream,
) -> proc_macro::TokenStream {
    diagnostic_kind::expand(input)
}
