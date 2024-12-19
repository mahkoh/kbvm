use {
    quote::quote,
    syn::{
        parse::{Parse, ParseStream},
        Expr, Index, Token, Type,
    },
};

pub(crate) fn expand(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let res = syn::parse_macro_input!(input as Input);
    let msg = &res.msg;
    let types = &res.types;
    let values = &res.values;
    let indices: Vec<_> = (0..values.len()).map(Index::from).collect();
    let derive;
    let fmt;
    match values.is_empty() {
        true => {
            derive = quote! {
                #[derive(Copy, Clone)]
            };
            fmt = quote! {
                f.write_str(#msg)
            };
        }
        false => {
            derive = quote! {};
            fmt = quote! {
                write!(f, #msg #(, self.#indices)*)
            };
        }
    }
    let res = quote! {
        {
            #derive
            struct AdHoc(#(#types),*);
            impl std::fmt::Display for AdHoc {
                fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                    #fmt
                }
            }
            AdHoc(#(#values),*)
        }
    };
    res.into()
}

struct Input {
    msg: Expr,
    values: Vec<Expr>,
    types: Vec<Type>,
}

impl Parse for Input {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let msg: Expr = input.parse()?;
        let mut types = Vec::new();
        let mut values = Vec::new();
        while input.parse::<Token![,]>().is_ok() {
            if input.is_empty() {
                break;
            }
            let val: Expr = input.parse()?;
            input.parse::<Token![=>]>()?;
            let ty: Type = input.parse()?;
            types.push(ty);
            values.push(val);
        }
        Ok(Input { msg, values, types })
    }
}
