use {
    proc_macro2::Ident,
    quote::quote,
    syn::{parse_quote, spanned::Spanned, Data, GenericParam, Index},
};

pub(crate) fn derive(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let mut item = syn::parse_macro_input!(input as syn::DeriveInput);
    let body = match &item.data {
        Data::Struct(s) => {
            let mut fields = vec![];
            for (idx, field) in s.fields.iter().enumerate() {
                let idx = Index::from(idx);
                let name = match &field.ident {
                    Some(id) => quote! { #id },
                    _ => quote! { #idx },
                };
                let ty = &field.ty;
                fields.push(quote! {
                    #name: <#ty as crate::xkb::clone_with_delta::CloneWithDelta>::clone_with_delta(&self.#name, delta),
                });
            }
            quote! {
                Self {
                    #(#fields)*
                }
            }
        }
        Data::Enum(e) => {
            let mut variants = vec![];
            for variant in &e.variants {
                let mut keys = vec![];
                let mut fields = vec![];
                for (idx, field) in variant.fields.iter().enumerate() {
                    let (name, ref_name) = match &field.ident {
                        Some(id) => (quote! { #id }, quote! { #id }),
                        _ => {
                            let ref_name = Ident::new(&format!("_{}", idx), field.ty.span());
                            let idx = Index::from(idx);
                            (quote! { #idx }, quote! { #ref_name })
                        }
                    };
                    let ty = &field.ty;
                    fields.push(quote! {
                        #name: <#ty as crate::xkb::clone_with_delta::CloneWithDelta>::clone_with_delta(#ref_name, delta),
                    });
                    keys.push(quote! {
                        #name: #ref_name,
                    });
                }
                let name = &variant.ident;
                variants.push(quote! {
                    Self::#name { #(#keys)* } => Self::#name { #(#fields)* },
                });
            }
            quote! {
                match self {
                    #(#variants)*
                }
            }
        }
        Data::Union(_) => unreachable!(),
    };
    let ident = item.ident;
    item.generics.make_where_clause();
    let where_clause = item.generics.where_clause.as_mut().unwrap();
    for field_ty in &item.generics.params {
        if let GenericParam::Type(ty) = field_ty {
            let ident = &ty.ident;
            where_clause.predicates.push(parse_quote! {
                #ident: crate::xkb::clone_with_delta::CloneWithDelta
            });
        }
    }
    let (impl_generics, type_generics, where_clause) = item.generics.split_for_impl();
    let tokens = quote! {
        impl #impl_generics crate::xkb::clone_with_delta::CloneWithDelta for #ident #type_generics #where_clause {
            fn clone_with_delta(&self, delta: u64) -> Self {
                #body
            }
        }
    };
    tokens.into()
}
