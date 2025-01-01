use {
    quote::quote,
    syn::{parse_quote, Meta, Path},
};

pub(crate) fn expand(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let mut item = syn::parse_macro_input!(input as syn::ItemEnum);
    let mut cases = vec![];
    for variant in &mut item.variants {
        let mut severity = None;
        variant.attrs.retain(|a| {
            let mut retain = true;
            if let Meta::NameValue(nv) = &a.meta {
                if nv.path.is_ident("severity") {
                    retain = false;
                    let expr = &nv.value;
                    severity = Some(parse_quote!(#expr));
                }
            }
            retain
        });
        let Some(severity) = severity else {
            panic!("no severity attribute found for variant {}", variant.ident,);
        };
        let severity: Path = severity;
        let name = &variant.ident;
        cases.push(quote! {
            Self::#name => Severity::#severity,
        });
        variant.attrs.push(parse_quote!(
            #[doc = ""]
        ));
        variant.attrs.push(parse_quote!(
            #[doc = "# Severity"]
        ));
        variant.attrs.push(parse_quote!(
            #[doc = ""]
        ));
        variant.attrs.push(parse_quote!(
            #[doc = concat!("[`", stringify!(#severity), "`](Severity::", stringify!(#severity), ")")]
        ));
        variant.attrs.push(parse_quote!(
            #[doc = ""]
        ));
    }
    let out = quote! {
        #item

        impl DiagnosticKind {
            fn severity_(self) -> Severity {
                match self {
                    #(#cases)*
                }
            }
        }
    };
    out.into()
}
