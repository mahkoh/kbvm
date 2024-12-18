use crate::xkb::{
    kccgst::ast::{
        CompatmapDecl, Component, ConfigItemType, Decl, Decls, DirectOrIncluded, GeometryDecl,
        Included, Item, ItemType, KeycodeDecl, ResolvedInclude, SymbolsDecl, TypesDecl,
    },
    span::Spanned,
};

pub(crate) fn embed(item: &mut Item) {
    match &mut item.ty {
        ItemType::Composite(e) => {
            for item in &mut e.config_items {
                embed_config_item_type(&mut item.val.item.specific);
            }
        }
        ItemType::Config(e) => {
            embed_config_item_type(e);
        }
    }
}

trait Embeddable: Sized {
    fn embed(decls: &mut DirectOrIncluded<Self>);

    fn unwrap_decls(i: ConfigItemType) -> Vec<Spanned<Decl<Self>>>;
}

macro_rules! s {
    ($($decl:ident, $var:ident;)*) => {
        fn embed_config_item_type(item: &mut ConfigItemType) {
            match item {
                $(
                    ConfigItemType::$var(e) => {
                        for doc in &mut e.decls.decls {
                            <$decl>::embed(&mut doc.val.ty);
                        }
                    },
                )*
            }
        }

        $(
            impl Embeddable for $decl {
                fn embed(decl: &mut DirectOrIncluded<Self>) {
                    if let DirectOrIncluded::Direct(x) = decl {
                        if let $decl::Include(i) = x {
                            if let Some(resolved) = i.resolved.take() {
                                let mut components = vec!();
                                embed_config_item_type2(&mut components, resolved);
                                *decl = DirectOrIncluded::Included(Included {
                                    components,
                                });
                            }
                        }
                    }
                }

                fn unwrap_decls(i: ConfigItemType) -> Vec<Spanned<Decl<Self>>> {
                    match i {
                        ConfigItemType::$var(s) => s.decls.decls,
                        _ => unreachable!(),
                    }
                }
            }
        )*
    };
}

s! {
    KeycodeDecl, Keycodes;
    TypesDecl, Types;
    CompatmapDecl, Compatmap;
    SymbolsDecl, Symbols;
    GeometryDecl, Geometry;
}

fn embed_config_item_type2<T: Embeddable>(dst: &mut Vec<Component<T>>, src: Vec<ResolvedInclude>) {
    for el in src {
        let mut decls = match el.item.val.ty {
            ItemType::Composite(_) => unreachable!(),
            ItemType::Config(c) => T::unwrap_decls(c),
        };
        for decl in &mut decls {
            T::embed(&mut decl.val.ty);
        }
        dst.push(Component {
            mm: el.mm,
            decls: Decls { decls },
            group: el.group,
        });
    }
}
