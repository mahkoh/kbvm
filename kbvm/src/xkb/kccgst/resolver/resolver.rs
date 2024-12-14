use crate::xkb::{
    interner::Interner,
    kccgst::{
        ast::{
            ConfigItemType, Decls, DirectOrIncluded, Item, ItemType, Types, TypesDecl, VModDecl,
            VModDef,
        },
        expr::eval_real_mods,
        meaning::MeaningCache,
        vmodmap::Vmodmap,
    },
    span::{SpanExt, Spanned},
};

struct Resolver<'a> {
    interner: &'a Interner,
    meaning_cache: &'a mut MeaningCache,
    vmods: &'a mut Vmodmap,
}

pub enum ResolverError {
    TooManyVmods,
}

impl Resolver<'_> {
    fn resolve_vmods(&mut self, e: &mut Item) -> Result<(), Spanned<ResolverError>> {
        match e.ty {
            ItemType::Composite(_) => {}
            ItemType::Config(_) => {}
        }
    }

    fn resolve_vmods_config(
        &mut self,
        e: &mut ConfigItemType,
    ) -> Result<(), Spanned<ResolverError>> {
        match e {
            ConfigItemType::Keycodes(_) => Ok(()),
            ConfigItemType::Types(e) => self.resolve_vmods_types(e),
            ConfigItemType::Compatmap(e) => {}
            ConfigItemType::Symbols(e) => {}
            ConfigItemType::Geometry(_) => Ok(()),
        }
    }

    fn resolve_vmods_types(&mut self, e: &mut Types) -> Result<(), Spanned<ResolverError>> {
        self.resolve_decls(&mut e.decls, &mut |slf, d| slf.resolve_vmods_types_decl(d))
    }

    fn resolve_vmods_types_decl(
        &mut self,
        e: &mut TypesDecl,
    ) -> Result<(), Spanned<ResolverError>> {
        if let TypesDecl::VMod(e) = e {
            self.resolve_vmod_decl(e)?;
        }
        Ok(())
    }

    fn resolve_vmod_decl(&mut self, e: &mut VModDecl) -> Result<(), Spanned<ResolverError>> {
        for e in &mut e.defs {
            self.resolve_vmod_def(&mut e.val)?;
        }
        Ok(())
    }

    fn resolve_vmod_def(&mut self, e: &mut VModDef) -> Result<(), Spanned<ResolverError>> {
        let mods = match e.val.as_ref() {
            Some(e) => Some(
                eval_real_mods(self.interner, self.meaning_cache, e.as_ref())?.spanned2(e.span),
            ),
            _ => None,
        };
        let (vmod, new) = match self.vmods.insert(e.name.val) {
            Some(r) => r,
            _ => return Err(ResolverError::TooManyVmods.spanned2(e.name.span)),
        };
        if new {
            vmod.def = mods;
        } else {
        }
        Ok(())
    }

    fn resolve_decls<T>(
        &mut self,
        e: &mut Decls<T>,
        f: &mut impl FnMut(&mut Self, &mut T) -> Result<(), Spanned<ResolverError>>,
    ) -> Result<(), Spanned<ResolverError>> {
        for decl in &mut e.decls {
            match &mut decl.val.ty {
                DirectOrIncluded::Direct(t) => f(self, t)?,
                DirectOrIncluded::Included(i) => {
                    for c in &mut i.components {
                        self.resolve_decls(&mut c.decls, f)?;
                    }
                }
            }
        }
        Ok(())
    }
}
