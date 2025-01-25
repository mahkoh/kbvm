use {
    crate::xkb::{
        code_loader::{CodeLoader, CodeType},
        code_map::CodeMap,
        diagnostic::{DiagnosticKind, DiagnosticSink},
        include::parse_include,
        interner::{Interned, Interner},
        kccgst::{
            ast::{
                Compat, CompatmapDecl, ConfigItemType, DirectOrIncluded, Geometry, GeometryDecl,
                Include, Item, ItemType, KeycodeDecl, Keycodes, LoadedInclude, Symbols,
                SymbolsDecl, Types, TypesDecl,
            },
            ast_cache::AstCache,
        },
        meaning::MeaningCache,
        span::SpanExt,
    },
    hashbrown::HashSet,
    kbvm_proc::ad_hoc_display,
};

#[expect(clippy::too_many_arguments)]
pub(crate) fn resolve_includes(
    diagnostics: &mut DiagnosticSink<'_, '_>,
    map: &mut CodeMap,
    cache: &mut AstCache,
    loader: &mut CodeLoader,
    interner: &mut Interner,
    meaning_cache: &mut MeaningCache,
    remaining_runtime: &mut u64,
    item: &mut Item,
    max_includes: u64,
    max_include_depth: u64,
) {
    let mut includer = Includer {
        diagnostics,
        map,
        cache,
        loader,
        meaning_cache,
        remaining_runtime,
        active_includes: Default::default(),
        num_includes: 0,
        max_includes,
        include_depth: 0,
        max_include_depth,
    };
    includer.process_includes(interner, item);
}

struct Includer<'a, 'b, 'c> {
    diagnostics: &'a mut DiagnosticSink<'b, 'c>,
    map: &'a mut CodeMap,
    cache: &'a mut AstCache,
    loader: &'a mut CodeLoader,
    meaning_cache: &'a mut MeaningCache,
    remaining_runtime: &'a mut u64,
    active_includes: HashSet<(CodeType, Interned, Option<Interned>)>,
    num_includes: u64,
    max_includes: u64,
    include_depth: u64,
    max_include_depth: u64,
}

macro_rules! process_ct {
    ($fn:ident, $ty:ident, $decl:ident, $ct:ident) => {
        fn $fn(&mut self, interner: &mut Interner, item: &mut $ty) {
            for decl in &mut item.decls.decls {
                if let DirectOrIncluded::Direct($decl::Include(i)) = &mut decl.val.ty {
                    self.process_include(interner, i, CodeType::$ct);
                }
            }
        }
    };
}

impl Includer<'_, '_, '_> {
    fn process_includes(&mut self, interner: &mut Interner, item: &mut Item) {
        self.include_depth += 1;
        match &mut item.ty {
            ItemType::Composite(c) => {
                for i in &mut c.config_items {
                    self.process_config_item_type(interner, &mut i.val.item.specific);
                }
            }
            ItemType::Config(c) => self.process_config_item_type(interner, c),
        }
        self.include_depth -= 1;
    }

    fn process_config_item_type(&mut self, interner: &mut Interner, ty: &mut ConfigItemType) {
        match ty {
            ConfigItemType::Keycodes(e) => self.process_keycodes_includes(interner, e),
            ConfigItemType::Types(e) => self.process_types_includes(interner, e),
            ConfigItemType::Compat(e) => self.process_compat_includes(interner, e),
            ConfigItemType::Symbols(e) => self.process_symbols_includes(interner, e),
            ConfigItemType::Geometry(e) => self.process_geometry_includes(interner, e),
        }
    }

    process_ct!(process_keycodes_includes, Keycodes, KeycodeDecl, Keycodes);
    process_ct!(process_types_includes, Types, TypesDecl, Types);
    process_ct!(process_compat_includes, Compat, CompatmapDecl, Compat);
    process_ct!(process_symbols_includes, Symbols, SymbolsDecl, Symbols);
    process_ct!(process_geometry_includes, Geometry, GeometryDecl, Geometry);

    fn process_include(&mut self, interner: &mut Interner, include: &mut Include, ty: CodeType) {
        if self.include_depth >= self.max_include_depth {
            self.diagnostics.push(
                self.map,
                DiagnosticKind::MaxIncludeDepthReached,
                ad_hoc_display!("maximum include depth ({}) reached", self.max_include_depth => u64).spanned2(include.path.span),
            );
            return;
        }
        let mut resolved = vec![];
        let mut iter = parse_include(interner, include.path);
        while let Some(i) = iter.next() {
            let i = match i {
                Ok(i) => i,
                Err(e) => {
                    self.diagnostics.push(self.map, e.val.diagnostic_kind(), e);
                    continue;
                }
            };
            self.num_includes += 1;
            if self.num_includes > self.max_includes {
                self.diagnostics.push(
                    self.map,
                    DiagnosticKind::MaxIncludesReached,
                    ad_hoc_display!("maximum number of includes reached").spanned2(i.file.span),
                );
                break;
            }
            let mut span = i.file.span;
            if let Some(map) = i.map {
                span.hi = map.span.hi + 1;
            }
            let key = (ty, i.file.val, i.map.map(|m| m.val));
            if !self.active_includes.insert(key) {
                self.diagnostics.push(
                    self.map,
                    DiagnosticKind::RecursiveInclude,
                    ad_hoc_display!("ignoring recursive include").spanned2(span),
                );
                continue;
            }
            let res = self.cache.get(
                self.diagnostics,
                self.map,
                self.loader,
                iter.interner(),
                self.meaning_cache,
                self.remaining_runtime,
                ty,
                i.file.val,
                i.map.map(|m| m.val),
                span,
            );
            match res {
                Ok(mut item) => {
                    self.process_includes(iter.interner(), &mut item.val);
                    resolved.push(LoadedInclude {
                        mm: i.merge_mode,
                        group: i.group,
                        item,
                    });
                }
                Err(e) => {
                    self.diagnostics.push(self.map, e.val.diagnostic_kind(), e);
                }
            }
            self.active_includes.remove(&key);
        }
        include.loaded = Some(resolved.into_boxed_slice());
    }
}
