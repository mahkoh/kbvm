use {
    crate::{
        config::DEFAULT_INCLUDE_DIR,
        xkb::{
            code::Code,
            code_loader::CodeLoader,
            code_map::CodeMap,
            diagnostic::{Diagnostic, DiagnosticSink},
            interner::{Interned, Interner},
            kccgst::{
                self,
                ast::{Include, Item},
                ast_cache::AstCache,
                embedder::embed,
                includer::resolve_includes,
                parser::parse_item,
                resolver::resolve,
            },
            keymap::Keymap,
            meaning::MeaningCache,
            rmlvo::{self, parser::MappingValue, resolver::Group},
            span::{SpanExt, Spanned},
            string_cooker::StringCooker,
        },
    },
    bstr::ByteSlice,
    isnt::std_1::primitive::IsntStrExt,
    std::{
        path::{Path, PathBuf},
        sync::Arc,
    },
};

pub enum MergeMode {
    Augment,
    Override,
}

pub struct Element {
    pub merge_mode: MergeMode,
    pub include: String,
}

#[derive(Default)]
pub struct Kccgst {
    pub keycodes: Vec<Element>,
    pub types: Vec<Element>,
    pub compat: Vec<Element>,
    pub symbols: Vec<Element>,
    pub geometry: Vec<Element>,
}

#[derive(Debug)]
pub struct Context {
    paths: Vec<Arc<PathBuf>>,
    max_includes: u64,
    max_include_depth: u64,
    home: Option<Vec<u8>>,
}

pub struct ContextBuilder {
    enable_system_dirs: bool,
    enable_environment: bool,
    max_includes: u64,
    max_include_depth: u64,
    prefix: Vec<PathBuf>,
    suffix: Vec<PathBuf>,
}

impl ContextBuilder {
    pub fn set_enable_system_dirs(&mut self, val: bool) {
        self.enable_system_dirs = val;
    }

    pub fn set_enable_environment(&mut self, val: bool) {
        self.enable_system_dirs = val;
    }

    pub fn prepend_path(&mut self, path: &(impl AsRef<Path> + ?Sized)) {
        self.prefix.push(path.as_ref().to_path_buf());
    }

    pub fn append_path(&mut self, path: &(impl AsRef<Path> + ?Sized)) {
        self.prefix.push(path.as_ref().to_path_buf());
    }

    pub fn build(mut self) -> Context {
        let mut home = None;
        let mut xdg_config_home = None;
        if self.enable_environment {
            home = std::env::var("HOME").ok();
            xdg_config_home = std::env::var("XDG_CONFIG_HOME").ok();
        }
        let mut paths = vec![];
        macro_rules! push {
            ($path:expr) => {
                paths.push(Arc::new($path.into()))
            };
        }
        while let Some(p) = self.prefix.pop() {
            push!(p);
        }
        if let Some(config) = &xdg_config_home {
            push!(format!("{config}/xkb"));
        } else if let Some(home) = &home {
            push!(format!("{home}/.config/xkb"));
        }
        if let Some(home) = &home {
            push!(format!("{home}/.xkb"));
        }
        if self.enable_system_dirs {
            push!("/etc/xkb".to_string());
            if let Some(did) = DEFAULT_INCLUDE_DIR {
                push!(did);
            }
        }
        for p in self.suffix.drain(..) {
            push!(p);
        }
        Context {
            paths,
            max_includes: self.max_includes,
            max_include_depth: self.max_include_depth,
            home: home.map(|h| h.into_bytes()),
        }
    }
}

pub struct RmlvoGroup<'a> {
    pub layout: &'a str,
    pub variant: &'a str,
}

impl Context {
    pub fn builder() -> ContextBuilder {
        ContextBuilder {
            enable_system_dirs: true,
            enable_environment: false,
            max_includes: 1024,
            max_include_depth: 128,
            prefix: vec![],
            suffix: vec![],
        }
    }

    pub fn create_keymap_from_rmlvo(
        &self,
        diagnostics: &mut DiagnosticSink,
        rules: &str,
        model: &str,
        groups: &[RmlvoGroup<'_>],
        options: &[&str],
    ) -> Keymap {
        let mut map = CodeMap::default();
        let mut interner = Interner::default();
        let mut cooker = StringCooker::default();
        let mut ast_cache = AstCache::default();
        let mut loader = CodeLoader::new(&self.paths);
        let mut meaning_cache = MeaningCache::default();
        let mut item = self.handle_rmlvo(
            diagnostics,
            &mut map,
            &mut interner,
            &mut loader,
            &mut meaning_cache,
            rules,
            model,
            groups,
            options,
            rmlvo::resolver::create_item,
        );
        self.handle_item(
            diagnostics,
            &mut map,
            &mut ast_cache,
            &mut loader,
            &mut interner,
            &mut meaning_cache,
            &mut cooker,
            &mut item,
        )
    }

    pub fn rmlvo_to_kccgst(
        &self,
        diagnostics: &mut DiagnosticSink,
        rules: &str,
        model: &str,
        groups: &[RmlvoGroup<'_>],
        options: &[&str],
    ) -> Kccgst {
        let mut map = CodeMap::default();
        let mut interner = Interner::default();
        let mut loader = CodeLoader::new(&self.paths);
        let mut meaning_cache = MeaningCache::default();
        let includes = self.handle_rmlvo(
            diagnostics,
            &mut map,
            &mut interner,
            &mut loader,
            &mut meaning_cache,
            rules,
            model,
            groups,
            options,
            rmlvo::resolver::create_includes,
        );
        let map = |includes: &[Spanned<Include>]| {
            includes
                .iter()
                .map(|i| Element {
                    merge_mode: match i.val.mm.val {
                        kccgst::MergeMode::Augment => MergeMode::Augment,
                        _ => MergeMode::Override,
                    },
                    include: interner.get(i.val.path.val).as_bstr().to_string(),
                })
                .collect()
        };
        Kccgst {
            keycodes: map(&includes[MappingValue::Keycodes]),
            types: map(&includes[MappingValue::Types]),
            compat: map(&includes[MappingValue::Compat]),
            symbols: map(&includes[MappingValue::Symbols]),
            geometry: map(&includes[MappingValue::Geometry]),
        }
    }

    #[expect(clippy::too_many_arguments)]
    fn handle_rmlvo<T>(
        &self,
        diagnostics: &mut DiagnosticSink,
        map: &mut CodeMap,
        interner: &mut Interner,
        loader: &mut CodeLoader,
        meaning_cache: &mut MeaningCache,
        rules: &str,
        model: &str,
        groups: &[RmlvoGroup<'_>],
        options: &[&str],
        f: impl FnOnce(
            &mut CodeMap,
            &mut Interner,
            &mut CodeLoader,
            &mut DiagnosticSink,
            &mut MeaningCache,
            Spanned<Interned>,
            Option<Interned>,
            &[Interned],
            &[Group],
            u64,
            Option<&[u8]>,
        ) -> T,
    ) -> T {
        let mut intern = |s: &str| {
            let code = Arc::new(s.as_bytes().to_vec());
            let code = Code::new(&code);
            let span = map.add(None, None, &code);
            interner.intern(&code.to_slice()).spanned2(span)
        };
        let rules = intern(rules);
        let model = model.is_not_empty().then(|| intern(model).val);
        let options: Vec<_> = options
            .iter()
            .cloned()
            .filter(|o| o.is_not_empty())
            .map(|o| intern(o).val)
            .collect();
        let groups: Vec<_> = groups
            .iter()
            .map(|group| {
                let layout = group.layout;
                let layout = layout.is_not_empty().then(|| intern(layout).val);
                let variant = group.variant;
                let variant = variant.is_not_empty().then(|| intern(variant).val);
                Group { layout, variant }
            })
            .collect();
        f(
            map,
            interner,
            loader,
            diagnostics,
            meaning_cache,
            rules,
            model,
            &options,
            &groups,
            self.max_includes,
            self.home.as_deref(),
        )
    }

    pub fn parse_keymap(
        &self,
        diagnostics: &mut DiagnosticSink,
        path: Option<&Path>,
        kccgst: &[u8],
    ) -> Result<Keymap, Diagnostic> {
        let mut map = CodeMap::default();
        let mut interner = Interner::default();
        let mut cooker = StringCooker::default();
        let mut ast_cache = AstCache::default();
        let mut loader = CodeLoader::new(&self.paths);
        let mut meaning_cache = MeaningCache::default();
        let code = Code::new(&Arc::new(kccgst.to_vec()));
        let span = map.add(
            path.map(|p| Arc::new(p.to_path_buf())).as_ref(),
            None,
            &code,
        );
        let mut lexer = kccgst::lexer::Lexer::new(None, &code, span.lo);
        let mut tokens = vec![];
        if let Err(e) = lexer.lex_item(&mut interner, &mut tokens) {
            return Err(Diagnostic::new(
                &mut map,
                e.val.diagnostic_kind(),
                e.val,
                e.span,
            ));
        }
        let parsed = parse_item(
            &mut map,
            diagnostics,
            &interner,
            &mut meaning_cache,
            &tokens,
            0,
        );
        tokens.clear();
        let mut parsed = match parsed {
            Ok(p) => p,
            Err(e) => {
                return Err(Diagnostic::new(
                    &mut map,
                    e.val.diagnostic_kind(),
                    e.val,
                    e.span,
                ));
            }
        };
        Ok(self.handle_item(
            diagnostics,
            &mut map,
            &mut ast_cache,
            &mut loader,
            &mut interner,
            &mut meaning_cache,
            &mut cooker,
            &mut parsed.val,
        ))
    }

    #[expect(clippy::too_many_arguments)]
    fn handle_item(
        &self,
        diagnostics: &mut DiagnosticSink<'_, '_>,
        map: &mut CodeMap,
        cache: &mut AstCache,
        loader: &mut CodeLoader,
        interner: &mut Interner,
        meaning_cache: &mut MeaningCache,
        cooker: &mut StringCooker,
        item: &mut Item,
    ) -> Keymap {
        resolve_includes(
            diagnostics,
            map,
            cache,
            loader,
            interner,
            meaning_cache,
            item,
            self.max_includes,
            self.max_include_depth,
        );
        embed(item);
        let resolved = resolve(map, diagnostics, interner, meaning_cache, cooker, item);
        Keymap::from_resolved(interner, &resolved)
    }
}
