use {
    crate::{
        config::DEFAULT_INCLUDE_DIR,
        xkb::{
            code::Code,
            code_loader::CodeLoader,
            code_map::CodeMap,
            diagnostic::{Diagnostic, DiagnosticHandler, DiagnosticSink},
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
            rmlvo::{self, parser::MappingValue, resolver::Group, Element},
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

///
#[derive(Clone, Debug)]
pub struct Context {
    paths: Vec<Arc<PathBuf>>,
    max_includes: u64,
    max_include_depth: u64,
    env: Environment,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct Environment {
    pub(crate) home: Option<String>,
    pub(crate) xlocaledir: String,
    pub(crate) xcomposefile: Option<String>,
    pub(crate) xkb_default_rules: String,
    pub(crate) xkb_default_model: String,
    pub(crate) xkb_default_layout: String,
    pub(crate) xkb_default_variant: String,
    pub(crate) xkb_default_options: String,
    pub(crate) xkb_config_extra_path: String,
    pub(crate) xkb_config_root: String,
}

pub struct ContextBuilder {
    enable_system_dirs: bool,
    enable_environment: bool,
    max_includes: u64,
    max_include_depth: u64,
    prefix: Vec<PathBuf>,
    suffix: Vec<PathBuf>,
}

impl Default for ContextBuilder {
    fn default() -> Self {
        Self {
            enable_system_dirs: true,
            enable_environment: false,
            max_includes: 1024,
            max_include_depth: 128,
            prefix: vec![],
            suffix: vec![],
        }
    }
}

impl ContextBuilder {
    pub fn enable_system_dirs(&mut self, val: bool) {
        self.enable_system_dirs = val;
    }

    pub fn enable_environment(&mut self, val: bool) {
        self.enable_environment = val;
    }

    pub fn prepend_path(&mut self, path: &(impl AsRef<Path> + ?Sized)) {
        self.prefix.push(path.as_ref().to_path_buf());
    }

    pub fn append_path(&mut self, path: &(impl AsRef<Path> + ?Sized)) {
        self.prefix.push(path.as_ref().to_path_buf());
    }

    pub fn max_includes(&mut self, val: u64) {
        self.max_includes = val;
    }

    pub fn max_include_depth(&mut self, val: u64) {
        self.max_include_depth = val;
    }

    pub fn build(mut self) -> Context {
        macro_rules! getenv {
            ($env:expr) => {{
                let mut tmp = None;
                if self.enable_environment {
                    tmp = std::env::var($env).ok();
                }
                tmp
            }};
            ($env:expr, $default:expr) => {
                match getenv!($env) {
                    Some(v) => v,
                    _ => $default.to_string(),
                }
            };
        }
        let home = getenv!("HOME");
        let xdg_config_home = getenv!("XDG_CONFIG_HOME");
        let xkb_config_extra_path = getenv!("XKB_CONFIG_EXTRA_PATH", "/etc/xkb");
        let xkb_config_root = getenv!("XKB_CONFIG_ROOT", DEFAULT_INCLUDE_DIR);
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
            push!(xkb_config_extra_path.clone());
            push!(xkb_config_root.clone());
        }
        for p in self.suffix.drain(..) {
            push!(p);
        }
        Context {
            paths,
            max_includes: self.max_includes,
            max_include_depth: self.max_include_depth,
            env: Environment {
                home,
                xlocaledir: getenv!("XLOCALEDIR", "/usr/share/X11/locale"),
                xcomposefile: getenv!("XCOMPOSEFILE"),
                xkb_default_rules: getenv!("XKB_DEFAULT_RULES", "evdev"),
                xkb_default_model: getenv!("XKB_DEFAULT_MODEL", "pc105"),
                xkb_default_layout: getenv!("XKB_DEFAULT_LAYOUT", "us"),
                xkb_default_variant: getenv!("XKB_DEFAULT_VARIANT", ""),
                xkb_default_options: getenv!("XKB_DEFAULT_OPTIONS", ""),
                xkb_config_extra_path,
                xkb_config_root,
            },
        }
    }
}

pub struct RmlvoGroup<'a> {
    pub layout: &'a str,
    pub variant: &'a str,
}

impl Context {
    pub fn builder() -> ContextBuilder {
        ContextBuilder::default()
    }

    pub fn keymap_from_names(
        &self,
        mut diagnostics: impl DiagnosticHandler,
        rules: Option<&str>,
        model: Option<&str>,
        groups: Option<&[RmlvoGroup<'_>]>,
        options: Option<&[&str]>,
    ) -> Keymap {
        self.keymap_from_names_(
            &mut DiagnosticSink::new(&mut diagnostics),
            rules,
            model,
            groups,
            options,
        )
    }

    fn keymap_from_names_(
        &self,
        diagnostics: &mut DiagnosticSink,
        rules: Option<&str>,
        model: Option<&str>,
        groups: Option<&[RmlvoGroup<'_>]>,
        options: Option<&[&str]>,
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

    pub fn expand_names(
        &self,
        mut diagnostics: impl DiagnosticHandler,
        rules: Option<&str>,
        model: Option<&str>,
        groups: Option<&[RmlvoGroup<'_>]>,
        options: Option<&[&str]>,
    ) -> rmlvo::Expanded {
        self.expand_names_(
            &mut DiagnosticSink::new(&mut diagnostics),
            rules,
            model,
            groups,
            options,
        )
    }

    fn expand_names_(
        &self,
        diagnostics: &mut DiagnosticSink,
        rules: Option<&str>,
        model: Option<&str>,
        groups: Option<&[RmlvoGroup<'_>]>,
        options: Option<&[&str]>,
    ) -> rmlvo::Expanded {
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
                        kccgst::MergeMode::Augment => rmlvo::MergeMode::Augment,
                        _ => rmlvo::MergeMode::Override,
                    },
                    include: interner.get(i.val.path.val).as_bstr().to_string(),
                })
                .collect()
        };
        rmlvo::Expanded {
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
        rules: Option<&str>,
        model: Option<&str>,
        groups: Option<&[RmlvoGroup<'_>]>,
        options: Option<&[&str]>,
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
            &Environment,
        ) -> T,
    ) -> T {
        let rules = rules.unwrap_or(&self.env.xkb_default_rules);
        let model = model.unwrap_or(&self.env.xkb_default_model);
        let mut default_options = vec![];
        let options = options.unwrap_or_else(|| {
            default_options.extend(self.env.xkb_default_options.split(','));
            &default_options
        });
        let mut default_groups = vec![];
        let groups = groups.unwrap_or_else(|| {
            let mut layouts = self.env.xkb_default_layout.split(',');
            let mut variants = self.env.xkb_default_variant.split(',');
            loop {
                let layout = layouts.next();
                let variant = variants.next();
                if layout.is_none() && variant.is_none() {
                    break;
                }
                default_groups.push(RmlvoGroup {
                    layout: layout.unwrap_or_default(),
                    variant: variant.unwrap_or_default(),
                });
            }
            &default_groups
        });
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
            &self.env,
        )
    }

    pub fn parse_keymap(
        &self,
        mut diagnostics: impl DiagnosticHandler,
        path: Option<&Path>,
        kccgst: &[u8],
    ) -> Result<Keymap, Diagnostic> {
        self.parse_keymap_(&mut DiagnosticSink::new(&mut diagnostics), path, kccgst)
    }

    fn parse_keymap_(
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
            return Err(diagnostics.push_fatal(&mut map, e.val.diagnostic_kind(), e));
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
                return Err(diagnostics.push_fatal(&mut map, e.val.diagnostic_kind(), e));
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
