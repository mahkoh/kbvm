#[cfg(test)]
mod tests;

use {
    crate::{
        config::DEFAULT_INCLUDE_DIR,
        xkb::{
            code::Code,
            code_loader::CodeLoader,
            code_map::CodeMap,
            diagnostic::{Diagnostic, DiagnosticHandler, DiagnosticKind, DiagnosticSink},
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
    kbvm_proc::ad_hoc_display,
    secure_execution::requires_secure_execution,
    std::{
        path::{Path, PathBuf},
        sync::Arc,
    },
};

/// An XKB context.
///
/// This type contains include paths, environment variables, and other setting related
/// to the processing of XKB files.
///
/// While this type is relatively cheap to create, there is usually no reason for an
/// application to create more than one of these objects.
///
/// # Environment Variables
///
/// If [enabled](ContextBuilder::enable_environment), this type makes use of the following
/// environment variables.
///
/// | name                    | default                 | description            |
/// | ----------------------- | ----------------------- | ---------------------- |
/// | `HOME`                  |                         |                        |
/// | `XDG_CONFIG_HOME`       |                         |                        |
/// | `XKB_CONFIG_EXTRA_PATH` | `/etc/xkb`              | The first system path  |
/// | `XKB_CONFIG_ROOT`       | `/usr/share/X11/xkb`    | The second system path |
/// | `XLOCALEDIR`            | `/usr/share/X11/locale` |                        |
/// | `XCOMPOSEFILE`          |                         |                        |
/// | `XKB_DEFAULT_RULES`     | `evdev`                 | The fallback rules     |
/// | `XKB_DEFAULT_MODEL`     | `pc105`                 | The fallback model     |
/// | `XKB_DEFAULT_LAYOUT`    | `us`                    | The fallback layout    |
/// | `XKB_DEFAULT_VARIANTS`  |                         | The fallback variant   |
/// | `XKB_DEFAULT_OPTIONS`   |                         | The fallback options   |
/// | `LC_ALL`                | `$LC_CTYPE`             | First locale choice    |
/// | `LC_CTYPE`              | `$LANG`                 | Second locale choice   |
/// | `LANG`                  | `C`                     | Third locale choice    |
///
/// The first and second system path are used as roots for relative include statements.
///
/// The fallback rules/model/layout/variants/options are used when the respective arguments
/// are `None` in [`Context::keymap_from_names`] and [`Context::expand_names`].
///
/// If environment variables are enabled, the defaults are used if the environment
/// variables are not set.
///
/// If environment variables are disabled, the defaults are used unconditionally.
///
/// # Include Paths
///
/// This type contains the following include paths:
///
/// - Paths added via [`ContextBuilder::prepend_path`] in reverse order.
/// - If default include paths are [enabled](ContextBuilder::enable_default_includes):
///   - If environment variables are [enabled](ContextBuilder::enable_environment):
///     - `$XDG_CONFIG_HOME/xkb` if `$XDG_CONFIG_HOME` is set.
///     - `$HOME/.config/xkb` if `$XDG_CONFIG_HOME` is not set and `$HOME` is set.
///     - `$HOME/.xkb` if `$HOME` is set.
///   - The first system path.
///   - The second system path.
/// - Paths added via [`ContextBuilder::append_path`].
#[derive(Clone, Debug)]
pub struct Context {
    pub(crate) paths: Vec<Arc<PathBuf>>,
    pub(crate) max_includes: u64,
    pub(crate) max_include_depth: u64,
    pub(crate) max_runtime: u64,
    #[cfg_attr(not(feature = "compose"), allow(dead_code))]
    pub(crate) max_compose_rules: u64,
    #[cfg_attr(not(feature = "registry"), allow(dead_code))]
    pub(crate) load_extra_rules: bool,
    pub(crate) env: Environment,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct Environment {
    pub(crate) home: Option<String>,
    #[cfg_attr(not(feature = "compose"), allow(dead_code))]
    pub(crate) xdg_config_home: Option<String>,
    pub(crate) locale: String,
    #[cfg_attr(not(feature = "compose"), allow(dead_code))]
    pub(crate) xlocaledir: String,
    #[cfg_attr(not(feature = "compose"), allow(dead_code))]
    pub(crate) xcomposefile: Option<String>,
    pub(crate) xkb_default_rules: String,
    pub(crate) xkb_default_model: String,
    pub(crate) xkb_default_layout: String,
    pub(crate) xkb_default_variant: String,
    pub(crate) xkb_default_options: String,
    pub(crate) xkb_config_extra_path: String,
    pub(crate) xkb_config_root: String,
}

type EnvironmentAccessor = Box<dyn FnMut(&str) -> Option<String>>;

/// A builder for [`Context`] objects.
pub struct ContextBuilder {
    enable_default_includes: bool,
    enable_environment: bool,
    load_extra_rules: bool,
    max_includes: u64,
    max_include_depth: u64,
    max_runtime: u64,
    max_compose_rules: u64,
    prefix: Vec<PathBuf>,
    suffix: Vec<PathBuf>,
    environment_accessor: Option<EnvironmentAccessor>,
}

impl Default for ContextBuilder {
    fn default() -> Self {
        Self {
            enable_default_includes: true,
            enable_environment: !requires_secure_execution(),
            load_extra_rules: false,
            max_includes: 1024,
            max_include_depth: 128,
            max_runtime: 500_000,
            max_compose_rules: 100_000,
            prefix: vec![],
            suffix: vec![],
            environment_accessor: None,
        }
    }
}

impl Default for Context {
    fn default() -> Self {
        Self::builder().build()
    }
}

impl ContextBuilder {
    /// Enables or disables the use of default include paths.
    ///
    /// The default is `true`.
    ///
    /// See the documentation of [`Context`] for the list of default include paths.
    pub fn enable_default_includes(&mut self, val: bool) {
        self.enable_default_includes = val;
    }

    /// Enables or disables the use of environment variables.
    ///
    /// See the documentation of [`Context`] for the list of used environment variables.
    ///
    /// The default depends on whether the application requires
    /// [secure execution](requires_secure_execution):
    ///
    /// - If the application requires secure execution, the default is `false`.
    /// - Otherwise the default is `true`.
    ///
    /// # Security
    ///
    /// Enabling this in a set-user-ID program can lead to information disclosure. For
    /// example, a file under `$HOME` might include `/etc/shadow`. Even if `/etc/shadow`
    /// cannot be parsed, its first line might get printed as a log message.
    pub fn enable_environment(&mut self, val: bool) {
        self.enable_environment = val;
    }

    /// Sets the function used to access environment variables.
    ///
    /// By default, the builder uses [`std::env::var`].
    ///
    /// If the environment is [disabled](Self::enable_environment), then this function
    /// will never be called.
    pub fn environment_accessor(&mut self, accessor: impl FnMut(&str) -> Option<String> + 'static) {
        self.environment_accessor = Some(Box::new(accessor));
    }

    #[allow(rustdoc::broken_intra_doc_links)]
    /// Enables or disables the loading of `.extras.xml` rules.
    ///
    /// The default is `false`.
    ///
    /// This only affects [`Context::registry`] and [`Context::default_registry`] and has
    /// no effect if the `registry` feature is not enabled.
    pub fn load_extra_rules(&mut self, val: bool) {
        self.load_extra_rules = val;
    }

    /// Prepends an include path.
    pub fn prepend_path(&mut self, path: &(impl AsRef<Path> + ?Sized)) {
        self.prefix.push(path.as_ref().to_path_buf());
    }

    /// Appends an include path.
    pub fn append_path(&mut self, path: &(impl AsRef<Path> + ?Sized)) {
        self.suffix.push(path.as_ref().to_path_buf());
    }

    /// Sets the maximum number of includes that will be processed by each function call.
    ///
    /// The default is `1024`.
    ///
    /// For RMLVO, each include line counts as 1 include. For XKB, each component of an
    /// include statement counts as 1 include.
    ///
    /// This setting exists to guard against the following situation where the total
    /// number of includes grows exponentially while the include depth stays low.
    ///
    /// ```xkb
    /// // file "1"
    /// include "2"
    /// include "2"
    ///
    /// // file "2"
    /// include "3"
    /// include "3"
    ///
    /// // ...
    /// ```
    pub fn max_includes(&mut self, val: u64) {
        self.max_includes = val;
    }

    /// Sets the maximum depth of processed includes.
    ///
    /// The default is `128`.
    pub fn max_include_depth(&mut self, val: u64) {
        self.max_include_depth = val;
    }

    /// Sets the maximum runtime of the parsing process.
    ///
    /// The default is `500_000`.
    ///
    /// This value has no unit and does not correspond to CPU or wall time. It limits how
    /// much work is performed by a single method call of the context but how it does this
    /// is unspecified.
    pub fn max_runtime(&mut self, val: u64) {
        self.max_runtime = val;
    }

    /// Sets the maximum number of compose rules.
    ///
    /// The default is `100_000`.
    pub fn max_compose_rules(&mut self, val: u64) {
        self.max_compose_rules = val;
    }

    /// Removes all paths from the builder and disables the default include paths.
    pub fn clear(&mut self) {
        self.enable_default_includes = false;
        self.prefix.clear();
        self.suffix.clear();
        self.environment_accessor = None;
    }

    /// Builds the context.
    pub fn build(mut self) -> Context {
        macro_rules! getenv {
            ($env:expr) => {{
                let mut tmp = None;
                if self.enable_environment {
                    tmp = match &mut self.environment_accessor {
                        Some(f) => f($env),
                        _ => std::env::var($env).ok(),
                    }
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
        if self.enable_default_includes {
            if let Some(config) = &xdg_config_home {
                push!(format!("{config}/xkb"));
            } else if let Some(home) = &home {
                push!(format!("{home}/.config/xkb"));
            }
            if let Some(home) = &home {
                push!(format!("{home}/.xkb"));
            }
            push!(xkb_config_extra_path.clone());
            push!(xkb_config_root.clone());
        }
        for p in self.suffix.drain(..) {
            push!(p);
        }
        let mut locale = String::new();
        for var in ["LC_ALL", "LC_CTYPE", "LANG"] {
            locale = getenv!(var, "");
            if locale.is_not_empty() {
                break;
            }
        }
        if locale.is_empty() {
            locale = "C".to_string();
        }
        Context {
            paths,
            max_includes: self.max_includes,
            max_include_depth: self.max_include_depth,
            max_runtime: self.max_runtime,
            max_compose_rules: self.max_compose_rules,
            load_extra_rules: self.load_extra_rules,
            env: Environment {
                home,
                xdg_config_home,
                locale,
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

impl Context {
    /// Creates a builder for a new context.
    pub fn builder() -> ContextBuilder {
        ContextBuilder::default()
    }

    /// Creates a keymap from RMLVO names.
    ///
    /// If a parameter is not given, a value from the environment or a default is used.
    /// This is described in the [`Context`] documentation.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::xkb::Context;
    /// # use kbvm::xkb::diagnostic::WriteToLog;
    /// # use kbvm::xkb::rmlvo::Group;
    /// let context = Context::default();
    /// let groups: Vec<_> = Group::from_layouts_and_variants(
    ///     "us,il,ru,de",
    ///     ",,phonetic,neo",
    /// ).collect();
    /// let map = context.keymap_from_names(
    ///     WriteToLog,
    ///     None, // rules default to "evdev"
    ///     None, // model default to "pc105"
    ///     Some(&groups),
    ///     None, // options default to &[],
    /// );
    /// ```
    pub fn keymap_from_names(
        &self,
        mut diagnostics: impl DiagnosticHandler,
        rules: Option<&str>,
        model: Option<&str>,
        groups: Option<&[rmlvo::Group<'_>]>,
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
        groups: Option<&[rmlvo::Group<'_>]>,
        options: Option<&[&str]>,
    ) -> Keymap {
        let mut map = CodeMap::default();
        let mut interner = Interner::default();
        let mut cooker = StringCooker::default();
        let mut ast_cache = AstCache::default();
        let mut loader = CodeLoader::new(&self.paths);
        let mut meaning_cache = MeaningCache::default();
        let mut remaining_runtime = self.max_runtime;
        let mut item = self.handle_rmlvo(
            diagnostics,
            &mut map,
            &mut interner,
            &mut loader,
            &mut meaning_cache,
            &mut remaining_runtime,
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
            &mut remaining_runtime,
            &mut cooker,
            &mut item,
        )
    }

    /// Expands RMLVO names to an unresolved XKB map.
    ///
    /// This can be used to display the output of the RMLVO compilation process. Most of
    /// the time, you want to use [`Self::keymap_from_names`] instead.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::xkb::Context;
    /// # use kbvm::xkb::diagnostic::WriteToLog;
    /// # use kbvm::xkb::rmlvo::Group;
    /// let context = Context::default();
    /// let groups: Vec<_> = Group::from_layouts_and_variants(
    ///     "us,il,ru,de",
    ///     ",,phonetic,neo",
    /// ).collect();
    /// let map = context.expand_names(
    ///     WriteToLog,
    ///     None, // rules default to "evdev"
    ///     None, // model default to "pc105"
    ///     Some(&groups),
    ///     None, // options default to &[],
    /// );
    /// println!("{:#}", map.format());
    /// ```
    ///
    /// This might print
    ///
    /// ```xkb
    /// xkb_keymap {
    ///     xkb_keycodes {
    ///         override "evdev"
    ///         override "aliases(qwerty)"
    ///     };
    ///     xkb_types {
    ///         override "complete"
    ///     };
    ///     xkb_compat {
    ///         override "complete"
    ///         override "caps(caps_lock):4"
    ///         override "misc(assign_shift_left_action):4"
    ///         override "level5(level5_lock):4"
    ///     };
    ///     xkb_symbols {
    ///         override "pc"
    ///         override "us"
    ///         override "il:2"
    ///         override "ru(phonetic):3"
    ///         override "de(neo):4"
    ///         override "inet(evdev)"
    ///     };
    ///     xkb_geometry {
    ///         override "pc(pc105)"
    ///     };
    /// };
    /// ```
    pub fn expand_names(
        &self,
        mut diagnostics: impl DiagnosticHandler,
        rules: Option<&str>,
        model: Option<&str>,
        groups: Option<&[rmlvo::Group<'_>]>,
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
        groups: Option<&[rmlvo::Group<'_>]>,
        options: Option<&[&str]>,
    ) -> rmlvo::Expanded {
        let mut map = CodeMap::default();
        let mut interner = Interner::default();
        let mut loader = CodeLoader::new(&self.paths);
        let mut meaning_cache = MeaningCache::default();
        let mut remaining_runtime = self.max_runtime;
        let includes = self.handle_rmlvo(
            diagnostics,
            &mut map,
            &mut interner,
            &mut loader,
            &mut meaning_cache,
            &mut remaining_runtime,
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
                        kccgst::MergeMode::Replace => rmlvo::MergeMode::Replace,
                        kccgst::MergeMode::Include | kccgst::MergeMode::Override => {
                            rmlvo::MergeMode::Override
                        }
                        _ => unreachable!(),
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
        remaining_runtime: &mut u64,
        rules: Option<&str>,
        model: Option<&str>,
        groups: Option<&[rmlvo::Group<'_>]>,
        options: Option<&[&str]>,
        f: impl FnOnce(
            &mut CodeMap,
            &mut Interner,
            &mut CodeLoader,
            &mut DiagnosticSink,
            &mut MeaningCache,
            &mut u64,
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
                default_groups.push(rmlvo::Group {
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
            remaining_runtime,
            rules,
            model,
            &options,
            &groups,
            self.max_includes,
            &self.env,
        )
    }

    /// Creates a keymap from an existing XKB map.
    ///
    /// If the buffer contains more than one keymap, then all but the first one are
    /// ignored.
    ///
    /// # Security
    ///
    /// XKB maps can contain arbitrary include paths. If you are executing code in a
    /// set-user-ID binary, care must be taken so that this does not lead to information
    /// disclosure. The [`requires_secure_execution`] function might be useful for this.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::xkb::Context;
    /// # use kbvm::xkb::diagnostic::WriteToLog;
    /// const MAP: &str = r#"
    ///     xkb_keymap {
    ///         xkb_keycodes {
    ///               <a>         = 38;
    ///               <leftshift> = 50;
    ///               <capslock>  = 66;
    ///         };
    ///         xkb_symbols {
    ///             key <a>         { [ a, A ] };
    ///             key <leftshift> { [ SetMods(modifiers=Shift) ] };
    ///             key <capslock>  { [ LockMods(modifiers=Lock) ] };
    ///         };
    ///     };
    /// "#;
    /// let context = Context::default();
    /// let map = context
    ///     .keymap_from_bytes(WriteToLog, None, MAP.as_bytes())
    ///     .unwrap();
    /// println!("{:#}", map.format());
    /// ```
    ///
    /// This might print
    ///
    /// ```xkb
    /// xkb_keymap {
    ///     xkb_keycodes {
    ///         minimum = 8;
    ///         maximum = 255;
    ///
    ///         indicator 1 = "DUMMY";
    ///
    ///         <a> = 38;
    ///         <capslock> = 66;
    ///         <leftshift> = 50;
    ///     };
    ///
    ///     xkb_types {
    ///         virtual_modifiers Dummy;
    ///
    ///         type "ALPHABETIC" {
    ///             modifiers = Shift+Lock;
    ///             level_name[Level1] = "Base";
    ///             level_name[Level2] = "Caps";
    ///             map[Shift] = Level2;
    ///             map[Lock] = Level2;
    ///         };
    ///
    ///         type "ONE_LEVEL" {
    ///             modifiers = None;
    ///             level_name[Level1] = "Any";
    ///             map[None] = Level1;
    ///         };
    ///     };
    ///
    ///     xkb_compat {
    ///         interpret VoidSymbol {
    ///             repeat = false;
    ///         };
    ///     };
    ///
    ///     xkb_symbols {
    ///         key.repeat = true;
    ///
    ///         key <a> {
    ///             type[Group1] = "ALPHABETIC",
    ///             symbols[Group1] = [ a, A ]
    ///         };
    ///         key <capslock> {
    ///             repeat = false,
    ///             type[Group1] = "ONE_LEVEL",
    ///             actions[Group1] = [ LockMods(modifiers = Lock) ]
    ///         };
    ///         key <leftshift> {
    ///             repeat = false,
    ///             type[Group1] = "ONE_LEVEL",
    ///             actions[Group1] = [ SetMods(modifiers = Shift) ]
    ///         };
    ///     };
    /// };
    /// ```
    pub fn keymap_from_bytes(
        &self,
        mut diagnostics: impl DiagnosticHandler,
        path: Option<&Path>,
        kccgst: impl AsRef<[u8]>,
    ) -> Result<Keymap, Diagnostic> {
        self.keymap_from_bytes_(
            &mut DiagnosticSink::new(&mut diagnostics),
            path,
            kccgst.as_ref(),
        )
    }

    fn keymap_from_bytes_(
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
        if tokens.is_empty() {
            return Err(diagnostics.push_fatal(
                &mut map,
                DiagnosticKind::UnexpectedEof,
                ad_hoc_display!("keymap is empty").spanned2(span),
            ));
        }
        let mut remaining_runtime = self.max_runtime;
        let parsed = parse_item(
            &mut map,
            diagnostics,
            &interner,
            &mut meaning_cache,
            &tokens,
            0,
            &mut remaining_runtime,
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
            &mut remaining_runtime,
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
        remaining_runtime: &mut u64,
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
            remaining_runtime,
            item,
            self.max_includes,
            self.max_include_depth,
        );
        embed(item);
        let resolved = resolve(map, diagnostics, interner, meaning_cache, cooker, item);
        Keymap::from_resolved(interner, &resolved)
    }

    /// Returns the locale of this context.
    ///
    /// If environment variables are [enabled](ContextBuilder::enable_environment), the
    /// locale is determined according to the POSIX rules for `LC_CTYPE`:
    ///
    /// - If `$LC_ALL` is set and non-empty, its value is used.
    /// - Otherwise, if `$LC_CTYPE` is set and non-empty, its value is used.
    /// - Otherwise, if `$LANG` is set and non-empty, its value is used.
    /// - Otherwise, the value is `"C"`.
    ///
    /// If environment variables are disabled, the value is always `"C"`.
    pub fn locale(&self) -> &str {
        &self.env.locale
    }
}
