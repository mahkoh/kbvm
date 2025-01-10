//! Types for working with the RMLVO registry.

mod xml;

#[expect(unused_imports)]
use crate::xkb::ContextBuilder;
use {
    crate::xkb::{
        code::Code,
        code_map::CodeMap,
        diagnostic::{DiagnosticHandler, DiagnosticKind, DiagnosticSink},
        span::SpanExt,
        Context,
    },
    arrayvec::ArrayVec,
    error_reporter::Report,
    std::{
        fs::File,
        io::{self, BufReader, ErrorKind},
        path::PathBuf,
        sync::Arc,
    },
    thiserror::Error,
};

/// The RMLVO registry.
///
/// You can retrieve the registry via [`Context::default_registry`] or
/// [`Context::registry`].
#[derive(Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub struct Registry {
    /// The models in the registry.
    pub models: Vec<Model>,
    /// The layouts in the registry.
    ///
    /// Layouts are not de-duplicated. The same layout can appear multiple times. It is
    /// up to the application to merge the variants lists if necessary.
    pub layouts: Vec<Layout>,
    /// The option groups in the registry.
    ///
    /// Option groups are not de-duplicated. The same option group can appear multiple
    /// times. It is up to the application to merge the options lists if necessary.
    pub options: Vec<OptGroup>,
}

/// The popularity of a config item.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Popularity {
    Standard,
    Exotic,
}

/// An RMLVO model.
#[derive(Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub struct Model {
    /// The name of the model.
    ///
    /// This can be used as the `model` parameter of [`Context::keymap_from_names`].
    pub name: String,
    /// The description of the model.
    pub description: Option<String>,
    /// The vendor of the model.
    pub vendor: Option<String>,
    /// The popularity of the model.
    pub popularity: Popularity,
}

/// An RMLVO layout with possible variants.
#[derive(Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub struct Layout {
    /// The name of the layout.
    ///
    /// This can be used as the layout in the `groups` parameter of
    /// [`Context::keymap_from_names`].
    pub name: String,
    /// The short description of the layout.
    pub short_description: Option<String>,
    /// The description of the layout.
    pub description: Option<String>,
    /// The popularity of the layout.
    pub popularity: Popularity,
    /// The ISO 639-3 language codes of the layout.
    pub languages: Vec<String>,
    /// The ISO 3166-1 alpha-2 country codes of the layout.
    pub countries: Vec<String>,
    /// The variants that can be used with this layout.
    pub variants: Vec<Variant>,
}

/// An RMLVO variant.
#[derive(Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub struct Variant {
    /// The name of the variant.
    ///
    /// This can be used as the variant in the `groups` parameter of
    /// [`Context::keymap_from_names`].
    pub name: String,
    /// The short description of the variant.
    pub short_description: Option<String>,
    /// The description of the variant.
    pub description: Option<String>,
    /// The popularity of the variant.
    pub popularity: Popularity,
    /// The ISO 639-3 language codes of the variant.
    pub languages: Vec<String>,
    /// The ISO 3166-1 alpha-2 country codes of the variant.
    pub countries: Vec<String>,
}

/// A group of XKB options.
#[derive(Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub struct OptGroup {
    /// Whether more than one of the options can be active at a time.
    pub allow_multiple: bool,
    /// The name of the group.
    pub name: String,
    /// The description of the group.
    pub description: Option<String>,
    /// The popularity of the group.
    pub popularity: Popularity,
    /// The options that are part of this group.
    pub options: Vec<Opt>,
}

/// A XKB options.
#[derive(Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub struct Opt {
    /// The name of the variant.
    ///
    /// This can be used in the `options` parameter of [`Context::keymap_from_names`].
    pub name: String,
    /// The short description of the option.
    pub short_description: Option<String>,
    /// The description of the option.
    pub description: Option<String>,
    /// The popularity of the option.
    pub popularity: Popularity,
}

impl Context {
    /// Loads the registry with the default rules name.
    ///
    /// See the documentation of [`Context`] for how the default rules name is determined.
    ///
    /// This function is a shorthand for calling [`Self::registry`] with the default rules
    /// name. See that function for more details.
    ///
    /// This function is only available if the `registry` feature is enabled.
    pub fn default_registry(&self, diagnostics: impl DiagnosticHandler) -> Registry {
        self.registry(diagnostics, &self.env.xkb_default_rules)
    }

    /// Loads the registry with the given rules name.
    ///
    /// This function is only available if the `registry` feature is enabled.
    ///
    /// This function iterates over the include paths in reverse order, parses the
    /// `rules/{rules}.xml` (and `rules/{rules}.extras.xml` if enabled via
    /// [`ContextBuilder::load_extra_rules`]), concatenates the results, and returns them.
    ///
    /// This function never fails. The only possible failure condition is a file not being
    /// readable or not containing valid XML. In this case the file is simply skipped and
    /// a diagnostic message is emitted.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::xkb::Context;
    /// # use kbvm::xkb::diagnostic::WriteToLog;
    /// let mut builder = Context::builder();
    /// builder.load_extra_rules(true);
    /// let context = builder.build();
    /// let _registry = context.registry(WriteToLog, "evdev");
    /// ```
    pub fn registry(&self, mut diagnostics: impl DiagnosticHandler, rules: &str) -> Registry {
        self.registry_(&mut DiagnosticSink::new(&mut diagnostics), rules)
    }

    fn registry_(&self, diagnostics: &mut DiagnosticSink<'_, '_>, ruleset: &str) -> Registry {
        let mut map = CodeMap::default();
        let mut res = Registry {
            models: Default::default(),
            layouts: Default::default(),
            options: Default::default(),
        };
        let mut rulesets = ArrayVec::<_, 2>::new();
        rulesets.push(format!("rules/{}.xml", ruleset));
        if self.load_extra_rules {
            rulesets.push(format!("rules/{}.extras.xml", ruleset));
        }
        for path in self.paths.iter().rev() {
            for ruleset in &rulesets {
                let path = path.join(ruleset);
                macro_rules! diag {
                    ($kind:expr, $err:expr) => {{
                        let code = Code::new(&Arc::new(path.display().to_string().into_bytes()));
                        let span = map.add(None, None, &code);
                        diagnostics.push(&mut map, $kind, Report::new($err).spanned2(span));
                    }};
                }
                let file = match File::open(&path) {
                    Ok(c) => BufReader::new(c),
                    Err(e) if e.kind() == ErrorKind::NotFound => continue,
                    Err(e) => {
                        #[derive(Debug, Error)]
                        #[error("could not open registry file `{}`", .0.display())]
                        struct E(PathBuf, #[source] io::Error);
                        diag!(DiagnosticKind::FileOpenFailed, E(path, e));
                        continue;
                    }
                };
                let mut r = match quick_xml::de::from_reader::<_, xml::Registry>(file) {
                    Ok(r) => Registry::from(r),
                    Err(e) => {
                        #[derive(Debug, Error)]
                        #[error("could not parse registry file `{}`", .0.display())]
                        struct E(PathBuf, #[source] quick_xml::DeError);
                        diag!(DiagnosticKind::DeserializeRegistryFailed, E(path, e));
                        continue;
                    }
                };
                macro_rules! take {
                    ($field:ident) => {
                        if res.$field.is_empty() {
                            res.$field = r.$field;
                        } else {
                            res.$field.append(&mut r.$field);
                        }
                    };
                }
                take!(models);
                take!(layouts);
                take!(options);
            }
        }
        res
    }
}
