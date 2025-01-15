//! XCompose helpers and types.
//!
//! The entry point to this module is the [`ComposeTable`].

pub use table::{ComposeTable, FeedResult, Iter, MatchRule, MatchStep, State};
#[expect(unused_imports)]
use {crate::xkb::ContextBuilder, secure_execution::requires_secure_execution};
use {
    crate::xkb::{
        code::Code,
        code_map::CodeMap,
        code_slice::CodeSlice,
        compose::{
            lexer::Lexer,
            parser::{parse_line, Line},
        },
        diagnostic::{DiagnosticHandler, DiagnosticKind, DiagnosticSink},
        interner::Interner,
        meaning::MeaningCache,
        span::SpanExt,
        Context,
    },
    bstr::ByteSlice,
    hashbrown::HashSet,
    kbvm_proc::ad_hoc_display,
    std::{
        fs::File,
        io::{BufRead, BufReader},
        mem,
        path::{Path, PathBuf},
        sync::Arc,
    },
    thiserror::Error,
};

#[macro_use]
mod macros;
mod lexer;
mod parser;
mod table;
#[cfg(test)]
mod tests;
mod token;

#[derive(Clone)]
enum TopLevel {
    Path(PathBuf),
    Buffer(Vec<u8>),
}

/// A builder for a compose table.
///
/// This type is created using [`Context::compose_table_builder`].
#[derive(Clone)]
pub struct ComposeTableBuilder<'a> {
    context: &'a Context,
    locale: Option<String>,
    top_level: Option<TopLevel>,
}

impl Context {
    /// Creates a builder for compose table.
    ///
    /// This function is only available when the `compose` feature is enabled.
    pub fn compose_table_builder(&self) -> ComposeTableBuilder<'_> {
        ComposeTableBuilder {
            context: self,
            locale: None,
            top_level: None,
        }
    }
}

impl ComposeTableBuilder<'_> {
    /// Sets the locale to use for the compose file.
    ///
    /// If this function is not called, the locale of the context is used. See [`Context::locale].
    pub fn locale(&mut self, locale: &str) {
        self.locale = Some(locale.to_string());
    }

    /// Sets the buffer that contains the compose table.
    ///
    /// This overwrites any buffer or file previously set for this builder.
    ///
    /// # Security
    ///
    /// Compose files can contain arbitrary include paths. If you are executing code in a
    /// set-user-ID binary, care must be taken so that this does not lead to information
    /// disclosure. The [`requires_secure_execution`] function might be useful for this.
    pub fn buffer(&mut self, buffer: impl AsRef<[u8]>) {
        self.top_level = Some(TopLevel::Buffer(buffer.as_ref().to_vec()));
    }

    /// Sets the path to the file that contains the compose table.
    ///
    /// This overwrites any buffer or file previously set for this builder.
    ///
    /// # Security
    ///
    /// Compose files can contain arbitrary include paths. If you are executing code in a
    /// set-user-ID binary, care must be taken so that this does not lead to information
    /// disclosure. The [`requires_secure_execution`] function might be useful for this.
    pub fn file(&mut self, path: impl AsRef<Path>) {
        self.top_level = Some(TopLevel::Path(path.as_ref().to_path_buf()));
    }

    /// Builds the compose table.
    ///
    /// Building a compose table requires access to the user's locale. If the [`Self::locale`]
    /// function was not called, the [locale of the context](Context::locale) is used.
    ///
    /// If [`Self::buffer`] or [`Self::file`] were called, that information is used to find the
    /// compose table.
    ///
    /// Otherwise, first the `<xlocaledir>` is determined:
    ///
    /// - If environment variables are [enabled](ContextBuilder::enable_environment) and
    ///   `$XLOCALEDIR` is set, that is the value of `<xlocaledir>`.
    /// - Otherwise, the value of `<xlocaledir>` is `/usr/share/X11/locale`.
    ///
    /// Then, the path to the compose table is determined as follows:
    ///
    /// - If environment variables are enabled, the first readable file from the following list is
    ///   used:
    ///
    ///   - `$XCOMPOSEFILE` if it is set.
    ///   - `$XDG_CONFIG_HOME/XCompose` if `$XDG_CONFIG_HOME` is set.
    ///   - `$HOME/.config/XCompose` if `$XDG_CONFIG_HOME` is not set and `$HOME` is set.
    ///   - `$HOME/.XCompose` if `$HOME` is set.
    ///
    /// - Otherwise, if the environment is disabled or none of the files exist, the following file
    ///   is used if it is readable:
    ///
    ///   - `<xlocaledir>/??`. The value of `??` is described below.
    ///
    /// - Otherwise, this function returns `None`.
    ///
    /// The value of `??` above is determined as follows:
    ///
    /// - First, the locale is resolved to the canonical locale name by reading
    ///   `<xlocaledir>/locale.alias`.
    /// - Then, the value of `??` determined by reading `<xlocaledir>/compose.dir`.
    ///
    ///   If this fails, this function returns `None`.
    pub fn build(self, mut diagnostics: impl DiagnosticHandler) -> Option<ComposeTable> {
        self.build_(&mut DiagnosticSink::new(&mut diagnostics))
    }

    fn build_(self, diagnostics: &mut DiagnosticSink<'_, '_>) -> Option<ComposeTable> {
        let locale = self.locale.as_deref().unwrap_or(&self.context.env.locale);
        let locale = self.resolve_locale(locale);
        let compose_file_path = self.get_compose_file(&locale);
        let mut toplevel_path = None;
        let toplevel = match self.top_level {
            Some(t) => t,
            None => 'toplevel: {
                enum Candidate {
                    Xcomposefile,
                    XdgConfigHome,
                    Home,
                    ComposeFilePath,
                }
                use Candidate::*;
                for candidate in [Xcomposefile, XdgConfigHome, Home, ComposeFilePath] {
                    let env = &self.context.env;
                    let path = match candidate {
                        Xcomposefile => match env.xcomposefile.as_deref() {
                            Some(f) => f.to_string(),
                            _ => continue,
                        },
                        XdgConfigHome => match env.xdg_config_home.as_deref() {
                            Some(f) => format!("{f}/XCompose"),
                            _ => match env.home.as_deref() {
                                Some(f) => format!("{f}/.config/XCompose"),
                                _ => continue,
                            },
                        },
                        Home => match env.home.as_deref() {
                            Some(f) => format!("{f}/.XCompose"),
                            _ => continue,
                        },
                        ComposeFilePath => match compose_file_path.as_ref() {
                            Some(f) => f.clone(),
                            None => continue,
                        },
                    };
                    if let Ok(c) = std::fs::read(&path) {
                        toplevel_path = Some(PathBuf::from(path));
                        break 'toplevel TopLevel::Buffer(c);
                    }
                }
                return None;
            }
        };
        let buffer = match toplevel {
            TopLevel::Buffer(b) => b,
            TopLevel::Path(p) => match std::fs::read(&p).ok() {
                Some(b) => {
                    toplevel_path = Some(p);
                    b
                }
                _ => return None,
            },
        };
        // println!("{}", buffer.as_bstr());
        let mut map = CodeMap::default();
        let mut interner = Interner::default();
        let path = toplevel_path.map(Arc::new);
        let code = Code::new(&Arc::new(buffer));
        let span = map.add(path.as_ref(), None, &code);
        let lexer = Lexer::new(&code, span.lo);
        let mut tokens = vec![];
        let mut meaning_cache = MeaningCache::default();
        let mut productions = vec![];
        let mut lexers = vec![(None, lexer)];
        let mut processed_includes = 0;
        let mut current_includes = HashSet::new();
        macro_rules! pop {
            () => {
                if let Some((Some(path), _)) = lexers.pop() {
                    current_includes.remove(&path);
                }
            };
        }
        while let Some((_, lexer)) = lexers.last_mut() {
            tokens.clear();
            let res = lexer.lex_line(&mut map, diagnostics, &mut interner, &mut tokens);
            if let Err(e) = res {
                pop!();
                diagnostics.push(&mut map, e.val.diagnostic_kind(), e);
                continue;
            }
            if tokens.is_empty() {
                pop!();
                continue;
            }
            let line = parse_line(
                &mut map,
                diagnostics,
                &mut interner,
                &mut meaning_cache,
                &tokens,
                &self.context.env,
                compose_file_path.as_deref(),
            );
            let line = match line {
                Ok(Some(l)) => l,
                Ok(None) => continue,
                Err(e) => {
                    diagnostics.push(&mut map, e.val.diagnostic_kind(), e);
                    continue;
                }
            };
            let include = match line.val {
                Line::Production(p) => {
                    productions.push(p.spanned2(line.span));
                    continue;
                }
                Line::Include(i) => i,
            };
            if processed_includes >= self.context.max_includes {
                diagnostics.push(
                    &mut map,
                    DiagnosticKind::MaxIncludesReached,
                    ad_hoc_display!("maximum number of includes reached").spanned2(line.span),
                );
                continue;
            }
            processed_includes += 1;
            if lexers.len() as u64 > self.context.max_include_depth {
                diagnostics.push(
                    &mut map,
                    DiagnosticKind::MaxIncludeDepthReached,
                    ad_hoc_display!("maximum include depth ({}) reached", self.context.max_include_depth => u64).spanned2(line.span),
                );
                continue;
            }
            if !current_includes.insert(include.to_owned()) {
                diagnostics.push(
                    &mut map,
                    DiagnosticKind::RecursiveInclude,
                    ad_hoc_display!("ignoring recursive include").spanned2(line.span),
                );
                continue;
            }
            let Ok(path) = include.as_bstr().to_str() else {
                diagnostics.push(
                    &mut map,
                    DiagnosticKind::NonUTF8Path,
                    ad_hoc_display!("ignoring non-UTF-8 path").spanned2(line.span),
                );
                continue;
            };
            let buffer = match std::fs::read(path) {
                Ok(b) => b,
                Err(e) => {
                    #[derive(Debug, Error)]
                    #[error("could not read file `{}`", .0.as_bstr())]
                    struct E(CodeSlice<'static>, #[source] std::io::Error);
                    diagnostics.push(
                        &mut map,
                        DiagnosticKind::FileReadFailed,
                        E(include, e).spanned2(line.span),
                    );
                    continue;
                }
            };
            let path = Arc::new(Path::new(path).to_path_buf());
            let code = Code::new(&Arc::new(buffer));
            let span = map.add(Some(&path), Some(line.span), &code);
            let lexer = Lexer::new(&code, span.lo);
            lexers.push((Some(include), lexer));
        }
        let table = ComposeTable::new(&mut map, diagnostics, &productions);
        Some(table)
    }

    fn get_compose_file(&self, mut resolved_locale: &str) -> Option<String> {
        if resolved_locale == "C" {
            resolved_locale = "en_US.UTF-8";
        }
        let mut ll = self.read_locale_lines("compose.dir")?;
        while let Some((l, r)) = ll.next() {
            if r == resolved_locale.as_bytes() {
                return Some(format!("{}/{}", self.context.env.xlocaledir, l.as_bstr()));
            }
        }
        None
    }

    fn resolve_locale(&self, locale: &str) -> String {
        let Some(mut ll) = self.read_locale_lines("locale.alias") else {
            return locale.to_string();
        };
        while let Some((l, r)) = ll.next() {
            if l == locale.as_bytes() {
                return r.as_bstr().to_string();
            }
        }
        locale.to_string()
    }

    fn read_locale_lines(&self, filename: &str) -> Option<LocaleLines> {
        let path = Path::new(&self.context.env.xlocaledir).join(filename);
        let file = File::open(&path).ok()?;
        Some(LocaleLines {
            buf: Vec::new(),
            reader: BufReader::new(file),
        })
    }
}

struct LocaleLines {
    buf: Vec<u8>,
    reader: BufReader<File>,
}

impl LocaleLines {
    fn next<'a>(&'a mut self) -> Option<(&'a [u8], &'a [u8])> {
        loop {
            self.buf.clear();
            if self.reader.read_until(b'\n', &mut self.buf).ok()? == 0 {
                return None;
            }
            let mut buf = self.buf.as_bytes();
            if let Some((l, _)) = buf.split_once_str(b"#") {
                buf = l;
            }
            let (l, r) = if let Some((l, r)) = buf.split_once_str(b":") {
                (l, r)
            } else if let Some(idx) = buf.find_byteset([b' ', b'\t']) {
                buf.split_at(idx)
            } else {
                continue;
            };
            unsafe {
                type Ret<'b> = Option<(&'b [u8], &'b [u8])>;
                // SAFETY: This is a compiler bug that will never be fixed.
                return mem::transmute::<Ret<'_>, Ret<'a>>(Some((l.trim_ascii(), r.trim_ascii())));
            }
        }
    }
}
