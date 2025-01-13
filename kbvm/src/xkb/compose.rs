pub use table::{ComposeTable, FeedResult, Iter, MatchRule, MatchStep, State};
use {
    crate::xkb::{
        code::Code,
        code_map::CodeMap,
        compose::{
            lexer::Lexer,
            parser::{parse_line, Line},
        },
        diagnostic::{DiagnosticSink, WriteToStderr},
        interner::Interner,
        meaning::MeaningCache,
        span::SpanExt,
        Context,
    },
    std::{
        fs::File,
        io::{BufRead, BufReader},
        mem,
        path::Path,
        sync::Arc,
    },
};

#[macro_use]
mod macros;
mod lexer;
mod parser;
mod table;
#[cfg(test)]
mod tests;
mod token;

impl Context {
    pub fn compose_from_locale(&self, locale: &str) {
        let locale = self.resolve_locale(locale);
        let locale_dir = self.get_compose_file(&locale);
        let mut interner = Interner::default();
        let path = Arc::new(Path::new("/usr/share/X11/locale/en_US.UTF-8/Compose").to_owned());
        // let path = Arc::new(Path::new("/home/julian/testcompose").to_owned());
        let code = std::fs::read(&*path).unwrap();
        let code = Code::new(&Arc::new(code));
        let mut map = CodeMap::default();
        let span = map.add(Some(&path), None, &code);
        let mut lexer = Lexer::new(&path, &code, span.lo);
        let mut handler = WriteToStderr;
        let mut sink = DiagnosticSink::new(&mut handler);
        let mut tokens = vec![];
        let mut meaning_cache = MeaningCache::default();
        let mut productions = vec![];
        loop {
            tokens.clear();
            let res = lexer.lex_line(&mut map, &mut sink, &mut interner, &mut tokens);
            if let Err(e) = res {
                sink.push(&mut map, e.val.diagnostic_kind(), e);
                return;
            }
            if tokens.is_empty() {
                break;
            }
            let line = parse_line(
                &mut map,
                &mut sink,
                &mut interner,
                &mut meaning_cache,
                &tokens,
                &self.env,
                locale_dir.as_deref(),
            );
            let line = match line {
                Ok(l) => l,
                Err(e) => {
                    sink.push(&mut map, e.val.diagnostic_kind(), e);
                    return;
                }
            };
            if let Line::Production(p) = line.val {
                productions.push(p.spanned2(line.span));
            }
        }
        let table = ComposeTable::new(&mut map, &mut sink, &productions);
        println!("{}", table.format());
    }

    fn get_compose_file(&self, resolved_locale: &str) -> Option<String> {
        let Some(mut ll) = self.read_locale_lines("compose.dir") else {
            return None;
        };
        while let Some((l, r)) = ll.next() {
            if r == resolved_locale {
                return Some(format!("{}/{}", self.env.xlocaledir, l));
            }
        }
        None
    }

    fn resolve_locale(&self, locale: &str) -> String {
        let Some(mut ll) = self.read_locale_lines("locale.alias") else {
            return locale.to_string();
        };
        while let Some((l, r)) = ll.next() {
            if l == locale {
                return r.to_string();
            }
        }
        locale.to_string()
    }

    fn read_locale_lines(&self, filename: &str) -> Option<LocaleLines> {
        let path = Path::new(&self.env.xlocaledir).join(filename);
        let file = File::open(&path).ok()?;
        Some(LocaleLines {
            buf: String::new(),
            reader: BufReader::new(file),
        })
    }
}

struct LocaleLines {
    buf: String,
    reader: BufReader<File>,
}

impl LocaleLines {
    fn next<'a>(&'a mut self) -> Option<(&'a str, &'a str)> {
        loop {
            self.buf.clear();
            if self.reader.read_line(&mut self.buf).ok()? == 0 {
                return None;
            }
            let mut buf = self.buf.as_str();
            if let Some((l, _)) = buf.split_once('#') {
                buf = l;
            }
            let (l, r) = if let Some((l, r)) = buf.split_once(':') {
                (l, r)
            } else if let Some((l, r)) = buf.split_once([' ', '\t']) {
                (l, r)
            } else {
                continue;
            };
            unsafe {
                type Ret<'b> = Option<(&'b str, &'b str)>;
                // SAFETY: This is a compiler bug that will never be fixed.
                return mem::transmute::<Ret<'_>, Ret<'a>>(Some((l.trim(), r.trim())));
            }
        }
    }
}
