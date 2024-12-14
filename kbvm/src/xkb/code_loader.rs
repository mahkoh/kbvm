use {
    crate::xkb::{
        code::Code,
        code_slice::CodeSlice,
        interner::{Interned, Interner},
    },
    hashbrown::HashMap,
    std::{
        ffi::OsStr,
        io::{self},
        ops::Range,
        path::PathBuf,
        sync::Arc,
    },
    thiserror::Error,
};

pub struct CodeLoader {
    include_paths: Vec<Arc<PathBuf>>,
    cache: HashMap<Key, Vec<Option<(Arc<PathBuf>, Code)>>>,
    full_path: PathBuf,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
struct Key {
    ty: CodeType,
    path: Interned,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub enum CodeType {
    Keycodes,
    Types,
    Compat,
    Symbols,
    Geometry,
    Keymap,
    Rules,
}

#[derive(Debug, Error)]
#[error("could not read file {path}")]
pub struct CodeLoaderError {
    path: String,
    #[source]
    err: io::Error,
}

impl CodeLoader {
    pub fn new(include_paths: &[Arc<PathBuf>]) -> Self {
        Self {
            include_paths: include_paths.to_vec(),
            cache: Default::default(),
            full_path: Default::default(),
        }
    }

    pub fn load(&mut self, interner: &Interner, ty: CodeType, path: Interned) -> CodeIter<'_> {
        let key = Key { ty, path };
        let entry = self.cache.entry(key).or_default();
        CodeIter {
            pos: 0..self.include_paths.len(),
            partial_path: interner.get(path).clone(),
            paths: &self.include_paths,
            entry,
            ty,
            full_path: &mut self.full_path,
        }
    }
}

pub struct CodeIter<'a> {
    pos: Range<usize>,
    partial_path: CodeSlice<'static>,
    paths: &'a [Arc<PathBuf>],
    entry: &'a mut Vec<Option<(Arc<PathBuf>, Code)>>,
    ty: CodeType,
    full_path: &'a mut PathBuf,
}

impl Iterator for CodeIter<'_> {
    type Item = Result<(Arc<PathBuf>, Code), CodeLoaderError>;

    fn next(&mut self) -> Option<Self::Item> {
        while let Some(pos) = self.pos.next() {
            if let Some(entry) = self.entry.get(pos) {
                if let Some(code) = entry {
                    return Some(Ok(code.clone()));
                }
                continue;
            }
            self.full_path.clone_from(&self.paths[pos]);
            let sub_dir = match self.ty {
                CodeType::Keycodes => "keycodes",
                CodeType::Types => "types",
                CodeType::Compat => "compat",
                CodeType::Symbols => "symbols",
                CodeType::Geometry => "geometry",
                CodeType::Keymap => "keymap",
                CodeType::Rules => "rules",
            };
            self.full_path.push(sub_dir);
            #[cfg(unix)]
            let partial_path =
                <OsStr as std::os::unix::ffi::OsStrExt>::from_bytes(self.partial_path.as_bytes());
            #[cfg(not(unix))]
            let partial_path = match std::str::from_utf8(self.partial_path.as_bytes()) {
                Ok(p) => OsStr::new(p),
                Err(e) => {
                    return Some(Err(CodeLoaderError {
                        path: self.partial_path.to_str_lossy().to_string(),
                        err: io::Error::new(io::ErrorKind::NotFound, e),
                    }));
                }
            };
            self.full_path.push(partial_path);
            let code = match std::fs::read(&self.full_path) {
                Ok(b) => Arc::new(b),
                Err(e) if e.kind() == io::ErrorKind::NotFound => {
                    self.entry.push(None);
                    continue;
                }
                Err(err) => {
                    return Some(Err(CodeLoaderError {
                        path: self.full_path.display().to_string(),
                        err,
                    }))
                }
            };
            let code = Code::new(&code);
            let res = (Arc::new(self.full_path.clone()), code);
            self.entry.push(Some(res.clone()));
            return Some(Ok(res));
        }
        None
    }
}
