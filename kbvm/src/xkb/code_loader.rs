#[cfg(test)]
mod tests;

use {
    crate::xkb::{
        code::Code,
        interner::{Interned, Interner},
    },
    cfg_if::cfg_if,
    hashbrown::HashMap,
    std::{
        io::{self},
        ops::Range,
        path::{Path, PathBuf},
        sync::Arc,
    },
    thiserror::Error,
};

pub(crate) struct CodeLoader {
    include_paths: Vec<Arc<PathBuf>>,
    cache: HashMap<Key, Vec<Option<CodeFromPath>>>,
    partial_path: Vec<u8>,
    full_path: PathBuf,
}

#[derive(Clone)]
struct CodeFromPath {
    path: Arc<PathBuf>,
    code: Code,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
struct Key {
    ty: CodeType,
    path: Interned,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub(crate) enum CodeType {
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
pub(crate) struct CodeLoaderError {
    path: String,
    #[source]
    err: io::Error,
}

impl CodeLoader {
    pub(crate) fn new(include_paths: &[Arc<PathBuf>]) -> Self {
        Self {
            include_paths: include_paths.to_vec(),
            cache: Default::default(),
            partial_path: vec![],
            full_path: Default::default(),
        }
    }

    pub(crate) fn load(
        &mut self,
        interner: &Interner,
        ty: CodeType,
        path: Interned,
    ) -> CodeIter<'_> {
        let key = Key { ty, path };
        let entry = self.cache.entry(key).or_default();
        let path = interner.get(path).clone();
        self.partial_path.clear();
        self.partial_path.extend_from_slice(&path);
        cfg_if! {
            if #[cfg(unix)] {
                use std::os::unix::ffi::OsStrExt;
                use std::ffi::OsStr;
                let path = Some(Path::new(OsStr::from_bytes(&self.partial_path)));
            } else {
                let path = match std::str::from_utf8(&self.partial_path) {
                    Ok(s) => Some(Path::new(s)),
                    _ => None,
                };
            }
        }
        let ty = match path {
            None => CodeIterTy::Absolute { path: None },
            Some(p) => match p.is_absolute() {
                true => CodeIterTy::Absolute { path: Some(p) },
                false => CodeIterTy::Relative {
                    pos: 0..self.include_paths.len(),
                    path: p,
                    paths: &self.include_paths,
                    ty,
                    full_path: &mut self.full_path,
                },
            },
        };
        CodeIter { entry, ty }
    }
}

pub(crate) struct CodeIter<'a> {
    entry: &'a mut Vec<Option<CodeFromPath>>,
    ty: CodeIterTy<'a>,
}

pub(crate) enum CodeIterTy<'a> {
    Absolute {
        path: Option<&'a Path>,
    },
    Relative {
        pos: Range<usize>,
        path: &'a Path,
        paths: &'a [Arc<PathBuf>],
        ty: CodeType,
        full_path: &'a mut PathBuf,
    },
}

impl Iterator for CodeIter<'_> {
    type Item = Result<(Arc<PathBuf>, Code), CodeLoaderError>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let (pos, path) = match &mut self.ty {
                CodeIterTy::Absolute { path } => (0, Some(path.take()?)),
                CodeIterTy::Relative { pos, .. } => (pos.next()?, None),
            };
            if let Some(entry) = self.entry.get(pos) {
                if let Some(code) = entry {
                    return Some(Ok((code.path.clone(), code.code.clone())));
                }
                continue;
            }
            let path = match path {
                Some(p) => p,
                _ => match &mut self.ty {
                    CodeIterTy::Absolute { path } => path.take()?,
                    CodeIterTy::Relative {
                        path,
                        paths,
                        ty,
                        full_path,
                        ..
                    } => {
                        let sub_dir = match ty {
                            CodeType::Keycodes => "keycodes",
                            CodeType::Types => "types",
                            CodeType::Compat => "compat",
                            CodeType::Symbols => "symbols",
                            CodeType::Geometry => "geometry",
                            CodeType::Keymap => "keymap",
                            CodeType::Rules => "rules",
                        };
                        full_path.clear();
                        full_path.push(&*paths[pos]);
                        full_path.push(sub_dir);
                        full_path.push(path);
                        full_path
                    }
                },
            };
            let code = match std::fs::read(path) {
                Ok(b) => Arc::new(b),
                Err(e) if e.kind() == io::ErrorKind::NotFound => {
                    self.entry.push(None);
                    continue;
                }
                Err(err) => {
                    return Some(Err(CodeLoaderError {
                        path: path.display().to_string(),
                        err,
                    }));
                }
            };
            let code = Code::new(&code);
            let res = CodeFromPath {
                path: Arc::new(path.to_path_buf()),
                code,
            };
            self.entry.push(Some(res.clone()));
            return Some(Ok((res.path, res.code)));
        }
    }
}
