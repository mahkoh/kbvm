use {
    crate::xkb::code::Code,
    std::{path::PathBuf, sync::Arc},
};

#[derive(Debug)]
pub(crate) struct Lexer {
    path: Option<Arc<PathBuf>>,
    code: Code,
    span_lo: u64,
    pos: usize,
}
