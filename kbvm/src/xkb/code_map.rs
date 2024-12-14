use {
    crate::xkb::{code::Code, span::Span},
    hashbrown::HashMap,
    std::{path::PathBuf, sync::Arc},
};

#[derive(Default)]
pub(crate) struct CodeMap {
    ranges: Vec<CodeRange>,
    canonical_idx: HashMap<*const u8, usize>,
}

struct CodeRange {
    span: Span,
    code: Code,
    file: Option<Arc<PathBuf>>,
    canonical_idx: Option<usize>,
    lines: Option<Vec<u64>>,
    include_span: Option<Span>,
}

pub(crate) struct CodeInfo<'a> {
    pub(crate) code: &'a Code,
    pub(crate) span: Span,
    pub(crate) file: Option<&'a Arc<PathBuf>>,
    pub(crate) lines: &'a [u64],
    pub(crate) lines_offset: u64,
    pub(crate) include_span: Option<Span>,
}

impl CodeRange {
    fn create_lines(&mut self) {
        if self.lines.is_some() {
            return;
        }
        let mut lines = vec![];
        let mut lo = self.span.lo;
        lines.push(lo);
        let mut code = self.code.as_bytes();
        while code.len() > 0 {
            let Some(pos) = code.iter().position(|&c| c == b'\n') else {
                break;
            };
            code = &code[pos + 1..];
            lo += pos as u64 + 1;
            lines.push(lo);
        }
        self.lines = Some(lines);
    }
}

impl CodeMap {
    pub(crate) fn add(
        &mut self,
        file: Option<&Arc<PathBuf>>,
        include_span: Option<Span>,
        code: &Code,
    ) -> Span {
        let lo = self
            .ranges
            .last()
            .map(|l| l.span.hi + 1)
            .unwrap_or_default();
        let span = Span {
            lo,
            hi: lo + code.len() as u64,
        };
        let canonical_idx = self
            .canonical_idx
            .try_insert(code.as_ptr(), self.ranges.len())
            .err()
            .map(|e| *e.entry.get());
        self.ranges.push(CodeRange {
            span,
            code: code.clone(),
            file: file.cloned(),
            canonical_idx,
            lines: None,
            include_span,
        });
        span
    }

    pub(crate) fn get(&mut self, span: Span) -> CodeInfo<'_> {
        let range = self.ranges.binary_search_by_key(&span.lo, |r| r.span.hi);
        let idx = range.unwrap_or_else(|i| i);
        let (lo, hi) = self.ranges.split_at_mut(idx);
        let range = &mut hi[0];
        let (lines, lines_offset) = match range.canonical_idx {
            Some(idx) => {
                let crange = &mut lo[idx];
                crange.create_lines();
                (
                    crange.lines.as_ref().unwrap(),
                    range.span.lo - crange.span.lo,
                )
            }
            _ => {
                range.create_lines();
                (range.lines.as_ref().unwrap(), 0)
            }
        };
        CodeInfo {
            code: &range.code,
            span: range.span,
            file: range.file.as_ref(),
            lines,
            lines_offset,
            include_span: range.include_span,
        }
    }
}
