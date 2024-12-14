use {
    crate::xkb::{code_map::CodeMap, code_slice::CodeSlice, span::Span},
    bstr::ByteSlice,
    std::{
        error::Error,
        fmt::{Debug, Display, Formatter},
        ops::Deref,
        path::PathBuf,
        sync::Arc,
    },
};

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub enum Severity {
    Warning,
    Error,
}

pub struct Diagnostic {
    severity: Severity,
    message: Box<dyn Display + Send + Sync>,
    source_file: Option<Arc<PathBuf>>,
    line: CodeSlice<'static>,
    line_num: usize,
    in_line_offset: usize,
    in_line_len: usize,
    inner: Option<Box<Diagnostic>>,
}

struct WithCode<'a> {
    diagnostic: &'a Diagnostic,
}

impl Diagnostic {
    pub(crate) fn new(
        map: &mut CodeMap,
        severity: Severity,
        message: impl Display + Send + Sync + 'static,
        span: Span,
    ) -> Self {
        let message = Box::new(message);
        Self::new_(map, severity, message, span, None)
    }

    fn new_(
        map: &mut CodeMap,
        severity: Severity,
        message: Box<dyn Display + Send + Sync>,
        span: Span,
        inner: Option<Box<Diagnostic>>,
    ) -> Self {
        let info = map.get(span);
        let lo = span.lo.max(info.span.lo);
        let hi = span.hi.min(info.span.hi);
        let line_idx = info
            .lines
            .binary_search_by(|r| r.cmp(&lo))
            .unwrap_or_else(|i| i - 1);
        let line_lo = info.lines[line_idx];
        let line_hi = info
            .lines
            .get(line_idx + 1)
            .map(|l| *l - 1)
            .unwrap_or(info.span.hi);
        let in_line_offset = (lo - line_lo) as usize;
        let in_line_len = (hi - lo) as usize;
        let line_lo = (line_lo - info.span.lo) as usize;
        let line_hi = (line_hi - info.span.lo) as usize;
        let slice = info.code.to_slice().slice(line_lo..line_hi).to_owned();
        let mut res = Self {
            severity,
            message,
            source_file: info.file.cloned(),
            line: slice,
            line_num: line_idx + 1,
            in_line_offset,
            in_line_len,
            inner,
        };
        if let Some(span) = info.include_span {
            struct CouldNotProcessInclude;
            impl Display for CouldNotProcessInclude {
                fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
                    f.write_str("could not process include")
                }
            }
            res = Self::new_(
                map,
                severity,
                Box::new(CouldNotProcessInclude),
                span,
                Some(Box::new(res)),
            )
        }
        res
    }

    pub fn with_code(&self) -> impl Display + use<'_> {
        WithCode { diagnostic: &self }
    }

    pub fn severity(&self) -> Severity {
        self.severity
    }

    fn fmt(&self, f: &mut Formatter<'_>, with_code: bool) -> std::fmt::Result {
        if let Some(path) = &self.source_file {
            write!(f, "at {}", path.display())?;
        } else {
            write!(f, "at <anonymous file>")?;
        }
        write!(
            f,
            " {}:{}: {}",
            self.line_num, self.in_line_offset, self.message
        )?;
        if with_code {
            f.write_str(":\n")?;
            write!(f, ">> {}\n   ", self.line.as_bytes().as_bstr())?;
            for _ in 0..self.in_line_offset {
                f.write_str(" ")?
            }
            f.write_str("^")?;
            for _ in 1..self.in_line_len {
                f.write_str("~")?;
            }
            f.write_str("\n")?;
            if let Some(inner) = &self.inner {
                inner.deref().fmt(f, true)?;
            }
        }
        Ok(())
    }
}

impl Display for WithCode<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        self.diagnostic.fmt(f, true)
    }
}

impl Debug for Diagnostic {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        self.fmt(f, false)
    }
}

impl Display for Diagnostic {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        self.fmt(f, false)
    }
}

impl Error for Diagnostic {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        self.inner.as_deref().map(|d| d as &dyn Error)
    }
}
