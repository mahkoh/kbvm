use {
    crate::xkb::{
        code_map::CodeMap,
        code_slice::CodeSlice,
        span::{Span, Spanned},
    },
    bstr::ByteSlice,
    debug_fn::debug_fn,
    std::{
        error::Error,
        fmt::{Debug, Display, Formatter},
        ops::Deref,
        path::PathBuf,
        sync::Arc,
    },
};

pub struct DiagnosticSink<'a> {
    diagnostics: &'a mut Vec<Diagnostic>,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
#[non_exhaustive]
pub enum DiagnosticKind {
    OctalOverflow,
    UnknownEscapeSequence,
    FileReadFailed,
    FileNotFound,
    UnexpectedConfigItemType,
    MultipleDefaultItems,
    DuplicateItemName,
    SyntaxError,
    UnexpectedDeclaration,
    InvalidKeysym,
    InvalidAction,
    RecursiveInclude,
    InvalidIndicatorName,
    InvalidIndicatorIndex,
    TooManyIndicators,
    UnknownVariable,
    InvalidModifierValue,
    InvalidLevelValue,
    InvalidLevelName,
    DuplicateKeyNameDefinition,
    DuplicateKeyCode,
    UnknownKeyAlias,
    DuplicateKeyTypeDefinition,
    UnknownKeysym,
    InvalidFilter,
    InvalidTypeField,
    InvalidInterpField,
    IgnoredInterpField,
    InvalidIndicatorField,
    IgnoredIndicatorField,
    InvalidActionDefault,
    UnknownModifier,
    InvalidModMapEntry,
    IgnoredModMapEntry,
    UnknownKey,
    InvalidSymbolsField,
    IgnoredSymbolsField,
    DiscardingGroup,
    MissingGroupName,
    InvalidGroupName,
    InvalidGroupIndex,
    TooManyVirtualModifiers,
}

pub struct Diagnostic {
    kind: DiagnosticKind,
    location: DiagnosticLocation,
}

pub(crate) struct DiagnosticLocation {
    message: Option<Box<dyn Display + Send + Sync>>,
    source_file: Option<Arc<PathBuf>>,
    line: CodeSlice<'static>,
    line_num: usize,
    in_line_offset: usize,
    in_line_len: usize,
    inner: Option<Box<DiagnosticLocation>>,
}

struct WithCode<'a> {
    diagnostic: &'a Diagnostic,
}

impl<'a> DiagnosticSink<'a> {
    pub fn new(diagnostics: &'a mut Vec<Diagnostic>) -> Self {
        Self { diagnostics }
    }

    pub(crate) fn push(
        &mut self,
        map: &mut CodeMap,
        kind: DiagnosticKind,
        message: Spanned<impl Display + Send + Sync + 'static>,
    ) {
        let diagnostic = Diagnostic::new(map, kind, message.val, message.span);
        self.diagnostics.push(diagnostic);
    }
}

impl DiagnosticLocation {
    fn new(
        map: &mut CodeMap,
        message: Option<Box<dyn Display + Send + Sync>>,
        span: Span,
        inner: Option<Box<DiagnosticLocation>>,
    ) -> Self {
        let info = map.get(span);
        let lo = span.lo.max(info.span.lo) - info.lines_offset;
        let hi = span.hi.min(info.span.hi) - info.lines_offset;
        let line_idx = info
            .lines
            .binary_search_by(|r| r.cmp(&lo))
            .unwrap_or_else(|i| i - 1);
        let line_lo = info.lines[line_idx];
        let line_hi = info
            .lines
            .get(line_idx + 1)
            .map(|l| *l - 1)
            .unwrap_or(info.span.hi - info.lines_offset);
        let in_line_offset = (lo - line_lo) as usize;
        let in_line_len = (hi - lo) as usize;
        let line_lo = (line_lo + info.lines_offset - info.span.lo) as usize;
        let line_hi = (line_hi + info.lines_offset - info.span.lo) as usize;
        let slice = info.code.to_slice().slice(line_lo..line_hi).to_owned();
        let mut res = Self {
            message,
            source_file: info.file.cloned(),
            line: slice,
            line_num: line_idx + 1,
            in_line_offset,
            in_line_len,
            inner,
        };
        if let Some(span) = info.include_span {
            res = Self::new(map, None, span, Some(Box::new(res)))
        }
        res
    }

    fn fmt(&self, f: &mut Formatter<'_>, with_code: bool) -> std::fmt::Result {
        write!(
            f,
            "at {} {}:{}: {}",
            debug_fn(|f| match &self.source_file {
                Some(p) => Display::fmt(&p.display(), f),
                None => f.write_str("<anonymous file>"),
            }),
            self.line_num,
            self.in_line_offset,
            debug_fn(|f| match &self.message {
                Some(m) => m.fmt(f),
                None => f.write_str("while processing include"),
            }),
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

impl Diagnostic {
    pub(crate) fn new(
        map: &mut CodeMap,
        kind: DiagnosticKind,
        message: impl Display + Send + Sync + 'static,
        span: Span,
    ) -> Self {
        Self {
            kind,
            location: DiagnosticLocation::new(map, Some(Box::new(message)), span, None),
        }
    }

    pub fn with_code(&self) -> impl Display + use<'_> {
        WithCode { diagnostic: &self }
    }

    pub fn kind(&self) -> DiagnosticKind {
        self.kind
    }
}

impl Display for WithCode<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        self.diagnostic.location.fmt(f, true)
    }
}

impl Debug for Diagnostic {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        self.location.fmt(f, false)
    }
}

impl Display for Diagnostic {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        self.location.fmt(f, false)
    }
}

impl Error for Diagnostic {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        self.location.source()
    }
}

impl Debug for DiagnosticLocation {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        self.fmt(f, false)
    }
}

impl Display for DiagnosticLocation {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        self.fmt(f, false)
    }
}

impl Error for DiagnosticLocation {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        self.inner.as_deref().map(|d| d as &dyn Error)
    }
}
