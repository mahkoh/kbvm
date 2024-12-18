use {
    crate::xkb::{
        code_slice::CodeSlice,
        diagnostic::DiagnosticKind,
        span::{Span, SpanExt, Spanned},
    },
    bstr::ByteSlice,
    thiserror::Error,
};

#[derive(Debug, Clone, PartialEq, Error)]
pub(crate) enum ParseIncludeError {
    #[error("invalid format")]
    InvalidFormat,
    #[error("missing merge mode")]
    MissingMergeMode,
    #[error("invalid group index `{}`", .0.as_bytes().as_bstr())]
    InvalidGroupIndex(CodeSlice<'static>),
}

impl ParseIncludeError {
    pub(crate) fn diagnostic_kind(&self) -> DiagnosticKind {
        DiagnosticKind::SyntaxError
    }
}

pub(super) fn invalid_format(span: Span) -> Spanned<ParseIncludeError> {
    ParseIncludeError::InvalidFormat.spanned2(span)
}

pub(super) fn missing_merge_mode(span: Span) -> Spanned<ParseIncludeError> {
    ParseIncludeError::MissingMergeMode.spanned2(span)
}

pub(super) fn invalid_group(s: &CodeSlice<'_>, span: Span) -> Spanned<ParseIncludeError> {
    ParseIncludeError::InvalidGroupIndex(s.to_owned()).spanned2(span)
}
