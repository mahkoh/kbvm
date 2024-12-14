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
    #[error("include component does not contain a file name")]
    MissingFileName,
    #[error("map name is not terminated with a `)`")]
    UnterminatedMapName,
    #[error("include component does not start with a merge mode")]
    MissingMergeMode,
    #[error("invalid group index `{}`", .0.as_bytes().as_bstr())]
    InvalidGroupIndex(CodeSlice<'static>),
}

impl ParseIncludeError {
    pub(crate) fn diagnostic_kind(&self) -> DiagnosticKind {
        match self {
            ParseIncludeError::MissingFileName => DiagnosticKind::MissingIncludeFileName,
            ParseIncludeError::UnterminatedMapName => DiagnosticKind::UnterminatedIncludeMapName,
            ParseIncludeError::MissingMergeMode => DiagnosticKind::MissingIncludeMergeMode,
            ParseIncludeError::InvalidGroupIndex(_) => DiagnosticKind::InvalidIncludeGroupIndex,
        }
    }
}

pub(super) fn missing_file_name(span: Span) -> Spanned<ParseIncludeError> {
    ParseIncludeError::MissingFileName.spanned2(span)
}

pub(super) fn unterminated_map_name(lo: u64, hi: u64) -> Spanned<ParseIncludeError> {
    ParseIncludeError::UnterminatedMapName.spanned(lo, hi)
}

pub(super) fn missing_merge_mode(span: Span) -> Spanned<ParseIncludeError> {
    ParseIncludeError::MissingMergeMode.spanned2(span)
}

pub(super) fn invalid_group(s: &CodeSlice<'_>, lo: u64, hi: u64) -> Spanned<ParseIncludeError> {
    ParseIncludeError::InvalidGroupIndex(s.to_owned()).spanned(lo, hi)
}
