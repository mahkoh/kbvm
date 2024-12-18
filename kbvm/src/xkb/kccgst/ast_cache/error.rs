use {
    crate::xkb::{
        code_slice::CodeSlice,
        diagnostic::DiagnosticKind,
        interner::{Interned, Interner},
        span::{Span, SpanExt, Spanned},
    },
    bstr::ByteSlice,
    debug_fn::debug_fn,
    std::{fmt, fmt::Formatter},
    thiserror::Error,
};

#[derive(Clone, Debug, Error)]
pub(crate) enum AstCacheError {
    #[error("not found: `{}{}`", .file.as_bytes().as_bstr(), debug_fn(|f| show_map(f, &.map)))]
    NotFound {
        file: CodeSlice<'static>,
        map: Option<CodeSlice<'static>>,
    },
}

impl AstCacheError {
    pub(crate) fn diagnostic_kind(&self) -> DiagnosticKind {
        DiagnosticKind::FileNotFound
    }
}

fn show_map(f: &mut Formatter<'_>, map: &Option<CodeSlice<'_>>) -> fmt::Result {
    if let Some(m) = map {
        write!(f, ":{}", m.as_bytes().as_bstr())?;
    }
    Ok(())
}

pub(super) fn not_found(
    interner: &Interner,
    name: Interned,
    map: Option<Interned>,
    span: Span,
) -> Spanned<AstCacheError> {
    AstCacheError::NotFound {
        file: interner.get(name).to_owned(),
        map: map.map(|m| interner.get(m).to_owned()),
    }
    .spanned2(span)
}
