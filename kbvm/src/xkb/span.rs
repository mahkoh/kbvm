use {
    crate::xkb::{
        code_map::CodeMap,
        diagnostic::{Diagnostic, Severity},
    },
    kbvm_proc::CloneWithDelta,
    std::{
        fmt::{Debug, Display, Formatter},
        ops::Add,
    },
};

#[derive(Copy, Clone, Eq, PartialEq)]
pub(crate) struct Span {
    pub(crate) lo: u64,
    pub(crate) hi: u64,
}

impl Add<u64> for Span {
    type Output = Self;

    fn add(self, rhs: u64) -> Self::Output {
        Self {
            lo: self.lo.wrapping_add(rhs),
            hi: self.hi.wrapping_add(rhs),
        }
    }
}

impl Debug for Span {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}..{}", self.lo, self.hi)
    }
}

#[derive(Copy, Clone, Eq, CloneWithDelta)]
pub(crate) struct Spanned<T> {
    pub(crate) span: Span,
    pub(crate) val: T,
}

impl<T> PartialEq for Spanned<T>
where
    T: PartialEq,
{
    fn eq(&self, other: &Self) -> bool {
        self.val == other.val
    }
}

impl<T: Debug> Debug for Spanned<T> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?} @ ", self.span)?;
        Debug::fmt(&self.val, f)
    }
}

impl<T> Spanned<T> {
    pub fn map<U>(self, f: impl FnOnce(T) -> U) -> Spanned<U> {
        Spanned {
            span: self.span,
            val: f(self.val),
        }
    }

    pub fn as_ref(&self) -> Spanned<&T> {
        Spanned {
            span: self.span,
            val: &self.val,
        }
    }
}

pub(crate) trait SpanExt: Sized {
    fn spanned(self, lo: u64, hi: u64) -> Spanned<Self>;
    fn spanned2(self, span: Span) -> Spanned<Self>;
}

impl<T> SpanExt for T {
    fn spanned(self, lo: u64, hi: u64) -> Spanned<Self> {
        self.spanned2(Span { lo, hi })
    }

    fn spanned2(self, span: Span) -> Spanned<Self> {
        Spanned { span, val: self }
    }
}

pub(crate) trait SpanMap<T>: Sized {
    type Output<U>;
    fn span_map<U>(self, f: impl FnOnce(T) -> U) -> Self::Output<U>;
}

impl<T, E> SpanMap<T> for Result<Spanned<T>, E> {
    type Output<U> = Result<Spanned<U>, E>;

    fn span_map<U>(self, f: impl FnOnce(T) -> U) -> Self::Output<U> {
        self.map(|s| s.map(f))
    }
}

impl<E> Spanned<E>
where
    E: Display + Send + Sync + 'static,
{
    pub fn into_diagnostic(self, map: &mut CodeMap, severity: Severity) -> Diagnostic {
        Diagnostic::new(map, severity, self.val, self.span)
    }
}
