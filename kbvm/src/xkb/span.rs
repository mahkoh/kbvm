use {
    kbvm_proc::CloneWithDelta,
    std::{
        fmt::{Debug, Formatter},
        hash::{Hash, Hasher},
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

impl<T> PartialEq<T> for Spanned<T>
where
    T: PartialEq,
{
    fn eq(&self, other: &T) -> bool {
        &self.val == other
    }
}

impl<T> Hash for Spanned<T>
where
    T: Hash,
{
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.val.hash(state)
    }
}

impl<T: Debug> Debug for Spanned<T> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?} @ ", self.span)?;
        Debug::fmt(&self.val, f)
    }
}

impl<T> Spanned<T> {
    pub(crate) fn map<U>(self, f: impl FnOnce(T) -> U) -> Spanned<U> {
        Spanned {
            span: self.span,
            val: f(self.val),
        }
    }

    pub(crate) fn _as_ref(&self) -> Spanned<&T> {
        Spanned {
            span: self.span,
            val: &self.val,
        }
    }

    pub(crate) fn as_ref(&self) -> Spanned<&T> {
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

pub(crate) trait SpanResult1<T, E>: Sized {
    fn span_map<U>(self, f: impl FnOnce(T) -> U) -> Result<Spanned<U>, E>;
}

impl<T, E> SpanResult1<T, E> for Result<Spanned<T>, E> {
    fn span_map<U>(self, f: impl FnOnce(T) -> U) -> Result<Spanned<U>, E> {
        self.map(|s| s.map(f))
    }
}

pub(crate) trait SpanResult2<T, E>: Sized {
    fn span_either(self, span: Span) -> Result<Spanned<T>, Spanned<E>>;
}

impl<T, E> SpanResult2<T, E> for Result<T, E> {
    fn span_either(self, span: Span) -> Result<Spanned<T>, Spanned<E>> {
        match self {
            Ok(e) => Ok(e.spanned2(span)),
            Err(e) => Err(e.spanned2(span)),
        }
    }
}

pub(crate) trait Despan {
    type Output;

    fn despan(self) -> Self::Output;
}

impl<T> Despan for Option<Spanned<T>> {
    type Output = Option<T>;

    fn despan(self) -> Self::Output {
        self.map(|v| v.val)
    }
}
