use crate::xkb::span::{Span, SpanUnit};

pub(crate) trait CloneWithDelta: Sized {
    fn clone_with_delta(&self, delta: SpanUnit) -> Self;
}

impl CloneWithDelta for Span {
    fn clone_with_delta(&self, delta: SpanUnit) -> Self {
        *self + delta
    }
}

impl<T> CloneWithDelta for Vec<T>
where
    T: CloneWithDelta,
{
    fn clone_with_delta(&self, delta: SpanUnit) -> Self {
        self.iter().map(|t| t.clone_with_delta(delta)).collect()
    }
}

impl<T> CloneWithDelta for Option<T>
where
    T: CloneWithDelta,
{
    fn clone_with_delta(&self, delta: SpanUnit) -> Self {
        self.as_ref().map(|t| t.clone_with_delta(delta))
    }
}

impl<T> CloneWithDelta for Box<T>
where
    T: CloneWithDelta,
{
    fn clone_with_delta(&self, delta: SpanUnit) -> Self {
        Box::new((**self).clone_with_delta(delta))
    }
}

impl<T> CloneWithDelta for Box<[T]>
where
    T: CloneWithDelta,
{
    fn clone_with_delta(&self, delta: SpanUnit) -> Self {
        self.iter().map(|t| t.clone_with_delta(delta)).collect()
    }
}

macro_rules! copy {
    ($ty:ty) => {
        impl CloneWithDelta for $ty {
            fn clone_with_delta(&self, _delta: SpanUnit) -> Self {
                *self
            }
        }
    };
}

copy!(usize);
copy!(u32);
copy!(u64);
copy!(bool);
copy!(f64);
copy!(i64);
