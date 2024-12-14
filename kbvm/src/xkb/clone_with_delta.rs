use crate::xkb::span::Span;

pub(crate) trait CloneWithDelta: Sized {
    fn clone_with_delta(&self, delta: u64) -> Self;
}

impl CloneWithDelta for Span {
    fn clone_with_delta(&self, delta: u64) -> Self {
        *self + delta
    }
}

impl<T> CloneWithDelta for Vec<T>
where
    T: CloneWithDelta,
{
    fn clone_with_delta(&self, delta: u64) -> Self {
        self.iter().map(|t| t.clone_with_delta(delta)).collect()
    }
}

impl<T> CloneWithDelta for Option<T>
where
    T: CloneWithDelta,
{
    fn clone_with_delta(&self, delta: u64) -> Self {
        self.as_ref().map(|t| t.clone_with_delta(delta))
    }
}

impl<T> CloneWithDelta for Box<T>
where
    T: CloneWithDelta,
{
    fn clone_with_delta(&self, delta: u64) -> Self {
        Box::new((**self).clone_with_delta(delta))
    }
}

macro_rules! copy {
    ($ty:ty) => {
        impl CloneWithDelta for $ty {
            fn clone_with_delta(&self, _delta: u64) -> Self {
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
