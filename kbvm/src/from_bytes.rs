#[cfg(test)]
mod tests;

pub trait FromBytes: Sized {
    fn from_bytes_dec(b: &[u8]) -> Option<Self>;

    fn from_bytes_hex(b: &[u8]) -> Option<Self>;

    fn from_bytes(b: &[u8], hex: bool) -> Option<Self> {
        match hex {
            true => Self::from_bytes_hex(b),
            false => Self::from_bytes_dec(b),
        }
    }
}

macro_rules! from_bytes {
    ($ty:ty, $signed:expr) => {
        impl FromBytes for $ty {
            fn from_bytes_dec(b: &[u8]) -> Option<Self> {
                if b.len() == 0 {
                    return None;
                }
                let mut res = 0 as $ty;
                for &c in b {
                    match c {
                        b'0'..=b'9' => res = res.checked_mul(10)?.checked_add((c - b'0') as $ty)?,
                        _ => return None,
                    }
                }
                Some(res)
            }

            fn from_bytes_hex(mut b: &[u8]) -> Option<Self> {
                if b.len() == 0 {
                    return None;
                }
                const MAX_BYTES: usize = <$ty>::BITS as usize / 8;
                const MAX_CHARS: usize = MAX_BYTES * 2;
                if b.len() > MAX_CHARS || ($signed && b.len() == MAX_CHARS) {
                    while b.len() > 0 && b[0] == 0 {
                        b = &b[1..];
                    }
                    if b.len() > MAX_CHARS {
                        return None;
                    }
                    if $signed && b.len() == MAX_CHARS {
                        if matches!(b[0], b'8'..=b'9' | b'A'..=b'F' | b'a'..=b'f') {
                            return None;
                        }
                    }
                }
                let mut res = 0;
                for &c in b {
                    res = res << 4
                        | match c {
                        b'0'..=b'9' => c - b'0',
                        b'A'..=b'F' => c - b'A' + 10,
                        b'a'..=b'f' => c - b'a' + 10,
                        _ => return None,
                    } as $ty;
                }
                Some(res)
            }
        }
    };
}

from_bytes!(u32, false);
from_bytes!(i64, true);
