use crate::from_bytes::FromBytes;

#[test]
fn test() {
    for i in 0..0xfff {
        assert_eq!(
            u32::from_bytes_hex(format!("{i:x}").as_bytes()),
            Some(i as u32)
        );
        assert_eq!(
            i64::from_bytes_hex(format!("{i:x}").as_bytes()),
            Some(i as i64)
        );
        assert_eq!(
            u32::from_bytes_dec(format!("{i}").as_bytes()),
            Some(i as u32)
        );
        assert_eq!(
            i64::from_bytes_dec(format!("{i}").as_bytes()),
            Some(i as i64)
        );
    }
    assert_eq!(u32::from_bytes_hex(b"ffffffff"), Some(u32::MAX));
    assert_eq!(i64::from_bytes_hex(b"7fffffffffffffff"), Some(i64::MAX));
    assert_eq!(
        u32::from_bytes_dec(u32::MAX.to_string().as_bytes()),
        Some(u32::MAX)
    );
    assert_eq!(
        i64::from_bytes_dec(i64::MAX.to_string().as_bytes()),
        Some(i64::MAX)
    );
    assert_eq!(u32::from_bytes_hex(b"100000000"), None);
    assert_eq!(i64::from_bytes_hex(b"8000000000000000"), None);
    assert_eq!(
        u32::from_bytes_dec((u32::MAX as i128 + 1).to_string().as_bytes()),
        None
    );
    assert_eq!(
        i64::from_bytes_dec((i64::MAX as i128 + 1).to_string().as_bytes()),
        None
    );
    assert_eq!(u32::from_bytes_hex(b"g"), None);
    assert_eq!(i64::from_bytes_hex(b"g"), None);
    assert_eq!(u32::from_bytes_dec(b"a"), None);
    assert_eq!(i64::from_bytes_dec(b"a"), None);
}
