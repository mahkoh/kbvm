use {
    crate::xkb::{
        code::Code,
        code_map::CodeMap,
        include::{error::ParseIncludeError, parse_include, IncludeIter},
        interner::{Interned, Interner},
        kccgst::MergeMode,
        span::SpanExt,
    },
    std::sync::Arc,
};

fn iter<'a>(map: &mut CodeMap, interner: &'a mut Interner, s: &str) -> IncludeIter<'a> {
    let code = Arc::new(s.as_bytes().to_vec());
    let code = Code::new(&code);
    let code_span = map.add(None, None, &code);
    let interned = interner.intern(&code.to_slice());
    parse_include(interner, interned.spanned2(code_span))
}

fn i(interner: &mut Interner, s: &str) -> Interned {
    let code = Arc::new(s.as_bytes().to_vec());
    let code = Code::new(&code);
    interner.intern(&code.to_slice())
}

#[test]
fn test() {
    let mut map = CodeMap::default();
    let mut interner = Interner::default();
    let abcd = i(&mut interner, "abcd");
    let ef = i(&mut interner, "ef");
    let tt = i(&mut interner, "22");
    let xx = i(&mut interner, "xx");
    let ayo = i(&mut interner, "ayo");
    let yy = i(&mut interner, "yy");
    let mut iter = iter(&mut map, &mut interner, "abcd(ef):22+xx|abcd(ayo)  +yy");

    let i = iter.next().unwrap().unwrap();
    assert_eq!(i.file.val, abcd);
    assert_eq!(i.map.unwrap().val, ef);
    assert_eq!(i.group.unwrap().group_name.val, tt);
    assert_eq!(i.group.unwrap().group.raw(), 22);
    assert_eq!(i.merge_mode, None);

    let i = iter.next().unwrap().unwrap();
    assert_eq!(i.file.val, xx);
    assert!(i.map.is_none());
    assert!(i.group.is_none());
    assert_eq!(i.merge_mode.unwrap(), MergeMode::Override);

    let i = iter.next().unwrap().unwrap();
    assert_eq!(i.file.val, abcd);
    assert_eq!(i.map.unwrap().val, ayo);
    assert!(i.group.is_none());
    assert_eq!(i.merge_mode.unwrap(), MergeMode::Augment);

    let i = iter.next().unwrap().unwrap();
    assert_eq!(i.file.val, yy);
    assert!(i.map.is_none());
    assert!(i.group.is_none());
    assert_eq!(i.merge_mode.unwrap(), MergeMode::Override);

    assert!(iter.next().is_none());
}

#[test]
fn invalid_format() {
    let mut map = CodeMap::default();
    let mut interner = Interner::default();
    let mut iter = iter(&mut map, &mut interner, "abc+");
    assert!(iter.next().unwrap().is_ok());
    let e = iter.next().unwrap().unwrap_err();
    assert_eq!(e.val, ParseIncludeError::MissingFileName);
}

#[test]
fn invalid_group() {
    let mut map = CodeMap::default();
    let mut interner = Interner::default();
    {
        let mut iter = iter(&mut map, &mut interner, "abc:9999999999999999999999");
        let e = iter.next().unwrap().unwrap_err();
        let ParseIncludeError::InvalidGroupIndex(idx) = e.val else {
            unreachable!();
        };
        assert_eq!(idx.as_bytes(), b"9999999999999999999999");
    }
    {
        let mut iter = iter(&mut map, &mut interner, "abc:33");
        let e = iter.next().unwrap().unwrap_err();
        let ParseIncludeError::InvalidGroupIndex(idx) = e.val else {
            unreachable!();
        };
        assert_eq!(idx.as_bytes(), b"33");
    }
    {
        let mut iter = iter(&mut map, &mut interner, "abc:xyz");
        let e = iter.next().unwrap().unwrap_err();
        let ParseIncludeError::InvalidGroupIndex(idx) = e.val else {
            unreachable!();
        };
        assert_eq!(idx.as_bytes(), b"x");
    }
}

#[test]
fn missing_merge_mode() {
    let mut map = CodeMap::default();
    let mut interner = Interner::default();
    let mut iter = iter(&mut map, &mut interner, "a(b)c");
    assert!(iter.next().unwrap().is_ok());
    let e = iter.next().unwrap().unwrap_err();
    assert_eq!(e.val, ParseIncludeError::MissingMergeMode);
}

#[test]
fn unterminated_map_name() {
    let mut map = CodeMap::default();
    let mut interner = Interner::default();
    let mut iter = iter(&mut map, &mut interner, "abc(def");
    let e = iter.next().unwrap().unwrap_err();
    assert_eq!(e.val, ParseIncludeError::UnterminatedMapName);
}
