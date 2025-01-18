use crate::{
    syms,
    xkb::{diagnostic::WriteToStderr, Context},
};

#[test]
fn iter() {
    const COMPOSE: &str = r#"
        <a> <a>: x
        <a> <b>: "y"
        <a> <c>: "z" z
        <q>: q
        <q> <x>: v
        <b>: b
    "#;
    let context = Context::default();
    let mut builder = context.compose_table_builder();
    builder.buffer(COMPOSE);
    let table = builder.build(WriteToStderr).unwrap();
    let mut iter = table.iter();
    {
        let r = iter.next().unwrap();
        assert_eq!(
            r.steps().iter().map(|s| s.keysym()).collect::<Vec<_>>(),
            [syms::a, syms::a],
        );
        assert_eq!(r.string(), None);
        assert_eq!(r.keysym(), Some(syms::x));
    }
    {
        let r = iter.next().unwrap();
        assert_eq!(
            r.steps().iter().map(|s| s.keysym()).collect::<Vec<_>>(),
            [syms::a, syms::b],
        );
        assert_eq!(r.string(), Some("y"));
        assert_eq!(r.keysym(), None);
    }
    {
        let r = iter.next().unwrap();
        assert_eq!(
            r.steps().iter().map(|s| s.keysym()).collect::<Vec<_>>(),
            [syms::a, syms::c],
        );
        assert_eq!(r.string(), Some("z"));
        assert_eq!(r.keysym(), Some(syms::z));
    }
    {
        let r = iter.next().unwrap();
        assert_eq!(
            r.steps().iter().map(|s| s.keysym()).collect::<Vec<_>>(),
            [syms::b],
        );
        assert_eq!(r.string(), None);
        assert_eq!(r.keysym(), Some(syms::b));
    }
    {
        let r = iter.next().unwrap();
        assert_eq!(
            r.steps().iter().map(|s| s.keysym()).collect::<Vec<_>>(),
            [syms::q, syms::x],
        );
        assert_eq!(r.string(), None);
        assert_eq!(r.keysym(), Some(syms::v));
    }
    assert!(iter.next().is_none());
}
