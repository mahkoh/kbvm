use {
    criterion::{black_box, criterion_group, criterion_main, Criterion},
    kbvm::{
        syms,
        xkb::{diagnostic::WriteToStderr, Context},
    },
};

fn shift_press_release(c: &mut Criterion) {
    let context = Context::default();
    let mut builder = context.compose_table_builder();
    builder.buffer(COMPOSE);
    let table = builder.build(WriteToStderr).unwrap();
    let mut state = table.create_state();
    c.bench_function("compose no-match", |b| {
        b.iter(|| {
            black_box(table.feed(&mut state, syms::a));
        })
    });
}

criterion_group!(benches, shift_press_release,);
criterion_main!(benches);

const COMPOSE: &str = r#"
<b>: ""
<c>: ""
<d>: ""
<e>: ""
<f>: ""
<g>: ""
<h>: ""
<i>: ""
<j>: ""
<k>: ""
<l>: ""
<m>: ""
<n>: ""
<o>: ""
<p>: ""
<q>: ""
<r>: ""
<s>: ""
<t>: ""
<u>: ""
<v>: ""
<w>: ""
<x>: ""
<y>: ""
<z>: ""
<B>: ""
<C>: ""
<D>: ""
<E>: ""
<F>: ""
<G>: ""
<H>: ""
<I>: ""
<J>: ""
<K>: ""
<L>: ""
<M>: ""
<N>: ""
<O>: ""
<P>: ""
<Q>: ""
<R>: ""
<S>: ""
<T>: ""
<U>: ""
<V>: ""
<W>: ""
<X>: ""
<Y>: ""
<Z>: ""
"#;
