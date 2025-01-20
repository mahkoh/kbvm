use {
    criterion::{black_box, criterion_group, criterion_main, Criterion},
    kbvm::Keysym,
    libxkbcommon_test_linker::{
        xkb_keysym_from_name, xkb_keysym_get_name, xkb_keysym_to_upper, xkb_keysym_to_utf32,
        xkb_utf32_to_keysym,
    },
};

const CHARS: [char; 4] = ['a', 'ァ', '字', 'Ü'];
// const CHARS: [char; 1] = ['字'];
// const CHARS: [char; 1] = ['ァ'];

pub fn from_char(c: &mut Criterion) {
    for char in CHARS {
        let t1 = format!("from_char - kbvm - {char}");
        c.bench_function(&t1, |b| b.iter(|| Keysym::from_char(char)));
        let t2 = format!("from_char - xkbc - {char}");
        c.bench_function(&t2, |b| b.iter(|| xkb_utf32_to_keysym(char as u32)));
    }
}

pub fn to_char(c: &mut Criterion) {
    const INVOCATIONS: usize = 1;
    for char in CHARS {
        let sym = Keysym::from_char(char);
        let t1 = format!("to_char - kbvm - {char}");
        c.bench_function(&t1, |b| {
            b.iter(|| {
                for _ in 0..INVOCATIONS {
                    black_box(sym.char());
                }
            })
        });
        let t2 = format!("to_char - xkbc - {char}");
        c.bench_function(&t2, |b| {
            b.iter(|| {
                for _ in 0..INVOCATIONS {
                    black_box(char::from_u32(xkb_keysym_to_utf32(sym.0)));
                }
            })
        });
    }
}

pub fn to_char_all(c: &mut Criterion) {
    let keysyms: Vec<_> = Keysym::all().collect();
    c.bench_function("to_char_all - kbvm", |b| {
        b.iter(|| {
            for sym in &keysyms {
                black_box(sym.char());
            }
        })
    });
    c.bench_function("to_char_all - xkbc", |b| {
        b.iter(|| {
            for sym in &keysyms {
                black_box(char::from_u32(xkb_keysym_to_utf32(sym.0)));
            }
        })
    });
}

pub fn name(c: &mut Criterion) {
    let mut buf = [0; 128];
    for char in CHARS {
        let sym = Keysym::from_char(char);
        let t1 = format!("name - kbvm - {char}");
        c.bench_function(&t1, |b| b.iter(|| sym.name()));
        let t2 = format!("name - xkbc - {char}");
        c.bench_function(&t2, |b| {
            b.iter(|| unsafe { xkb_keysym_get_name(sym.0, buf.as_mut_ptr(), buf.len()) })
        });
    }
}

pub fn from_name(c: &mut Criterion) {
    let mut buf = [0; 128];
    for char in CHARS {
        let Some(name) = Keysym::from_char(char).name() else {
            continue;
        };
        buf[..name.len()].copy_from_slice(name.as_bytes());
        buf[name.len()] = 0;
        let t1 = format!("from_name - kbvm - {char}");
        c.bench_function(&t1, |b| b.iter(|| Keysym::from_str(name)));
        let t2 = format!("from_name - xkbc - {char}");
        c.bench_function(&t2, |b| {
            b.iter(|| unsafe { xkb_keysym_from_name(buf.as_ptr(), 0) })
        });
    }
}

pub fn from_name_xf86(c: &mut Criterion) {
    let mut buf = [0; 128];
    let name = "XF86_MonBrightnessCycle";
    buf[..name.len()].copy_from_slice(name.as_bytes());
    buf[name.len()] = 0;
    let t1 = format!("from_name_xf86 - kbvm - {name}");
    c.bench_function(&t1, |b| b.iter(|| Keysym::from_str(name)));
    let t2 = format!("from_name_xf86 - xkbc - {name}");
    c.bench_function(&t2, |b| {
        b.iter(|| unsafe { xkb_keysym_from_name(buf.as_ptr(), 0) })
    });
}

pub fn from_name_insensitive(c: &mut Criterion) {
    let mut buf = [0; 128];
    for char in CHARS {
        let Some(name) = Keysym::from_char(char).name() else {
            continue;
        };
        let name = name.to_ascii_uppercase();
        buf[..name.len()].copy_from_slice(name.as_bytes());
        buf[name.len()] = 0;
        let t1 = format!("from_name_insensitive - kbvm - {char}");
        c.bench_function(&t1, |b| b.iter(|| Keysym::from_str_insensitive(&name)));
        let t2 = format!("from_name_insensitive - xkbc - {char}");
        c.bench_function(&t2, |b| {
            b.iter(|| unsafe { xkb_keysym_from_name(buf.as_ptr(), 1) })
        });
    }
}

pub fn from_name_numbers(c: &mut Criterion) {
    let mut buf = [0; 128];
    let char = '字';
    for hex in [false, true] {
        let name = match hex {
            true => format!("0x{:x}", char as u32),
            false => format!("U{:x}", char as u32),
        };
        buf[..name.len()].copy_from_slice(name.as_bytes());
        buf[name.len()] = 0;
        let t1 = format!("from_name_numbers (hex = {hex}) - kbvm - {char}");
        c.bench_function(&t1, |b| b.iter(|| Keysym::from_str(&name)));
        let t2 = format!("from_name_numbers (hex = {hex}) - xkbc - {char}");
        c.bench_function(&t2, |b| {
            b.iter(|| unsafe { xkb_keysym_from_name(buf.as_ptr(), 0) })
        });
    }
}

pub fn to_upper(c: &mut Criterion) {
    for char in CHARS {
        let sym = Keysym::from_char(char);
        let t1 = format!("to_upper - kbvm - {char}");
        c.bench_function(&t1, |b| b.iter(|| black_box(sym).to_uppercase()));
        let t2 = format!("to_upper - xkbc - {char}");
        c.bench_function(&t2, |b| b.iter(|| xkb_keysym_to_upper(sym.0)));
    }
}

criterion_group!(
    benches,
    from_char,
    to_char,
    to_char_all,
    name,
    from_name,
    from_name_xf86,
    from_name_insensitive,
    from_name_numbers,
    to_upper,
);
criterion_main!(benches);
