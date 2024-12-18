#[cfg(test)]
mod tests;

use {
    crate::xkb::{
        code::Code,
        code_map::CodeMap,
        code_slice::CodeSlice,
        diagnostic::{DiagnosticKind, DiagnosticSink},
        interner::{Interned, Interner},
        span::{Span, SpanExt, Spanned},
    },
    hashbrown::{hash_map::Entry, HashMap},
    std::sync::Arc,
    thiserror::Error,
};

#[derive(Default)]
pub(crate) struct StringCooker {
    cache: HashMap<Interned, Interned>,
}

#[derive(Debug, Error, Eq, PartialEq)]
pub(crate) enum StringCookerError {
    #[error("The octal literal overflows u8")]
    OctalOverflow,
    #[error("Unknown escape sequence {:?}", *.0 as char)]
    UnknownSequence(u8),
}

impl StringCooker {
    pub(crate) fn cook(
        &mut self,
        map: &mut CodeMap,
        diagnostics: &mut DiagnosticSink,
        interner: &mut Interner,
        interned: Spanned<Interned>,
    ) -> Interned {
        let string = interned.val;
        let entry = match self.cache.entry(string) {
            Entry::Occupied(e) => return *e.into_mut(),
            Entry::Vacant(e) => e,
        };
        let uncooked = interner.get(string);
        if !uncooked.contains(&b'\\') {
            return *entry.insert(string);
        }
        let lo = interned.span.lo + 1;
        let mut res = Vec::with_capacity(uncooked.len());
        let mut i = 0;
        while i < uncooked.len() {
            let mut b = uncooked[i];
            if b != b'\\' {
                res.push(b);
                i += 1;
                continue;
            }
            i += 1;
            b = uncooked[i];
            i += 1;
            let c = match b {
                b'\\' => b'\\',
                b'n' => b'\n',
                b't' => b'\t',
                b'r' => b'\r',
                b'b' => b'\x08',
                b'f' => b'\x0c',
                b'v' => b'\x0b',
                b'e' => b'\x1b',
                b'0'..=b'7' => {
                    let start = i - 1;
                    let mut c = b as u32 - b'0' as u32;
                    macro_rules! next {
                        () => {
                            'next: {
                                if i < uncooked.len() {
                                    b = uncooked[i];
                                    if matches!(b, b'0'..=b'7') {
                                        c = c << 3 | (b as u32 - b'0' as u32);
                                        i += 1;
                                        break 'next true;
                                    }
                                }
                                false
                            }
                        };
                    }
                    let _ = next!() && next!();
                    if c > 0xff {
                        let span = Span {
                            lo: lo + start as u64,
                            hi: lo + i as u64,
                        };
                        diagnostics.push(
                            map,
                            DiagnosticKind::OctalOverflow,
                            StringCookerError::OctalOverflow.spanned2(span),
                        );
                        continue;
                    }
                    c as u8
                }
                _ => {
                    let span = Span {
                        lo: lo + i as u64 - 1,
                        hi: lo + i as u64,
                    };
                    diagnostics.push(
                        map,
                        DiagnosticKind::UnknownEscapeSequence,
                        StringCookerError::UnknownSequence(b).spanned2(span),
                    );
                    continue;
                }
            };
            res.push(c);
        }
        let code = Code::new(&Arc::new(res));
        let slice = CodeSlice::new_owned(&code, 0..code.len());
        let interned = interner.intern(&slice);
        entry.insert(interned);
        interned
    }
}
