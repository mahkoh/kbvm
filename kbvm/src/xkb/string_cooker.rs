#[cfg(test)]
mod tests;

use {
    crate::xkb::{
        code::Code,
        code_slice::CodeSlice,
        interner::{Interned, Interner},
    },
    hashbrown::{hash_map::Entry, HashMap},
    std::{ops::Range, sync::Arc},
    thiserror::Error,
};

#[derive(Default)]
pub struct StringCooker {
    cache: HashMap<Interned, Interned>,
}

#[derive(Eq, PartialEq, Debug)]
pub struct StringCookerDiagnostic {
    range: Range<usize>,
    error: StringCookerError,
}

#[derive(Debug, Error, Eq, PartialEq)]
pub enum StringCookerError {
    #[error("The octal literal overflows u8")]
    OctalOverflow,
    #[error("Unknown escape sequence {:?}", *.0 as char)]
    UnknownSequence(u8),
}

impl StringCooker {
    pub fn cook(
        &mut self,
        diagnostics: &mut Vec<StringCookerDiagnostic>,
        interner: &mut Interner,
        interned: Interned,
    ) -> Interned {
        let entry = match self.cache.entry(interned) {
            Entry::Occupied(e) => return *e.into_mut(),
            Entry::Vacant(e) => e,
        };
        let uncooked = interner.get(interned);
        if !uncooked.contains(&b'\\') {
            return *entry.insert(interned);
        }
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
                        diagnostics.push(StringCookerDiagnostic {
                            range: start..i,
                            error: StringCookerError::OctalOverflow,
                        });
                        continue;
                    }
                    c as u8
                }
                _ => {
                    diagnostics.push(StringCookerDiagnostic {
                        range: i - 1..i,
                        error: StringCookerError::UnknownSequence(b),
                    });
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
