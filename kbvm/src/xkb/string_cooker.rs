#[cfg(test)]
mod tests;

use {
    crate::{
        from_bytes::FromBytes,
        xkb::{
            code::Code,
            code_map::CodeMap,
            code_slice::CodeSlice,
            diagnostic::{DiagnosticKind, DiagnosticSink},
            interner::{Interned, Interner},
            span::{Span, SpanExt, SpanUnit, Spanned},
        },
    },
    bstr::ByteSlice,
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
    #[error("Unterminated unicode escape sequence {:?}", .0.as_bstr())]
    UnterminatedUnicodeEscape(CodeSlice<'static>),
    #[error("Unicode escape sequence doesn't start with a `{{`")]
    UnopenedUnicodeEscape,
    #[error("Unicode escape sequence contains invalid representation {:?}", .0.as_bstr())]
    InvalidUnicodeEscapeRepresentation(CodeSlice<'static>),
    #[error("Code point is not valid: U+{:X}", .0)]
    InvalidUnicodeCodepoint(u32),
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
        'outer: while i < uncooked.len() {
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
                            lo: lo + start as SpanUnit,
                            hi: lo + i as SpanUnit,
                        };
                        diagnostics.push(
                            map,
                            DiagnosticKind::OctalStringEscapeOverflow,
                            StringCookerError::OctalOverflow.spanned2(span),
                        );
                        continue;
                    }
                    c as u8
                }
                b'u' => {
                    let start = i - 1;
                    loop {
                        if i == uncooked.len() {
                            let span = Span {
                                lo: lo + start as SpanUnit - 1,
                                hi: lo + i as SpanUnit,
                            };
                            diagnostics.push(
                                map,
                                DiagnosticKind::UnterminatedUnicodeEscape,
                                StringCookerError::UnterminatedUnicodeEscape(
                                    uncooked.slice(start - 1..i).to_owned(),
                                )
                                .spanned2(span),
                            );
                            continue 'outer;
                        }
                        let b = uncooked[i];
                        i += 1;
                        if b == b'}' {
                            break;
                        }
                    }
                    if uncooked[start + 1] != b'{' {
                        let span = Span {
                            lo: lo + start as SpanUnit + 1,
                            hi: lo + start as SpanUnit + 2,
                        };
                        diagnostics.push(
                            map,
                            DiagnosticKind::UnopenedUnicodeEscape,
                            StringCookerError::UnopenedUnicodeEscape.spanned2(span),
                        );
                        continue;
                    }
                    let start = start + 2;
                    let end = i - 1;
                    let bytes = &uncooked[start..end];
                    let span = Span {
                        lo: lo + start as SpanUnit,
                        hi: lo + end as SpanUnit,
                    };
                    let Some(codepoint) = u32::from_bytes_hex(bytes) else {
                        diagnostics.push(
                            map,
                            DiagnosticKind::InvalidUnicodeEscapeRepresentation,
                            StringCookerError::InvalidUnicodeEscapeRepresentation(
                                uncooked.slice(start..end).to_owned(),
                            )
                            .spanned2(span),
                        );
                        continue;
                    };
                    let Some(codepoint) = char::from_u32(codepoint) else {
                        diagnostics.push(
                            map,
                            DiagnosticKind::InvalidUnicodeCodepoint,
                            StringCookerError::InvalidUnicodeCodepoint(codepoint).spanned2(span),
                        );
                        continue;
                    };
                    let mut buf = [0; 4];
                    res.extend_from_slice(codepoint.encode_utf8(&mut buf).as_bytes());
                    continue;
                }
                _ => {
                    let span = Span {
                        lo: lo + i as SpanUnit - 1,
                        hi: lo + i as SpanUnit,
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
