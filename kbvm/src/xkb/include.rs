mod error;
#[cfg(test)]
mod tests;

use {
    crate::{
        from_bytes::FromBytes,
        xkb::{
            code_slice::CodeSlice,
            group::GroupIdx,
            include::error::{
                invalid_format, invalid_group, missing_merge_mode, ParseIncludeError,
            },
            interner::{Interned, Interner},
            kccgst::MergeMode,
            span::{Span, SpanExt, Spanned},
        },
    },
    kbvm_proc::CloneWithDelta,
};

#[derive(Debug)]
pub(crate) struct Include {
    pub(crate) merge_mode: Option<Spanned<MergeMode>>,
    pub(crate) file: Spanned<Interned>,
    pub(crate) map: Option<Spanned<Interned>>,
    pub(crate) group: Option<IncludeGroup>,
}

#[derive(Copy, Clone, Debug, CloneWithDelta)]
pub(crate) struct IncludeGroup {
    pub(crate) group_name: Spanned<Interned>,
    pub(crate) group: GroupIdx,
}

pub(crate) struct IncludeIter<'a> {
    interner: &'a mut Interner,
    s: CodeSlice<'static>,
    pos: usize,
    span_lo: u64,
    first: bool,
}

pub(crate) fn parse_include(interner: &mut Interner, include: Spanned<Interned>) -> IncludeIter {
    IncludeIter {
        s: interner.get(include.val).to_owned(),
        interner,
        pos: 0,
        span_lo: include.span.lo + 1,
        first: true,
    }
}

impl IncludeIter<'_> {
    pub(crate) fn interner(&mut self) -> &mut Interner {
        self.interner
    }
}

impl<'a> Iterator for IncludeIter<'a> {
    type Item = Result<Include, Spanned<ParseIncludeError>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.pos < self.s.len() && self.s[self.pos].is_ascii_whitespace() {
            self.pos += 1;
        }
        assert!(self.pos <= self.s.len());
        if self.pos == self.s.len() {
            return None;
        }
        let captures = match capture(&self.s, self.span_lo, self.pos) {
            Ok(c) => c,
            Err(e) => {
                self.pos = self.s.len();
                return Some(Err(e));
            }
        };
        self.pos = captures.pos;
        let file = captures.file;
        let mm = captures.mm;
        if mm.is_none() && !self.first {
            let span = Span {
                lo: file.span.lo,
                hi: file.span.lo + 1,
            };
            return Some(Err(missing_merge_mode(span)));
        }
        let file = self.interner.intern(&file.val).spanned2(file.span);
        let map = captures
            .map
            .map(|g| self.interner.intern(&g.val).spanned2(g.span));
        let mut group = None;
        if let Some(g) = captures.group {
            let group_name = self.interner.intern(&g.val).spanned2(g.span);
            let Some(group_idx) = u32::from_bytes_dec(g.val.as_bytes()) else {
                return Some(Err(invalid_group(&g.val, g.span)));
            };
            let Some(group_idx) = GroupIdx::new(group_idx) else {
                return Some(Err(invalid_group(&g.val, g.span)));
            };
            group = Some(IncludeGroup {
                group_name,
                group: group_idx,
            });
        };
        self.first = false;
        Some(Ok(Include {
            merge_mode: mm,
            file,
            map,
            group,
        }))
    }
}

struct Capture<'a> {
    mm: Option<Spanned<MergeMode>>,
    file: Spanned<CodeSlice<'a>>,
    map: Option<Spanned<CodeSlice<'a>>>,
    group: Option<Spanned<CodeSlice<'a>>>,
    pos: usize,
}

fn capture<'a>(
    slice: &'a CodeSlice<'static>,
    lo: u64,
    mut pos: usize,
) -> Result<Capture<'a>, Spanned<ParseIncludeError>> {
    let mut mm = None;
    let mm_lo = lo + pos as u64;
    match slice[pos] {
        b'|' => {
            mm = Some(MergeMode::Augment.spanned(mm_lo, mm_lo + 1));
            pos += 1;
        }
        b'+' => {
            mm = Some(MergeMode::Override.spanned(mm_lo, mm_lo + 1));
            pos += 1;
        }
        _ => {}
    };
    if matches!(slice.get(pos), None | Some(b'(' | b'|' | b'+' | b':')) {
        return Err(invalid_format(Span {
            lo: lo + pos as u64,
            hi: lo + pos as u64 + 1,
        }));
    }
    let file_start = pos;
    pos += 1;
    while pos < slice.len() && !matches!(slice[pos], b'(' | b'|' | b'+' | b':') {
        pos += 1;
    }
    let file_end = pos;
    let mut map = None;
    if pos < slice.len() && slice[pos] == b'(' {
        pos += 1;
        let map_start = pos;
        let map_end = loop {
            match slice.get(pos) {
                None => {
                    return Err(invalid_format(Span {
                        lo: lo + pos as u64,
                        hi: lo + pos as u64 + 1,
                    }));
                }
                Some(b')') => break pos,
                _ => pos += 1,
            }
        };
        pos += 1;
        map = Some(
            slice
                .slice(map_start..map_end)
                .spanned(lo + map_start as u64, lo + map_end as u64),
        );
    }
    let mut group = None;
    if pos < slice.len() && slice[pos] == b':' {
        pos += 1;
        let group_start = pos;
        if !matches!(slice.get(pos), Some(b'0'..=b'9')) {
            return Err(invalid_format(Span {
                lo: lo + pos as u64,
                hi: lo + pos as u64 + 1,
            }));
        }
        pos += 1;
        while matches!(slice.get(pos), Some(b'0'..=b'9')) {
            pos += 1;
        }
        let group_end = pos;
        group = Some(
            slice
                .slice(group_start..group_end)
                .spanned(lo + group_start as u64, lo + group_end as u64),
        );
    }
    Ok(Capture {
        mm,
        file: slice
            .slice(file_start..file_end)
            .spanned(lo + file_start as u64, lo + file_end as u64),
        map,
        group,
        pos,
    })
}
