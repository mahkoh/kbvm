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
    regex::bytes::Regex,
    std::sync::LazyLock,
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
        span_lo: include.span.lo,
        first: true,
    }
}

static COMP_REGEX: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(
        r#"(?x-u)
            (?<mm>[|+])?
            (?<file>[^(|+:]+)
            (
                \(
                    (?<map>[^)]+)
                \)
            )?
            (
                :
                (?<group>[0-9]+)
            )?
        "#,
    )
    .unwrap()
});

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
        if self.pos == self.s.len() {
            return None;
        }
        let pos = self.pos;
        let lo = self.span_lo + pos as u64 + 1;
        let Some(captures) = COMP_REGEX.captures(&self.s[pos..]) else {
            return Some(Err(invalid_format(Span {
                lo,
                hi: self.span_lo + self.s.len() as u64 + 1,
            })));
        };
        self.pos += captures.get(0).unwrap().len();
        let file = captures.name("file").unwrap();
        let file_span = Span {
            lo: lo + file.range().start as u64,
            hi: lo + file.range().end as u64,
        };
        let mm = match captures.name("mm") {
            Some(mm) => {
                let span = Span {
                    lo: lo + mm.range().start as u64,
                    hi: lo + mm.range().end as u64,
                };
                let mm = match mm.as_bytes() {
                    b"+" => MergeMode::Override,
                    b"|" => MergeMode::Augment,
                    _ => unreachable!(),
                };
                Some(mm.spanned2(span))
            }
            _ if self.first => None,
            _ => {
                let span = Span {
                    lo: file_span.lo,
                    hi: file_span.lo + 1,
                };
                return Some(Err(missing_merge_mode(span)));
            }
        };
        let file = self
            .s
            .slice(pos + file.range().start..pos + file.range().end);
        let file = self.interner.intern(&file).spanned2(file_span);
        let map = captures.name("map").map(|g| {
            let span = Span {
                lo: lo + g.range().start as u64,
                hi: lo + g.range().end as u64,
            };
            let slice = self.s.slice(pos + g.range().start..pos + g.range().end);
            self.interner.intern(&slice).spanned2(span)
        });
        let mut group = None;
        if let Some(g) = captures.name("group") {
            let span = Span {
                lo: lo + g.range().start as u64,
                hi: lo + g.range().end as u64,
            };
            let slice = self.s.slice(pos + g.range().start..pos + g.range().end);
            let group_name = self.interner.intern(&slice).spanned2(span);
            let Some(group_idx) = u32::from_bytes_dec(g.as_bytes()) else {
                return Some(Err(invalid_group(&slice, span)));
            };
            let Some(group_idx) = GroupIdx::new(group_idx) else {
                return Some(Err(invalid_group(&slice, span)));
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
