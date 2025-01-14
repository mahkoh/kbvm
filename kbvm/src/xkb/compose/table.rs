use {
    crate::{
        keysym::Keysym,
        modifier::ModifierMask,
        syms,
        xkb::{
            code_map::CodeMap,
            compose::parser::{Production, Step},
            diagnostic::{DiagnosticKind, DiagnosticSink},
            format::FormatFormat,
            span::{SpanExt, Spanned},
        },
    },
    bstr::ByteSlice,
    kbvm_proc::ad_hoc_display,
    std::{
        cell::Cell,
        fmt::{Debug, Display, Formatter},
        ops::Range,
    },
};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Payload {
    pub(crate) string: Option<String>,
    pub(crate) keysym: Option<Keysym>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct Node {
    pub(crate) keysym: Keysym,
    pub(crate) mask: u8,
    pub(crate) mods: u8,
    data: [u32; 2],
}

#[derive(Clone)]
enum NodeType {
    Intermediate { range: Range<usize> },
    Leaf { payload: usize },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ComposeTable {
    nodes: Box<[Node]>,
    payloads: Box<[Payload]>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct State {
    range: Range<usize>,
}

pub enum FeedResult<'a> {
    Pending,
    Aborted,
    Composed {
        string: Option<&'a str>,
        keysym: Option<Keysym>,
    },
}

impl NodeType {
    fn serialize(self) -> [u32; 2] {
        match self {
            NodeType::Intermediate { range } => [range.start as u32, range.end as u32],
            NodeType::Leaf { payload } => [!0, payload as u32],
        }
    }
}

impl Node {
    fn deserialize(&self) -> NodeType {
        if self.data[0] == !0 {
            NodeType::Leaf {
                payload: self.data[1] as usize,
            }
        } else {
            NodeType::Intermediate {
                range: (self.data[0] as usize)..(self.data[1] as usize),
            }
        }
    }
}

impl ComposeTable {
    pub(crate) fn new(
        map: &mut CodeMap,
        diagnostics: &mut DiagnosticSink<'_, '_>,
        productions: &[Spanned<Production>],
    ) -> Self {
        struct PreData {
            step_range: Range<usize>,
            payload: Option<Spanned<u32>>,
        }

        let mut payloads = Vec::with_capacity(productions.len());
        let mut steps = vec![];
        let mut pre_datas = vec![];

        for production in productions.iter().rev() {
            let start = steps.len();
            if u32::MAX as usize - steps.len() - 1 <= production.val.steps.len() {
                // ensure that the maximum number of nodes fits into u32
                break;
            }
            for step in production.val.steps.iter() {
                steps.push(*step);
                pre_datas.push(PreData {
                    step_range: start..steps.len(),
                    payload: None,
                });
            }
            pre_datas.last_mut().unwrap().payload =
                Some((payloads.len() as u32).spanned2(production.span));
            payloads.push(Payload {
                string: production
                    .val
                    .string
                    .as_ref()
                    .map(|s| s.as_bstr().to_string()),
                keysym: production.val.keysym,
            });
        }

        pre_datas.sort_by_key(|k| &steps[k.step_range.clone()]);

        struct Data {
            step: Step,
            payload: Option<Spanned<u32>>,
            num_children: u32,
            parent: Option<u32>,
            heap_pos: Cell<u32>,
            children_heap_pos: Cell<u32>,
            next_child_pos: Cell<u32>,
        }

        let mut datas: Vec<Data> = vec![];
        let mut num_root = 0;

        let mut stack: Vec<u32> = vec![];
        let mut prev_step = None;
        let mut prev_len = 0;
        let mut prev_payload = None::<Spanned<u32>>;

        for pre_data in pre_datas {
            let step = steps[pre_data.step_range.end - 1];
            let len = pre_data.step_range.len();
            let is_duplicate = (Some(step), len) == (prev_step, prev_len);
            if is_duplicate {
                if let Some(pl) = pre_data.payload {
                    if let Some(prev) = prev_payload {
                        if payloads[prev.val as usize] != payloads[pl.val as usize] {
                            diagnostics.push(
                                map,
                                DiagnosticKind::IgnoringDuplicateComposeEntry,
                                ad_hoc_display!("ignoring duplicate compose entry")
                                    .spanned2(pl.span),
                            );
                        }
                    } else {
                        diagnostics.push(
                            map,
                            DiagnosticKind::IgnoringComposePrefix,
                            ad_hoc_display!("ignoring compose prefix").spanned2(pl.span),
                        );
                    }
                }
                continue;
            }
            prev_payload = pre_data.payload;
            if len <= prev_len {
                for _ in 0..prev_len - len + 1 {
                    assert!(stack.pop().is_some());
                }
            }
            let parent = match stack.last() {
                Some(&idx) => {
                    let data = &mut datas[idx as usize];
                    data.num_children += 1;
                    if let Some(pl) = data.payload.take() {
                        diagnostics.push(
                            map,
                            DiagnosticKind::IgnoringComposePrefix,
                            ad_hoc_display!("ignoring compose prefix").spanned2(pl.span),
                        );
                    }
                    Some(idx)
                }
                _ => {
                    num_root += 1;
                    None
                }
            };
            prev_step = Some(step);
            prev_len = len;
            stack.push(datas.len() as u32);
            datas.push(Data {
                step,
                payload: pre_data.payload,
                num_children: 0,
                parent,
                heap_pos: Cell::new(0),
                children_heap_pos: Cell::new(0),
                next_child_pos: Cell::new(0),
            });
        }

        let mut next_free_node_position = num_root as u32 + 1;
        let mut next_root_pos = 1;

        for data in &datas {
            match data.parent {
                Some(parent_idx) => {
                    let parent = &datas[parent_idx as usize];
                    let next_child_pos = parent.next_child_pos.get();
                    data.heap_pos.set(next_child_pos);
                    parent.next_child_pos.set(next_child_pos + 1);
                }
                _ => {
                    data.heap_pos.set(next_root_pos);
                    next_root_pos += 1;
                }
            }
            data.children_heap_pos.set(next_free_node_position);
            data.next_child_pos.set(next_free_node_position);
            next_free_node_position += data.num_children;
        }

        datas.sort_by_key(|d| d.heap_pos.get());

        let mut nodes = Vec::with_capacity(datas.len() + 1);
        nodes.push(Node {
            keysym: Default::default(),
            mask: 0,
            mods: 0,
            data: NodeType::Intermediate {
                range: 1..1 + num_root,
            }
            .serialize(),
        });

        for data in datas {
            let ty = match data.payload {
                Some(payload) => NodeType::Leaf {
                    payload: payload.val as usize,
                },
                _ => {
                    let lo = data.children_heap_pos.get() as usize;
                    let hi = lo + data.num_children as usize;
                    NodeType::Intermediate { range: lo..hi }
                }
            };
            nodes.push(Node {
                keysym: data.step.keysym,
                mask: data.step.mask,
                mods: data.step.mods,
                data: ty.serialize(),
            });
        }

        Self {
            nodes: nodes.into_boxed_slice(),
            payloads: payloads.into_boxed_slice(),
        }
    }

    /// Creates a new [`State`] for use with this table.
    ///
    /// The returned state is in the initial state.
    pub fn state(&self) -> State {
        let NodeType::Intermediate { range } = self.nodes[0].deserialize() else {
            unreachable!();
        };
        State { range }
    }

    /// Advance the compose state.
    ///
    /// The modifiers should be the effective modifiers at the time the keysym was
    /// generated.
    ///
    /// The `state` should have been created by [`Self::state`]. Otherwise this function
    /// might panic.
    ///
    /// This function returns `None` if the call had no effect. This happens in two
    /// situations:
    ///
    /// - The keysym is a modifier (e.g. `Shift_L`).
    /// - The state is in the initial state (that is, there is no ongoing compose
    ///   sequence) and the keysym does not start a compose sequence.
    ///
    /// Otherwise, this function returns a [`FeedResult`] as follows:
    ///
    /// - If the keysym/modifiers combination matches no candidate,
    ///   [`FeedResult::Aborted`] is returned and the state is reset to the initial state.
    /// - Otherwise, the matching candidate with the highest priority is chosen. Note that
    ///   there can only be multiple matching candidates if they only differ by modifiers.
    ///
    ///   - If this candidate completes the compose sequence, [`FeedResult::Composed`] is
    ///     returned with the output and `state` is reset to the initial state.
    ///   - Otherwise, [`FeedResult::Pending`] is returned and `state` is updated to match
    ///     the new pending state.
    pub fn feed(
        &self,
        state: &mut State,
        mods: ModifierMask,
        sym: Keysym,
    ) -> Option<FeedResult<'_>> {
        if sym >= syms::Shift_L && sym <= syms::Hyper_R {
            return None;
        }
        if sym >= syms::ISO_Lock && sym <= syms::ISO_Level5_Lock {
            return None;
        }
        if sym == syms::Mode_switch || sym == syms::Num_Lock {
            return None;
        }
        let range = &self.nodes[state.range.clone()];
        let range = find_candidates(range, sym);
        for n in range {
            let mask = ModifierMask(n.mask as i8 as u32); // force sign extension
            let expected = ModifierMask(n.mods as u32);
            if mods & mask == expected {
                let res = match n.deserialize() {
                    NodeType::Intermediate { range } => {
                        state.range = range;
                        FeedResult::Pending
                    }
                    NodeType::Leaf { payload } => {
                        *state = self.state();
                        let payload = &self.payloads[payload];
                        FeedResult::Composed {
                            string: payload.string.as_deref(),
                            keysym: payload.keysym,
                        }
                    }
                };
                return Some(res);
            }
        }
        if state.range.start == 1 {
            return None;
        }
        *state = self.state();
        Some(FeedResult::Aborted)
    }

    pub fn iter(&self) -> Iter<'_> {
        Iter {
            table: self,
            stack: vec![],
            child_range: vec![self.state().range],
        }
    }

    pub fn format(&self) -> impl Display + use<'_> {
        FormatFormat(self)
    }
}

fn find_candidates(range: &[Node], sym: Keysym) -> &[Node] {
    const MAX_LINEAR: usize = 64;
    if range.len() <= MAX_LINEAR {
        let mut iter = range.iter().enumerate();
        for (lo, n) in iter.by_ref() {
            if n.keysym > sym {
                return &[];
            }
            if n.keysym == sym {
                for (hi, n) in iter {
                    if n.keysym > sym {
                        return &range[lo..hi];
                    }
                }
                return &range[lo..];
            }
        }
        return &[];
    }
    let Ok(pos) = range.binary_search_by_key(&sym, |n| n.keysym) else {
        return &[];
    };
    let mut lo = pos;
    while lo > 0 && range[lo - 1].keysym == sym {
        lo -= 1;
    }
    let mut hi = pos + 1;
    while hi < range.len() && range[hi].keysym == sym {
        hi += 1;
    }
    &range[lo..hi]
}

pub struct MatchStep<'a> {
    pub(crate) node: &'a Node,
}

pub struct MatchRule<'a, 'b> {
    pub(crate) steps: &'a [MatchStep<'b>],
    pub(crate) payload: &'b Payload,
}

impl<'b> MatchRule<'_, 'b> {
    pub fn steps(&self) -> &[MatchStep<'b>] {
        self.steps
    }

    pub fn payload(&self) -> &'b Payload {
        self.payload
    }
}

impl Debug for MatchRule<'_, '_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        FormatFormat(self).fmt(f)
    }
}

impl Debug for MatchStep<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        FormatFormat(self).fmt(f)
    }
}

pub struct Iter<'a> {
    table: &'a ComposeTable,
    stack: Vec<MatchStep<'a>>,
    child_range: Vec<Range<usize>>,
}

impl<'a> Iter<'a> {
    pub fn next(&mut self) -> Option<MatchRule<'_, 'a>> {
        self.stack.pop();
        loop {
            let mut range = self.child_range.pop()?;
            let Some(idx) = range.next() else {
                self.stack.pop();
                continue;
            };
            self.child_range.push(range);
            let node = &self.table.nodes[idx];
            self.stack.push(MatchStep { node });
            match node.deserialize() {
                NodeType::Intermediate { range } => {
                    self.child_range.push(range);
                }
                NodeType::Leaf { payload } => {
                    let payload = &self.table.payloads[payload];
                    return Some(MatchRule {
                        steps: &self.stack,
                        payload,
                    });
                }
            }
        }
    }
}
