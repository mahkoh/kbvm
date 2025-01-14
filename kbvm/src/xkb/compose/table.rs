use {
    crate::{
        keysym::Keysym,
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
    data: [u32; 2],
}

#[derive(Clone, PartialEq)]
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
            production: Option<usize>,
        }

        let mut steps = vec![];
        let mut pre_datas = vec![];

        for (idx, production) in productions.iter().enumerate().rev() {
            let start = steps.len();
            if u32::MAX as usize - steps.len() - 1 <= production.val.steps.len() {
                // ensure that the maximum number of nodes fits into u32
                break;
            }
            for step in production.val.steps.iter() {
                steps.push(*step);
                pre_datas.push(PreData {
                    step_range: start..steps.len(),
                    production: None,
                });
            }
            pre_datas.last_mut().unwrap().production = Some(idx);
        }

        pre_datas.sort_by_key(|k| &steps[k.step_range.clone()]);

        struct Data {
            step: Step,
            production: Option<usize>,
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
        let mut prev_production = None::<usize>;

        for pre_data in pre_datas {
            let step = steps[pre_data.step_range.end - 1];
            let len = pre_data.step_range.len();
            let is_duplicate = (Some(step), len) == (prev_step, prev_len);
            if is_duplicate {
                if let Some(pl) = pre_data.production {
                    let pl = &productions[pl];
                    if let Some(prev) = prev_production {
                        let prev = &productions[prev];
                        if (&pl.val.string, pl.val.keysym) != (&prev.val.string, prev.val.keysym) {
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
            prev_production = pre_data.production;
            if len <= prev_len {
                for _ in 0..prev_len - len + 1 {
                    assert!(stack.pop().is_some());
                }
            }
            let parent = match stack.last() {
                Some(&idx) => {
                    let data = &mut datas[idx as usize];
                    data.num_children += 1;
                    if let Some(pl) = data.production.take() {
                        let pl = &productions[pl];
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
                production: pre_data.production,
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

        let mut payloads = vec![];
        let mut nodes = Vec::with_capacity(datas.len() + 1);
        nodes.push(Node {
            keysym: Default::default(),
            data: NodeType::Intermediate {
                range: 1..1 + num_root,
            }
            .serialize(),
        });

        for data in datas {
            let ty = match data.production {
                Some(idx) => {
                    let production = &productions[idx];
                    let pos = payloads.len();
                    payloads.push(Payload {
                        string: production
                            .val
                            .string
                            .as_ref()
                            .map(|s| s.as_bstr().to_string()),
                        keysym: production.val.keysym,
                    });
                    NodeType::Leaf { payload: pos }
                }
                _ => {
                    let lo = data.children_heap_pos.get() as usize;
                    let hi = lo + data.num_children as usize;
                    NodeType::Intermediate { range: lo..hi }
                }
            };
            nodes.push(Node {
                keysym: data.step.keysym,
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
    pub fn feed(&self, state: &mut State, sym: Keysym) -> Option<FeedResult<'_>> {
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
        if let Some(node) = find_match(range, sym) {
            let res = match node.deserialize() {
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

fn find_match(range: &[Node], sym: Keysym) -> Option<&Node> {
    const MAX_LINEAR: usize = 64;
    if range.len() <= MAX_LINEAR {
        for n in range {
            if n.keysym == sym {
                return Some(n);
            }
            if n.keysym > sym {
                return None;
            }
        }
        return None;
    }
    let Ok(pos) = range.binary_search_by_key(&sym, |n| n.keysym) else {
        return None;
    };
    Some(&range[pos])
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
