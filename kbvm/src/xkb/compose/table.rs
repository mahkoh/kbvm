#[cfg(test)]
mod tests;

#[allow(unused_imports)]
use crate::{syms, xkb::Context};
use {
    crate::{
        xkb::{
            code_map::CodeMap,
            compose::parser::{Production, Step},
            diagnostic::{DiagnosticKind, DiagnosticSink},
            format::FormatFormat,
            span::{SpanExt, Spanned},
        },
        Keysym,
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
pub(crate) struct Payload {
    pub(crate) string: Option<Box<str>>,
    pub(crate) keysym: Option<Keysym>,
}

/// A node in the table.
///
/// Except for the root node, keysym is the keysym that must be fed for this node to
/// match.
///
/// This node is either an intermediate node or a leaf node. The type is determined by the
/// contents of `data`. See [`Node::deserialize`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct Node {
    pub(crate) keysym: Keysym,
    data: [u32; 2],
}

#[derive(Clone, PartialEq)]
enum NodeType {
    /// The range is the range of the children of this node in the table.
    Intermediate { range: Range<usize> },
    /// The payload is the index of the payload in the table.
    Leaf { payload: usize },
}

/// A compose table.
///
/// This table contains the logic necessary to handle compose sequences. It is created via
/// [`Context::compose_table_builder`].
///
/// # Example
///
/// ```
/// # use kbvm::syms;
/// # use kbvm::xkb::Context;
/// # use kbvm::xkb::diagnostic::WriteToLog;
/// let context = Context::default();
/// let table = context.compose_table_builder().build(WriteToLog).unwrap();
/// let mut state = table.create_state();
/// let res = table.feed(&mut state, syms::dead_acute);
/// println!("{:?}", res);
/// let res = table.feed(&mut state, syms::dead_acute);
/// println!("{:?}", res);
/// ```
///
/// This might print
///
/// ```txt
/// Some(Pending)
/// Some(Composed { string: Some("´"), keysym: Some(acute) })
/// ```
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ComposeTable {
    nodes: Box<[Node]>,
    payloads: Box<[Payload]>,
}

/// The state of a compose operation.
///
/// This object is created via [`ComposeTable::create_state`] and should only be used with the
/// table that it was created from.
///
/// This object is essentially a pointer into the compose table and is cheap to clone.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct State {
    range: Range<usize>,
}

/// The result of a [`ComposeTable::feed`] call.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FeedResult<'a> {
    /// The call started or continued a compose sequence and the sequence is not yet
    /// completed.
    Pending,
    /// The call caused the compose sequence to be aborted because the keysym was not a
    /// valid continuation.
    Aborted,
    /// The call completed a compose sequence. The `string` and `keysym` fields contain
    /// the generated output.
    Composed {
        string: Option<&'a str>,
        keysym: Option<Keysym>,
    },
}

impl NodeType {
    fn serialize(self) -> [u32; 2] {
        // This is fine because we ensure that there are no more than u32::MAX nodes in
        // the table and we generally assume that usize >= u32.
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

        // the next step will add all prefixes of all productions to these vectors.
        // for example, given the input
        //
        //     <Multi_key> <a> <b>: X
        //     <Multi_key> <c>:     Y
        //     <Multi_key> <c>:     Z
        //
        // these vectors will the contain
        //
        //     <Multi_key>:         (no production)
        //     <Multi_key> <c>:     Z
        //     <Multi_key>:         (no production)
        //     <Multi_key> <c>:     Y
        //     <Multi_key>:         (no production)
        //     <Multi_key> <a>:     (no production)
        //     <Multi_key> <a> <b>: X
        //
        // note that the order of productions has been reversed
        let mut steps = vec![];
        let mut pre_datas = vec![];

        for (idx, production) in productions.iter().enumerate().rev() {
            let start = steps.len();
            if u32::MAX as usize - steps.len() - 1 <= production.val.steps.len() {
                // ensure that the maximum number of nodes fits into u32 with 1 node to
                // spare for the root node.
                break;
            }
            for step in production.val.steps.iter() {
                steps.push(*step);
                pre_datas.push(PreData {
                    step_range: start..steps.len(),
                    production: None,
                });
            }
            // unwrap is fine because the parser guarantees that each production has at
            // least 1 step.
            pre_datas.last_mut().unwrap().production = Some(idx);
        }

        // this sort is stable and therefore preserves the order of compose rules that
        // have the same set of steps. note that above we inserted into pre_datas in
        // reverse order, meaning that later rules overwrite earlier rules.
        //
        // after this step, pre_datas has the following form in our example
        //
        //     <Multi_key>:         (no production)
        //     <Multi_key>:         (no production)
        //     <Multi_key> <a>:     (no production)
        //     <Multi_key> <a> <b>: X
        //     <Multi_key> <c>:     Z
        //     <Multi_key> <c>:     Y
        //
        // the duplicates are removed by hte next step
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

        // the next steps deduplicates the contents of pre_datas and counts for each entry
        // how many children it has. in our example
        //
        //     root node:           1 child
        //     <Multi_key>:         2 children
        //     <Multi_key> <a>:     1 child
        //     <Multi_key> <a> <b>: 0 children (production = X)
        //     <Multi_key> <c>:     0 children (production = Z, the Y production was discarded)
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
                // this block contains no business logic, only diagnostics
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
                // pop siblings, nephews, etc. off the stack
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

        // the next steps assigns each node a position in the heap and tells it where its
        // children will be positioned in the heap. in our example
        //
        //     <Multi_key>:         pos 1, children start at pos 2
        //     <Multi_key> <a>:     pos 2, children start at pos 4
        //     <Multi_key> <a> <b>: pos 4
        //     <Multi_key> <c>:     pos 3
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

        // sort the nodes by their position in the heap. in our example
        //
        //     <Multi_key>:         pos 1, children start at pos 2
        //     <Multi_key> <a>:     pos 2, children start at pos 4
        //     <Multi_key> <c>:     pos 3
        //     <Multi_key> <a> <b>: pos 4
        datas.sort_unstable_by_key(|d| d.heap_pos.get());

        // the next step converts the datas to the final node structure
        let mut payloads = vec![];
        let mut nodes = Vec::with_capacity(datas.len() + 1);
        // the root node
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
                            .map(|s| s.as_bstr().to_string().into_boxed_str()),
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
    pub fn create_state(&self) -> State {
        let NodeType::Intermediate { range } = self.nodes[0].deserialize() else {
            unreachable!();
        };
        State { range }
    }

    /// Advance the compose state.
    ///
    /// The `state` should have been created by [`Self::create_state`]. Otherwise this function
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
    /// - Otherwise, if the candidate completes the compose sequence,
    ///   [`FeedResult::Composed`] is returned with the output and `state` is reset to the
    ///   initial state.
    /// - Otherwise, [`FeedResult::Pending`] is returned and `state` is updated to match
    ///   the new pending state.
    pub fn feed(&self, state: &mut State, sym: Keysym) -> Option<FeedResult<'_>> {
        if sym.is_modifier() {
            return None;
        }
        let range = &self.nodes[state.range.clone()];
        if let Ok(node) = range.binary_search_by_key(&sym, |n| n.keysym) {
            let node = &range[node];
            let res = match node.deserialize() {
                NodeType::Intermediate { range } => {
                    state.range = range;
                    FeedResult::Pending
                }
                NodeType::Leaf { payload } => {
                    *state = self.create_state();
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
        *state = self.create_state();
        Some(FeedResult::Aborted)
    }

    /// Creates an iterator over the rules in this table.
    ///
    /// Due to the data structure used by this table, the iterator must allocate and
    /// returns references to the allocated memory. Since this memory is re-used for each
    /// returned rule, the iterator cannot implement the `Iterator` trait. Instead, you
    /// have to call [`Iter::next`] manually.
    pub fn iter(&self) -> Iter<'_> {
        Iter {
            table: self,
            stack: vec![],
            child_range: vec![self.create_state().range],
        }
    }

    /// Returns a [`Display`] type that can be used to format the table in XCompose format.
    ///
    /// # Example
    ///
    /// If the input was
    ///
    /// ```xcompose
    /// <b>: B
    /// <a> <d>: D
    /// <a> <c>: C
    /// ```
    ///
    /// then the output would look like this:
    ///
    /// ```xcompose
    /// <a> <c>: C
    /// <a> <d>: D
    /// <b>: B
    /// ```
    pub fn format(&self) -> impl Display + use<'_> {
        FormatFormat(self)
    }
}

/// A matching step in a [`MatchRule`].
///
/// # Example
///
/// ```xcompose
/// <a> <b> <c>: asciitilde
/// ```
///
/// This step might correspond to `<a>`, `<b>`, or `<c>`.
#[derive(Copy, Clone)]
pub struct MatchStep<'a> {
    pub(crate) node: &'a Node,
}

impl MatchStep<'_> {
    /// Returns the keysym required by this step.
    ///
    /// # Example
    ///
    /// ```xcompose
    /// <a>: asciitilde
    /// ```
    ///
    /// This function returns [`syms::a`].
    pub fn keysym(&self) -> Keysym {
        self.node.keysym
    }
}

/// A rule from an `XCompose` file.
///
/// # Example
///
/// ```xcompose
/// <Multi_key> <a>: at
/// <Multi_key> <c>: copyright
/// ```
///
/// This rule might correspond to `<Multi_key> <a>` or `<Multi_key> <c>`.
#[derive(Copy, Clone)]
pub struct MatchRule<'a, 'b> {
    pub(crate) steps: &'a [MatchStep<'b>],
    pub(crate) payload: &'b Payload,
}

impl<'b> MatchRule<'_, 'b> {
    /// Returns the inputs required to match this rule.
    ///
    /// # Example
    ///
    /// ```xcompose
    /// <a> <b> <c>: asciitilde
    /// ```
    ///
    /// This function returns three steps, one each for `<a>`, `<b>`, and `<c>`.
    pub fn steps(&self) -> &[MatchStep<'b>] {
        self.steps
    }

    /// Returns the string produced by this rule if it matches.
    ///
    /// # Example
    ///
    /// ```xcompose
    /// <a>: "hello" asciitilde
    /// <b>: asciitilde
    /// <c>: "hello"
    /// ```
    ///
    /// If this rule corresponds to `<a>` or `<c>`, this function returns `Some("hello")`.
    /// Otherwise this function returns `None`.
    pub fn string(&self) -> Option<&'b str> {
        self.payload.string.as_deref()
    }

    /// Returns the keysym produced by this rule if it matches.
    ///
    /// # Example
    ///
    /// ```xcompose
    /// <a>: "hello" asciitilde
    /// <b>: asciitilde
    /// <c>: "hello"
    /// ```
    ///
    /// If this rule corresponds to `<a>` or `<b>`, this function returns
    /// `Some(asciitilde)`. Otherwise this function returns `None`.
    pub fn keysym(&self) -> Option<Keysym> {
        self.payload.keysym
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

/// An iterator over the [`MatchRule`] in a table.
///
/// This type is created using [`ComposeTable::iter`].
///
/// This type does not implement `Iterator` because the returned values borrow from the
/// iterator. Call [`Iter::next`] manually instead.
///
/// # Example
///
/// ```xcompose
/// <Multi_key> <a>: at
/// <Multi_key> <c>: copyright
/// ```
///
/// This rule returns two elements, one for `<Multi_key> <a>` and one for
/// `<Multi_key> <c>`.
#[derive(Clone, Debug)]
pub struct Iter<'a> {
    table: &'a ComposeTable,
    stack: Vec<MatchStep<'a>>,
    child_range: Vec<Range<usize>>,
}

impl<'a> Iter<'a> {
    /// Returns the next element of the iterator.
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
