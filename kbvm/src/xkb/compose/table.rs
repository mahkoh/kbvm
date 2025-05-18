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
    data: [u32; 4],
}

#[derive(Debug, Clone, PartialEq)]
struct NodeData {
    children: Range<usize>,
    payload: Option<usize>,
    is_old_terminal: bool,
}

#[derive(Debug)]
struct NodePayload<'a> {
    payload: &'a Payload,
    is_old_terminal: bool,
}

#[derive(Debug)]
enum NodeType<'a> {
    Intermediate {
        children: Range<usize>,
        payload: Option<NodePayload<'a>>,
    },
    Leaf {
        payload: &'a Payload,
    },
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
///
/// # Data Structure
///
/// This type represents a tree parsed from one or more compose files. For example, the compose file
///
/// ```compose
/// <a>:         "foo"
/// <a> <d>:     "bar"
/// <a> <a> <d>: "baz"
/// <c> <c>:     "yolo"
/// ```
///
/// would produce the following tree (with an implicit root node):
///
/// ```txt
///          ┌────────┐                   ┌───┐
///          │  <a>   │                   │<c>│
///          │        │                   └─┬─┘
///          │-> "foo"│                     │
///          └────┬───┘                     │
///               │                         │
///     ┌─────────┴───────┐                 │
///     ▼                 ▼                 ▼
/// ┌────────┐          ┌───┐          ┌─────────┐
/// │  <d>   │          │<a>│          │   <c>   │
/// │        │          └─┬─┘          │         │
/// │-> "bar"│            │            │-> "yolo"│
/// └────────┘            │            └─────────┘
///                       │
///                       │
///                       ▼
///                   ┌────────┐
///                   │  <d>   │
///                   │        │
///                   │-> "baz"│
///                   └────────┘
/// ```
///
/// A [`State`] object is a pointer into this tree, encoding the current position. Feeding a
/// [`Keysym`] into the [`State`] advances the [`State`] to the next child or resets it to the root
/// node if no continuation exists.
///
/// # Intermediate Nodes with Data
///
/// Classic compose APIs do not support non-leaf nodes producing an output. In the example above,
/// the top-left `<a>` node is such a node. Those APIs behave as if the `<a>` node did not produce
/// an output.
///
/// This is the behavior of the [`ComposeTable::feed`] function. If more control is desired, then
/// the [`ComposeTable::feed2`] function should be used instead. It allows the application decide
/// if the intermediate node should
///
/// - be treated as a leaf node, and the state be reset to the root node, or
/// - be treated as an intermediate node without an output.
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

/// The output of a [`ComposeTable::feed2`] call.
///
/// Dropping this object without calling [`Self::apply`] does not update the state.
#[derive(Debug)]
pub struct Feed2Result<'a, 'b> {
    state: &'b mut State,
    table: &'a ComposeTable,
    node: Option<NodeType<'a>>,
    accept: Option<bool>,
}

impl<'a> Feed2Result<'a, '_> {
    /// Returns the continuation-state of the node.
    ///
    /// This function returns `None` if no node was found or the node is not an intermediate node.
    ///
    /// # Example
    ///
    /// ```compose
    /// <a>:     X
    /// <a> <b>: Y
    /// ```
    ///
    /// After feeding `<a>`, this function returns `Some`.
    ///
    /// After feeding `<u>`, this function returns `None`.
    ///
    /// ```compose
    /// <a>: Y
    /// ```
    ///
    /// After feeding `<a>`, this function returns `None`.
    pub fn continuation(&self) -> Option<State> {
        let node = self.node.as_ref()?;
        match node {
            NodeType::Intermediate { children, .. } => Some(State {
                range: children.clone(),
            }),
            NodeType::Leaf { .. } => None,
        }
    }

    /// Returns the output produced by the node.
    ///
    /// If no node was found, [`FeedResult::Aborted`] is returned. If the node produces no output
    /// [`FeedResult::Pending`] is returned. Otherwise this function returns
    /// [`FeedResult::Composed`].
    ///
    /// If this function return [`FeedResult::Composed`], then [`Feed2Result::continuation`] can be
    /// used to determine if this is an intermediate node or a leaf node.
    ///
    /// # Example
    ///
    /// ```compose
    /// <a>:     X
    /// <a> <b>: Y
    /// ```
    ///
    /// After feeding `<a>`, this function returns `X`.
    ///
    /// ```compose
    /// <a> <b>: Y
    /// ```
    ///
    /// After feeding `<a>`, this function returns [`FeedResult::Pending`].
    pub fn output(&self) -> FeedResult<'a> {
        let Some(node) = &self.node else {
            return FeedResult::Aborted;
        };
        match node {
            NodeType::Intermediate {
                payload: Some(NodePayload { payload, .. }),
                ..
            }
            | NodeType::Leaf { payload } => payload.to_composed(),
            NodeType::Intermediate { payload: None, .. } => FeedResult::Pending,
        }
    }

    /// Returns whether the selected node is a terminal in the xkbcommon API.
    ///
    /// This function returns true if the node is a leaf node or it is an intermediate node and its
    /// production was declared after all productions of all child nodes.
    ///
    /// # Examples
    ///
    /// ```compose
    /// <a>:     X
    /// <a> <b>: Y
    /// ```
    ///
    /// After feeding `<a>`, this function returns `false`.
    ///
    /// ```compose
    /// <a> <b>: Y
    /// <a>:     X
    /// ```
    ///
    /// After feeding `<a>`, this function returns `true`.
    pub fn is_classic_terminal(&self) -> bool {
        let Some(node) = &self.node else {
            return false;
        };
        matches!(
            node,
            NodeType::Intermediate {
                payload: Some(NodePayload {
                    is_old_terminal: true,
                    ..
                }),
                ..
            } | NodeType::Leaf { .. }
        )
    }

    /// Uses the output of an intermediate node and resets the state to the initial state.
    ///
    /// This function has no effect if the selected node is not an intermediate node with output.
    ///
    /// This function only configures this object. The effect is not applied until
    /// [`Feed2Result::apply`] is called.
    ///
    /// # Example
    ///
    /// ```compose
    /// <a>:     X
    /// <a> <b>: Y
    /// ```
    ///
    /// After feeding `<a>`, calling this function causes the result to be [`FeedResult::Composed`].
    pub fn use_intermediate_composed(&mut self) {
        self.accept = Some(true);
    }

    /// Skips the output of an intermediate node and proceeds to its children.
    ///
    /// This function has no effect if the selected node is not an intermediate node with output.
    ///
    /// This function only configures this object. The effect is not applied until
    /// [`Feed2Result::apply`] is called.
    ///
    /// # Example
    ///
    /// ```compose
    /// <a>:     X
    /// <a> <b>: Y
    /// ```
    ///
    /// After feeding `<a>`, calling this function causes the result to be [`FeedResult::Pending`]
    /// and the state to accept `<b>` as the next input.
    pub fn skip_intermediate_composed(&mut self) {
        self.accept = Some(false);
    }

    /// Updates the state and returns its result.
    ///
    /// If no matching node was found, the state is reset to the initial state and the result is
    /// [`FeedResult::Aborted`].
    ///
    /// Otherwise, if the node is a leaf node, the state is reset to the initial state and the
    /// result is [`FeedResult::Composed`].
    ///
    /// Otherwise, if the node is an intermediate node without output, the state is set to the node
    /// and the result is [`FeedResult::Pending`].
    ///
    /// Otherwise the node is an intermediate node with output and the behavior is as follows:
    ///
    /// - If [`Feed2Result::use_intermediate_composed`] was called, the state is reset to the
    ///   initial state and the result is [`FeedResult::Composed`].
    /// - If [`Feed2Result::skip_intermediate_composed`] was called, the state is set to the node
    ///   and the result is [`FeedResult::Pending`].
    /// - If neither function was called:
    ///   - If the production for the output was declared after all productions for any of the child
    ///     nodes, the state is reset to the initial state and the result is
    ///     [`FeedResult::Composed`].
    ///   - Otherwise, the state is set to the node and the result is [`FeedResult::Pending`].
    ///
    /// If [`Feed2Result::use_intermediate_composed`] and [`Feed2Result::skip_intermediate_composed`]
    /// are both called, only the last function call is considered.
    pub fn apply(self) -> FeedResult<'a> {
        let Some(node) = &self.node else {
            *self.state = self.table.create_state();
            return FeedResult::Aborted;
        };
        match node {
            NodeType::Leaf { payload } => {
                *self.state = self.table.create_state();
                payload.to_composed()
            }
            NodeType::Intermediate {
                children,
                payload: None,
            } => {
                self.state.range = children.clone();
                FeedResult::Pending
            }
            NodeType::Intermediate {
                children,
                payload: Some(pl),
            } => {
                let accept = self.accept.unwrap_or(pl.is_old_terminal);
                match accept {
                    true => {
                        *self.state = self.table.create_state();
                        pl.payload.to_composed()
                    }
                    false => {
                        self.state.range = children.clone();
                        FeedResult::Pending
                    }
                }
            }
        }
    }
}

const HAS_PAYLOAD: u32 = 1 << 0;
const IS_OLD_TERMINAL: u32 = 1 << 1;

impl NodeData {
    fn serialize(self) -> [u32; 4] {
        let mut res = [0; 4];
        res[0] = self.children.start as u32;
        res[1] = self.children.end as u32;
        if let Some(payload) = self.payload {
            res[2] = payload as u32;
            res[3] |= HAS_PAYLOAD;
            if self.is_old_terminal {
                res[3] |= IS_OLD_TERMINAL;
            }
        }
        res
    }
}

impl Node {
    fn deserialize(&self) -> NodeData {
        let flags = self.data[3];
        let has_payload = flags & HAS_PAYLOAD != 0;
        let is_old_terminal = flags & IS_OLD_TERMINAL != 0;
        NodeData {
            children: (self.data[0] as usize)..(self.data[1] as usize),
            payload: has_payload.then_some(self.data[2] as usize),
            is_old_terminal,
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
            is_old_terminal: bool,
        }

        // the next step will add all prefixes of all productions to these vectors.
        // for example, given the input
        //
        //     <Multi_key> <a> <b>: X
        //     <Multi_key> <c>:     Y
        //     <Multi_key> <c>:     Z
        //     <Multi_key> <d> <e>: U
        //     <Multi_key> <d>:     V
        //     <Multi_key> <f>:     I
        //     <Multi_key> <f> <g>: J
        //
        // these vectors will the contain
        //
        //     <Multi_key>:         (no production)
        //     <Multi_key> <a>:     (no production)
        //     <Multi_key> <a> <b>: X
        //     <Multi_key>:         (no production)
        //     <Multi_key> <c>:     Y
        //     <Multi_key>:         (no production)
        //     <Multi_key> <c>:     Z
        //     <Multi_key>:         (no production)
        //     <Multi_key> <d>:     (no production)
        //     <Multi_key> <d> <e>: U
        //     <Multi_key>:         (no production)
        //     <Multi_key> <d>:     V
        //     <Multi_key>:         (no production)
        //     <Multi_key> <f>:     I
        //     <Multi_key>:         (no production)
        //     <Multi_key> <f>:     (no production)
        //     <Multi_key> <f> <g>: J
        let mut steps = vec![];
        let mut pre_datas = vec![];

        for (idx, production) in productions.iter().enumerate() {
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
                    is_old_terminal: false,
                });
            }
            // unwrap is fine because the parser guarantees that each production has at
            // least 1 step.
            let last = pre_datas.last_mut().unwrap();
            last.production = Some(idx);
            last.is_old_terminal = true;
        }

        // this sort is stable and therefore preserves the order of compose rules that
        // have the same set of steps.
        //
        // after this step, pre_datas has the following form in our example
        //
        //     <Multi_key>:         (no production)
        //     <Multi_key>:         (no production)
        //     <Multi_key>:         (no production)
        //     <Multi_key>:         (no production)
        //     <Multi_key>:         (no production)
        //     <Multi_key>:         (no production)
        //     <Multi_key>:         (no production)
        //     <Multi_key> <a>:     (no production)
        //     <Multi_key> <a> <b>: X
        //     <Multi_key> <c>:     Y
        //     <Multi_key> <c>:     Z
        //     <Multi_key> <d>:     (no production)
        //     <Multi_key> <d>:     V
        //     <Multi_key> <d> <e>: U
        //     <Multi_key> <f>:     I
        //     <Multi_key> <f>:     (no production)
        //     <Multi_key> <f> <g>: J
        //
        // the duplicates are removed by the next step
        pre_datas.sort_by_key(|k| &steps[k.step_range.clone()]);

        // this step deduplicates events. there are two scenarios:
        //
        // 1. for each list of steps, we choose the last PreData with this list
        // 2. if we've selected a PreData X that contains a production, then any PreData whose list
        //    of steps is an extension of X's list of steps is discarded
        //
        // since pre_datas is sorted in order in which the rules were parsed, this implies that
        // later rules overwrite earlier rules.
        //
        // after this step, pre_datas_dedup has the following form in our example
        //
        //     <Multi_key>:         (no production)
        //     <Multi_key> <a>:     (no production)
        //     <Multi_key> <a> <b>: X
        //     <Multi_key> <c>:     Z
        //     <Multi_key> <d>:     V
        //     <Multi_key> <f>:     (no production)
        //     <Multi_key> <f> <g>: J
        let mut pre_datas_dedup = Vec::<&mut PreData>::new();
        let mut prev_step = None;
        let mut prev_len = 0;
        for k in &mut pre_datas {
            let len = k.step_range.len();
            let step = steps[k.step_range.end - 1];
            if (Some(step), len) == (prev_step, prev_len) {
                let prev = pre_datas_dedup.pop().unwrap();
                if k.production.is_some() {
                    if let Some(pl) = prev.production {
                        let pl = &productions[pl];
                        diagnostics.push(
                            map,
                            DiagnosticKind::ComposeProductionOverwritten,
                            ad_hoc_display!("compose production has been overwritten")
                                .spanned2(pl.span),
                        );
                    }
                } else {
                    k.production = prev.production;
                }
            }
            prev_step = Some(step);
            prev_len = len;
            pre_datas_dedup.push(k);
        }

        struct Data {
            step: Step,
            production: Option<usize>,
            num_children: u32,
            parent: Option<u32>,
            heap_pos: Cell<u32>,
            children_heap_pos: Cell<u32>,
            next_child_pos: Cell<u32>,
            is_old_terminal: bool,
        }

        // the next steps deduplicates the contents of pre_datas and counts for each entry
        // how many children it has. in our example
        //
        //     root node:                 1 child
        //       <Multi_key>:             4 children
        //         <Multi_key> <a>:       1 child
        //           <Multi_key> <a> <b>: 0 children (production = X)
        //         <Multi_key> <c>:       0 children (production = Z)
        //         <Multi_key> <d>:       0 children (production = V)
        //         <Multi_key> <f>:       1 child
        //           <Multi_key> <f> <g>: 0 children (production = J)
        let mut datas: Vec<Data> = vec![];
        let mut num_root = 0;

        let mut stack: Vec<u32> = vec![];
        let mut prev_len = 0;

        for pre_data in pre_datas_dedup {
            let len = pre_data.step_range.len();
            let step = steps[pre_data.step_range.end - 1];
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
                    Some(idx)
                }
                _ => {
                    num_root += 1;
                    None
                }
            };
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
                is_old_terminal: pre_data.is_old_terminal,
            });
        }

        // the next steps assigns each node a position in the heap and tells it where its
        // children will be positioned in the heap. in our example
        //
        //     <Multi_key>:         pos 1, children start at pos 2
        //     <Multi_key> <a>:     pos 2, children start at pos 6
        //     <Multi_key> <a> <b>: pos 6
        //     <Multi_key> <c>:     pos 3
        //     <Multi_key> <d>:     pos 4
        //     <Multi_key> <f>:     pos 5, children start at pos 7
        //     <Multi_key> <f> <g>: pos 7
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
        //     <Multi_key> <a>:     pos 2, children start at pos 6
        //     <Multi_key> <c>:     pos 3
        //     <Multi_key> <d>:     pos 4
        //     <Multi_key> <f>:     pos 5
        //     <Multi_key> <a> <b>: pos 6
        //     <Multi_key> <f> <g>: pos 7
        datas.sort_unstable_by_key(|d| d.heap_pos.get());

        // the next step converts the datas to the final node structure
        let mut payloads = vec![];
        let mut nodes = Vec::with_capacity(datas.len() + 1);
        let root_children = 1..1 + num_root;
        // the root node
        nodes.push(Node {
            keysym: Default::default(),
            data: NodeData {
                children: root_children.clone(),
                payload: None,
                is_old_terminal: false,
            }
            .serialize(),
        });

        for data in datas {
            let payload = match data.production {
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
                    Some(pos)
                }
                _ => None,
            };
            let children = if data.num_children > 0 {
                let lo = data.children_heap_pos.get() as usize;
                let hi = lo + data.num_children as usize;
                lo..hi
            } else {
                root_children.clone()
            };
            let ty = NodeData {
                children,
                payload,
                is_old_terminal: data.is_old_terminal,
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
        State {
            range: self.nodes[0].deserialize().children,
        }
    }

    /// Advances the compose state.
    ///
    /// The `state` should have been created by [`Self::create_state`]. Otherwise this function
    /// might panic.
    ///
    /// This function is the same as calling [`Self::feed2`] immediately followed by
    /// [`Feed2Result::apply`]. This corresponds to the behavior of xkbcommon.
    ///
    /// This function does not support productions were one is a prefix of another. For example, the
    /// following compose files behave the same with this function:
    ///
    /// ```compose
    /// <a> <b>:     X
    /// ```
    ///
    /// ```compose
    /// <a>:         Y
    /// <a> <b>:     X
    /// ```
    ///
    /// ```compose
    /// <a>:         Y
    /// <a> <b> <c>: Y
    /// <a> <b>:     X
    /// ```
    ///
    /// If more control is needed over the behavior of overlapping productions, [`Self::feed2`]
    /// should be used instead.
    pub fn feed(&self, state: &mut State, sym: Keysym) -> Option<FeedResult<'_>> {
        self.feed2(state, sym).map(|o| o.apply())
    }

    /// Finds the next step in a compose sequence.
    ///
    /// The `state` should have been created by [`Self::create_state`]. Otherwise this function
    /// might panic.
    ///
    /// This function does not update the `state`. The state can be updated by calling
    /// [`Feed2Result::apply`].
    ///
    /// This function returns `None` if the call had no effect. This happens in two
    /// situations:
    ///
    /// - The keysym is a modifier (e.g. `Shift_L`).
    /// - The state is in the initial state (that is, there is no ongoing compose
    ///   sequence) and the keysym does not start a compose sequence.
    ///
    /// Otherwise, the [`Feed2Result`] can be used to inspect the selected node. See the
    /// documentation of [`ComposeTable`] for a description of nodes.
    pub fn feed2<'a, 'b>(
        &'a self,
        state: &'b mut State,
        sym: Keysym,
    ) -> Option<Feed2Result<'a, 'b>> {
        if sym.is_modifier() {
            return None;
        }
        let range = &self.nodes[state.range.clone()];
        let node = match range.binary_search_by_key(&sym, |n| n.keysym) {
            Ok(node) => {
                let node = range[node].deserialize();
                let ty = if node.children.start > 1 {
                    NodeType::Intermediate {
                        children: node.children.clone(),
                        payload: node.payload.map(|pl| NodePayload {
                            payload: &self.payloads[pl],
                            is_old_terminal: node.is_old_terminal,
                        }),
                    }
                } else {
                    NodeType::Leaf {
                        payload: &self.payloads[node.payload.unwrap()],
                    }
                };
                Some(ty)
            }
            Err(..) => {
                if state.range.start == 1 {
                    return None;
                }
                None
            }
        };
        Some(Feed2Result {
            state,
            table: self,
            node,
            accept: None,
        })
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

impl Payload {
    fn to_composed(&self) -> FeedResult<'_> {
        FeedResult::Composed {
            string: self.string.as_deref(),
            keysym: self.keysym,
        }
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
        loop {
            let mut range = self.child_range.pop()?;
            let Some(idx) = range.next() else {
                self.stack.pop();
                continue;
            };
            self.child_range.push(range);
            let node = &self.table.nodes[idx];
            self.stack.push(MatchStep { node });
            let data = node.deserialize();
            let child_range = if data.children.start > 1 {
                data.children
            } else {
                0..0
            };
            self.child_range.push(child_range);
            if let Some(payload) = data.payload {
                let payload = &self.table.payloads[payload];
                return Some(MatchRule {
                    steps: &self.stack,
                    payload,
                });
            }
        }
    }
}
