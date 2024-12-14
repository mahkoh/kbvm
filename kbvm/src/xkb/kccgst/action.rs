use crate::{
    modifier::ModifierMask,
    xkb::{kccgst::expr::Sign, span::Spanned},
};

pub enum Action {
    ModsSet(ModsSet),
    ModsLatch(ModsLatch),
    ModsLock(ModsLock),
    GroupSet(GroupSet),
    GroupLatch(GroupLatch),
    GroupLock(GroupLock),
}

#[derive(Copy, Clone, Debug)]
pub enum ActionMods {
    ModMap,
    Explicit(ModifierMask),
}

#[derive(Copy, Clone, Debug)]
pub struct ActionAffect {
    pub lock: bool,
    pub unlock: bool,
}

#[derive(Default)]
pub struct ModsSet {
    pub clear_locks: Option<Spanned<bool>>,
    pub modifiers: Option<Spanned<ActionMods>>,
}

#[derive(Default)]
pub struct ModsLatch {
    pub clear_locks: Option<Spanned<bool>>,
    pub latch_to_lock: Option<Spanned<bool>>,
    pub modifiers: Option<Spanned<ActionMods>>,
}

#[derive(Default)]
pub struct ModsLock {
    pub modifiers: Option<Spanned<ActionMods>>,
    pub affect: Option<Spanned<ActionAffect>>,
}

#[derive(Default)]
pub struct GroupSet {
    pub group: Option<Spanned<(u32, Option<Sign>)>>,
    pub clear_locks: Option<Spanned<bool>>,
}

#[derive(Default)]
pub struct GroupLatch {
    pub group: Option<Spanned<(u32, Option<Sign>)>>,
    pub clear_locks: Option<Spanned<bool>>,
    pub latch_to_lock: Option<Spanned<bool>>,
}

#[derive(Default)]
pub struct GroupLock {
    pub group: Option<Spanned<(u32, Option<Sign>)>>,
}
