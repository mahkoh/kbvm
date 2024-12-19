use {
    crate::{
        keysym::Keysym,
        modifier::{ModifierIndex, ModifierMask},
        state_machine::Keycode,
        xkb::{
            controls::ControlMask,
            group::{GroupChange, GroupIdx, GroupMask},
            group_component::GroupComponentMask,
            indicator::IndicatorIdx,
            interner::Interned,
            level::Level,
            mod_component::ModComponentMask,
            modmap::Vmodmap,
            span::{Span, Spanned},
        },
    },
    hashbrown::HashMap,
    indexmap::IndexMap,
    linearize::Linearize,
    smallvec::SmallVec,
};

#[derive(Debug)]
pub(crate) struct Resolved {
    pub(crate) name: Option<Spanned<Interned>>,
    pub(crate) mods: Vmodmap,
    pub(crate) keycodes: ResolvedKeycodes,
    pub(crate) types: ResolvedTypes,
    pub(crate) compat: ResolvedCompat,
    pub(crate) symbols: ResolvedSymbols,
}

#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug)]
pub(crate) struct Filter {
    pub(crate) predicate: Predicate,
    pub(crate) mask: Spanned<ModifierMask>,
}

#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug)]
pub(crate) enum Predicate {
    NoneOf,
    AnyOfOrNone,
    AnyOf,
    AllOf,
    Exactly,
}

#[derive(Default, Debug)]
pub(crate) struct ResolvedKeycodes {
    pub(crate) indicators: Vec<Indicator>,
    pub(crate) keycode_to_name: HashMap<Keycode, Interned>,
    pub(crate) name_to_key: HashMap<Interned, ResolvedKey>,
}

#[derive(Debug)]
pub(crate) struct ResolvedKey {
    pub(crate) name: Spanned<Interned>,
    pub(crate) kind: ResolvedKeyKind,
}

#[derive(Copy, Clone, Debug)]
pub(crate) enum ResolvedKeyKind {
    Real(Spanned<Keycode>),
    Alias(Spanned<Interned>, Keycode, Option<Spanned<Interned>>),
}

#[derive(Debug)]
pub(crate) struct Indicator {
    pub(crate) idx: Spanned<IndicatorIdx>,
    pub(crate) name: Spanned<Interned>,
    pub(crate) virt: Option<Span>,
}

#[derive(Default, Debug)]
pub(crate) struct ResolvedTypes {
    pub(crate) default: ResolvedKeyType,
    pub(crate) key_types: HashMap<Interned, ResolvedKeyTypeWithName>,
}

#[derive(Clone, Debug)]
pub(crate) struct ResolvedKeyTypeWithName {
    pub(crate) name: Spanned<Interned>,
    pub(crate) ty: ResolvedKeyType,
}

#[derive(Clone, Default, Debug)]
pub(crate) struct ResolvedKeyType {
    pub(crate) modifiers: Option<Spanned<ModifierMask>>,
    pub(crate) map: HashMap<ModifierMask, Spanned<Level>>,
    pub(crate) preserved: HashMap<ModifierMask, Spanned<ModifierMask>>,
    pub(crate) names: HashMap<Level, Spanned<Interned>>,
}

#[derive(Default, Debug)]
pub(crate) struct ResolvedCompat {
    pub(crate) interp_default: Interp,
    pub(crate) interps: IndexMap<(Option<Keysym>, Option<Filter>), InterpWithKey>,
    pub(crate) interps_sorted: Vec<InterpWithKey>,
    pub(crate) indicator_map_default: IndicatorMap,
    pub(crate) indicator_maps: IndexMap<Interned, IndicatorMapWithKey>,
    pub(crate) action_defaults: ActionDefaults,
}

#[derive(Default, Debug)]
pub(crate) struct ActionDefaults {
    pub(crate) no_action: ResolvedNoAction,
    pub(crate) mods_set: ResolvedModsSet,
    pub(crate) mods_latch: ResolvedModsLatch,
    pub(crate) mods_lock: ResolvedModsLock,
    pub(crate) group_set: ResolvedGroupSet,
    pub(crate) group_latch: ResolvedGroupLatch,
    pub(crate) group_lock: ResolvedGroupLock,
}

#[derive(Debug)]
pub(crate) struct InterpWithKey {
    pub(crate) keysym: Option<Spanned<Keysym>>,
    pub(crate) filter: Option<Spanned<Filter>>,
    pub(crate) interp: Interp,
    pub(crate) version: u64,
}

#[derive(Clone, Default, Debug)]
pub(crate) struct Interp {
    pub(crate) action: Option<Spanned<ResolvedAction>>,
    pub(crate) virtual_modifier: Option<Spanned<ModifierIndex>>,
    pub(crate) repeat: Option<Spanned<bool>>,
    pub(crate) locking: Option<Spanned<bool>>,
    pub(crate) level_one_only: Option<Spanned<bool>>,
}

impl Interp {
    pub(crate) fn level_one_only(&self) -> bool {
        self.level_one_only.map(|l| l.val).unwrap_or_default()
    }
}

#[derive(Debug)]
pub(crate) struct IndicatorMapWithKey {
    pub(crate) name: Spanned<Interned>,
    pub(crate) indicator_map: IndicatorMap,
}

#[derive(Clone, Default, Debug)]
pub(crate) struct IndicatorMap {
    pub(crate) idx: Option<IndicatorIdx>,
    pub(crate) virt: bool,
    pub(crate) modifiers: Option<Spanned<ModifierMask>>,
    pub(crate) groups: Option<Spanned<GroupMask>>,
    pub(crate) controls: Option<Spanned<ControlMask>>,
    pub(crate) whichmodifierstate: Option<Spanned<ModComponentMask>>,
    pub(crate) whichgroupstate: Option<Spanned<GroupComponentMask>>,
}

#[derive(Default, Clone, Debug)]
pub(crate) struct SymbolsKey {
    pub(crate) groups: Vec<SymbolsKeyGroup>,
    pub(crate) default_key_type: Option<Spanned<Interned>>,
    pub(crate) virtualmodifiers: Option<Spanned<ModifierMask>>,
    pub(crate) repeating: Option<Spanned<Option<bool>>>,
    pub(crate) groups_redirect: Option<Spanned<GroupsRedirect>>,
    pub(crate) modmap: ModifierMask,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) enum GroupsRedirect {
    Wrap,
    Clamp,
    Redirect(GroupIdx),
}

#[derive(Default, Clone, Debug)]
pub(crate) struct SymbolsKeyGroup {
    pub(crate) levels: Vec<SymbolsKeyLevel>,
    pub(crate) key_type: Option<KeyTypeRef>,
    pub(crate) has_explicit_symbols: bool,
    pub(crate) has_explicit_actions: bool,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Linearize)]
pub(crate) enum BuiltInKeytype {
    OneLevel,
    Alphabetic,
    Keypad,
    TwoLevel,
    FourLevelAlphabetic,
    FourLevelSemialphabetic,
    FourLevelKeypad,
    FourLevel,
}

impl BuiltInKeytype {
    pub(crate) const fn name(self) -> &'static str {
        match self {
            BuiltInKeytype::OneLevel => "ONE_LEVEL",
            BuiltInKeytype::Alphabetic => "ALPHABETIC",
            BuiltInKeytype::Keypad => "KEYPAD",
            BuiltInKeytype::TwoLevel => "TWO_LEVEL",
            BuiltInKeytype::FourLevelAlphabetic => "FOUR_LEVEL_ALPHABETIC",
            BuiltInKeytype::FourLevelSemialphabetic => "FOUR_LEVEL_SEMIALPHABETIC",
            BuiltInKeytype::FourLevelKeypad => "FOUR_LEVEL_KEYPAD",
            BuiltInKeytype::FourLevel => "FOUR_LEVEL",
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) enum KeyTypeRef {
    BuiltIn(BuiltInKeytype),
    Named(Spanned<Interned>),
}

#[derive(Default, Clone, Debug)]
pub(crate) struct SymbolsKeyLevel {
    pub(crate) symbols: SmallVec<[Option<Spanned<Keysym>>; 1]>,
    pub(crate) actions: SmallVec<[Option<Spanned<ResolvedAction>>; 1]>,
}

#[derive(Debug)]
pub(crate) struct SymbolsKeyWithKey {
    pub(crate) name: Spanned<Interned>,
    pub(crate) code: Keycode,
    pub(crate) key: SymbolsKey,
}

#[derive(Debug)]
pub(crate) struct ModMapEntryWithKey {
    pub(crate) key: Spanned<ModMapField>,
    pub(crate) modifier: Option<Spanned<ModifierIndex>>,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub(crate) enum ModMapField {
    Keysym(Keysym),
    Keycode(Keycode),
}

#[derive(Default, Debug)]
pub(crate) struct ResolvedSymbols {
    pub(crate) mod_map_entries: HashMap<ModMapField, ModMapEntryWithKey>,
    pub(crate) action_defaults: ActionDefaults,
    pub(crate) key_default: SymbolsKey,
    pub(crate) keys: HashMap<Keycode, SymbolsKeyWithKey>,
    pub(crate) group_names: Vec<Option<(GroupIdx, Spanned<Interned>)>>,
}

#[derive(Copy, Clone, Debug)]
pub(crate) enum ResolvedAction {
    ResolvedNoAction(ResolvedNoAction),
    ResolvedModsSet(ResolvedModsSet),
    ResolvedModsLatch(ResolvedModsLatch),
    ResolvedModsLock(ResolvedModsLock),
    ResolvedGroupSet(ResolvedGroupSet),
    ResolvedGroupLatch(ResolvedGroupLatch),
    ResolvedGroupLock(ResolvedGroupLock),
}

#[derive(Copy, Clone, Debug, Default)]
pub(crate) struct ResolvedNoAction;

#[derive(Copy, Clone, Debug)]
pub(crate) enum ResolvedActionMods {
    ModMap,
    Explicit(ModifierMask),
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct ResolvedActionAffect {
    pub(crate) lock: bool,
    pub(crate) unlock: bool,
}

#[derive(Copy, Clone, Default, Debug)]
pub(crate) struct ResolvedModsSet {
    pub(crate) clear_locks: Option<Spanned<bool>>,
    pub(crate) modifiers: Option<Spanned<ResolvedActionMods>>,
}

#[derive(Copy, Clone, Default, Debug)]
pub(crate) struct ResolvedModsLatch {
    pub(crate) clear_locks: Option<Spanned<bool>>,
    pub(crate) latch_to_lock: Option<Spanned<bool>>,
    pub(crate) modifiers: Option<Spanned<ResolvedActionMods>>,
}

#[derive(Copy, Clone, Default, Debug)]
pub(crate) struct ResolvedModsLock {
    pub(crate) modifiers: Option<Spanned<ResolvedActionMods>>,
    pub(crate) affect: Option<Spanned<ResolvedActionAffect>>,
}

#[derive(Copy, Clone, Default, Debug)]
pub(crate) struct ResolvedGroupSet {
    pub(crate) group: Option<Spanned<GroupChange>>,
    pub(crate) clear_locks: Option<Spanned<bool>>,
}

#[derive(Copy, Clone, Default, Debug)]
pub(crate) struct ResolvedGroupLatch {
    pub(crate) group: Option<Spanned<GroupChange>>,
    pub(crate) clear_locks: Option<Spanned<bool>>,
    pub(crate) latch_to_lock: Option<Spanned<bool>>,
}

#[derive(Copy, Clone, Default, Debug)]
pub(crate) struct ResolvedGroupLock {
    pub(crate) group: Option<Spanned<GroupChange>>,
}
