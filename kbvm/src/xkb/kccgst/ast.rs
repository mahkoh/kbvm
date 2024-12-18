use {
    crate::{
        state_machine::Keycode,
        xkb::{
            include::IncludeGroup,
            interner::Interned,
            kccgst::token::Token,
            span::{Span, Spanned},
        },
    },
    kbvm_proc::CloneWithDelta,
    std::fmt::Debug,
};

#[derive(Debug, CloneWithDelta)]
pub(crate) struct Item {
    pub(crate) flags: Flags,
    pub(crate) ty: ItemType,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct FlagWrapper {
    pub(crate) name: Spanned<Interned>,
    pub(crate) flag: Flag,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct Flags {
    pub(crate) flags: Vec<FlagWrapper>,
}

#[derive(Copy, Clone, Debug, PartialEq, CloneWithDelta)]
pub(crate) enum Flag {
    Partial,
    Default,
    Hidden,
    AlphanumericKeys,
    ModifierKeys,
    KeypadKeys,
    FunctionKeys,
    AlternateGroup,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) enum ItemType {
    Composite(CompositeMap),
    Config(ConfigItemType),
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct CompositeMap {
    pub(crate) ty: Spanned<Interned>,
    pub(crate) name: Option<Spanned<Interned>>,
    pub(crate) config_items: Vec<Spanned<NestedConfigItem>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct NestedConfigItem {
    pub(crate) flags: Flags,
    pub(crate) item: ConfigItem,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct ConfigItem {
    pub(crate) ty: Spanned<Interned>,
    pub(crate) specific: ConfigItemType,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) enum ConfigItemType {
    Keycodes(Keycodes),
    Types(Types),
    Compatmap(Compatmap),
    Symbols(Symbols),
    Geometry(Geometry),
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct Keycodes {
    pub(crate) name: Option<Spanned<Interned>>,
    pub(crate) decls: Decls<KeycodeDecl>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct Decls<T> {
    pub(crate) decls: Vec<Spanned<Decl<T>>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct Decl<T> {
    pub(crate) merge_mode: Option<Spanned<MergeMode>>,
    pub(crate) ty: DirectOrIncluded<T>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) enum DirectOrIncluded<T> {
    Direct(T),
    Included(Included<T>),
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct Included<T> {
    pub(crate) components: Vec<Component<T>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct Component<T> {
    pub(crate) mm: Option<Spanned<MergeMode>>,
    pub(crate) decls: Decls<T>,
    pub(crate) group: Option<IncludeGroup>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) enum KeycodeDecl {
    Include(Include),
    KeyName(KeyNameDecl),
    KeyAlias(KeyAliasDecl),
    Var(VarDecl),
    IndicatorName(IndicatorNameDecl),
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct Types {
    pub(crate) name: Option<Spanned<Interned>>,
    pub(crate) decls: Decls<TypesDecl>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) enum TypesDecl {
    Include(Include),
    KeyType(KeyTypeDecl),
    Var(VarDecl),
    VMod(VModDecl),
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct Compatmap {
    pub(crate) name: Option<Spanned<Interned>>,
    pub(crate) decls: Decls<CompatmapDecl>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) enum CompatmapDecl {
    Include(Include),
    Interpret(InterpretDecl),
    GroupCompat(GroupCompatDecl),
    IndicatorMap(LedMapDecl),
    Var(VarDecl),
    VMod(VModDecl),
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct Symbols {
    pub(crate) name: Option<Spanned<Interned>>,
    pub(crate) decls: Decls<SymbolsDecl>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) enum SymbolsDecl {
    Include(Include),
    Symbols(KeySymbolsDecl),
    Var(VarDecl),
    VMod(VModDecl),
    ModMap(ModMapDecl),
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct Geometry {
    pub(crate) name: Option<Spanned<Interned>>,
    pub(crate) decls: Decls<GeometryDecl>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) enum GeometryDecl {
    Include(Include),
    KeyAlias(KeyAliasDecl),
    Var(VarDecl),
    Shape(ShapeDecl),
    Section(SectionDecl),
    LedMap(LedMapDecl),
    Doodad(DoodadDecl),
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct VarDecl {
    pub(crate) var: Var,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct Var {
    pub(crate) not: Option<Span>,
    pub(crate) path: Spanned<Path>,
    pub(crate) expr: Option<Spanned<Expr>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct InterpretDecl {
    pub(crate) match_: Spanned<InterpretMatch>,
    pub(crate) vars: Vec<Spanned<VarDecl>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct InterpretMatch {
    pub(crate) sym: Spanned<Interned>,
    pub(crate) filter: Option<Spanned<Expr>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct KeyNameDecl {
    pub(crate) name: Spanned<Interned>,
    pub(crate) code_str: Spanned<Interned>,
    pub(crate) code: Keycode,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct KeyAliasDecl {
    pub(crate) name: Spanned<Interned>,
    pub(crate) alias_for: Spanned<Interned>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct KeyTypeDecl {
    pub(crate) name: Spanned<Interned>,
    pub(crate) decls: Vec<Spanned<VarDecl>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct KeySymbolsDecl {
    pub(crate) key: Spanned<Interned>,
    pub(crate) vars: Vec<Spanned<VarOrExpr>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) enum VarOrExpr {
    Var(Var),
    Expr(Expr),
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct ModMapDecl {
    pub(crate) modifier: Spanned<Interned>,
    pub(crate) keys: Vec<Spanned<Expr>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct GroupCompatDecl {
    pub(crate) group_name: Spanned<Interned>,
    pub(crate) group: u32,
    pub(crate) val: Spanned<Expr>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct LedMapDecl {
    pub(crate) name: Spanned<Interned>,
    pub(crate) decls: Vec<Spanned<VarDecl>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct IndicatorNameDecl {
    pub(crate) virt: Option<Span>,
    pub(crate) idx_name: Spanned<Interned>,
    pub(crate) idx: u32,
    pub(crate) val: Spanned<Expr>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct ShapeDecl {
    pub(crate) name: Spanned<Interned>,
    pub(crate) ty: Spanned<ShapeDeclType>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) enum ShapeDeclType {
    OutlineList(Vec<Spanned<Outline>>),
    CoordList(Vec<Spanned<Coord>>),
}

#[derive(Debug, CloneWithDelta)]
pub(crate) enum Outline {
    CoordList(Vec<Spanned<Coord>>),
    ExprAssignment(ExprAssignment),
    CoordAssignment(CoordAssignment),
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct ExprAssignment {
    pub(crate) name: Spanned<Interned>,
    pub(crate) expr: Spanned<Expr>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct CoordAssignment {
    pub(crate) name: Spanned<Interned>,
    pub(crate) coords: Spanned<Vec<Spanned<Coord>>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct Coord {
    pub(crate) x: Spanned<Expr>,
    pub(crate) y: Spanned<Expr>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct SectionDecl {
    pub(crate) name: Spanned<Interned>,
    pub(crate) items: Vec<Spanned<SectionItem>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) enum SectionItem {
    Row(RowBody),
    Var(VarDecl),
    Doodad(DoodadDecl),
    LedMap(LedMapDecl),
    Overlay(OverlayDecl),
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct RowBody {
    pub(crate) items: Vec<Spanned<RowBodyItem>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) enum RowBodyItem {
    Keys(Keys),
    Var(VarDecl),
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct Keys {
    pub(crate) keys: Vec<Spanned<Key>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) enum Key {
    Name(Interned),
    Exprs(KeyExprs),
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct KeyExprs {
    pub(crate) exprs: Vec<Spanned<VarOrExpr>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct OverlayDecl {
    pub(crate) name: Spanned<Interned>,
    pub(crate) items: Vec<Spanned<OverlayItem>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct OverlayItem {
    pub(crate) name: Spanned<Interned>,
    pub(crate) alias: Spanned<Interned>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct DoodadDecl {
    pub(crate) ty: Spanned<DoodadType>,
    pub(crate) name: Spanned<Interned>,
    pub(crate) decls: Vec<Spanned<VarDecl>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct VModDecl {
    pub(crate) defs: Vec<Spanned<VModDef>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct VModDef {
    pub(crate) name: Spanned<Interned>,
    pub(crate) val: Option<Spanned<Expr>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct Include {
    pub(crate) mm: Spanned<MergeMode>,
    pub(crate) path: Spanned<Interned>,
    pub(crate) resolved: Option<Vec<ResolvedInclude>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct ResolvedInclude {
    pub(crate) mm: Option<Spanned<MergeMode>>,
    pub(crate) item: Spanned<Item>,
    pub(crate) group: Option<IncludeGroup>,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, CloneWithDelta)]
pub(crate) enum MergeMode {
    Include,
    Augment,
    Override,
    Replace,
    Alternate,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, CloneWithDelta)]
pub(crate) enum DoodadType {
    Text,
    Outline,
    Solid,
    Logo,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) enum CallArg {
    Expr(Expr),
    NamedParam(NamedParam),
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct NamedParam {
    pub(crate) name: Spanned<Path>,
    pub(crate) expr: Spanned<Expr>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) enum Expr {
    UnMinus(Box<Spanned<Expr>>),
    UnPlus(Box<Spanned<Expr>>),
    UnNot(Box<Spanned<Expr>>),
    UnInverse(Box<Spanned<Expr>>),
    Path(Path),
    Call(Call),
    String(Interned),
    Integer(Interned, i64),
    Float(Interned, f64),
    KeyName(Interned),
    Parenthesized(Box<Spanned<Expr>>),
    Mul(Box<Spanned<Expr>>, Box<Spanned<Expr>>),
    Div(Box<Spanned<Expr>>, Box<Spanned<Expr>>),
    Add(Box<Spanned<Expr>>, Box<Spanned<Expr>>),
    Sub(Box<Spanned<Expr>>, Box<Spanned<Expr>>),
    BracketList(Vec<Spanned<Expr>>),
    BraceList(Vec<Spanned<Expr>>),
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct Call {
    pub(crate) path: Spanned<Path>,
    pub(crate) args: Vec<Spanned<CallArg>>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct Path {
    pub(crate) components: Vec<PathComponent>,
}

impl Path {
    pub(crate) fn unique_ident(&self) -> Option<Interned> {
        if self.components.len() == 1 && self.components[0].index.is_none() {
            return Some(self.components[0].ident.val);
        }
        None
    }
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct PathComponent {
    pub(crate) ident: Spanned<Interned>,
    pub(crate) index: Option<PathIndex>,
}

#[derive(Debug, CloneWithDelta)]
pub(crate) struct PathIndex {
    pub(crate) obracket: Spanned<Token>,
    pub(crate) index: Spanned<Expr>,
}
