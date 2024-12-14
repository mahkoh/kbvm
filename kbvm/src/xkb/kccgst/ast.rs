use {
    crate::{
        state_machine::Keycode,
        xkb::{
            interner::Interned,
            kccgst::{token::Token, vmodmap::Vmodmap},
            span::Spanned,
        },
    },
    kbvm_proc::CloneWithDelta,
    std::fmt::Debug,
};

#[derive(Debug, CloneWithDelta)]
pub struct Item {
    pub flags: Flags,
    pub ty: ItemType,
}

#[derive(Debug, CloneWithDelta)]
pub struct FlagWrapper {
    pub name: Spanned<Interned>,
    pub flag: Flag,
}

#[derive(Debug, CloneWithDelta)]
pub struct Flags {
    pub flags: Vec<FlagWrapper>,
}

#[derive(Copy, Clone, Debug, PartialEq, CloneWithDelta)]
pub enum Flag {
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
pub enum ItemType {
    Composite(CompositeMap),
    Config(ConfigItemType),
}

#[derive(Debug, CloneWithDelta)]
pub struct CompositeMap {
    pub ty: Spanned<Interned>,
    pub name: Option<Spanned<Interned>>,
    pub config_items: Vec<Spanned<NestedConfigItem>>,
}

#[derive(Debug, CloneWithDelta)]
pub struct NestedConfigItem {
    pub flags: Flags,
    pub item: ConfigItem,
}

#[derive(Debug, CloneWithDelta)]
pub struct ConfigItem {
    pub ty: Spanned<Interned>,
    pub specific: ConfigItemType,
}

#[derive(Debug, CloneWithDelta)]
pub enum ConfigItemType {
    Keycodes(Keycodes),
    Types(Types),
    Compatmap(Compatmap),
    Symbols(Symbols),
    Geometry(Geometry),
}

#[derive(Debug, CloneWithDelta)]
pub struct Keycodes {
    pub name: Option<Spanned<Interned>>,
    pub decls: Decls<KeycodeDecl>,
}

#[derive(Debug, CloneWithDelta)]
pub struct Decls<T> {
    pub decls: Vec<Spanned<Decl<T>>>,
}

#[derive(Debug, CloneWithDelta)]
pub struct Decl<T> {
    pub merge_mode: Option<Spanned<MergeMode>>,
    pub ty: DirectOrIncluded<T>,
}

#[derive(Debug, CloneWithDelta)]
pub enum DirectOrIncluded<T> {
    Direct(T),
    Included(Included<T>),
}

#[derive(Debug, CloneWithDelta)]
pub struct Included<T> {
    pub vmods: Vmodmap,
    pub components: Vec<Component<T>>,
}

#[derive(Debug, CloneWithDelta)]
pub struct Component<T> {
    pub mm: Option<Spanned<MergeMode>>,
    pub decls: Decls<T>,
}

#[derive(Debug, CloneWithDelta)]
pub enum KeycodeDecl {
    Include(Include),
    KeyName(KeyNameDecl),
    KeyAlias(KeyAliasDecl),
    Var(VarDecl),
    LedName(LedNameDecl),
}

#[derive(Debug, CloneWithDelta)]
pub struct Types {
    pub name: Option<Spanned<Interned>>,
    pub decls: Decls<TypesDecl>,
}

#[derive(Debug, CloneWithDelta)]
pub enum TypesDecl {
    Include(Include),
    KeyType(KeyTypeDecl),
    Var(VarDecl),
    VMod(VModDecl),
}

#[derive(Debug, CloneWithDelta)]
pub struct Compatmap {
    pub name: Option<Spanned<Interned>>,
    pub decls: Decls<CompatmapDecl>,
}

#[derive(Debug, CloneWithDelta)]
pub enum CompatmapDecl {
    Include(Include),
    Interpret(InterpretDecl),
    GroupCompat(GroupCompatDecl),
    LedMap(LedMapDecl),
    Var(VarDecl),
    VMod(VModDecl),
}

#[derive(Debug, CloneWithDelta)]
pub struct Symbols {
    pub name: Option<Spanned<Interned>>,
    pub decls: Decls<SymbolsDecl>,
}

#[derive(Debug, CloneWithDelta)]
pub enum SymbolsDecl {
    Include(Include),
    Symbols(KeySymbolsDecl),
    Var(VarDecl),
    VMod(VModDecl),
    ModMap(ModMapDecl),
}

#[derive(Debug, CloneWithDelta)]
pub struct Geometry {
    pub name: Option<Spanned<Interned>>,
    pub decls: Decls<GeometryDecl>,
}

#[derive(Debug, CloneWithDelta)]
pub enum GeometryDecl {
    Include(Include),
    KeyAlias(KeyAliasDecl),
    Var(VarDecl),
    Shape(ShapeDecl),
    Section(SectionDecl),
    LedMap(LedMapDecl),
    Doodad(DoodadDecl),
}

#[derive(Debug, CloneWithDelta)]
pub enum VarDecl {
    VarAssign(VarAssign),
    Ident(VarDeclIdent),
}

#[derive(Debug, CloneWithDelta)]
pub struct VarAssign {
    pub path: Spanned<Path>,
    pub expr: Spanned<Expr>,
}

#[derive(Debug, CloneWithDelta)]
pub struct VarDeclIdent {
    pub not: bool,
    pub ident: Spanned<Interned>,
}

#[derive(Debug, CloneWithDelta)]
pub struct InterpretDecl {
    pub match_: Spanned<InterpretMatch>,
    pub vars: Vec<Spanned<VarDecl>>,
}

#[derive(Debug, CloneWithDelta)]
pub struct InterpretMatch {
    pub sym: Spanned<Interned>,
    pub filter: Option<Spanned<Expr>>,
}

#[derive(Debug, CloneWithDelta)]
pub struct KeyNameDecl {
    pub name: Spanned<Interned>,
    pub code_str: Spanned<Interned>,
    pub code: Keycode,
}

#[derive(Debug, CloneWithDelta)]
pub struct KeyAliasDecl {
    pub name: Spanned<Interned>,
    pub alias_for: Spanned<Interned>,
}

#[derive(Debug, CloneWithDelta)]
pub struct KeyTypeDecl {
    pub name: Spanned<Interned>,
    pub decls: Vec<Spanned<VarDecl>>,
}

#[derive(Debug, CloneWithDelta)]
pub struct KeySymbolsDecl {
    pub key: Spanned<Interned>,
    pub vars: Vec<Spanned<SymbolsVarDecl>>,
}

#[derive(Debug, CloneWithDelta)]
pub enum SymbolsVarDecl {
    ExprAssign(SymbolsExprAssign),
    ArrayAssign(SymbolsArrayAssign),
    Ident(SymbolsIdent),
    Array(ArrayInit),
}

#[derive(Debug, CloneWithDelta)]
pub struct SymbolsIdent {
    pub not: bool,
    pub ident: Spanned<Interned>,
}

#[derive(Debug, CloneWithDelta)]
pub struct SymbolsExprAssign {
    pub lhs: Spanned<Path>,
    pub expr: Spanned<Expr>,
}

#[derive(Debug, CloneWithDelta)]
pub struct SymbolsArrayAssign {
    pub lhs: Spanned<Path>,
    pub array: Spanned<ArrayInit>,
}

#[derive(Debug, CloneWithDelta)]
pub struct ArrayInit {
    pub elements: Vec<Spanned<ArrayElement>>,
}

#[derive(Debug, CloneWithDelta)]
pub enum ArrayElement {
    Braced(Vec<Spanned<Expr>>),
    Expr(Expr),
}

#[derive(Debug, CloneWithDelta)]
pub struct ModMapDecl {
    pub modifier: Spanned<Interned>,
    pub keys: Vec<Spanned<Expr>>,
}

#[derive(Debug, CloneWithDelta)]
pub struct GroupCompatDecl {
    pub group_name: Spanned<Interned>,
    pub group: u32,
    pub val: Spanned<Expr>,
}

#[derive(Debug, CloneWithDelta)]
pub struct LedMapDecl {
    pub name: Spanned<Interned>,
    pub decls: Vec<Spanned<VarDecl>>,
}

#[derive(Debug, CloneWithDelta)]
pub struct LedNameDecl {
    pub virt: bool,
    pub idx_name: Spanned<Interned>,
    pub idx: u32,
    pub val: Spanned<Expr>,
}

#[derive(Debug, CloneWithDelta)]
pub struct ShapeDecl {
    pub name: Spanned<Interned>,
    pub ty: Spanned<ShapeDeclType>,
}

#[derive(Debug, CloneWithDelta)]
pub enum ShapeDeclType {
    OutlineList(Vec<Spanned<Outline>>),
    CoordList(Vec<Spanned<Coord>>),
}

#[derive(Debug, CloneWithDelta)]
pub enum Outline {
    CoordList(Vec<Spanned<Coord>>),
    ExprAssignment(ExprAssignment),
    CoordAssignment(CoordAssignment),
}

#[derive(Debug, CloneWithDelta)]
pub struct ExprAssignment {
    pub name: Spanned<Interned>,
    pub expr: Spanned<Expr>,
}

#[derive(Debug, CloneWithDelta)]
pub struct CoordAssignment {
    pub name: Spanned<Interned>,
    pub coords: Spanned<Vec<Spanned<Coord>>>,
}

#[derive(Debug, CloneWithDelta)]
pub struct Coord {
    pub x: Spanned<Expr>,
    pub y: Spanned<Expr>,
}

#[derive(Debug, CloneWithDelta)]
pub struct SectionDecl {
    pub name: Spanned<Interned>,
    pub items: Vec<Spanned<SectionItem>>,
}

#[derive(Debug, CloneWithDelta)]
pub enum SectionItem {
    Row(RowBody),
    Var(VarDecl),
    Doodad(DoodadDecl),
    LedMap(LedMapDecl),
    Overlay(OverlayDecl),
}

#[derive(Debug, CloneWithDelta)]
pub struct RowBody {
    pub items: Vec<Spanned<RowBodyItem>>,
}

#[derive(Debug, CloneWithDelta)]
pub enum RowBodyItem {
    Keys(Keys),
    Var(VarDecl),
}

#[derive(Debug, CloneWithDelta)]
pub struct Keys {
    pub keys: Vec<Spanned<Key>>,
}

#[derive(Debug, CloneWithDelta)]
pub enum Key {
    Name(Interned),
    Exprs(KeyExprs),
}

#[derive(Debug, CloneWithDelta)]
pub struct KeyExprs {
    pub exprs: Vec<Spanned<Expr>>,
}

#[derive(Debug, CloneWithDelta)]
pub struct OverlayDecl {
    pub name: Spanned<Interned>,
    pub items: Vec<Spanned<OverlayItem>>,
}

#[derive(Debug, CloneWithDelta)]
pub struct OverlayItem {
    pub name: Spanned<Interned>,
    pub alias: Spanned<Interned>,
}

#[derive(Debug, CloneWithDelta)]
pub struct DoodadDecl {
    pub ty: Spanned<DoodadType>,
    pub name: Spanned<Interned>,
    pub decls: Vec<Spanned<VarDecl>>,
}

#[derive(Debug, CloneWithDelta)]
pub struct VModDecl {
    pub defs: Vec<Spanned<VModDef>>,
}

#[derive(Debug, CloneWithDelta)]
pub struct VModDef {
    pub name: Spanned<Interned>,
    pub val: Option<Spanned<Expr>>,
}

#[derive(Debug, CloneWithDelta)]
pub struct Include {
    pub mm: Spanned<MergeMode>,
    pub path: Spanned<Interned>,
    pub resolved: Option<Vec<ResolvedInclude>>,
}

#[derive(Debug, CloneWithDelta)]
pub struct ResolvedInclude {
    pub mm: Option<Spanned<MergeMode>>,
    pub item: Spanned<Item>,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, CloneWithDelta)]
pub enum MergeMode {
    Include,
    Augment,
    Override,
    Replace,
    Alternate,
}

impl MergeMode {
    pub fn merge(self, inner: Self) -> Self {
        if inner == MergeMode::Replace {
            inner
        } else {
            self
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, CloneWithDelta)]
pub enum DoodadType {
    Text,
    Outline,
    Solid,
    Logo,
}

#[derive(Debug, CloneWithDelta)]
pub enum CallArg {
    Expr(Expr),
    NamedParam(NamedParam),
}

#[derive(Debug, CloneWithDelta)]
pub struct NamedParam {
    pub name: Spanned<Path>,
    pub expr: Spanned<Expr>,
}

#[derive(Debug, CloneWithDelta)]
pub enum Expr {
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
}

#[derive(Debug, CloneWithDelta)]
pub struct Call {
    pub path: Spanned<Path>,
    pub args: Vec<Spanned<CallArg>>,
}

#[derive(Debug, CloneWithDelta)]
pub struct Path {
    pub components: Vec<PathComponent>,
}

impl Path {
    pub fn unique_ident(&self) -> Option<Interned> {
        if self.components.len() == 1 && self.components[0].index.is_none() {
            return Some(self.components[0].ident.val);
        }
        None
    }
}

#[derive(Debug, CloneWithDelta)]
pub struct PathComponent {
    pub ident: Spanned<Interned>,
    pub index: Option<PathIndex>,
}

#[derive(Debug, CloneWithDelta)]
pub struct PathIndex {
    pub obracket: Spanned<Token>,
    pub index: Spanned<Expr>,
}
