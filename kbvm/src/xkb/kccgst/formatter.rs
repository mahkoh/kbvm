use {
    crate::xkb::{
        interner::{Interned, Interner},
        kccgst::ast::{
            Call, CallArg, Compat, CompatmapDecl, Component, CompositeMap, ConfigItem,
            ConfigItemType, Coord, Decl, Decls, DirectOrIncluded, DoodadDecl, DoodadType, Expr,
            Flags, Geometry, GeometryDecl, GroupCompatDecl, Include, Included, IndicatorNameDecl,
            InterpretDecl, InterpretMatch, Item, ItemType, Key, KeyAliasDecl, KeyExprs,
            KeyNameDecl, KeySymbolsDecl, KeyTypeDecl, KeycodeDecl, Keycodes, Keys, LedMapDecl,
            MergeMode, ModMapDecl, NamedParam, NestedConfigItem, Outline, OverlayDecl, Path,
            RowBody, RowBodyItem, SectionDecl, SectionItem, ShapeDecl, ShapeDeclType, Symbols,
            SymbolsDecl, Types, TypesDecl, VModDecl, VModDef, Var, VarDecl, VarOrExpr,
        },
    },
    std::io::{self, Write},
};

pub(crate) struct Formatter<'a, W> {
    interner: &'a Interner,
    nesting: usize,
    out: W,
}

pub(crate) trait Format {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write;
}

impl<'a, W> Formatter<'a, W>
where
    W: Write,
{
    pub(crate) fn new(interner: &'a Interner, out: W) -> Self {
        Self {
            interner,
            nesting: 0,
            out,
        }
    }

    fn write_all(&mut self, s: &str) -> io::Result<()> {
        self.out.write_all(s.as_bytes())
    }

    fn write_interned(&mut self, interned: Interned) -> io::Result<()> {
        let s = self.interner.get(interned);
        self.out.write_all(s.as_bytes())?;
        Ok(())
    }

    fn write_keyname(&mut self, interned: Interned) -> io::Result<()> {
        self.write_all("<")?;
        self.write_interned(interned)?;
        self.write_all(">")?;
        Ok(())
    }

    fn write_string(&mut self, interned: Interned) -> io::Result<()> {
        self.write_all("\"")?;
        self.write_interned(interned)?;
        self.write_all("\"")?;
        Ok(())
    }

    fn write_nesting(&mut self) -> io::Result<()> {
        let spaces = self.nesting * 4;
        write!(self.out, "{:spaces$}", "", spaces = spaces)?;
        Ok(())
    }

    fn write_multiline_block<T>(
        &mut self,
        items: &[T],
        mut f: impl FnMut(&mut Self, &T) -> io::Result<()>,
    ) -> io::Result<()> {
        self.write_all("\n")?;
        self.nesting += 1;
        for item in items {
            self.write_nesting()?;
            f(self, item)?;
            self.write_all("\n")?;
        }
        self.nesting -= 1;
        self.write_nesting()?;
        Ok(())
    }

    fn write_inline_list<T>(
        &mut self,
        items: &[T],
        mut f: impl FnMut(&mut Self, &T) -> io::Result<()>,
    ) -> io::Result<()> {
        for (idx, item) in items.iter().enumerate() {
            if idx == 0 {
                self.write_all(" ")?;
            } else {
                self.write_all(", ")?;
            }
            f(self, item)?;
        }
        Ok(())
    }
}

impl Format for Item {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        self.flags.format(f)?;
        self.ty.format(f)
    }
}

impl Format for ItemType {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        match self {
            ItemType::Composite(e) => e.format(f),
            ItemType::Config(e) => e.format(f),
        }
    }
}

impl Format for CompositeMap {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("xkb_keymap ")?;
        if let Some(name) = &self.name {
            f.write_string(name.val)?;
            f.write_all(" ")?;
        }
        f.write_all("{")?;
        f.write_multiline_block(&self.config_items, |f, e| e.val.format(f))?;
        f.write_all("};")?;
        Ok(())
    }
}

impl Format for NestedConfigItem {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        self.flags.format(f)?;
        self.item.format(f)
    }
}

impl Format for Flags {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        for flag in &self.flags {
            f.write_interned(flag.name.val)?;
            f.write_all(" ")?;
        }
        Ok(())
    }
}

impl Format for ConfigItem {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        self.specific.format(f)
    }
}

impl Format for ConfigItemType {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        match self {
            ConfigItemType::Keycodes(e) => e.format(f),
            ConfigItemType::Types(e) => e.format(f),
            ConfigItemType::Compat(e) => e.format(f),
            ConfigItemType::Symbols(e) => e.format(f),
            ConfigItemType::Geometry(e) => e.format(f),
        }
    }
}

impl Format for Keycodes {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("xkb_keycodes ")?;
        if let Some(name) = &self.name {
            f.write_string(name.val)?;
            f.write_all(" ")?;
        }
        self.decls.format(f)?;
        Ok(())
    }
}

impl Format for KeycodeDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        match self {
            KeycodeDecl::Include(e) => e.format(f),
            KeycodeDecl::KeyName(e) => e.format(f),
            KeycodeDecl::KeyAlias(e) => e.format(f),
            KeycodeDecl::Var(e) => e.format(f),
            KeycodeDecl::IndicatorName(e) => e.format(f),
        }
    }
}

impl Format for Types {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("xkb_types ")?;
        if let Some(name) = &self.name {
            f.write_string(name.val)?;
            f.write_all(" ")?;
        }
        self.decls.format(f)?;
        Ok(())
    }
}

impl Format for TypesDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        match self {
            TypesDecl::Include(e) => e.format(f),
            TypesDecl::KeyType(e) => e.format(f),
            TypesDecl::Var(e) => e.format(f),
            TypesDecl::VMod(e) => e.format(f),
        }
    }
}

impl Format for Compat {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("xkb_compatibility ")?;
        if let Some(name) = &self.name {
            f.write_string(name.val)?;
            f.write_all(" ")?;
        }
        self.decls.format(f)?;
        Ok(())
    }
}

impl Format for CompatmapDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        match self {
            CompatmapDecl::Include(e) => e.format(f),
            CompatmapDecl::Interpret(e) => e.format(f),
            CompatmapDecl::GroupCompat(e) => e.format(f),
            CompatmapDecl::IndicatorMap(e) => e.format(f),
            CompatmapDecl::Var(e) => e.format(f),
            CompatmapDecl::VMod(e) => e.format(f),
        }
    }
}

impl Format for Symbols {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("xkb_symbols ")?;
        if let Some(name) = &self.name {
            f.write_string(name.val)?;
            f.write_all(" ")?;
        }
        self.decls.format(f)?;
        Ok(())
    }
}

impl Format for SymbolsDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        match self {
            SymbolsDecl::Include(e) => e.format(f),
            SymbolsDecl::Symbols(e) => e.format(f),
            SymbolsDecl::Var(e) => e.format(f),
            SymbolsDecl::VMod(e) => e.format(f),
            SymbolsDecl::ModMap(e) => e.format(f),
        }
    }
}

impl Format for Geometry {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("xkb_geometry ")?;
        if let Some(name) = &self.name {
            f.write_string(name.val)?;
            f.write_all(" ")?;
        }
        self.decls.format(f)?;
        Ok(())
    }
}

impl Format for GeometryDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        match self {
            GeometryDecl::Include(e) => e.format(f),
            GeometryDecl::KeyAlias(e) => e.format(f),
            GeometryDecl::Var(e) => e.format(f),
            GeometryDecl::Shape(e) => e.format(f),
            GeometryDecl::Section(e) => e.format(f),
            GeometryDecl::LedMap(e) => e.format(f),
            GeometryDecl::Doodad(e) => e.format(f),
        }
    }
}

impl<T> Format for Decls<T>
where
    T: Format,
{
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("{")?;
        f.write_multiline_block(&self.decls, |f, e| e.val.format(f))?;
        f.write_all("};")?;
        Ok(())
    }
}

impl<T> Format for Decl<T>
where
    T: Format,
{
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        if let Some(mm) = self.merge_mode {
            mm.val.format(f)?;
            f.write_all(" ")?;
        }
        self.ty.format(f)
    }
}

impl<T> Format for DirectOrIncluded<T>
where
    T: Format,
{
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        match self {
            DirectOrIncluded::Direct(e) => e.format(f),
            DirectOrIncluded::Included(e) => e.format(f),
        }
    }
}

impl<T> Format for Included<T>
where
    T: Format,
{
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("{")?;
        f.write_multiline_block(&self.components, |f, e| e.format(f))?;
        f.write_all("};")?;
        Ok(())
    }
}

impl<T> Format for Component<T>
where
    T: Format,
{
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        if let Some(mm) = self.mm {
            mm.val.format(f)?;
            f.write_all(" ")?;
        }
        self.decls.format(f)
    }
}

impl Format for InterpretDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("interpret ")?;
        self.match_.val.format(f)?;
        f.write_all(" {")?;
        f.write_multiline_block(&self.vars, |f, e| e.val.format(f))?;
        f.write_all("};")?;
        Ok(())
    }
}

impl Format for InterpretMatch {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_interned(self.sym.val)?;
        if let Some(filter) = &self.filter {
            f.write_all(" + ")?;
            filter.val.format(f)?;
        }
        Ok(())
    }
}

impl Format for KeyNameDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_keyname(self.name.val)?;
        f.write_all(" = ")?;
        f.write_interned(self.code_str.val)?;
        f.write_all(";")?;
        Ok(())
    }
}

impl Format for KeyAliasDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("alias ")?;
        f.write_keyname(self.name.val)?;
        f.write_all(" = ")?;
        f.write_keyname(self.alias_for.val)?;
        f.write_all(";")?;
        Ok(())
    }
}

impl Format for KeyTypeDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("type ")?;
        f.write_string(self.name.val)?;
        f.write_all(" {")?;
        f.write_multiline_block(&self.decls, |f, e| e.val.format(f))?;
        f.write_all("};")?;
        Ok(())
    }
}

impl Format for KeySymbolsDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("key ")?;
        f.write_keyname(self.key.val)?;
        f.write_all(" {")?;
        if self.vars.len() < 2 {
            f.write_inline_list(&self.vars, |f, e| e.val.format(f))?;
            f.write_all(" };")?;
        } else {
            f.nesting += 1;
            for (idx, var) in self.vars.iter().enumerate() {
                if idx > 0 {
                    f.write_all(",\n")?;
                } else {
                    f.write_all("\n")?;
                }
                f.write_nesting()?;
                var.val.format(f)?;
            }
            f.nesting -= 1;
            f.write_all("\n")?;
            f.write_nesting()?;
            f.write_all("};")?;
        }
        Ok(())
    }
}

impl Format for VarOrExpr {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        match self {
            VarOrExpr::Var(e) => e.format(f),
            VarOrExpr::Expr(e) => e.format(f),
        }
    }
}

impl Format for ModMapDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("modifier_map ")?;
        f.write_interned(self.modifier.val)?;
        f.write_all(" {")?;
        f.write_inline_list(&self.keys, |f, e| e.val.format(f))?;
        f.write_all(" };")?;
        Ok(())
    }
}

impl Format for GroupCompatDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("group ")?;
        f.write_interned(self.group_name.val)?;
        f.write_all(" = ")?;
        self.val.val.format(f)?;
        f.write_all(";")?;
        Ok(())
    }
}

impl Format for IndicatorNameDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        if self.virt.is_some() {
            f.write_all("virtual ")?;
        }
        f.write_all("indicator ")?;
        f.write_interned(self.idx_name.val)?;
        f.write_all(" = ")?;
        self.val.val.format(f)?;
        f.write_all(";")?;
        Ok(())
    }
}

impl Format for ShapeDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("shape ")?;
        f.write_string(self.name.val)?;
        f.write_all(" {")?;
        match &self.ty.val {
            ShapeDeclType::OutlineList(e) => {
                f.write_inline_list(e, |f, e| e.val.format(f))?;
            }
            ShapeDeclType::CoordList(e) => {
                f.write_inline_list(e, |f, e| e.val.format(f))?;
            }
        }
        f.write_all(" };")?;
        Ok(())
    }
}

impl Format for Outline {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        match self {
            Outline::CoordList(e) => {
                f.write_all("{")?;
                f.write_inline_list(e, |f, e| e.val.format(f))?;
                f.write_all(" }")?;
                Ok(())
            }
            Outline::ExprAssignment(e) => {
                f.write_interned(e.name.val)?;
                f.write_all(" = ")?;
                e.expr.val.format(f)?;
                Ok(())
            }
            Outline::CoordAssignment(e) => {
                f.write_interned(e.name.val)?;
                f.write_all(" = ")?;
                f.write_all("{")?;
                f.write_inline_list(&e.coords.val, |f, e| e.val.format(f))?;
                f.write_all(" }")?;
                Ok(())
            }
        }
    }
}

impl Format for Coord {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("[")?;
        self.x.val.format(f)?;
        f.write_all(", ")?;
        self.y.val.format(f)?;
        f.write_all("]")?;
        Ok(())
    }
}

impl Format for SectionDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("section ")?;
        f.write_string(self.name.val)?;
        f.write_all(" {")?;
        f.write_multiline_block(&self.items, |f, i| i.val.format(f))?;
        f.write_all("};")?;
        Ok(())
    }
}

impl Format for SectionItem {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        match self {
            SectionItem::Row(e) => e.format(f),
            SectionItem::Var(e) => e.format(f),
            SectionItem::Doodad(e) => e.format(f),
            SectionItem::LedMap(e) => e.format(f),
            SectionItem::Overlay(e) => e.format(f),
        }
    }
}

impl Format for LedMapDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("indicator ")?;
        f.write_string(self.name.val)?;
        f.write_all(" {")?;
        f.write_multiline_block(&self.decls, |f, i| i.val.format(f))?;
        f.write_all("};")?;
        Ok(())
    }
}

impl Format for RowBody {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("row {")?;
        f.write_multiline_block(&self.items, |f, i| i.val.format(f))?;
        f.write_all("};")?;
        Ok(())
    }
}

impl Format for RowBodyItem {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        match self {
            RowBodyItem::Keys(k) => k.format(f),
            RowBodyItem::Var(v) => v.format(f),
        }
    }
}

impl Format for VarDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        self.var.format(f)?;
        f.write_all(";")?;
        Ok(())
    }
}

impl Format for Var {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        if self.not.is_some() {
            f.write_all("!")?;
        }
        self.path.val.format(f)?;
        if let Some(e) = &self.expr {
            f.write_all(" = ")?;
            e.val.format(f)?;
        }
        Ok(())
    }
}

impl Format for Keys {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("keys {")?;
        f.write_inline_list(&self.keys, |f, k| k.val.format(f))?;
        f.write_all(" };")?;
        Ok(())
    }
}

impl Format for Key {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        match self {
            Key::Name(n) => f.write_keyname(*n),
            Key::Exprs(e) => e.format(f),
        }
    }
}

impl Format for KeyExprs {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("{")?;
        f.write_inline_list(&self.exprs, |f, e| e.val.format(f))?;
        f.write_all(" }")?;
        Ok(())
    }
}

impl Format for OverlayDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("overlay ")?;
        f.write_string(self.name.val)?;
        f.write_all(" {")?;
        f.write_inline_list(&self.items, |f, i| {
            f.write_keyname(i.val.name.val)?;
            f.write_all(" = ")?;
            f.write_keyname(i.val.alias.val)?;
            Ok(())
        })?;
        f.write_all(" };")?;
        Ok(())
    }
}

impl Format for DoodadDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        self.ty.val.format(f)?;
        f.write_all(" ")?;
        f.write_string(self.name.val)?;
        f.write_all(" {")?;
        f.write_multiline_block(&self.decls, |f, decl| decl.val.format(f))?;
        f.write_all("};")?;
        Ok(())
    }
}

impl Format for VModDecl {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_all("virtual_modifiers")?;
        for (idx, def) in self.defs.iter().enumerate() {
            if idx == 0 {
                f.write_all(" ")?
            } else {
                f.write_all(", ")?
            }
            def.val.format(f)?;
        }
        f.write_all(";")?;
        Ok(())
    }
}

impl Format for VModDef {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_interned(self.name.val)?;
        if let Some(val) = &self.val {
            f.write_all(" = ")?;
            val.val.format(f)?;
        }
        Ok(())
    }
}

impl Format for Include {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        f.write_string(self.path.val)
    }
}

impl Format for MergeMode {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        let v = match self {
            MergeMode::Include => "include",
            MergeMode::Augment => "augment",
            MergeMode::Override => "override",
            MergeMode::Replace => "replace",
            MergeMode::Alternate => "alternate",
        };
        f.out.write_all(v.as_bytes())
    }
}

impl Format for DoodadType {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        let v = match self {
            DoodadType::Text => "text",
            DoodadType::Outline => "outline",
            DoodadType::Solid => "solid",
            DoodadType::Logo => "logo",
        };
        f.out.write_all(v.as_bytes())
    }
}

impl Format for Path {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        for (idx, component) in self.components.iter().enumerate() {
            if idx > 0 {
                f.write_all(".")?;
            }
            f.write_interned(component.ident.val)?;
            if let Some(idx) = &component.index {
                f.write_all("[")?;
                idx.index.val.format(f)?;
                f.write_all("]")?;
            }
        }
        Ok(())
    }
}

impl Format for Call {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        self.path.val.format(f)?;
        f.write_all("(")?;
        for (idx, arg) in self.args.iter().enumerate() {
            if idx > 0 {
                f.write_all(", ")?;
            }
            arg.val.format(f)?;
        }
        f.write_all(")")?;
        Ok(())
    }
}

impl Format for CallArg {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        match self {
            CallArg::Expr(e) => e.format(f),
            CallArg::NamedParam(e) => e.format(f),
        }
    }
}

impl Format for NamedParam {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        self.name.val.format(f)?;
        f.write_all(" = ")?;
        self.expr.val.format(f)?;
        Ok(())
    }
}

impl Format for Expr {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        match self {
            Expr::UnMinus(e) => {
                f.write_all("-")?;
                e.val.format(f)?;
            }
            Expr::UnPlus(e) => {
                f.write_all("+")?;
                e.val.format(f)?;
            }
            Expr::UnNot(e) => {
                f.write_all("!")?;
                e.val.format(f)?;
            }
            Expr::UnInverse(e) => {
                f.write_all("~")?;
                e.val.format(f)?;
            }
            Expr::Path(p) => {
                p.format(f)?;
            }
            Expr::Call(c) => {
                c.format(f)?;
            }
            Expr::String(s) => {
                f.write_all("\"")?;
                f.write_interned(*s)?;
                f.write_all("\"")?;
            }
            Expr::Integer(i, _) => {
                f.write_interned(*i)?;
            }
            Expr::Float(i, _) => {
                f.write_interned(*i)?;
            }
            Expr::KeyName(i) => {
                f.write_all("<")?;
                f.write_interned(*i)?;
                f.write_all(">")?;
            }
            Expr::Parenthesized(e) => {
                f.write_all("(")?;
                e.val.format(f)?;
                f.write_all(")")?;
            }
            Expr::Mul(l, r) => {
                l.val.format(f)?;
                f.write_all(" * ")?;
                r.val.format(f)?;
            }
            Expr::Div(l, r) => {
                l.val.format(f)?;
                f.write_all(" / ")?;
                r.val.format(f)?;
            }
            Expr::Add(l, r) => {
                l.val.format(f)?;
                f.write_all(" + ")?;
                r.val.format(f)?;
            }
            Expr::Sub(l, r) => {
                l.val.format(f)?;
                f.write_all(" - ")?;
                r.val.format(f)?;
            }
            Expr::BracketList(e) => {
                f.write_all("[")?;
                f.write_inline_list(e, |f, e| e.val.format(f))?;
                f.write_all(" ]")?;
            }
            Expr::BraceList(e) => {
                f.write_all("{")?;
                f.write_inline_list(e, |f, e| e.val.format(f))?;
                f.write_all(" }")?;
            }
        }
        Ok(())
    }
}
