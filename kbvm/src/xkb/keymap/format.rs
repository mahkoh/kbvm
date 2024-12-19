use {
    super::*,
    debug_fn::debug_fn,
    isnt::std_1::vec::IsntVecExt,
    std::{
        fmt,
        fmt::{Display, Formatter},
    },
};

impl Display for Keymap {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let mut writer = Writer {
            nesting: 0,
            alternate: f.alternate(),
            newline: match f.alternate() {
                true => "\n",
                false => " ",
            },
            f,
        };
        self.format(&mut writer)
    }
}

struct Writer<'a, 'b> {
    nesting: usize,
    alternate: bool,
    newline: &'static str,
    f: &'a mut Formatter<'b>,
}

trait Format {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result;
}

impl<'a, 'b> Writer<'a, 'b> {
    fn write(&mut self, s: &str) -> fmt::Result {
        self.f.write_str(s)
    }

    fn write_newline(&mut self) -> fmt::Result {
        self.f.write_str(self.newline)
    }

    fn write_string(&mut self, s: &str) -> fmt::Result {
        self.write("\"")?;
        for c in s.chars() {
            if c == '\\' {
                self.write(r"\\")?;
            } else if (c as u32) < 0x20 || c == '\x7f' {
                write!(self.f, r"\{:03o}", c as u32)?;
            } else {
                self.f.write_char(c)?;
            }
        }
        self.write("\"")?;
        Ok(())
    }

    fn write_nesting(&mut self) -> fmt::Result {
        if self.alternate {
            let spaces = self.nesting * 4;
            write!(self.f, "{:spaces$}", "", spaces = spaces)?;
        }
        Ok(())
    }

    fn write_nested(&mut self, f: impl FnOnce(&mut Self) -> fmt::Result) -> fmt::Result {
        self.write_newline()?;
        self.nesting += 1;
        f(self)?;
        self.nesting -= 1;
        Ok(())
    }

    fn write_inline_list<T>(
        &mut self,
        items: &[T],
        mut f: impl FnMut(&mut Self, &T) -> fmt::Result,
    ) -> fmt::Result {
        for (idx, item) in items.iter().enumerate() {
            if idx == 0 {
                self.write(" ")?;
            } else {
                self.write(", ")?;
            }
            f(self, item)?;
        }
        Ok(())
    }
}

impl Format for Keymap {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        f.write_nesting()?;
        f.write("xkb_keymap ")?;
        if let Some(name) = &self.name {
            f.write_string(name)?;
            f.write(" ")?;
        }
        f.write("{")?;
        f.write_nested(|f| {
            Keycodes(self).format(f)?;
            f.write_newline()?;
            f.write_newline()?;
            Types(self).format(f)?;
            f.write_newline()?;
            f.write_newline()?;
            Compat(self).format(f)?;
            f.write_newline()?;
            f.write_newline()?;
            Symbols(self).format(f)?;
            f.write_newline()?;
            Ok(())
        })?;
        f.write_nesting()?;
        f.write("};")?;
        Ok(())
    }
}

struct Keycodes<'a>(&'a Keymap);

impl Format for Keycodes<'_> {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        let m = self.0;
        f.write_nesting()?;
        f.write("xkb_keycodes {")?;
        f.write_nested(|f| {
            f.write_nesting()?;
            write!(f.f, "minimum = {};", m.min_keycode)?;
            f.write_newline()?;
            f.write_nesting()?;
            write!(f.f, "maximum = {};", m.max_keycode)?;
            f.write_newline()?;
            f.write_newline()?;
            for i in &m.indicators {
                KeycodeIndicator(i).format(f)?;
                f.write_newline()?;
            }
            if m.keycodes.is_not_empty() {
                f.write_newline()?;
                KeycodeKeys(m).format(f)?;
            }
            Ok(())
        })?;
        f.write_nesting()?;
        f.write("};")?;
        Ok(())
    }
}

struct Compat<'a>(&'a Keymap);

impl Format for Compat<'_> {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        let m = self.0;
        f.write_nesting()?;
        f.write("xkb_compat {")?;
        f.write_nested(|f| {
            let mut wrote_any_indicators = false;
            for i in &m.indicators {
                let write = i.modifier_mask.0 != 0
                    || i.group_mask.0 != 0
                    || i.controls != ControlMask::NONE
                    || i.mod_components != ModComponentMask::NONE
                    || i.group_components != GroupComponentMask::NONE;
                if write {
                    wrote_any_indicators = true;
                    CompatIndicator(i).format(f)?;
                    f.write_newline()?;
                }
            }
            if wrote_any_indicators {
                f.write_newline()?;
            }
            // https://gitlab.freedesktop.org/xorg/lib/libxkbfile/-/issues/12
            f.write_nesting()?;
            f.write("interpret VoidSymbol {")?;
            f.write_nested(|f| {
                f.write_nesting()?;
                f.write("repeat = false;")?;
                f.write_newline()
            })?;
            f.write_nesting()?;
            f.write("};")?;
            f.write_newline()?;
            Ok(())
        })?;
        f.write_nesting()?;
        f.write("};")?;
        Ok(())
    }
}

struct KeycodeIndicator<'a>(&'a Indicator);

impl Format for KeycodeIndicator<'_> {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        let i = self.0;
        f.write_nesting()?;
        if i.virt {
            f.write("virtual ")?;
        }
        f.write("indicator ")?;
        write!(f.f, "{}", i.index.raw())?;
        f.write(" = ")?;
        f.write_string(&i.name)?;
        f.write(";")?;
        Ok(())
    }
}

struct CompatIndicator<'a>(&'a Indicator);

impl Format for CompatIndicator<'_> {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        let i = self.0;
        f.write_nesting()?;
        f.write("indicator ")?;
        f.write_string(&i.name)?;
        f.write(" {")?;
        f.write_nested(|f| {
            if i.modifier_mask.0 != 0 {
                f.write_nesting()?;
                write!(f.f, "modifiers = {};", modifier_mask(i.modifier_mask))?;
                f.write_newline()?;
                if i.mod_components != ModComponentMask::EFFECTIVE {
                    f.write_nesting()?;
                    write!(f.f, "whichModState = {};", i.mod_components)?;
                    f.write_newline()?;
                }
            }
            if i.group_mask.0 != 0 {
                f.write_nesting()?;
                write!(f.f, "groups = 0x{:08x};", i.group_mask.0)?;
                f.write_newline()?;
                if i.group_components != GroupComponentMask::EFFECTIVE {
                    f.write_nesting()?;
                    write!(f.f, "whichGroupState = {};", i.group_components)?;
                    f.write_newline()?;
                }
            }
            if i.controls != ControlMask::NONE {
                f.write_nesting()?;
                write!(f.f, "controls = {};", i.controls)?;
                f.write_newline()?;
            }
            Ok(())
        })?;
        f.write_nesting()?;
        f.write("};")?;
        Ok(())
    }
}

struct KeycodeKeys<'a>(&'a Keymap);

impl Format for KeycodeKeys<'_> {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        for k in &self.0.keycodes {
            f.write_nesting()?;
            write!(f.f, "<{}> = {};", k.name, k.keycode.0)?;
            f.write_newline()?;
        }
        Ok(())
    }
}

struct Types<'a>(&'a Keymap);

impl Format for Types<'_> {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        let m = self.0;
        f.write_nesting()?;
        f.write("xkb_types {")?;
        f.write_nested(|f| {
            VirtualModifiers(m).format(f)?;
            for i in &m.types {
                f.write_newline()?;
                TypesKeyType(i).format(f)?;
                f.write_newline()?;
            }
            Ok(())
        })?;
        f.write_nesting()?;
        f.write("};")?;
        Ok(())
    }
}

struct TypesKeyType<'a>(&'a KeyType);

impl Format for TypesKeyType<'_> {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        let m = self.0;
        f.write_nesting()?;
        f.write("type ")?;
        f.write_string(&m.name)?;
        f.write(" {")?;
        f.write_nested(|f| {
            if m.modifiers.0 != 0 {
                f.write_nesting()?;
                write!(f.f, "modifiers = {};", modifier_mask(m.modifiers))?;
                f.write_newline()?;
            }
            for (l, n) in &m.level_names {
                f.write_nesting()?;
                write!(f.f, "level_name[Level{}] = ", l.raw())?;
                f.write_string(n)?;
                f.write(";")?;
                f.write_newline()?;
            }
            for level in &m.mappings {
                f.write_nesting()?;
                let mask = modifier_mask(level.modifiers);
                write!(f.f, "map[{}] = Level{};", mask, level.level.raw())?;
                f.write_newline()?;
                if level.preserved.0 != 0 {
                    f.write_nesting()?;
                    write!(
                        f.f,
                        "preserve[{}] = {};",
                        mask,
                        modifier_mask(level.preserved)
                    )?;
                    f.write_newline()?;
                }
            }
            Ok(())
        })?;
        f.write_nesting()?;
        f.write("};")?;
        Ok(())
    }
}

struct VirtualModifiers<'a>(&'a Keymap);

impl Format for VirtualModifiers<'_> {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        let m = self.0;
        for v in &m.virtual_modifiers {
            f.write_nesting()?;
            f.write("virtual_modifiers ")?;
            f.write(&v.name)?;
            if v.values.0 != 0 {
                f.write(" = ")?;
                write!(f.f, "{}", modifier_mask(v.values))?;
            }
            f.write(";")?;
            f.write_newline()?;
        }
        Ok(())
    }
}

impl Format for Action {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        match self {
            Action::ModsSet(e) => e.format(f),
            Action::ModsLatch(e) => e.format(f),
            Action::ModsLock(e) => e.format(f),
            Action::GroupSet(e) => e.format(f),
            Action::GroupLatch(e) => e.format(f),
            Action::GroupLock(e) => e.format(f),
        }
    }
}

impl Format for ModsSet {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        f.write("SetMods(")?;
        write!(f.f, "modifiers = {}", modifier_mask(self.modifiers))?;
        if self.clear_locks {
            f.write(", clearLocks")?;
        }
        f.write(")")?;
        Ok(())
    }
}

impl Format for ModsLatch {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        f.write("LatchMods(")?;
        write!(f.f, "modifiers = {}", modifier_mask(self.modifiers))?;
        if self.clear_locks {
            f.write_string(", clearLocks")?;
        }
        if self.latch_to_lock {
            f.write_string(", latchToLock")?;
        }
        f.write(")")?;
        Ok(())
    }
}

impl Format for ModsLock {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        f.write("LockMods(")?;
        write!(f.f, "modifiers = {}", modifier_mask(self.modifiers))?;
        'affect: {
            let affect = match (self.lock, self.unlock) {
                (false, false) => "neither",
                (true, false) => "lock",
                (false, true) => "unlock",
                _ => break 'affect,
            };
            write!(f.f, ", affect={}", affect)?;
        }
        f.write(")")?;
        Ok(())
    }
}

impl Format for GroupSet {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        f.write("SetGroup(")?;
        write!(f.f, "group = {}", group_change(self.group))?;
        if self.clear_locks {
            f.write_string(", clearLocks")?;
        }
        f.write(")")?;
        Ok(())
    }
}

impl Format for GroupLatch {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        f.write("LatchGroup(")?;
        write!(f.f, "group = {}", group_change(self.group))?;
        if self.clear_locks {
            f.write_string(", clearLocks")?;
        }
        if self.latch_to_lock {
            f.write_string(", latchToLock")?;
        }
        f.write(")")?;
        Ok(())
    }
}

impl Format for GroupLock {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        f.write("LockGroup(")?;
        write!(f.f, "group = {}", group_change(self.group))?;
        f.write(")")?;
        Ok(())
    }
}

struct Symbols<'a>(&'a Keymap);

impl Format for Symbols<'_> {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        let m = self.0;
        f.write_nesting()?;
        f.write("xkb_symbols {")?;
        f.write_nested(|f| {
            let mut need_newline = false;
            if m.mod_maps.is_not_empty() {
                ModMaps(m).format(f)?;
                need_newline = true;
            }
            if m.group_names.is_not_empty() {
                if need_newline {
                    f.write_newline()?;
                }
                GroupNames(m).format(f)?;
                need_newline = true;
            }
            if m.keys.len() > 0 {
                if need_newline {
                    f.write_newline()?;
                }
                Keys(m).format(f)?;
            }
            Ok(())
        })?;
        f.write_nesting()?;
        f.write("};")?;
        Ok(())
    }
}

struct Keys<'a>(&'a Keymap);

impl Format for Keys<'_> {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        let m = self.0;
        f.write_nesting()?;
        f.write("key.repeat = true;")?;
        f.write_newline()?;
        f.write_newline()?;
        for key in m.keys.values() {
            f.write_nesting()?;
            f.write("key <")?;
            f.write(&key.key_name)?;
            f.write("> {")?;
            f.write_nested(|f| {
                let mut needs_newline = false;
                fn handle_newline(needs_newline: &mut bool, f: &mut Writer<'_, '_>) -> fmt::Result {
                    if *needs_newline {
                        f.write(",")?;
                        f.write_newline()?;
                    }
                    *needs_newline = false;
                    Ok(())
                }
                if !key.repeat {
                    f.write_nesting()?;
                    f.write("repeat = false")?;
                    needs_newline = true;
                }
                if key.redirect != GroupsRedirect::Wrap {
                    handle_newline(&mut needs_newline, f)?;
                    f.write_nesting()?;
                    match key.redirect {
                        GroupsRedirect::Wrap => {}
                        GroupsRedirect::Clamp => {
                            f.write("groupsClamp = true")?;
                        }
                        GroupsRedirect::Redirect(n) => {
                            write!(f.f, "groupsRedirect = Group{}", n.raw())?;
                        }
                    }
                    needs_newline = true;
                }
                for (offset, group) in key.groups.iter().enumerate() {
                    if let Some(group) = group {
                        if group.levels.len() == 0 {
                            continue;
                        }
                        let idx = offset + 1;
                        handle_newline(&mut needs_newline, f)?;
                        f.write_nesting()?;
                        write!(f.f, "type[Group{idx}] = ")?;
                        f.write_string(&group.key_type.name)?;
                        needs_newline = true;
                    }
                }
                for (offset, group) in key.groups.iter().enumerate() {
                    if let Some(group) = group {
                        let idx = offset + 1;
                        fn write_levels<T: Format>(
                            needs_newline: &mut bool,
                            idx: usize,
                            group: &SymbolGroup,
                            f: &mut Writer<'_, '_>,
                            name: &str,
                            absent: &str,
                            field: impl Fn(&SymbolLevel) -> &SmallVec<[T; 1]>,
                        ) -> fmt::Result {
                            if group.levels.iter().all(|l| field(l).is_empty()) {
                                return Ok(());
                            }
                            handle_newline(needs_newline, f)?;
                            f.write_nesting()?;
                            write!(f.f, "{name}[Group{idx}] = [")?;
                            f.write_inline_list(&group.levels, |f, l| {
                                let list = field(l);
                                if list.is_empty() {
                                    f.write(absent)?;
                                } else if list.len() == 1 {
                                    list[0].format(f)?;
                                } else {
                                    f.write("{")?;
                                    f.write_inline_list(list, |f, a| a.format(f))?;
                                    f.write(" }")?;
                                }
                                Ok(())
                            })?;
                            f.write(" ]")?;
                            *needs_newline = true;
                            Ok(())
                        }
                        write_levels(
                            &mut needs_newline,
                            idx,
                            group,
                            f,
                            "symbols",
                            "NoSymbol",
                            |l| &l.symbols,
                        )?;
                        write_levels(
                            &mut needs_newline,
                            idx,
                            group,
                            f,
                            "actions",
                            "NoAction()",
                            |l| &l.actions,
                        )?;
                    }
                }
                if !needs_newline {
                    f.write_nesting()?;
                    f.write("repeat = true")?;
                }
                f.write_newline()?;
                Ok(())
            })?;
            f.write_nesting()?;
            f.write("};")?;
            f.write_newline()?;
        }
        Ok(())
    }
}

impl Format for Keysym {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        if let Some(name) = self.name() {
            f.write(name)
        } else if self.is_in_unicode_range() {
            let d = self.0 & 0xff_ff_ff;
            write!(f.f, "U{d:x}")
        } else {
            write!(f.f, "0x{:08x}", self.0)
        }
    }
}

struct ModMaps<'a>(&'a Keymap);

impl Format for ModMaps<'_> {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        for (idx, kc) in &self.0.mod_maps {
            f.write_nesting()?;
            write!(f.f, "modmap {} {{ ", modifier_mask(idx.to_mask()))?;
            if let Some(s) = kc.key_sym {
                s.format(f)?;
            } else {
                write!(f.f, "<{}>", kc.key_name)?;
            }
            f.write(" };")?;
            if kc.key_sym.is_some() && f.alternate {
                write!(f.f, " // <{}>", kc.key_name)?;
            }
            f.write_newline()?;
        }
        Ok(())
    }
}

struct GroupNames<'a>(&'a Keymap);

impl Format for GroupNames<'_> {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        for (idx, kc) in &self.0.group_names {
            f.write_nesting()?;
            write!(f.f, "name[Group{}] = ", idx.raw())?;
            f.write_string(kc)?;
            f.write(";")?;
            f.write_newline()?;
        }
        Ok(())
    }
}

fn group_change(gc: GroupChange) -> impl Display {
    debug_fn(move |f| match gc {
        GroupChange::Absolute(g) => {
            write!(f, "Group{}", g.raw())
        }
        GroupChange::Rel(r) => {
            if r > 0 {
                f.write_str("+")?;
            }
            Display::fmt(&r, f)
        }
    })
}

fn modifier_mask(mm: ModifierMask) -> impl Display {
    debug_fn(move |f| {
        if mm.0 == 0 {
            f.write_str("None")
        } else if mm.0 & !0xff != 0 {
            write!(f, "0x{:08x}", mm.0)
        } else {
            let mut first = true;
            macro_rules! one {
                ($n:expr, $name:expr) => {
                    if mm.0 & (1 << $n) != 0 {
                        if !first {
                            f.write_str("+")?;
                        }
                        first = false;
                        f.write_str($name)?;
                    }
                };
            }
            one!(0, "Shift");
            one!(1, "Lock");
            one!(2, "Control");
            one!(3, "Mod1");
            one!(4, "Mod2");
            one!(5, "Mod3");
            one!(6, "Mod4");
            one!(7, "Mod5");
            let _ = first;
            Ok(())
        }
    })
}
