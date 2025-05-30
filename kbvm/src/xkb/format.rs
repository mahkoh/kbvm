use {
    crate::{
        xkb::{
            controls::ControlMask,
            group::GroupChange,
            group_component::GroupComponent,
            keymap::{
                self,
                actions::{
                    ControlsLockAction, ControlsSetAction, GroupLatchAction, GroupLockAction,
                    GroupSetAction, ModsLatchAction, ModsLockAction, ModsSetAction,
                    RedirectKeyAction,
                },
                Action, Indicator, KeyBehavior, KeyGroup, KeyLevel, KeyOverlay, KeyType,
            },
            mod_component::ModComponentMask,
            resolved::GroupsRedirect,
            rmlvo::{self, MergeMode},
            Keymap,
        },
        Keysym, ModifierMask,
    },
    debug_fn::debug_fn,
    hashbrown::{HashMap, HashSet},
    isnt::std_1::vec::IsntVecExt,
    smallvec::SmallVec,
    std::{
        fmt::{self, Display, Formatter, Write},
        sync::Arc,
    },
};

pub(crate) struct FormatFormat<'a, T>(pub(crate) &'a T);

impl<T> Display for FormatFormat<'_, T>
where
    T: Format,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let mut writer = Writer {
            nesting: 0,
            multi_line: f.alternate(),
            lookup_only: false,
            omit_multiple_actions: false,
            long_keys: None,
            newline: match f.alternate() {
                true => "\n",
                false => " ",
            },
            f,
        };
        self.0.format(&mut writer)
    }
}

struct Writer<'a, 'b> {
    nesting: usize,
    multi_line: bool,
    lookup_only: bool,
    omit_multiple_actions: bool,
    long_keys: Option<HashMap<Arc<String>, String>>,
    newline: &'static str,
    f: &'a mut Formatter<'b>,
}

trait Format {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result;
}

impl Writer<'_, '_> {
    fn write(&mut self, s: &str) -> fmt::Result {
        self.f.write_str(s)
    }

    fn write_newline(&mut self) -> fmt::Result {
        self.f.write_str(self.newline)
    }

    fn write_string(&mut self, s: &str) -> fmt::Result {
        self.write("\"")?;
        for c in s.chars() {
            match c {
                '\\' => self.write(r"\\")?,
                '\n' => self.write(r"\n")?,
                '\r' => self.write(r"\r")?,
                '\t' => self.write(r"\t")?,
                _ if (c as u32) < 0x20 || c == '\x7f' || c == '"' => {
                    write!(self.f, r"\{:03o}", c as u32)?
                }
                _ => self.f.write_char(c)?,
            }
        }
        self.write("\"")?;
        Ok(())
    }

    fn write_key_name(&mut self, name: &Arc<String>) -> fmt::Result {
        if name.len() > 4 {
            if let Some(names) = &mut self.long_keys {
                if let Some(name) = names.get(name) {
                    return self.f.write_str(name);
                }
            }
        }
        self.write(name)
    }

    fn write_nesting(&mut self) -> fmt::Result {
        if self.multi_line {
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

impl Display for keymap::Formatter<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let mut long_keys = None;
        if self.rename_long_keys {
            let mut has_any = false;
            let mut conflicts = HashSet::new();
            for key in &self.keymap.keycodes {
                if key.name.len() > 4 {
                    has_any = true;
                }
                if key.name.len() == 4
                    && key.name.as_bytes()[0] == b'K'
                    && key.name.as_bytes()[1..].iter().all(|b| b.is_ascii_digit())
                {
                    conflicts.insert(key.name.clone());
                }
            }
            let mut names = HashMap::new();
            if has_any {
                let mut next = 0u64;
                for key in &self.keymap.keycodes {
                    if key.name.len() <= 4 {
                        continue;
                    }
                    let name = loop {
                        let name = format!("K{:03}", next);
                        next += 1;
                        if !conflicts.contains(&name) {
                            break name;
                        }
                    };
                    names.insert(key.name.clone(), name);
                }
            }
            if !names.is_empty() {
                long_keys = Some(names);
            }
        }
        let mut writer = Writer {
            nesting: 0,
            multi_line: !self.single_line,
            lookup_only: self.lookup_only,
            omit_multiple_actions: !self.multiple_actions_per_level,
            long_keys,
            newline: match self.single_line {
                false => "\n",
                true => " ",
            },
            f,
        };
        Format::format(self.keymap, &mut writer)
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
            f.write("minimum = 8;")?;
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
                    || i.group_component != GroupComponent::Effective
                    || i.controls != ControlMask::NONE;
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
            }
            if i.group_component != GroupComponent::Effective {
                f.write_nesting()?;
                write!(f.f, "whichGroupState = {};", i.group_component)?;
                f.write_newline()?;
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
            f.write("<")?;
            f.write_key_name(&k.name)?;
            write!(f.f, "> = {};", k.keycode.0)?;
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
            f.write_nesting()?;
            write!(f.f, "modifiers = {};", modifier_mask(m.modifiers))?;
            f.write_newline()?;
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
            Action::RedirectKey(e) => e.format(f),
            Action::ControlsSet(e) => e.format(f),
            Action::ControlsLock(e) => e.format(f),
        }
    }
}

impl Format for ModsSetAction {
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

impl Format for ModsLatchAction {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        f.write("LatchMods(")?;
        write!(f.f, "modifiers = {}", modifier_mask(self.modifiers))?;
        if self.clear_locks {
            f.write(", clearLocks")?;
        }
        if self.latch_to_lock {
            f.write(", latchToLock")?;
        }
        f.write(")")?;
        Ok(())
    }
}

impl Format for ModsLockAction {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        write!(
            f.f,
            "LockMods(modifiers = {}{})",
            modifier_mask(self.modifiers),
            affect(self.lock, self.unlock),
        )
    }
}

impl Format for GroupSetAction {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        f.write("SetGroup(")?;
        write!(f.f, "group = {}", group_change(self.group))?;
        if self.clear_locks {
            f.write(", clearLocks")?;
        }
        f.write(")")?;
        Ok(())
    }
}

impl Format for GroupLatchAction {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        f.write("LatchGroup(")?;
        write!(f.f, "group = {}", group_change(self.group))?;
        if self.clear_locks {
            f.write(", clearLocks")?;
        }
        if self.latch_to_lock {
            f.write(", latchToLock")?;
        }
        f.write(")")?;
        Ok(())
    }
}

impl Format for GroupLockAction {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        f.write("LockGroup(")?;
        write!(f.f, "group = {}", group_change(self.group))?;
        f.write(")")?;
        Ok(())
    }
}

impl Format for RedirectKeyAction {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        f.write("RedirectKey(")?;
        f.write("key = <")?;
        f.write_key_name(&self.key_name)?;
        f.write(">")?;
        if self.mods_to_clear.0 != 0 {
            write!(f.f, ", clearMods = {}", modifier_mask(self.mods_to_clear))?;
        }
        if self.mods_to_set.0 != 0 {
            write!(f.f, ", mods = {}", modifier_mask(self.mods_to_set))?;
        }
        f.write(")")?;
        Ok(())
    }
}

impl Format for ControlsSetAction {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        write!(f.f, "SetControls(controls = {})", self.controls)?;
        Ok(())
    }
}

impl Format for ControlsLockAction {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        write!(
            f.f,
            "LockControls(controls = {}{})",
            self.controls,
            affect(self.lock, self.unlock),
        )
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
            f.write_key_name(&key.key_name)?;
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
                if !f.lookup_only {
                    if let Some(behavior) = &key.behavior {
                        handle_newline(&mut needs_newline, f)?;
                        f.write_nesting()?;
                        match behavior {
                            KeyBehavior::Lock => {
                                f.write("locks = true")?;
                            }
                            KeyBehavior::Overlay(b) => {
                                let idx = match b.overlay() {
                                    KeyOverlay::Overlay1 => 1,
                                    KeyOverlay::Overlay2 => 2,
                                };
                                write!(f.f, "overlay{idx} = <")?;
                                f.write_key_name(&b.key_name)?;
                                f.write(">")?;
                            }
                            KeyBehavior::RadioGroup(g) => {
                                if g.allow_none {
                                    write!(f.f, "allownone")?;
                                    needs_newline = true;
                                    handle_newline(&mut needs_newline, f)?;
                                    f.write_nesting()?;
                                }
                                write!(f.f, "radiogroup = {}", g.radio_group.raw())?;
                            }
                        }
                        needs_newline = true;
                    }
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
                        #[expect(clippy::too_many_arguments)]
                        fn write_levels<T: Format>(
                            needs_newline: &mut bool,
                            idx: usize,
                            group: &KeyGroup,
                            f: &mut Writer<'_, '_>,
                            name: &str,
                            absent: &str,
                            field: impl Fn(&KeyLevel) -> &SmallVec<[T; 1]>,
                            is_actions: bool,
                        ) -> fmt::Result {
                            if group.levels.iter().all(|l| field(l).is_empty()) {
                                return Ok(());
                            }
                            handle_newline(needs_newline, f)?;
                            f.write_nesting()?;
                            write!(f.f, "{name}[Group{idx}] = [")?;
                            f.write_inline_list(&group.levels, |f, l| {
                                let mut list = &**field(l);
                                if is_actions && f.omit_multiple_actions && list.len() > 1 {
                                    list = &[];
                                }
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
                            false,
                        )?;
                        if !f.lookup_only {
                            write_levels(
                                &mut needs_newline,
                                idx,
                                group,
                                f,
                                "actions",
                                "NoAction()",
                                |l| &l.actions,
                                true,
                            )?;
                        }
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
        Display::fmt(self, f.f)
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
                f.write("<")?;
                f.write_key_name(&kc.key_name)?;
                f.write(">")?;
            }
            f.write(" };")?;
            if kc.key_sym.is_some() && f.multi_line {
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

fn affect(lock: bool, unlock: bool) -> impl Display {
    debug_fn(move |f| {
        let affect = match (lock, unlock) {
            (false, false) => "neither",
            (true, false) => "lock",
            (false, true) => "unlock",
            _ => return Ok(()),
        };
        write!(f, ", affect={}", affect)
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

impl Format for rmlvo::Expanded {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        f.write_nesting()?;
        f.write("xkb_keymap {")?;
        f.write_nested(|f| {
            let elements = [
                ("xkb_keycodes", &self.keycodes),
                ("xkb_types", &self.types),
                ("xkb_compat", &self.compat),
                ("xkb_symbols", &self.symbols),
                ("xkb_geometry", &self.geometry),
            ];
            for (name, elements) in elements {
                f.write_nesting()?;
                f.write(name)?;
                f.write(" {")?;
                f.write_nested(|f| RmlvoIncludes(elements).format(f))?;
                f.write_nesting()?;
                f.write("};")?;
                f.write_newline()?;
            }
            Ok(())
        })?;
        f.write_nesting()?;
        f.write("};")?;
        Ok(())
    }
}

struct RmlvoIncludes<'a>(&'a [rmlvo::Element]);

impl Format for RmlvoIncludes<'_> {
    fn format(&self, f: &mut Writer<'_, '_>) -> fmt::Result {
        for e in self.0 {
            f.write_nesting()?;
            match e.merge_mode {
                MergeMode::Augment => f.write("augment ")?,
                MergeMode::Override => f.write("override ")?,
                MergeMode::Replace => f.write("replace ")?,
            }
            f.write_string(&e.include)?;
            f.write_newline()?;
        }
        Ok(())
    }
}

#[cfg(feature = "compose")]
mod compose {
    use crate::xkb::{
        compose::{ComposeTable, MatchRule, MatchStep},
        format::{Format, Writer},
    };

    impl Format for MatchStep<'_> {
        fn format(&self, f: &mut Writer<'_, '_>) -> std::fmt::Result {
            f.write("<")?;
            self.node.keysym.format(f)?;
            f.write(">")?;
            Ok(())
        }
    }

    impl Format for MatchRule<'_, '_> {
        fn format(&self, f: &mut Writer<'_, '_>) -> std::fmt::Result {
            for (idx, step) in self.steps.iter().enumerate() {
                if idx > 0 {
                    f.write(" ")?;
                }
                step.format(f)?;
            }
            f.write(":")?;
            if let Some(s) = &self.payload.string {
                f.write(" ")?;
                f.write_string(s)?;
            }
            if let Some(s) = self.payload.keysym {
                f.write(" ")?;
                s.format(f)?;
            }
            Ok(())
        }
    }

    impl Format for ComposeTable {
        fn format(&self, f: &mut Writer<'_, '_>) -> std::fmt::Result {
            let mut iter = self.iter();
            let mut first = true;
            while let Some(rule) = iter.next() {
                if !first {
                    f.write("\n")?;
                }
                first = false;
                rule.format(f)?;
            }
            Ok(())
        }
    }
}
