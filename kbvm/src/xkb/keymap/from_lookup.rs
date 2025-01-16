use {
    crate::{
        builder::Redirect,
        group_type,
        lookup::LookupTable,
        syms,
        xkb::{
            group::GroupIdx,
            keymap::{
                Indicator, Key, KeyGroup, KeyLevel, KeyType, KeyTypeMapping, Keycode, ModMapValue,
                VirtualModifier,
            },
            level::Level,
            resolved::GroupsRedirect,
            Keymap,
        },
        ModifierIndex,
    },
    hashbrown::{hash_map::Entry, HashMap},
    std::sync::Arc,
};

impl LookupTable {
    /// Creates a client-side XKB keymap from a lookup table.
    ///
    /// The created keymap does not contain any actions. Clients trying to implement
    /// how-to-type logic will not be able to determine which key presses trigger which
    /// modifiers or groups.
    ///
    /// The keymap will contain modmap entries mapping all keys producing `Alt_L` or
    /// `Alt_R` to `Mod1`.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::builder::{Builder, GroupBuilder, KeyBuilder, LevelBuilder};
    /// # use kbvm::GroupType;
    /// # use kbvm::syms;
    /// # use kbvm::ModifierMask;
    /// # use kbvm::Keycode;
    /// let mut builder = Builder::default();
    /// {
    ///     let gt = GroupType::builder(ModifierMask::SHIFT)
    ///         .map(ModifierMask::SHIFT, 1)
    ///         .build();
    ///     let mut gb = GroupBuilder::new(0, &gt);
    ///     let mut level = LevelBuilder::new(0);
    ///     level.keysyms(&[syms::a]);
    ///     gb.add_level(level);
    ///     let mut level = LevelBuilder::new(1);
    ///     level.keysyms(&[syms::A]);
    ///     gb.add_level(level);
    ///     let mut key = KeyBuilder::new(Keycode::from_x11(9));
    ///     key.repeats(true);
    ///     key.add_group(gb);
    ///     builder.add_key(key)
    /// }
    /// {
    ///     let gt = GroupType::builder(ModifierMask::NONE).build();
    ///     let mut level = LevelBuilder::new(0);
    ///     level.keysyms(&[syms::Alt_L]);
    ///     let mut gb = GroupBuilder::new(0, &gt);
    ///     gb.add_level(level);
    ///     let mut key = KeyBuilder::new(Keycode::from_x11(10));
    ///     key.add_group(gb);
    ///     builder.add_key(key)
    /// }
    /// let map = builder.build_lookup_table().to_xkb_keymap();
    /// println!("{:#}", map.format());
    /// ```
    ///
    /// Outputs
    ///
    /// ```xkb
    /// xkb_keymap {
    ///     xkb_keycodes {
    ///         minimum = 8;
    ///         maximum = 255;
    ///
    ///         indicator 1 = "DUMMY";
    ///
    ///         <9> = 9;
    ///         <10> = 10;
    ///     };
    ///
    ///     xkb_types {
    ///         virtual_modifiers Dummy;
    ///
    ///         type "type0" {
    ///             modifiers = Shift;
    ///             map[Shift] = Level2;
    ///         };
    ///
    ///         type "type1" {
    ///             modifiers = None;
    ///         };
    ///     };
    ///
    ///     xkb_compat {
    ///         interpret VoidSymbol {
    ///             repeat = false;
    ///         };
    ///     };
    ///
    ///     xkb_symbols {
    ///         modmap Mod1 { <10> };
    ///
    ///         key.repeat = true;
    ///
    ///         key <9> {
    ///             type[Group1] = "type0",
    ///             symbols[Group1] = [ a, A ]
    ///         };
    ///         key <10> {
    ///             repeat = false,
    ///             type[Group1] = "type1",
    ///             symbols[Group1] = [ Alt_L ]
    ///         };
    ///     };
    /// };
    /// ```
    pub fn to_xkb_keymap(&self) -> Keymap {
        let mut max_keycode = 255;
        let mut keycodes = Vec::with_capacity(self.keys.len());
        let mut keys = Vec::with_capacity(self.keys.len());
        let mut key_types = HashMap::<_, Arc<KeyType>>::new();
        let mut mod_maps = vec![];
        for (kc, kg) in self.keys.iter().enumerate() {
            let Some(kg) = kg else {
                continue;
            };
            let kc = crate::Keycode(kc as u32);
            max_keycode = max_keycode.max(kc.0);
            let name = Arc::new(kc.0.to_string());
            keycodes.push(Keycode {
                name: name.clone(),
                keycode: kc,
            });
            let mut groups = Vec::with_capacity(kg.groups.len());
            for group in kg.groups.iter() {
                let Some(group) = group else {
                    groups.push(None);
                    continue;
                };
                let mut levels = Vec::with_capacity(group.levels.len());
                macro_rules! syms_to_map {
                    ($($syms:pat => $mask:ident,)*) => {{
                        for &sym in group.levels.iter().flat_map(|l| l.symbols.iter()) {
                            let idx = $(
                                if let $syms = sym {
                                    ModifierIndex::$mask
                                } else
                            )* {
                                continue;
                            };
                            mod_maps.push((
                                idx,
                                ModMapValue {
                                    key_name: name.clone(),
                                    key_sym: None,
                                },
                            ));
                            break;
                        }
                    }};
                }
                syms_to_map! {
                    syms::Shift_L | syms::Shift_R => SHIFT,
                    syms::Caps_Lock => LOCK,
                    syms::Control_L | syms::Control_R => CONTROL,
                    syms::Alt_L | syms::Alt_R => MOD1,
                    syms::Num_Lock => MOD2,
                    syms::Super_L | syms::Super_R => MOD4,
                }
                for level in &group.levels {
                    levels.push(KeyLevel {
                        symbols: level.symbols.clone(),
                        actions: Default::default(),
                    });
                }
                let num_key_types = key_types.len();
                let key_type = match key_types.entry(&*group.ty.data as *const group_type::Data) {
                    Entry::Occupied(v) => v.get().clone(),
                    Entry::Vacant(v) => {
                        let mut mappings = Vec::with_capacity(group.ty.data.cases.len());
                        for case in &group.ty.data.cases {
                            if case.level <= u32::MAX as usize {
                                mappings.push(KeyTypeMapping {
                                    modifiers: case.mods,
                                    preserved: case.mods & !case.consumed,
                                    level: Level::new(case.level as u32 + 1).unwrap(),
                                });
                            }
                        }
                        let kt = KeyType {
                            name: Arc::new(format!("type{}", num_key_types)),
                            modifiers: group.ty.data.mask,
                            mappings,
                            level_names: vec![],
                        };
                        let kt = Arc::new(kt);
                        v.insert(kt.clone());
                        kt
                    }
                };
                groups.push(Some(KeyGroup { key_type, levels }));
            }
            if groups.last() == Some(&None) {
                groups.pop();
            }
            let redirect = match kg.redirect {
                Redirect::Wrap => GroupsRedirect::Wrap,
                Redirect::Clamp => GroupsRedirect::Clamp,
                Redirect::Fixed(f) => {
                    if f >= u32::MAX as usize {
                        GroupsRedirect::Redirect(GroupIdx::ONE)
                    } else {
                        GroupsRedirect::Redirect(
                            GroupIdx::new(f as u32 + 1).unwrap_or(GroupIdx::ONE),
                        )
                    }
                }
            };
            let symbol = Key {
                key_name: name.clone(),
                key_code: kc,
                groups,
                repeat: kg.repeats,
                redirect,
            };
            keys.push((kc, symbol));
        }
        let mut types: Vec<_> = key_types.into_values().collect();
        types.sort_unstable_by_key(|kt| kt.name.clone());
        keys.sort_by_key(|k| k.0);
        Keymap {
            name: None,
            max_keycode,
            indicators: vec![Indicator::dummy()],
            keycodes,
            types,
            virtual_modifiers: vec![VirtualModifier::dummy()],
            mod_maps,
            group_names: vec![],
            keys: keys.into_iter().collect(),
        }
    }
}
