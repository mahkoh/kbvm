use {
    crate::{
        group::{GroupDelta, GroupIndex},
        modifier::{ModifierIndex, ModifierMask},
        state_machine,
        xkb::{
            controls::ControlMask,
            group::{GroupIdx, GroupMask},
            group_component::GroupComponent,
            indicator::IndicatorIdx,
            keymap::{self, Indicator, Key, KeyType, KeyTypeMapping, ModMapValue, VirtualModifier},
            level::Level,
            mod_component::ModComponentMask,
            x11::sealed::Sealed,
            Keymap,
        },
        Components, Keycode,
    },
    bstr::ByteSlice,
    hashbrown::{hash_map::Entry, DefaultHashBuilder, HashMap},
    indexmap::IndexMap,
    std::sync::Arc,
    thiserror::Error,
    x11rb::{
        connection::RequestConnection,
        cookie::Cookie,
        errors::{ConnectionError, ReplyError},
        protocol::{
            xkb::{
                self, BoolCtrl, ConnectionExt as E2, DeviceSpec, GetIndicatorMapReply, GetMapReply,
                GetNamesReply, IDSpec, IMGroupsWhich, IMModsWhich, LedClass, MapPart, NameDetail,
                SetOfGroup, VMod, XIFeature, ID,
            },
            xproto::{Atom, ConnectionExt as E1, GetAtomNameReply},
        },
    },
};

#[derive(Debug, Error)]
pub enum X11Error {
    #[error("could not send xkb_get_state request")]
    GetState(#[source] ConnectionError),
    #[error("could not retrieve xkb_get_state reply")]
    GetStateReply(#[source] ReplyError),
    #[error("could not fetch XKB extension info")]
    ExtensionInfo(#[source] ConnectionError),
    #[error("the XKB extension is not available")]
    Unavailable,
    #[error("could not send xkb_use_extension request")]
    UseExtension(#[source] ConnectionError),
    #[error("could not retrieve xkb_use_extension reply")]
    UseExtensionReply(#[source] ReplyError),
    #[error("could not send xkb_get_device_info request")]
    GetDeviceInfo(#[source] ConnectionError),
    #[error("could not retrieve xkb_get_device_info reply")]
    GetDeviceInfoReply(#[source] ReplyError),
    #[error("could not send xkb_get_map request")]
    GetMap(#[source] ConnectionError),
    #[error("could not retrieve xkb_get_map reply")]
    GetMapReply(#[source] ReplyError),
    #[error("could not send xkb_get_indicator_map request")]
    GetIndicatorMap(#[source] ConnectionError),
    #[error("could not retrieve xkb_get_indicator_map reply")]
    GetIndicatorMapReply(#[source] ReplyError),
    #[error("could not send xkb_get_compat_map request")]
    GetCompatMap(#[source] ConnectionError),
    #[error("could not retrieve xkb_get_compat_map reply")]
    GetCompatMapReply(#[source] ReplyError),
    #[error("could not send xkb_get_names request")]
    GetNames(#[source] ConnectionError),
    #[error("could not retrieve xkb_get_names reply")]
    GetNamesReply(#[source] ReplyError),
    #[error("could not send xkb_get_controls request")]
    GetControls(#[source] ConnectionError),
    #[error("could not retrieve xkb_get_controls reply")]
    GetControlsReply(#[source] ReplyError),
    #[error("could not send get_atom_name request")]
    GetAtomName(#[source] ConnectionError),
    #[error("could not retrieve get_atom_name reply")]
    GetAtomNameReply(#[source] ReplyError),
    #[error("server-sent type names with an invalid length")]
    TypeNamesLen,
    #[error("server-sent per-type level count with an invalid length")]
    LevelNamesCountLen,
    #[error("server-sent level names with an invalid length")]
    LevelNamesLen,
    #[error("server-sent key-name list misses a key name")]
    MissingKeyName,
    #[error("server-sent group names have an invalid length")]
    GroupNamesLen,
}

pub trait KbvmX11Ext: Sealed {
    fn setup_xkb_extension(&self) -> Result<ExtensionInfo, X11Error>;

    fn get_xkb_core_device_id(&self) -> Result<DeviceSpec, X11Error>;

    fn get_keymap(&self, device: DeviceSpec) -> Result<Keymap, X11Error>;

    fn get_xkb_components(&self, device: DeviceSpec) -> Result<Components, X11Error>;
}

mod sealed {
    pub trait Sealed {}
}

impl<T> Sealed for T where T: RequestConnection {}

pub struct ExtensionInfo {
    pub first_event: u8,
    pub first_error: u8,
}

impl<T> KbvmX11Ext for T
where
    T: RequestConnection,
{
    fn setup_xkb_extension(&self) -> Result<ExtensionInfo, X11Error> {
        let Some(info) = self
            .extension_information(xkb::X11_EXTENSION_NAME)
            .map_err(X11Error::ExtensionInfo)?
        else {
            return Err(X11Error::Unavailable);
        };
        let reply = self
            .xkb_use_extension(1, 0)
            .map_err(X11Error::UseExtension)?
            .reply()
            .map_err(X11Error::UseExtensionReply)?;
        if !reply.supported {
            return Err(X11Error::Unavailable);
        }
        Ok(ExtensionInfo {
            first_event: info.first_event,
            first_error: info.first_error,
        })
    }

    fn get_xkb_core_device_id(&self) -> Result<DeviceSpec, X11Error> {
        let reply = self
            .xkb_get_device_info(
                ID::USE_CORE_KBD.into(),
                XIFeature::default(),
                false,
                0,
                0,
                LedClass::default(),
                IDSpec::default(),
            )
            .map_err(X11Error::GetDeviceInfo)?
            .reply()
            .map_err(X11Error::GetDeviceInfoReply)?;
        Ok(reply.device_id as DeviceSpec)
    }

    fn get_keymap(&self, device: DeviceSpec) -> Result<Keymap, X11Error> {
        get_keymap(self, device)
    }

    fn get_xkb_components(&self, device: DeviceSpec) -> Result<Components, X11Error> {
        let cookie = self.xkb_get_state(device).map_err(X11Error::GetState)?;
        let reply = cookie.reply().map_err(X11Error::GetStateReply)?;
        Ok(Components {
            mods_pressed: ModifierMask(reply.base_mods.into()),
            mods_latched: ModifierMask(reply.latched_mods.into()),
            mods_locked: ModifierMask(reply.locked_mods.into()),
            mods: ModifierMask(reply.mods.into()),
            group_pressed: GroupDelta(reply.base_group as i32 as u32),
            group_latched: GroupDelta(reply.latched_group as i32 as u32),
            group_locked: GroupIndex(reply.locked_group.into()),
            group: GroupIndex(reply.group.into()),
        })
    }
}

fn get_keymap<C>(c: &C, device: DeviceSpec) -> Result<Keymap, X11Error>
where
    C: RequestConnection,
{
    let map_components = MapPart::KEY_TYPES
        | MapPart::KEY_SYMS
        | MapPart::MODIFIER_MAP
        | MapPart::EXPLICIT_COMPONENTS
        | MapPart::KEY_ACTIONS
        | MapPart::VIRTUAL_MODS
        | MapPart::VIRTUAL_MOD_MAP;
    let names = NameDetail::KEYCODES
        | NameDetail::SYMBOLS
        | NameDetail::TYPES
        | NameDetail::COMPAT
        | NameDetail::KEY_TYPE_NAMES
        | NameDetail::KT_LEVEL_NAMES
        | NameDetail::INDICATOR_NAMES
        | NameDetail::KEY_NAMES
        | NameDetail::KEY_ALIASES
        | NameDetail::VIRTUAL_MOD_NAMES
        | NameDetail::GROUP_NAMES;

    let map_cookie = c
        .xkb_get_map(
            device,
            map_components,
            MapPart::default(),
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            VMod::default(),
            0,
            0,
            0,
            0,
            0,
            0,
        )
        .map_err(X11Error::GetMap)?;
    let indicator_map_cookie = c
        .xkb_get_indicator_map(device, !0)
        .map_err(X11Error::GetIndicatorMap)?;
    let compat_map_cookie = c
        .xkb_get_compat_map(device, SetOfGroup::default(), true, 0, 0)
        .map_err(X11Error::GetCompatMap)?;
    let names_cookie = c.xkb_get_names(device, names).map_err(X11Error::GetNames)?;

    let map_reply = map_cookie.reply().map_err(X11Error::GetMapReply)?;
    let indicator_map_reply = indicator_map_cookie
        .reply()
        .map_err(X11Error::GetIndicatorMapReply)?;
    let compat_map_reply = compat_map_cookie
        .reply()
        .map_err(X11Error::GetCompatMapReply)?;
    let names_reply = names_cookie.reply().map_err(X11Error::GetNamesReply)?;

    MapBuilder {
        atoms: Interner {
            atoms: Default::default(),
            c,
        },
        c,
        names: names_reply,
        map: map_reply,
        indicator_map: indicator_map_reply,
        key_names: Default::default(),
        keycodes: Default::default(),
        indicators: Default::default(),
        types: Default::default(),
        virtual_modifiers: Default::default(),
        modmap: Default::default(),
        group_names: Default::default(),
        keys: Default::default(),
    }
    .build_map()
}

enum AtomState<'a, C>
where
    C: RequestConnection,
{
    Pending(Cookie<'a, C, GetAtomNameReply>),
    Present(Arc<String>),
}

struct Interner<'a, C>
where
    C: RequestConnection,
{
    atoms: HashMap<Atom, AtomState<'a, C>>,
    c: &'a C,
}

struct MapBuilder<'a, C>
where
    C: RequestConnection,
{
    atoms: Interner<'a, C>,
    c: &'a C,

    names: GetNamesReply,
    map: GetMapReply,
    indicator_map: GetIndicatorMapReply,

    key_names: HashMap<Keycode, Arc<String>>,

    keycodes: Vec<keymap::Keycode>,
    indicators: Vec<Indicator>,
    types: Vec<Arc<KeyType>>,
    virtual_modifiers: Vec<VirtualModifier>,
    modmap: Vec<(ModifierIndex, ModMapValue)>,
    group_names: Vec<(GroupIdx, Arc<String>)>,
    keys: IndexMap<Keycode, Key, DefaultHashBuilder>,
}

impl<C> Interner<'_, C>
where
    C: RequestConnection,
{
    fn prefetch(&mut self, atom: Atom) -> Result<(), X11Error> {
        if let Entry::Vacant(e) = self.atoms.entry(atom) {
            e.insert(AtomState::Pending(
                self.c.get_atom_name(atom).map_err(X11Error::GetAtomName)?,
            ));
        }
        Ok(())
    }

    fn get(&mut self, atom: Atom) -> Result<Arc<String>, X11Error> {
        let state = match self.atoms.entry(atom) {
            Entry::Vacant(e) => e.insert_entry(AtomState::Pending(
                self.c.get_atom_name(atom).map_err(X11Error::GetAtomName)?,
            )),
            Entry::Occupied(o) => o,
        };
        if let AtomState::Present(p) = state.get() {
            return Ok(p.clone());
        }
        let p = state.remove();
        let AtomState::Pending(p) = p else {
            unreachable!();
        };
        let reply = p.reply().map_err(X11Error::GetAtomNameReply)?;
        let s = Arc::new(reply.name.as_bstr().to_string());
        self.atoms.insert(atom, AtomState::Present(s.clone()));
        Ok(s)
    }
}

impl<C> MapBuilder<'_, C>
where
    C: RequestConnection,
{
    fn prefetch_atoms(&mut self) -> Result<(), X11Error> {
        let vl = &self.names.value_list;
        macro_rules! map_name {
            ($name:ident) => {
                if let Some(name) = &vl.$name {
                    self.atoms.prefetch(*name)?;
                }
            };
        }
        map_name!(keycodes_name);
        map_name!(symbols_name);
        map_name!(phys_symbols_name);
        map_name!(types_name);
        map_name!(compat_name);
        macro_rules! prefetch_all {
            ($name:ident) => {
                if let Some(els) = &vl.$name {
                    for el in els {
                        self.atoms.prefetch(*el)?;
                    }
                }
            };
        }
        prefetch_all!(type_names);
        prefetch_all!(indicator_names);
        prefetch_all!(virtual_mod_names);
        prefetch_all!(groups);
        prefetch_all!(radio_group_names);
        if let Some(els) = &vl.kt_level_names {
            for el in &els.kt_level_names {
                self.atoms.prefetch(*el)?;
            }
        }
        if let Some(kn) = &vl.key_names {
            for (idx, kn) in kn.iter().enumerate() {
                let keycode = Keycode(idx as u32 + self.names.min_key_code as u32);
                let mut name = &kn.name[..];
                if let Some(p) = name.iter().position(|n| *n == 0) {
                    name = &name[..p];
                }
                self.key_names
                    .insert(keycode, Arc::new(name.as_bstr().to_string()));
            }
        }
        Ok(())
    }

    // fn map_keycodes(&mut self) -> Result<(), X11Error> {
    //     // self.map.map
    // }

    fn map_modmap(&mut self) -> Result<(), X11Error> {
        let mm = self.map.map.modmap_rtrn.as_deref().unwrap_or_default();
        for mm in mm {
            let Some(key_name) = self.key_names.get(&Keycode(mm.keycode as u32)) else {
                return Err(X11Error::MissingKeyName);
            };
            for idx in ModifierMask(mm.mods.into()) {
                self.modmap.push((
                    idx,
                    ModMapValue {
                        key_name: key_name.clone(),
                        key_sym: None,
                    },
                ));
            }
        }
        Ok(())
    }

    fn map_virtual_modifiers(&mut self) -> Result<(), X11Error> {
        let vmods = self.map.map.vmods_rtrn.as_deref().unwrap_or_default();
        let names = self
            .names
            .value_list
            .virtual_mod_names
            .as_deref()
            .unwrap_or_default();
        for (mask, name) in vmods.iter().zip(names.iter()) {
            self.virtual_modifiers.push(VirtualModifier {
                name: self.atoms.get(*name)?,
                values: ModifierMask((*mask).into()),
            });
        }
        Ok(())
    }

    fn map_types(&mut self) -> Result<(), X11Error> {
        let tys = self.map.map.types_rtrn.as_deref().unwrap_or_default();
        let names = self
            .names
            .value_list
            .type_names
            .as_deref()
            .unwrap_or_default();
        if tys.len() != names.len() {
            return Err(X11Error::TypeNamesLen);
        }
        let n_levels_per_type = self
            .names
            .value_list
            .kt_level_names
            .as_ref()
            .map(|n| &n.n_levels_per_type[..])
            .unwrap_or_default();
        let kt_level_names = self
            .names
            .value_list
            .kt_level_names
            .as_ref()
            .map(|n| &n.kt_level_names[..])
            .unwrap_or_default();
        if n_levels_per_type.len() != tys.len() {
            return Err(X11Error::LevelNamesCountLen);
        }
        if n_levels_per_type.iter().map(|n| *n as usize).sum::<usize>() != kt_level_names.len() {
            return Err(X11Error::LevelNamesLen);
        }
        let mut lo = 0;
        for (idx, (ty, name)) in tys.iter().zip(names.iter()).enumerate() {
            let hi = lo + n_levels_per_type[idx] as usize;
            let mut names = vec![];
            for (idx, name) in kt_level_names[lo..hi].iter().enumerate() {
                names.push((Level::new(idx as u32 + 1).unwrap(), self.atoms.get(*name)?));
            }
            let mut mappings = vec![];
            for (idx, m) in ty.map.iter().enumerate() {
                if !m.active {
                    continue;
                }
                let mut preserved = ModifierMask::default();
                if let Some(p) = ty.preserve.get(idx) {
                    preserved.0 = p.mask.into();
                }
                mappings.push(KeyTypeMapping {
                    modifiers: ModifierMask(m.mods_mask.into()),
                    preserved,
                    level: Level::new(m.level as u32 + 1).unwrap(),
                });
            }
            let ret = KeyType {
                name: self.atoms.get(*name)?,
                modifiers: ModifierMask(ty.mods_mask.into()),
                mappings,
                level_names: names,
            };
            self.types.push(Arc::new(ret));
            lo = hi;
        }
        Ok(())
    }

    fn map_group_names(&mut self) -> Result<(), X11Error> {
        let names = self.names.value_list.groups.as_deref().unwrap_or_default();
        let mut names = names.into_iter();
        for i in 0..4 {
            if self.names.group_names.contains(1 << i) {
                let Some(name) = names.next() else {
                    return Err(X11Error::GroupNamesLen);
                };
                self.group_names
                    .push((GroupIdx::new(i + 1).unwrap(), self.atoms.get(*name)?));
            }
        }
        Ok(())
    }

    fn map_indicators(&mut self) -> Result<(), X11Error> {
        let names = self
            .names
            .value_list
            .indicator_names
            .as_deref()
            .unwrap_or_default();
        for (idx, (map, name)) in self.indicator_map.maps.iter().zip(names.iter()).enumerate() {
            let group_components = match map.which_groups {
                _ if map.which_groups == IMGroupsWhich::default() => GroupComponent::None,
                IMGroupsWhich::USE_BASE => GroupComponent::Base,
                IMGroupsWhich::USE_LATCHED => GroupComponent::Latched,
                IMGroupsWhich::USE_LOCKED => GroupComponent::Locked,
                _ => GroupComponent::Effective,
            };
            macro_rules! map_mask {
                (
                    $lf:ident,
                    $rf:ident,
                    $lt:ident,
                    $rt:ident,
                    $(
                        $ln:ident => $rn:ident,
                    )*
                ) => {
                    $(
                        if map.$lf.contains($lt::$ln) {
                            $rf |= $rt::$rn;
                        }
                    )*
                };
            }
            let mut mod_components = ModComponentMask::default();
            map_mask! {
                which_mods, mod_components, IMModsWhich, ModComponentMask,
                USE_BASE => BASE,
                USE_LATCHED => LATCHED,
                USE_LOCKED => LOCKED,
                USE_EFFECTIVE => EFFECTIVE,
                USE_COMPAT => EFFECTIVE,
            }
            let mut controls = ControlMask::default();
            map_mask! {
                ctrls, controls, BoolCtrl, ControlMask,
                REPEAT_KEYS              => REPEAT_KEYS,
                SLOW_KEYS                => SLOW_KEYS,
                BOUNCE_KEYS              => BOUNCE_KEYS,
                STICKY_KEYS              => STICKY_KEYS,
                MOUSE_KEYS               => MOUSE_KEYS,
                MOUSE_KEYS_ACCEL         => MOUSE_KEYS_ACCEL,
                ACCESS_X_KEYS            => ACCESS_X_KEYS,
                ACCESS_X_TIMEOUT_MASK    => ACCESS_X_TIMEOUT,
                ACCESS_X_FEEDBACK_MASK   => ACCESS_X_FEEDBACK,
                AUDIBLE_BELL_MASK        => AUDIBLE_BELL,
                OVERLAY1_MASK            => OVERLAY1,
                OVERLAY2_MASK            => OVERLAY2,
                IGNORE_GROUP_LOCK_MASK   => IGNORE_GROUP_LOCK,
            }
            self.indicators.push(Indicator {
                virt: idx >= self.indicator_map.real_indicators as usize,
                index: IndicatorIdx::new(idx as u32 + 1).unwrap(),
                name: self.atoms.get(*name)?,
                modifier_mask: ModifierMask(map.mods.into()),
                group_mask: GroupMask(map.groups.into()),
                controls,
                mod_components,
                group_components,
            });
        }
        Ok(())
    }

    fn map_keycodes(&mut self) {
        for (&keycode, name) in &self.key_names {
            if name.is_empty() {
                continue;
            }
            self.keycodes.push(keymap::Keycode {
                name: name.clone(),
                keycode,
            });
        }
        self.keycodes.sort_by_key(|kc| kc.keycode);
    }

    fn map_keys(&mut self) {
        // for (&keycode, name) in &self.map.map.vmods_rtrn {
        //     if name.is_empty() {
        //         continue;
        //     }
        //     self.keycodes.push(keymap::Keycode {
        //         name: name.clone(),
        //         keycode,
        //     });
        // }
        // self.keycodes.sort_by_key(|kc| kc.keycode);
    }

    fn build_map(mut self) -> Result<Keymap, X11Error> {
        self.prefetch_atoms()?;
        self.map_types()?;
        self.map_modmap()?;
        self.map_virtual_modifiers()?;
        self.map_group_names()?;
        self.map_indicators()?;
        self.map_keycodes();
        self.map_keys();
        let map = Keymap {
            name: None,
            max_keycode: 255,
            indicators: self.indicators,
            keycodes: self.keycodes,
            types: self.types,
            virtual_modifiers: self.virtual_modifiers,
            mod_maps: self.modmap,
            group_names: self.group_names,
            keys: self.keys,
        };
        Ok(map)
    }
}

#[cfg(test)]
mod tests {
    use crate::xkb::x11::KbvmX11Ext;

    #[test]
    fn test() {
        let (con, _) = x11rb::connect(None).unwrap();
        let _ = con.setup_xkb_extension().unwrap();
        let id = con.get_xkb_core_device_id().unwrap();
        let map = con.get_keymap(id).unwrap();
        println!("{:#}", map);
    }
}
