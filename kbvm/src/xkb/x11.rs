//! Integration with the `x11rb` crate.
//!
//! This module provides the extension trait [`KbvmX11Ext`] that can be used to create
//! keymaps from [`RequestConnection`] objects.

use {
    crate::{
        group::{GroupDelta, GroupIndex},
        xkb::{
            controls::ControlMask,
            group::{GroupChange, GroupIdx, GroupMask},
            group_component::GroupComponent,
            indicator::IndicatorIdx,
            keymap::{
                self,
                actions::{
                    GroupLatchAction, GroupLockAction, GroupSetAction, ModsLatchAction,
                    ModsLockAction, ModsSetAction,
                },
                Action, Indicator, Key, KeyGroup, KeyLevel, KeyType, KeyTypeMapping, ModMapValue,
                VirtualModifier,
            },
            level::Level,
            mod_component::ModComponentMask,
            resolved::GroupsRedirect,
            x11::sealed::Sealed,
            Keymap,
        },
        Components, Keycode, Keysym, ModifierIndex, ModifierMask,
    },
    bstr::ByteSlice,
    hashbrown::{hash_map::Entry, DefaultHashBuilder, HashMap, HashSet},
    indexmap::IndexMap,
    std::sync::Arc,
    thiserror::Error,
    x11rb::{
        connection::RequestConnection,
        cookie::Cookie,
        errors::{ConnectionError, ReplyError},
        protocol::{
            xkb::{
                self, BoolCtrl, ConnectionExt as E2, DeviceSpec, GetControlsReply,
                GetIndicatorMapReply, GetMapReply, GetNamesReply, IDSpec, IMGroupsWhich,
                IMModsWhich, LedClass, MapPart, NameDetail, SAIsoLockFlag, SAType, VMod, XIFeature,
                ID, SA,
            },
            xproto::{Atom, ConnectionExt as E1, GetAtomNameReply},
        },
    },
};

/// An error produced by one of the [`KbvmX11Ext`] functions.
#[derive(Debug, Error)]
pub enum X11Error {
    /// Could not send `xkb_get_state` request.
    #[error("could not send xkb_get_state request")]
    GetState(#[source] ConnectionError),
    /// Could not retrieve `xkb_get_state` reply.
    #[error("could not retrieve xkb_get_state reply")]
    GetStateReply(#[source] ReplyError),
    /// Could not fetch XKB extension info.
    #[error("could not fetch XKB extension info")]
    ExtensionInfo(#[source] ConnectionError),
    /// The XKB extension is not available.
    #[error("the XKB extension is not available")]
    Unavailable,
    /// Could not send `xkb_use_extension` request.
    #[error("could not send xkb_use_extension request")]
    UseExtension(#[source] ConnectionError),
    /// Could not retrieve `xkb_use_extension` reply.
    #[error("could not retrieve xkb_use_extension reply")]
    UseExtensionReply(#[source] ReplyError),
    /// Could not send `xkb_get_device_info` request.
    #[error("could not send xkb_get_device_info request")]
    GetDeviceInfo(#[source] ConnectionError),
    /// Could not retrieve `xkb_get_device_info` reply.
    #[error("could not retrieve xkb_get_device_info reply")]
    GetDeviceInfoReply(#[source] ReplyError),
    /// Could not send `xkb_get_map` request.
    #[error("could not send xkb_get_map request")]
    GetMap(#[source] ConnectionError),
    /// Could not retrieve `xkb_get_map` reply.
    #[error("could not retrieve xkb_get_map reply")]
    GetMapReply(#[source] ReplyError),
    /// Could not send `xkb_get_indicator_map` request.
    #[error("could not send xkb_get_indicator_map request")]
    GetIndicatorMap(#[source] ConnectionError),
    /// Could not retrieve `xkb_get_indicator_map` reply.
    #[error("could not retrieve xkb_get_indicator_map reply")]
    GetIndicatorMapReply(#[source] ReplyError),
    /// Could not send `xkb_get_names` request.
    #[error("could not send xkb_get_names request")]
    GetNames(#[source] ConnectionError),
    /// Could not retrieve `xkb_get_names` reply.
    #[error("could not retrieve xkb_get_names reply")]
    GetNamesReply(#[source] ReplyError),
    /// Could not send `xkb_get_controls` request.
    #[error("could not send xkb_get_controls request")]
    GetControls(#[source] ConnectionError),
    /// Could not retrieve `xkb_get_controls` reply.
    #[error("could not retrieve xkb_get_controls reply")]
    GetControlsReply(#[source] ReplyError),
    /// Could not send `get_atom_name` request.
    #[error("could not send get_atom_name request")]
    GetAtomName(#[source] ConnectionError),
    /// Could not retrieve `get_atom_name` reply.
    #[error("could not retrieve get_atom_name reply")]
    GetAtomNameReply(#[source] ReplyError),
}

/// A [`RequestConnection`] extension to create keymaps and fetch components.
///
/// This trait is automatically implemented for all types that implement
/// [`RequestConnection`].
///
/// # Example
///
/// ```no_run
/// # use x11rb::rust_connection::RustConnection;
/// # use kbvm::xkb::x11::KbvmX11Ext;
/// let (con, _) = RustConnection::connect(None).unwrap();
/// // You must call this function before using any of the other functions.
/// con.setup_xkb_extension().unwrap();
/// let core_device_id = con.get_xkb_core_device_id().unwrap();
/// let keymap = con.get_xkb_keymap(core_device_id).unwrap();
/// ```
pub trait KbvmX11Ext: Sealed {
    /// Initializes the XKB extension for this connection.
    ///
    /// You must call this function before calling any of the other functions of this
    /// trait.
    fn setup_xkb_extension(&self) -> Result<ExtensionInfo, X11Error>;

    /// Returns the core keyboard device ID.
    ///
    /// You must call [`Self::setup_xkb_extension`] before calling this function.
    fn get_xkb_core_device_id(&self) -> Result<DeviceSpec, X11Error>;

    /// Returns [`Keymap`] of the given device.
    ///
    /// You must call [`Self::setup_xkb_extension`] before calling this function.
    fn get_xkb_keymap(&self, device: DeviceSpec) -> Result<Keymap, X11Error>;

    /// Returns the current [`Components`] of the given device.
    ///
    /// You must call [`Self::setup_xkb_extension`] before calling this function.
    fn get_xkb_components(&self, device: DeviceSpec) -> Result<Components, X11Error>;
}

mod sealed {
    pub trait Sealed {}
}

impl<T> Sealed for T where T: RequestConnection + ?Sized {}

/// Information about the XKB extension.
#[derive(Copy, Clone, Debug)]
pub struct ExtensionInfo {
    /// The first event code of the extension.
    pub first_event: u8,
    /// The first error code of the extension.
    pub first_error: u8,
}

impl<T> KbvmX11Ext for T
where
    T: RequestConnection + ?Sized,
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

    fn get_xkb_keymap(&self, device: DeviceSpec) -> Result<Keymap, X11Error> {
        get_keymap(self, device)
    }

    fn get_xkb_components(&self, device: DeviceSpec) -> Result<Components, X11Error> {
        let reply = self
            .xkb_get_state(device)
            .map_err(X11Error::GetState)?
            .reply()
            .map_err(X11Error::GetStateReply)?;
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
    C: RequestConnection + ?Sized,
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
    let names_cookie = c.xkb_get_names(device, names).map_err(X11Error::GetNames)?;
    let controls_cookie = c.xkb_get_controls(device).map_err(X11Error::GetControls)?;

    let map_reply = map_cookie.reply().map_err(X11Error::GetMapReply)?;
    let indicator_map_reply = indicator_map_cookie
        .reply()
        .map_err(X11Error::GetIndicatorMapReply)?;
    let names_reply = names_cookie.reply().map_err(X11Error::GetNamesReply)?;
    let controls_reply = controls_cookie
        .reply()
        .map_err(X11Error::GetControlsReply)?;

    MapBuilder {
        atoms: Interner {
            atoms: Default::default(),
            c,
        },
        names: names_reply,
        map: map_reply,
        indicator_map: indicator_map_reply,
        controls: controls_reply,
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
    C: RequestConnection + ?Sized,
{
    Pending(Cookie<'a, C, GetAtomNameReply>),
    Present(Arc<String>),
}

struct Interner<'a, C>
where
    C: RequestConnection + ?Sized,
{
    atoms: HashMap<Atom, AtomState<'a, C>>,
    c: &'a C,
}

struct MapBuilder<'a, C>
where
    C: RequestConnection + ?Sized,
{
    atoms: Interner<'a, C>,

    names: GetNamesReply,
    map: GetMapReply,
    indicator_map: GetIndicatorMapReply,
    controls: GetControlsReply,

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
    C: RequestConnection + ?Sized,
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
    C: RequestConnection + ?Sized,
{
    fn prefetch_atoms(&mut self) -> Result<(), X11Error> {
        let vl = &self.names.value_list;
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
                if name.is_empty() {
                    continue;
                }
                self.key_names
                    .insert(keycode, Arc::new(name.as_bstr().to_string()));
            }
        }
        Ok(())
    }

    fn map_modmap(&mut self) {
        let mm = self.map.map.modmap_rtrn.as_deref().unwrap_or_default();
        for mm in mm {
            let Some(key_name) = self.key_names.get(&Keycode(mm.keycode as u32)) else {
                continue;
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
        let mut n_levels_per_type = self
            .names
            .value_list
            .kt_level_names
            .as_ref()
            .map(|n| &n.n_levels_per_type[..])
            .unwrap_or_default()
            .iter()
            .map(|n| *n as usize);
        let mut kt_level_names = self
            .names
            .value_list
            .kt_level_names
            .as_ref()
            .map(|n| &n.kt_level_names[..])
            .unwrap_or_default()
            .iter()
            .copied();
        for (ty, name) in tys.iter().zip(names.iter()) {
            let mut names = vec![];
            let names_of_type = kt_level_names
                .by_ref()
                .take(n_levels_per_type.next().unwrap_or_default())
                .enumerate();
            for (idx, name) in names_of_type {
                names.push((Level::new(idx as u32 + 1).unwrap(), self.atoms.get(name)?));
            }
            let mut mappings = vec![];
            for (idx, m) in ty.map.iter().enumerate() {
                if !m.active {
                    continue;
                }
                let mut preserved = ModifierMask(0);
                if let Some(p) = ty.preserve.get(idx) {
                    preserved.0 = p.mask.into();
                }
                mappings.push(KeyTypeMapping {
                    modifiers: ModifierMask(m.mods_mask.into()),
                    preserved,
                    level: Level::new(m.level as u32 + 1).unwrap(),
                });
            }
            mappings.sort_unstable_by_key(|k| (k.level, k.modifiers.0));
            let ret = KeyType {
                name: self.atoms.get(*name)?,
                modifiers: ModifierMask(ty.mods_mask.into()),
                mappings,
                level_names: names,
            };
            self.types.push(Arc::new(ret));
        }
        Ok(())
    }

    fn map_group_names(&mut self) -> Result<(), X11Error> {
        let mut names = self
            .names
            .value_list
            .groups
            .as_deref()
            .unwrap_or_default()
            .iter();
        for i in 0..4 {
            if self.names.group_names.contains(1 << i) {
                let Some(name) = names.next() else {
                    continue;
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
            let group_mask = GroupMask(map.groups.into());
            let group_component = match map.which_groups {
                _ if group_mask.0 == 0 => GroupComponent::Effective,
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
            let modifier_mask = ModifierMask(map.mods.into());
            let mut mod_components = ModComponentMask::default();
            if modifier_mask.0 != 0 {
                map_mask! {
                    which_mods, mod_components, IMModsWhich, ModComponentMask,
                    USE_BASE => BASE,
                    USE_LATCHED => LATCHED,
                    USE_LOCKED => LOCKED,
                    USE_EFFECTIVE => EFFECTIVE,
                    USE_COMPAT => EFFECTIVE,
                }
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
                virt: (self.indicator_map.real_indicators & (1 << idx)) == 0,
                index: IndicatorIdx::new(idx as u32 + 1).unwrap(),
                name: self.atoms.get(*name)?,
                modifier_mask,
                group_mask,
                controls,
                mod_components,
                group_component,
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
        let syms = self.map.map.syms_rtrn.as_deref().unwrap_or_default();
        let mut actions_count = self
            .map
            .map
            .key_actions
            .as_ref()
            .map(|a| &a.acts_rtrn_count[..])
            .unwrap_or_default()
            .iter()
            .copied();
        let mut actions = self
            .map
            .map
            .key_actions
            .as_ref()
            .map(|a| &a.acts_rtrn_acts[..])
            .unwrap_or_default()
            .iter();
        for (idx, sym) in syms.iter().enumerate() {
            let mut groups = vec![];
            let width = sym.width as usize;
            let types = &sym.kt_index;
            let num_groups = (sym.group_info as usize & 0xf).min(types.len());
            let mut syms = sym.syms.iter();
            let has_actions = actions_count.next().unwrap_or_default() > 0;
            #[expect(clippy::needless_range_loop)]
            for i in 0..num_groups {
                let mut levels = vec![];
                for sym in syms.by_ref().take(width) {
                    let mut level = KeyLevel::default();
                    if *sym != 0 {
                        level.symbols.push(Keysym(*sym));
                    }
                    if has_actions {
                        if let Some(action) = actions.next() {
                            level.actions.extend(map_action(action));
                        }
                    }
                    levels.push(level);
                }
                while let Some(last) = levels.last() {
                    if last.actions.is_empty() && last.symbols.is_empty() {
                        levels.pop();
                    } else {
                        break;
                    }
                }
                if levels.is_empty() {
                    groups.push(None);
                    continue;
                }
                let Some(ty) = self.types.get(types[i] as usize) else {
                    continue;
                };
                groups.push(Some(KeyGroup {
                    key_type: ty.clone(),
                    levels,
                }));
            }
            let kc = Keycode(idx as u32 + self.map.min_key_code as u32);
            let Some(name) = self.key_names.get(&kc) else {
                continue;
            };
            let repeat = {
                let idx = kc.0 / 8;
                let offset = kc.0 % 8;
                match self.controls.per_key_repeat.get(idx as usize) {
                    Some(r) => r & (1 << offset) != 0,
                    _ => continue,
                }
            };
            let redirect = if sym.group_info & (1 << 6) != 0 {
                GroupsRedirect::Clamp
            } else if sym.group_info & (1 << 7) != 0 {
                let mut idx = GroupIdx::new(((sym.group_info & 0x30) >> 4) as u32).unwrap();
                if idx.to_offset() >= groups.len() {
                    idx = GroupIdx::ONE
                };
                GroupsRedirect::Redirect(idx)
            } else {
                GroupsRedirect::Wrap
            };
            self.keys.insert(
                kc,
                Key {
                    key_name: name.clone(),
                    key_code: kc,
                    groups,
                    repeat,
                    redirect,
                },
            );
        }
    }

    fn remove_unused(&mut self) {
        self.keycodes
            .retain(|kc| self.keys.contains_key(&kc.keycode));
        let used_types: HashSet<_> = self
            .keys
            .values()
            .flat_map(|k| k.groups.iter().flatten())
            .map(|g| &*g.key_type as *const KeyType)
            .collect();
        self.types
            .retain(|t| used_types.contains(&(&**t as *const KeyType)));
    }

    fn sort(&mut self) {
        self.types.sort_unstable_by_key(|t| t.name.clone());
        self.keycodes.sort_unstable_by_key(|k| k.name.clone());
        self.virtual_modifiers.sort_by_key(|k| k.name.clone());
        let mut keys: Vec<_> = self.keys.drain(..).collect();
        keys.sort_unstable_by_key(|k| k.1.key_name.clone());
        self.keys = keys.into_iter().collect();
        self.modmap.sort_unstable();
    }

    fn build_map(mut self) -> Result<Keymap, X11Error> {
        self.prefetch_atoms()?;
        self.map_types()?;
        self.map_modmap();
        self.map_virtual_modifiers()?;
        self.map_group_names()?;
        self.map_indicators()?;
        self.map_keycodes();
        self.map_keys();
        self.remove_unused();
        self.sort();
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

fn map_action(action: &xkb::Action) -> Option<Action> {
    macro_rules! map_group {
        ($a:expr) => {
            match $a.flags.contains(SA::GROUP_ABSOLUTE) {
                true => GroupChange::Absolute(GroupIdx::new($a.group as u32 + 1).unwrap()),
                false => GroupChange::Rel($a.group as i32),
            }
        };
    }
    let action = match action.as_type() {
        SAType::SET_MODS => {
            let a = action.as_setmods();
            Action::ModsSet(ModsSetAction {
                clear_locks: a.flags.contains(SA::CLEAR_LOCKS),
                modifiers: ModifierMask(a.mask.into()),
            })
        }
        SAType::LATCH_MODS => {
            let a = action.as_latchmods();
            Action::ModsLatch(ModsLatchAction {
                clear_locks: a.flags.contains(SA::CLEAR_LOCKS),
                latch_to_lock: a.flags.contains(SA::LATCH_TO_LOCK),
                modifiers: ModifierMask(a.mask.into()),
            })
        }
        SAType::LOCK_MODS => {
            let a = action.as_lockmods();
            Action::ModsLock(ModsLockAction {
                modifiers: ModifierMask(a.mask.into()),
                lock: !a.flags.contains(SAIsoLockFlag::NO_LOCK),
                unlock: !a.flags.contains(SAIsoLockFlag::NO_UNLOCK),
            })
        }
        SAType::SET_GROUP => {
            let a = action.as_setgroup();
            Action::GroupSet(GroupSetAction {
                group: map_group!(a),
                clear_locks: a.flags.contains(SA::CLEAR_LOCKS),
            })
        }
        SAType::LATCH_GROUP => {
            let a = action.as_latchgroup();
            Action::GroupLatch(GroupLatchAction {
                group: map_group!(a),
                clear_locks: a.flags.contains(SA::CLEAR_LOCKS),
                latch_to_lock: a.flags.contains(SA::LATCH_TO_LOCK),
            })
        }
        SAType::LOCK_GROUP => {
            let a = action.as_lockgroup();
            Action::GroupLock(GroupLockAction {
                group: map_group!(a),
            })
        }
        _ => return None,
    };
    Some(action)
}
