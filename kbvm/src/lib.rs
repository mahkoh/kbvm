//! KBVM is an implementation of the XKB specification and associated protocols.
//!
//! At its core, KBVM provides two types:
//!
//! - [`StateMachine`](state_machine::StateMachine), a compositor-side keyboard state
//!   machine.
//! - [`LookupTable`](lookup::LookupTable), a client-side lookup table that can be used
//!   to look up keysyms.
//!
//! These types can be created from XKB keymaps or RMLVO names by using an
//! [`xkb::Context`].
//!
//! Additionally, KBVM provides other tools from the XKB ecosystem:
//!
//! - [`ComposeTables`](xkb::compose::ComposeTable) can be created from XCompose files as
//!   a simple input method.
//! - XKB keymaps can be loaded from X11 connections via
//!   [integration](xkb::x11::KbvmX11Ext) with the x11rb crate.
//! - The RMLVO [registry](xkb::registry::Registry) can be loaded to display the available
//!   RMLVO names to users.
//!
//! While XKB keymaps can be used to create `StateMachines` and `LookupTables`, it is also
//! possible to created these objects manually with the [`Builder`](builder::Builder)
//! type. To retain compatibility with XKB, any `LookupTable` can be
//! [turned](lookup::LookupTable::to_xkb_keymap) into an XKB keymap.
//!
//! Manually created `StateMachines` allow you to run arbitrary logic when keys are
//! pressed and released. For example, you can use this logic to implement sticky keys,
//! radio groups, locked keys, key redirection, latching keys, etc.
//!
//! # The common compositor pattern
//!
//! If you are developing a wayland compositor, you might use this crate as follows:
//!
//! 1. For each seat, the user configures either an XKB map directly or a set of RMLVO
//!    names from which you have to create an XKB map.
//! 2. Either way, you use an [`xkb::Context`] and either
//!    [`keymap_from_bytes`](xkb::Context::keymap_from_bytes) or
//!    [`keymap_from_names`](xkb::Context::keymap_from_names) to create an
//!    [`xkb::Keymap`].
//! 3. You can format this keymap as a string by using
//!    [`Keymap::format`](xkb::Keymap::format). You send this keymap to clients via the
//!    `wl_keyboard.keymap` event.
//! 4. You then use [`Keymap::to_builder`](xkb::Keymap::to_builder) followed by
//!    [`Builder::build_state_machine`](builder::Builder::build_state_machine) to create
//!    a [`StateMachine`](state_machine::StateMachine).
//! 5. Whenever you receive a libinput key event, you feed it into the `StateMachine` with
//!    [`StateMachine::handle_key`](state_machine::StateMachine::handle_key). This in turn
//!    produces a number of [`Events`](state_machine::Event) that you forward to clients.
//!
//!    The `handle_key` documentation contains an example showing how to do this
//!    correctly.
//!
//! If you are also handling keyboard shortcuts in your compositor, you will likely also
//! want to create a `LookupTable` as described in the next section.
//!
//! # The common client pattern
//!
//! If you are developing a wayland client, you might use this crate as follows:
//!
//! 1. For each seat, you receive a keymap via the `wl_keyboard.keymap` event.
//! 2. You use an [`xkb::Context`] and
//!    [`keymap_from_bytes`](xkb::Context::keymap_from_bytes) to create an [`xkb::Keymap`]
//!    from the buffer from the event.
//! 3. You then use [`Keymap::to_builder`](xkb::Keymap::to_builder) followed by
//!    [`Builder::build_lookup_table`](builder::Builder::build_lookup_table) to create
//!    a [`LookupTable`](lookup::LookupTable).
//! 4. You create a [`Components`] object to store the active modifiers and group of the
//!    keyboard.
//! 5. Whenever you receive a `wl_keyboard.modifiers` event, you update the [`Components`]
//!    as shown in [`Components::update_effective`].
//! 6. Whenever you receive a `wl_keyboard.key` event for a key press, you use
//!    [`LookupTable::lookup`](lookup::LookupTable::lookup) to look up the keysyms
//!    produced by this event, using the effective modifiers and group from your
//!    [`Components`].

#![expect(
    clippy::collapsible_else_if,
    clippy::collapsible_if,
    clippy::field_reassign_with_default,
    clippy::should_implement_trait,
    clippy::assertions_on_constants,
    clippy::len_zero,
    clippy::manual_range_contains
)]

pub use {
    components::Components,
    controls::ControlsMask,
    group::{GroupDelta, GroupIndex},
    group_type::hidden::GroupType,
    keycode::Keycode,
    keysym::hidden::Keysym,
    modifier::hidden::{ModifierIndex, ModifierMask},
};

#[macro_use]
mod macros;
pub mod builder;
mod components;
mod config;
mod controls;
pub mod evdev;
mod from_bytes;
mod group;
pub mod group_type;
mod key_storage;
mod keycode;
pub mod keysym;
pub mod lookup;
pub mod modifier;
mod phf;
mod phf_map;
pub mod routine;
pub mod state_machine;
pub mod syms;
pub mod xkb;
