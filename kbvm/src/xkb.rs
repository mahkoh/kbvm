//! Integration with the XKB ecosystem.
//!
//! This module provides integration with the XKB ecosystem, including
//!
//! - loading keymaps from buffers,
//! - loading keymaps from RMLVO names,
//! - loading keymaps from X11 connections,
//! - loading the RMLVO registry,
//! - loading XCompose files.
//!
//! # Example
//!
//! ```
//! # use kbvm::xkb::Context;
//! # use kbvm::xkb::diagnostic::WriteToLog;
//! const MAP: &str = r#"
//!     xkb_keymap {
//!         // This empty map is not very useful. Under wayland, you would get
//!         // this map from the wl_keyboard.keymap event.
//!     };
//! "#;
//! let context = Context::default();
//! let keymap = context.keymap_from_bytes(WriteToLog, None, MAP).unwrap();
//! let builder = keymap.to_builder();
//! let _state_machine = builder.build_state_machine();
//! let _lookup_table = builder.build_lookup_table();
//! ```
//!
//! # Features
//!
//! The following features are disabled by default:
//!
//! - `registry` - Provides access to the RMLVO registry. Most applications have no need
//!   for this.
//! - `compose` - Allows loading XCompose files.
//! - `x11` - Allows loading keymaps from X11 connections.
//!
//! # Logging
//!
//! This crate does not write to STDERR and does not use the `log` crate. Instead,
//! diagnostic messages are handled via the
//! [`DiagnosticHandler`](diagnostic::DiagnosticHandler) trait that is accepted by
//! functions that might produce diagnostic messages. This allows you to display these
//! messages as you see fit.
//!
//! To keep simple things simple, this crate provides the
//! [`WriteToStderr`](diagnostic::WriteToStderr) and
//! [`WriteToLog`](diagnostic::WriteToLog) implementations that do the obvious things.
//!
//! The `WriteToLog` type depends on the `log` crate and the feature of the same name,
//! which is enabled by default.

pub use {
    context::{Context, ContextBuilder},
    keymap::Keymap,
};

#[macro_use]
mod macros;
pub(crate) mod clone_with_delta;
mod code;
mod code_loader;
mod code_map;
mod code_slice;
#[cfg(feature = "compose")]
pub mod compose;
mod context;
mod controls;
pub mod diagnostic;
pub(crate) mod format;
mod group;
mod group_component;
mod include;
mod indicator;
mod interner;
mod kccgst;
pub mod keymap;
mod level;
mod meaning;
mod mod_component;
mod modmap;
mod radio_group;
#[cfg(feature = "registry")]
pub mod registry;
mod resolved;
pub mod rmlvo;
mod span;
mod string_cooker;
#[cfg(feature = "x11")]
pub mod x11;
