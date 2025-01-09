#![expect(
    clippy::collapsible_else_if,
    clippy::collapsible_if,
    clippy::field_reassign_with_default,
    clippy::should_implement_trait,
    clippy::assertions_on_constants,
    clippy::len_zero,
    clippy::manual_range_contains
)]

pub use {components::Components, state_machine::Keycode};

pub mod builder;
pub mod components;
mod config;
pub mod evdev;
mod from_bytes;
pub mod group;
pub mod group_type;
pub mod keysym;
pub mod lookup;
pub mod modifier;
mod phf;
mod phf_map;
pub mod routine;
pub mod state_machine;
pub mod syms;
pub mod xkb;
