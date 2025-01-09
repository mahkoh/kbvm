#![expect(
    clippy::collapsible_else_if,
    clippy::collapsible_if,
    clippy::field_reassign_with_default,
    clippy::should_implement_trait,
    clippy::assertions_on_constants,
    clippy::len_zero,
    clippy::manual_range_contains
)]

pub mod builder;
pub mod components;
mod config;
mod from_bytes;
pub mod group;
pub mod group_type;
pub mod keysym;
pub mod syms;
pub mod lookup;
pub mod modifier;
mod phf;
mod phf_map;
pub mod routine;
pub mod state_machine;
pub mod xkb;

pub use components::Components;
