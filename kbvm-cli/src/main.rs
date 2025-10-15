#![allow(clippy::single_match)]
#![allow(clippy::collapsible_else_if)]

use {error_reporter::Report, log::LevelFilter};

mod cli;
mod compile_rmlvo;
mod compile_xkb;
#[cfg(unix)]
mod dump_x11;
mod evdev;
mod expand_rmlvo;
mod generate;
#[cfg_attr(not(unix), expect(dead_code))]
mod output;
#[cfg(unix)]
mod test_wayland;
mod utils;
#[cfg(unix)]
mod wayland_protocols;

fn main() {
    env_logger::builder()
        .filter_level(LevelFilter::Info)
        .parse_default_env()
        .init();
    if let Err(e) = utf8_console::enable() {
        log::error!("could not enable UTF-8: {}", Report::new(e));
    }
    cli::main();
}
