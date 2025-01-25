#![allow(clippy::single_match)]

use {error_reporter::Report, log::LevelFilter};

mod cli;
mod compile_rmlvo;
mod compile_xkb;
mod evdev;
mod expand_rmlvo;
mod generate;
mod output;
mod test_wayland;
mod utils;

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
