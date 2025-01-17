#![allow(clippy::single_match)]

use log::LevelFilter;

mod cli;
mod compile_rmlvo;
mod compile_xkb;
mod evdev;
mod expand_rmlvo;
mod output;
mod test_wayland;
mod utils;

fn main() {
    env_logger::builder()
        .filter_level(LevelFilter::Info)
        .parse_default_env()
        .init();
    cli::main();
}
