use log::LevelFilter;

mod cli;
mod evdev;
mod output;
mod test_wayland;

fn main() {
    env_logger::builder()
        .filter_level(LevelFilter::Info)
        .parse_default_env()
        .init();
    cli::main();
}
