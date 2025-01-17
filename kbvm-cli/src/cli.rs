use {
    crate::{test_wayland, test_wayland::TestWaylandArgs},
    clap::{Parser, Subcommand},
};

#[derive(Parser, Debug)]
struct Kbvm {
    #[clap(subcommand)]
    command: Cmd,
}

#[derive(Subcommand, Debug)]
enum Cmd {
    TestWayland(TestWaylandArgs),
}

pub fn main() {
    let args = Kbvm::parse();
    match args.command {
        Cmd::TestWayland(args) => test_wayland::main(args),
    }
}
