use {
    crate::{
        compile_rmlvo::{self, CompileRmlvoArgs},
        compile_xkb::{self, CompileXkbArgs},
        expand_rmlvo::{self, ExpandRmlvoArgs},
        test_wayland::{self, TestWaylandArgs},
    },
    clap::{Args, Parser, Subcommand},
    kbvm::xkb::ContextBuilder,
};

#[derive(Parser, Debug)]
struct Kbvm {
    #[clap(subcommand)]
    command: Cmd,
}

#[derive(Subcommand, Debug)]
enum Cmd {
    TestWayland(TestWaylandArgs),
    ExpandRmlvo(ExpandRmlvoArgs),
    CompileRmlvo(CompileRmlvoArgs),
    CompileXkb(CompileXkbArgs),
}

#[derive(Args, Debug, Default)]
pub struct CompileArgs {
    #[clap(long)]
    pub no_default_paths: bool,
    #[clap(long)]
    pub prepend_path: Vec<String>,
    #[clap(long)]
    pub path: Vec<String>,
}

impl CompileArgs {
    pub fn apply(&self, builder: &mut ContextBuilder) {
        if self.no_default_paths {
            builder.clear();
        }
        for dir in self.prepend_path.iter().rev() {
            builder.prepend_path(dir);
        }
        for dir in &self.path {
            builder.append_path(dir);
        }
    }
}

pub fn main() {
    let args = Kbvm::parse();
    match args.command {
        Cmd::TestWayland(args) => test_wayland::main(args),
        Cmd::ExpandRmlvo(args) => expand_rmlvo::main(args),
        Cmd::CompileRmlvo(args) => compile_rmlvo::main(args),
        Cmd::CompileXkb(args) => compile_xkb::main(args),
    }
}
