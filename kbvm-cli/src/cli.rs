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

/// KBVM test utility.
#[derive(Parser, Debug)]
struct Kbvm {
    #[clap(subcommand)]
    command: Cmd,
}

#[derive(Subcommand, Debug)]
enum Cmd {
    /// Test keymaps and compose files via wayland input events.
    TestWayland(TestWaylandArgs),
    /// Expand RMLVO names.
    ExpandRmlvo(ExpandRmlvoArgs),
    /// Compile a keymap from RMLVO names.
    CompileRmlvo(CompileRmlvoArgs),
    /// Compile a keymap from an XKB file.
    CompileXkb(CompileXkbArgs),
}

#[derive(Args, Debug, Default)]
pub struct CompileArgs {
    /// Disable all default include paths.
    #[clap(long)]
    pub no_default_includes: bool,
    /// Prepend an include path.
    #[clap(long)]
    pub prepend_include: Vec<String>,
    /// Append an include path.
    #[clap(long)]
    pub append_include: Vec<String>,
}

impl CompileArgs {
    pub fn apply(&self, builder: &mut ContextBuilder) {
        if self.no_default_includes {
            builder.clear();
        }
        for dir in self.prepend_include.iter().rev() {
            builder.prepend_path(dir);
        }
        for dir in &self.append_include {
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
