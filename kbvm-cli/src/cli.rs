#[cfg(unix)]
use crate::test_wayland::{self, TestWaylandArgs};
use {
    crate::{
        compile_rmlvo::{self, CompileRmlvoArgs},
        compile_xkb::{self, CompileXkbArgs},
        expand_rmlvo::{self, ExpandRmlvoArgs},
        generate::{self, GenerateArgs},
    },
    clap::{Args, Parser, Subcommand, ValueHint},
    kbvm::xkb::{ContextBuilder, keymap::Formatter},
};

/// KBVM test utility.
#[derive(Parser, Debug)]
pub struct Kbvm {
    #[clap(subcommand)]
    command: Cmd,
}

#[derive(Subcommand, Debug)]
enum Cmd {
    /// Test keymaps and compose files via wayland input events.
    #[cfg(unix)]
    TestWayland(TestWaylandArgs),
    /// Expand RMLVO names.
    ExpandRmlvo(ExpandRmlvoArgs),
    /// Compile a keymap from RMLVO names.
    CompileRmlvo(CompileRmlvoArgs),
    /// Compile a keymap from an XKB file.
    CompileXkb(CompileXkbArgs),
    /// Generate shell completion scripts for kbvm.
    GenerateCompletion(GenerateArgs),
}

#[derive(Args, Debug, Default)]
pub struct CompileArgs {
    /// Disable all default include paths.
    #[clap(long)]
    pub no_default_includes: bool,
    /// Prepend an include path.
    #[clap(long, value_hint = ValueHint::DirPath)]
    pub prepend_include: Vec<String>,
    /// Append an include path.
    #[clap(long, value_hint = ValueHint::DirPath)]
    pub append_include: Vec<String>,
    /// Don't include key actions or behaviors.
    #[clap(long)]
    pub lookup_only: bool,
    /// Rename long key names for compatibility with other XKB implementations.
    #[clap(long)]
    pub rename_long_names: bool,
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

    pub fn apply2<'a>(&self, mut formatter: Formatter<'a>) -> Formatter<'a> {
        if self.lookup_only {
            formatter = formatter.lookup_only(true);
        }
        if self.rename_long_names {
            formatter = formatter.rename_long_keys(true);
        }
        formatter.multiple_actions_per_level(true)
    }
}

pub fn main() {
    let args = Kbvm::parse();
    match args.command {
        #[cfg(unix)]
        Cmd::TestWayland(args) => test_wayland::main(args),
        Cmd::ExpandRmlvo(args) => expand_rmlvo::main(args),
        Cmd::CompileRmlvo(args) => compile_rmlvo::main(args),
        Cmd::CompileXkb(args) => compile_xkb::main(args),
        Cmd::GenerateCompletion(args) => generate::main(args),
    }
}
