use crate::compile_rmlvo;
use crate::compile_rmlvo::CompileRmlvoArgs;
use crate::compile_xkb;
use crate::compile_xkb::CompileXkbArgs;
#[cfg(unix)]
use crate::dump_x11;
#[cfg(unix)]
use crate::dump_x11::DumpX11Args;
use crate::expand_rmlvo;
use crate::expand_rmlvo::ExpandRmlvoArgs;
use crate::generate;
use crate::generate::GenerateArgs;
#[cfg(unix)]
use crate::test_wayland;
#[cfg(unix)]
use crate::test_wayland::TestWaylandArgs;
use clap::Args;
use clap::Parser;
use clap::Subcommand;
use clap::ValueHint;
use kbvm::xkb::ContextBuilder;
use kbvm::xkb::keymap::Formatter;

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
    /// Loads the keymap from the X server and writes it to stdout.
    #[cfg(unix)]
    DumpX11(DumpX11Args),
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
}

#[derive(Args, Debug, Default)]
pub struct FormatArgs {
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
}

impl FormatArgs {
    pub fn apply<'a>(&self, mut formatter: Formatter<'a>) -> Formatter<'a> {
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
        #[cfg(unix)]
        Cmd::DumpX11(args) => dump_x11::main(args),
    }
}
