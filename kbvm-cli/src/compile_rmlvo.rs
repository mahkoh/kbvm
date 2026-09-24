use crate::cli::CompileArgs;
use crate::cli::FormatArgs;
use crate::compile_xkb::format_keymap;
use crate::expand_rmlvo::RmlvoArgs;
use clap::Args;
use kbvm::xkb::Context;
use kbvm::xkb::diagnostic::WriteToLog;

#[derive(Args, Debug, Default)]
pub struct CompileRmlvoArgs {
    #[clap(flatten)]
    compile_args: CompileArgs,
    #[clap(flatten)]
    format_args: FormatArgs,
    #[clap(flatten)]
    rmlvo: RmlvoArgs,
}

pub fn main(args: CompileRmlvoArgs) {
    let mut context = Context::builder();
    args.compile_args.apply(&mut context);
    let context = context.build();
    let (rules, model, groups, options) = args.rmlvo.expand();
    let expanded = context.keymap_from_names(
        WriteToLog,
        rules,
        model,
        groups.as_deref(),
        options.as_deref(),
    );
    format_keymap(args.format_args.apply(expanded.format()));
}
