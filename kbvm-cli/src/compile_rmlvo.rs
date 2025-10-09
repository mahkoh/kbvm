use {
    crate::{cli::CompileArgs, compile_xkb::format_keymap, expand_rmlvo::RmlvoArgs},
    clap::Args,
    kbvm::xkb::{Context, diagnostic::WriteToLog},
};

#[derive(Args, Debug, Default)]
pub struct CompileRmlvoArgs {
    #[clap(flatten)]
    compile_args: CompileArgs,
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
    format_keymap(expanded.format().multiple_actions_per_level(true));
}
