use {
    crate::cli::CompileArgs,
    clap::Args,
    isnt::std_1::vec::IsntVecExt,
    kbvm::xkb::{diagnostic::WriteToLog, rmlvo::Group, Context},
};

#[derive(Args, Debug, Default)]
pub struct ExpandRmlvoArgs {
    #[clap(flatten)]
    compile_args: CompileArgs,
    #[clap(flatten)]
    rmlvo: RmlvoArgs,
}

#[derive(Args, Debug, Default)]
pub struct RmlvoArgs {
    #[clap(long)]
    rules: Option<String>,
    #[clap(long)]
    model: Option<String>,
    #[clap(long)]
    layout: Vec<String>,
    #[clap(long)]
    variant: Vec<String>,
    #[clap(long)]
    options: Vec<String>,
}

impl RmlvoArgs {
    pub fn expand(
        &self,
    ) -> (
        Option<&str>,
        Option<&str>,
        Option<Vec<Group<'_>>>,
        Option<Vec<&str>>,
    ) {
        let mut groups = vec![];
        for (idx, layout) in self.layout.iter().enumerate() {
            let variant = self.variant.get(idx).map(|s| &**s).unwrap_or_default();
            groups.push(Group { layout, variant });
        }
        let options: Vec<_> = self.options.iter().map(|s| &**s).collect();
        (
            self.rules.as_deref(),
            self.model.as_deref(),
            self.layout.is_not_empty().then_some(groups),
            self.options.is_not_empty().then_some(options),
        )
    }
}

pub fn main(args: ExpandRmlvoArgs) {
    let mut context = Context::builder();
    args.compile_args.apply(&mut context);
    let context = context.build();
    let (rules, model, groups, options) = args.rmlvo.expand();
    let expanded = context.expand_names(
        WriteToLog,
        rules,
        model,
        groups.as_deref(),
        options.as_deref(),
    );
    println!("{:#}", expanded.format());
}
