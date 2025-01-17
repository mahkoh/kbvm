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
    /// The name of the rules file.
    #[clap(long)]
    rules: Option<String>,
    /// The name of the model.
    #[clap(long)]
    model: Option<String>,
    /// A comma-separated list of layouts.
    #[clap(long, value_delimiter = ',', num_args = 1..)]
    layout: Vec<String>,
    /// A comma-separated list of layout variants.
    #[clap(long, value_delimiter = ',', num_args = 1..)]
    variant: Vec<String>,
    /// A comma-separated list of options.
    #[clap(long, value_delimiter = ',', num_args = 1..)]
    options: Vec<String>,
}

impl RmlvoArgs {
    #[allow(clippy::type_complexity)]
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
