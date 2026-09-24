use crate::cli::Kbvm;
use clap::Args;
use clap::CommandFactory;
use clap_complete::Shell;
use std::io::stdout;

#[derive(Args, Debug)]
pub struct GenerateArgs {
    /// The shell to generate completions for
    #[clap(value_enum)]
    shell: Shell,
}

pub fn main(args: GenerateArgs) {
    let stdout = stdout();
    let mut stdout = stdout.lock();
    clap_complete::generate(args.shell, &mut Kbvm::command(), "kbvm", &mut stdout);
}
