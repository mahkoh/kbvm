use {
    crate::cli::Kbvm,
    clap::{Args, CommandFactory},
    clap_complete::Shell,
    std::io::stdout,
};

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
