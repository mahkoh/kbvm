use {
    crate::{cli::CompileArgs, utils::read_path},
    clap::Args,
    kbvm::xkb::{diagnostic::WriteToLog, Context},
};

#[derive(Args, Debug, Default)]
pub struct CompileXkbArgs {
    #[clap(flatten)]
    compile_args: CompileArgs,
    xkb: Option<String>,
}

pub fn main(args: CompileXkbArgs) {
    let mut context = Context::builder();
    args.compile_args.apply(&mut context);
    let context = context.build();
    let path = args.xkb.as_deref().unwrap_or("-");
    let (path, source) = read_path(path);
    let expanded = context.keymap_from_bytes(WriteToLog, Some(path.as_ref()), &source);
    match expanded {
        Ok(map) => {
            println!("{:#}", map.format());
        }
        Err(_) => {
            log::error!("could not compile keymap");
            std::process::exit(1);
        }
    }
}
