use crate::cli::CompileArgs;
use crate::cli::FormatArgs;
use crate::utils::read_path;
use clap::Args;
use clap::ValueHint;
use error_reporter::Report;
use kbvm::xkb::Context;
use kbvm::xkb::diagnostic::WriteToLog;
use raw_stdio::raw_stdout;
use std::fmt::Display;
use std::io::BufWriter;
use std::io::Write;

#[derive(Args, Debug, Default)]
pub struct CompileXkbArgs {
    #[clap(flatten)]
    compile_args: CompileArgs,
    #[clap(flatten)]
    format_args: FormatArgs,
    /// The path to the keymap.
    ///
    /// If the path is not specified or if the path is `-`, the keymap is read from stdin.
    #[clap(value_hint = ValueHint::FilePath)]
    path: Option<String>,
}

pub fn main(args: CompileXkbArgs) {
    let mut context = Context::builder();
    args.compile_args.apply(&mut context);
    let context = context.build();
    let path = args.path.as_deref().unwrap_or("-");
    let (path, source) = read_path(path);
    let expanded = context.keymap_from_bytes(WriteToLog, Some(path.as_ref()), &source);
    match expanded {
        Ok(map) => {
            format_keymap(args.format_args.apply(map.format()));
        }
        Err(_) => {
            log::error!("could not compile keymap");
            std::process::exit(1);
        }
    }
}

pub(crate) fn format_keymap(map: impl Display) {
    let mut stdout = BufWriter::new(raw_stdout());
    let res = writeln!(stdout, "{}", map);
    if let Err(e) = res {
        log::error!("could not format keymap: {}", Report::new(e));
    }
}
