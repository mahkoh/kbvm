use {
    pkg_config::get_variable,
    std::{
        env,
        fs::{File, OpenOptions},
        io::{self, BufWriter, Write},
        path::PathBuf,
    },
};

fn main() {
    let default_include_dir =
        get_variable("xkeyboard-config", "xkb_base").unwrap_or("/usr/share/X11/xkb".into());
    let mut config = open("config.rs").unwrap();
    writeln!(
        config,
        "pub const DEFAULT_INCLUDE_DIR: &str = \"{}\";",
        default_include_dir
    )
    .unwrap();
}

fn open(s: &str) -> io::Result<BufWriter<File>> {
    let path: PathBuf = vec![env::var("OUT_DIR").unwrap().as_str(), s]
        .into_iter()
        .collect();
    Ok(BufWriter::new(
        OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open(path)?,
    ))
}
