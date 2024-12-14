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
    let mut default_include_dir = None;
    if default_include_dir.is_none() {
        if let Ok(var) = get_variable("xkeyboard-config", "xkb_base") {
            default_include_dir = Some(var);
        }
    }
    let mut config = open("config.rs").unwrap();
    writeln!(
        config,
        "pub const DEFAULT_INCLUDE_DIR: &str = \"{}\";",
        default_include_dir
            .as_deref()
            .unwrap_or("/usr/share/X11/xkb"),
    )
    .unwrap();
}

fn open(s: &str) -> io::Result<BufWriter<File>> {
    let mut path = PathBuf::from(env::var("OUT_DIR").unwrap());
    path.push(s);
    Ok(BufWriter::new(
        OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open(path)?,
    ))
}
