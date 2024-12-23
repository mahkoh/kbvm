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
    #[cfg(unix)]
    if default_include_dir.is_none() {
        default_include_dir = Some("/usr/share/X11/xkb".to_string());
    }
    let mut config = open("config.rs").unwrap();
    if let Some(dir) = &default_include_dir {
        writeln!(
            config,
            "pub const DEFAULT_INCLUDE_DIR: Option<&'static str> = Some(\"{}\");",
            dir
        )
        .unwrap();
    } else {
        writeln!(
            config,
            "pub const DEFAULT_INCLUDE_DIR: Option<&'static str> = None;",
        )
        .unwrap();
    }
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
