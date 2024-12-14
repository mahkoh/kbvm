use std::path::Path;

fn main() {
    let path = Path::new("../libxkbcommon/build/libxkbcommon.a");
    if !path.exists() {
        panic!("You have to build libxkbcommon.a in the libxkbcommon submodule to run tests");
    }
    println!(
        "cargo::rustc-link-search={}",
        path.parent().unwrap().canonicalize().unwrap().display()
    );
    println!("cargo::rustc-link-lib=static=xkbcommon");
}
