use std::path::Path;

fn main() {
    let path = Path::new("../libxkbcommon/build/libxkbcommon.a");
    if !path.exists() {
        panic!(concat!(
            "Failed to find libxkbcommon.a for static linking.\n",
            "You have to build libxkbcommon.a in the libxkbcommon submodule to run tests:\n",
            "  $ git submodule --init update\n",
            "  $ cd libxkbcommon\n",
            "  $ meson setup -Ddefault_library=both build/\n",
            "  $ meson compile -C build/"
        ));
    }
    println!(
        "cargo::rustc-link-search={}",
        path.parent().unwrap().canonicalize().unwrap().display()
    );
    println!("cargo::rustc-link-lib=static=xkbcommon");
}
