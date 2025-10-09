use {
    error_reporter::Report,
    kbvm::xkb::{Context, diagnostic::WriteToLog, x11::KbvmX11Ext},
    std::{
        fs::File,
        io::{Read, Write},
        os::unix::process::CommandExt,
        process::{Command, Stdio},
    },
    uapi::{
        OwnedFd,
        c::{self},
        fork, kill,
    },
    x11rb::rust_connection::RustConnection,
};

fn main() {
    let xvfb = Xvfb::launch();
    xvfb.set_keymap(include_str!("host.xkb"));
    // xvfb.print_keymap();
    xvfb.con.setup_xkb_extension().unwrap();
    let id = xvfb.con.get_xkb_core_device_id().unwrap();
    let map = xvfb.con.get_xkb_keymap(id).unwrap();
    let actual = format!("{:#}\n", map.format());
    if actual != include_str!("expected.xkb") {
        let actual_path = env!("CARGO_MANIFEST_DIR").to_owned() + "/src/actual.xkb";
        std::fs::write(&actual_path, actual).unwrap();
        eprintln!("expected differs from actual");
        std::process::exit(1);
    }
    let mut context = Context::builder();
    context.clear();
    context.enable_environment(false);
    let roundtrip = context
        .build()
        .keymap_from_bytes(WriteToLog, None, actual.as_bytes())
        .unwrap();
    if map != roundtrip {
        let actual_path = env!("CARGO_MANIFEST_DIR").to_owned() + "/src/roundtrip.xkb";
        std::fs::write(&actual_path, format!("{:#}\n", roundtrip.format())).unwrap();
        eprintln!("roundtrip failed");
        eprintln!("roundtrip: {roundtrip:#?}");
        eprintln!("expected:  {map:#?}");
        std::process::exit(1);
    }
}

struct Xvfb {
    _exit_write: OwnedFd,
    display: String,
    con: RustConnection,
}

impl Xvfb {
    fn launch() -> Self {
        // x11-tests
        // |
        // \-- intermediate process that kills xvfb when x11-tests exits
        //     |
        //     \-- xvfb
        let (exit_read, exit_write) = uapi::pipe().unwrap();
        let (display_read, display_write) = uapi::pipe().unwrap();
        let pid = unsafe { fork().unwrap() };
        if pid == 0 {
            unsafe {
                c::daemon(0, 0);
            }
            let pid = unsafe { fork().unwrap() };
            if pid == 0 {
                drop([exit_read, exit_write, display_read]);
                let err = Command::new("Xvfb")
                    .arg("-displayfd")
                    .arg(display_write.raw().to_string())
                    .exec();
                eprintln!("could not spawn xvfb: {}", Report::new(err));
                std::process::exit(0);
            }
            drop([exit_write, display_read, display_write]);
            let mut buf = 0u8;
            uapi::read(exit_read.raw(), &mut buf).unwrap();
            kill(pid, c::SIGKILL).unwrap();
            std::process::exit(0);
        }
        drop([exit_read, display_write]);
        let mut display = ":".to_string();
        File::from(display_read)
            .read_to_string(&mut display)
            .unwrap();
        display.truncate(display.trim_end().len());
        let (con, _) = RustConnection::connect(Some(&display)).unwrap();
        Xvfb {
            _exit_write: exit_write,
            display,
            con,
        }
    }

    #[allow(dead_code)]
    fn print_keymap(&self) {
        let status = Command::new("xkbcomp")
            .arg(&self.display)
            .arg("-")
            .spawn()
            .unwrap()
            .wait()
            .unwrap();
        assert!(status.success());
    }

    fn set_keymap(&self, map: &str) {
        let mut child = Command::new("xkbcomp")
            .arg("-I")
            .arg("-")
            .arg(&self.display)
            .stdin(Stdio::piped())
            .spawn()
            .unwrap();
        child
            .stdin
            .take()
            .unwrap()
            .write_all(map.as_bytes())
            .unwrap();
        let exit = child.wait().unwrap();
        assert!(exit.success());
    }
}
