use {
    crate::{cli::FormatArgs, compile_xkb::format_keymap},
    clap::Args,
    error_reporter::Report,
    kbvm::xkb::x11::KbvmX11Ext,
};

#[derive(Args, Debug, Default)]
pub struct DumpX11Args {
    #[clap(flatten)]
    format_args: FormatArgs,
}

pub fn main(args: DumpX11Args) {
    let (con, _) = match x11rb::connect(None) {
        Ok(c) => c,
        Err(e) => {
            log::error!("could not connect to X11: {}", Report::new(e));
            std::process::exit(1);
        }
    };
    if let Err(e) = con.setup_xkb_extension() {
        log::error!("could not enable the XKB extension: {}", Report::new(e));
        std::process::exit(1);
    }
    let dev_id = match con.get_xkb_core_device_id() {
        Ok(i) => i,
        Err(e) => {
            log::error!("could not retrieve the core device ID: {}", Report::new(e));
            std::process::exit(1);
        }
    };
    let expanded = con.get_xkb_keymap(dev_id);
    match expanded {
        Ok(map) => {
            format_keymap(args.format_args.apply(map.format()));
        }
        Err(e) => {
            log::error!("could not retrieve the keymap from X11: {}", Report::new(e));
            std::process::exit(1);
        }
    }
}
