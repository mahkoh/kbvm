use {
    error_reporter::Report,
    std::io::{Read, stdin},
};

pub fn read_path(path: &str) -> (&str, Vec<u8>) {
    match path {
        "-" => {
            let mut buf = vec![];
            if let Err(e) = stdin().lock().read_to_end(&mut buf) {
                log::error!("could not read <stdin>: {}", Report::new(e));
                std::process::exit(1);
            }
            ("<stdin>", buf)
        }
        _ => match std::fs::read(path) {
            Ok(v) => (path, v),
            Err(e) => {
                log::error!("could not read {path:?}: {}", Report::new(e));
                std::process::exit(1);
            }
        },
    }
}
