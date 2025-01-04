#[rustfmt::skip]
mod generated;
#[path = "../../kbvm/src/phf.rs"]
mod phf;
#[path = "../../kbvm/src/phf_map.rs"]
mod phf_map;

use {
    crate::generated::{CODE_TO_NAME, NAME_TO_CODE},
    error_reporter::Report,
    isnt::std_1::vec::IsntVecExt,
    kbvm::{
        modifier::ModifierMask,
        state_machine::{Keycode, LogicalEvent, State},
        xkb::{
            diagnostic::{Diagnostic, DiagnosticSink},
            Context,
        },
    },
    parking_lot::Mutex,
    phf_map::PhfMap,
    std::{
        fmt::Write,
        fs::read_dir,
        io::{self, ErrorKind},
        path::{Path, PathBuf},
        str::FromStr,
        sync::{
            atomic::{AtomicUsize, Ordering::Relaxed},
            Arc,
        },
        thread::available_parallelism,
    },
    thiserror::Error,
};

// const SINGLE: Option<&str> = Some("t0358");
const SINGLE: Option<&str> = None;
const WRITE_MISSING: bool = true;
const WRITE_FAILED: bool = false;
const SHOW_ALL_DIAGNOSTICS_IF_SINGLE: bool = true;
const SHOW_ALL_DIAGNOSTICS: bool = false;

fn main() {
    std::env::set_current_dir(env!("CARGO_MANIFEST_DIR")).unwrap();
    let path = Path::new("./testcases");
    let mut cases = vec![];
    for f in read_dir(path).unwrap() {
        let f = f.unwrap();
        if !f.metadata().unwrap().is_dir() {
            continue;
        }
        for f in read_dir(f.path()).unwrap() {
            let f = f.unwrap();
            if !f.metadata().unwrap().is_dir() {
                continue;
            }
            for f in read_dir(f.path()).unwrap() {
                let f = f.unwrap();
                if let Some(s) = SINGLE {
                    if f.file_name() != s {
                        continue;
                    }
                }
                cases.push(f.path());
            }
        }
    }
    let results = Arc::new(Results {
        idx: Default::default(),
        cases,
        results: Default::default(),
    });
    let mut threads = vec![];
    for _ in 0..available_parallelism().unwrap().get() {
        let results = results.clone();
        threads.push(std::thread::spawn(move || test_thread(results)));
    }
    for t in threads {
        t.join().unwrap();
    }
    let results = &mut *results.results.lock();
    results.sort_unstable_by(|l, r| l.case.cmp(&r.case));
    let mut any_failed = false;
    for result in results {
        let show_all_dignostics =
            SHOW_ALL_DIAGNOSTICS || (SHOW_ALL_DIAGNOSTICS_IF_SINGLE && SINGLE.is_some());
        if result.result.is_ok() && (!show_all_dignostics || result.diagnostics.is_empty()) {
            continue;
        }
        let case: &Path = result.case.file_name().unwrap().as_ref();
        eprintln!("Case {}:", case.display());
        let write_diagnostic = |diag: &Diagnostic| {
            let lines = diag.with_code().to_string();
            let mut lines = lines.lines();
            eprintln!("       == {}", lines.next().unwrap());
            for line in lines {
                eprintln!("          {}", line);
            }
        };
        if result.diagnostics.is_not_empty() {
            eprintln!("    Diagnostics");
            for diag in &result.diagnostics {
                write_diagnostic(diag);
            }
        }
        if let Err(err) = &result.result {
            any_failed = true;
            eprintln!("    Error: {}", Report::new(err));
            match err {
                ResultError::ParsingFailed(e) => {
                    write_diagnostic(e);
                }
                _ => {}
            }
        }
    }
    if any_failed {
        std::process::exit(1);
    }
}

struct Results {
    idx: AtomicUsize,
    cases: Vec<PathBuf>,
    results: Mutex<Vec<TestResult>>,
}

struct TestResult {
    case: PathBuf,
    diagnostics: Vec<Diagnostic>,
    result: Result<(), ResultError>,
}

#[derive(Debug, Error)]
enum ResultError {
    #[error("could not read map file")]
    ReadMapFailed(#[source] io::Error),
    #[error("could not read input file")]
    ReadInputFailed(#[source] io::Error),
    #[error("could not read expected file")]
    ReadExpectedFailed(#[source] io::Error),
    #[error("invalid key name `{0}`")]
    InvalidKeyName(String),
    #[error("unknown command `{0}`")]
    UnknownCommand(String),
    #[error("invalid repeat count `{0}`")]
    InvalidRepeatCount(String),
    #[error("could not parse keymap")]
    ParsingFailed(#[source] Diagnostic),
    #[error("could not write expected file")]
    WriteExpectedFailed(#[source] io::Error),
    #[error("text comparison failed")]
    TextComparisonFailed,
    #[error("could not write actual file")]
    WriteActualFailed(#[source] io::Error),
    #[error("invalid key code `{0}`")]
    InvalidKeyCode(u32),
}

fn test_thread(results: Arc<Results>) {
    let digits = (results.cases.len() as f64 + 1.0).log10().ceil() as usize;
    loop {
        let idx = results.idx.fetch_add(1, Relaxed);
        if idx >= results.cases.len() {
            return;
        }
        let path = &results.cases[idx];
        eprintln!(
            "testing {:0digits$}/{}: {}",
            idx + 1,
            results.cases.len(),
            Path::new(path.file_name().unwrap()).display(),
        );
        test_case(&results, path)
    }
}

fn test_case(results: &Results, case: &Path) {
    let mut diagnostics = Vec::new();
    let result = TestResult {
        result: test_case2(&mut diagnostics, case),
        diagnostics,
        case: case.to_path_buf(),
    };
    results.results.lock().push(result);
}

fn test_case2(diagnostics: &mut Vec<Diagnostic>, case: &Path) -> Result<(), ResultError> {
    let map_path = case.join("map.xkb");
    let map = std::fs::read(&map_path).map_err(ResultError::ReadMapFailed)?;

    let mut diagnostics = DiagnosticSink::new(diagnostics);
    let mut context = Context::default();
    context.add_include_path(case);
    context.add_include_path(&case.join("extra-includes"));
    context.add_include_path("./include".as_ref());
    let map = context
        .parse_keymap(&mut diagnostics, Some(&map_path), &map)
        .map_err(ResultError::ParsingFailed)?;
    let builder = map.to_builder();
    let state_machine = builder.build_state_machine();
    let lookup_table = builder.build_lookup_table();
    let mut state = State::default();

    let mut actual = String::new();

    let input_path = case.join("input.txt");
    let input = std::fs::read_to_string(&input_path).map_err(ResultError::ReadInputFailed)?;
    let mut mods = ModifierMask(0);
    let group = 0;
    let mut events = vec![];
    let mut repeating_key = None;
    let key_name = |code: Keycode| {
        let (name, kc) = CODE_TO_NAME[&code.0];
        if kc != code.0 {
            return Err(ResultError::InvalidKeyCode(kc));
        }
        Ok(name)
    };
    let handle_down = |actual: &mut String,
                       kc: Keycode,
                       repeating_key: &mut Option<Keycode>,
                       mods: ModifierMask,
                       is_repeat: bool| {
        writeln!(
            actual,
            "key_{}({})",
            if is_repeat { "repeat" } else { "down" },
            key_name(kc)?,
        )
        .unwrap();
        let lookup = lookup_table.lookup(group, mods, kc);
        println!("{:?}", lookup);
        if lookup.repeats() {
            *repeating_key = Some(kc);
        }
        for p in lookup {
            write!(actual, "sym = {:?}", p.keysym()).unwrap();
            if let Some(char) = p.char() {
                write!(actual, ", char = {:?}", char).unwrap();
            }
            writeln!(actual).unwrap();
        }
        Ok(())
    };
    for mut line in input.lines() {
        if let Some((pre, _)) = line.split_once("#") {
            line = pre;
        }
        line = line.trim();
        if line.is_empty() {
            continue;
        }
        let mut arg = "";
        let command = match line.split_once(' ') {
            Some((c, a)) => {
                arg = a;
                c
            },
            _ => line,
        };
        let command = command.trim();
        let arg = arg.trim();
        let get_key = || {
            let (kn, kc) = NAME_TO_CODE[arg];
            if arg != kn {
                return Err(ResultError::InvalidKeyName(arg.to_string()));
            }
            Ok(Keycode(kc))
        };
        events.clear();
        let mut key = |down: bool| {
            state_machine.handle_key(&mut state, &mut events, get_key()?, down);
            Ok(())
        };
        match command {
            "down" => key(true)?,
            "up" => key(false)?,
            "both" => {
                key(true)?;
                key(false)?;
            }
            "repeat" => {
                if let Some(kc) = repeating_key {
                    handle_down(&mut actual, kc, &mut repeating_key, mods, true)?;
                }
            }
            _ => return Err(ResultError::UnknownCommand(command.to_string())),
        }
        for event in events.drain(..) {
            match event {
                LogicalEvent::ModsPressed(m) => {
                    writeln!(actual, "mods_pressed = {m:?}").unwrap();
                }
                LogicalEvent::ModsLatched(m) => {
                    writeln!(actual, "mods_latched = {m:?}").unwrap();
                }
                LogicalEvent::ModsLocked(m) => {
                    writeln!(actual, "mods_locked = {m:?}").unwrap();
                }
                LogicalEvent::ModsEffective(m) => {
                    mods = m;
                    writeln!(actual, "mods_effective = {m:?}").unwrap();
                }
                LogicalEvent::KeyDown(kc) => {
                    handle_down(&mut actual, kc, &mut repeating_key, mods, false)?;
                }
                LogicalEvent::KeyUp(kc) => {
                    if repeating_key == Some(kc) {
                        repeating_key = None;
                    }
                    writeln!(actual, "key_up({})", key_name(kc)?).unwrap();
                }
            };
        }
    }

    let expected_path = case.join("expected.txt");
    let expected = match std::fs::read_to_string(&expected_path) {
        Ok(e) => e,
        Err(e) if e.kind() == ErrorKind::NotFound && WRITE_MISSING => {
            return std::fs::write(&expected_path, &actual)
                .map_err(ResultError::WriteExpectedFailed);
        }
        Err(e) => return Err(ResultError::ReadExpectedFailed(e)),
    };

    if actual != expected {
        let actual_path = match WRITE_FAILED {
            true => "expected.txt",
            false => "actual.txt",
        };
        let actual_path = case.join(actual_path);
        std::fs::write(&actual_path, &actual).map_err(ResultError::WriteActualFailed)?;
        return Err(ResultError::TextComparisonFailed);
    }

    Ok(())
}
