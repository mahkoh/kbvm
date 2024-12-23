use {
    error_reporter::Report,
    isnt::std_1::{primitive::IsntStrExt, vec::IsntVecExt},
    kbvm::xkb::{
        diagnostic::{Diagnostic, DiagnosticSink},
        Context, Element, Kccgst, Keymap, MergeMode, RmlvoGroup,
    },
    parking_lot::Mutex,
    serde::{Deserialize, Serialize},
    std::{
        fs::read_dir,
        io::{self, ErrorKind},
        path::{Path, PathBuf},
        sync::{
            atomic::{AtomicUsize, Ordering::Relaxed},
            Arc,
        },
        thread::available_parallelism,
    },
    thiserror::Error,
};

// const SINGLE: Option<&str> = Some("t0154");
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
            if let Some(s) = SINGLE {
                if f.file_name() != s {
                    continue;
                }
            }
            cases.push(f.path());
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
                ResultError::ParsingFailed(e) | ResultError::ParsingExpectedFailed(e) => {
                    write_diagnostic(e);
                }
                ResultError::MemoryComparisonFailed { expected, actual } => {
                    eprintln!("       expected {:?}", expected);
                    eprintln!("         actual {:?}", actual);
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
    #[error("could not read input file")]
    ReadInputFailed(#[source] io::Error),
    #[error("could not deserialize input file")]
    DeserializeInputFailed(#[source] toml::de::Error),
    #[error("could not deserialize expected file")]
    DeserializeExpectedFailed(#[source] toml::de::Error),
    #[error("could not parse keymap")]
    ParsingFailed(#[source] Diagnostic),
    #[error("could not parse expected keymap")]
    ParsingExpectedFailed(#[source] Diagnostic),
    #[error("could not write expected file")]
    WriteExpectedFailed(#[source] io::Error),
    #[error("could not read expected file")]
    ReadExpected(#[source] io::Error),
    #[error("text comparison failed")]
    TextComparisonFailed,
    #[error("memory comparison failed")]
    MemoryComparisonFailed { expected: Keymap, actual: Keymap },
    #[error("text comparison failed")]
    OutputComparisonFailed,
    #[error("round trip failed")]
    RoundTripFailed,
    #[error("could not write actual file")]
    WriteActualFailed(#[source] io::Error),
    #[error("could not write round-trip file")]
    WriteRoundTripFailed(#[source] io::Error),
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
    let input_path = case.join("input.xkb");
    if input_path.exists() {
        test_kccgst(diagnostics, case)
    } else {
        test_rmlvo(diagnostics, case)
    }
}

fn test_kccgst(diagnostics: &mut Vec<Diagnostic>, case: &Path) -> Result<(), ResultError> {
    let input_path = case.join("input.xkb");
    let input = std::fs::read(&input_path).map_err(ResultError::ReadInputFailed)?;

    let mut diagnostics = DiagnosticSink::new(diagnostics);
    let mut context = Context::default();
    context.add_include_path(case);
    context.add_include_path("./include".as_ref());
    let map_actual = context
        .parse_keymap(&mut diagnostics, Some(&input_path), &input)
        .map_err(ResultError::ParsingFailed)?;
    let map = format!("{:#}\n", map_actual);

    let expected_path = case.join("expected.xkb");
    let expected = match std::fs::read(&expected_path) {
        Ok(e) => Ok(e),
        Err(e) => {
            if e.kind() == ErrorKind::NotFound && WRITE_MISSING {
                return std::fs::write(&expected_path, &map)
                    .map_err(ResultError::WriteExpectedFailed);
            }
            Err(e)
        }
    };

    if Some(map.as_bytes()) != expected.as_deref().ok() {
        let actual = match WRITE_FAILED {
            true => "expected.xkb",
            false => "actual.xkb",
        };
        let actual = case.join(actual);
        std::fs::write(&actual, &map).map_err(ResultError::WriteActualFailed)?;
        return Err(ResultError::TextComparisonFailed);
    }

    let expected = expected.map_err(ResultError::ReadExpected)?;
    let map_expected = context
        .parse_keymap(&mut diagnostics, Some(&expected_path), &expected)
        .map_err(ResultError::ParsingExpectedFailed)?;

    if map_expected != map_actual {
        return Err(ResultError::MemoryComparisonFailed {
            expected: map_expected,
            actual: map_actual,
        });
    }

    let round_trip = format!("{:#}\n", map_expected);
    if round_trip.as_bytes() != expected {
        let rt = case.join("round_trip.xkb");
        std::fs::write(&rt, &round_trip).map_err(ResultError::WriteRoundTripFailed)?;
        return Err(ResultError::RoundTripFailed);
    }

    Ok(())
}

#[derive(Deserialize, Default)]
#[serde(default)]
struct RmlvoInputGroup {
    layout: String,
    variant: String,
}

#[derive(Deserialize, Default)]
#[serde(default)]
struct RmlvoInput {
    rules: String,
    model: String,
    options: Vec<String>,
    groups: Vec<RmlvoInputGroup>,
}

#[derive(Serialize, Deserialize, PartialEq, Default)]
#[serde(default)]
struct RmlvoOutput {
    keycodes: String,
    types: String,
    compat: String,
    symbols: String,
    geometry: String,
}

impl From<Kccgst> for RmlvoOutput {
    fn from(value: Kccgst) -> Self {
        let convert = |elements: &[Element]| {
            let mut res = String::new();
            for e in elements {
                if res.is_not_empty() {
                    let mm = match e.merge_mode {
                        MergeMode::Augment => "|",
                        MergeMode::Override => "+",
                    };
                    res.push_str(mm);
                }
                res.push_str(&e.include);
            }
            res
        };
        macro_rules! convert {
            ($($name:ident,)*) => {
                Self {
                    $($name: convert(&value.$name),)*
                }
            };
        }
        convert! {
            keycodes,
            types,
            compat,
            symbols,
            geometry,
        }
    }
}

fn test_rmlvo(diagnostics: &mut Vec<Diagnostic>, case: &Path) -> Result<(), ResultError> {
    let input_path = case.join("input.toml");
    let input = std::fs::read_to_string(&input_path).map_err(ResultError::ReadInputFailed)?;
    let input: RmlvoInput = toml::from_str(&input).map_err(ResultError::DeserializeInputFailed)?;

    let groups: Vec<_> = input
        .groups
        .iter()
        .map(|g| RmlvoGroup {
            layout: &g.layout,
            variant: &g.variant,
        })
        .collect();
    let options: Vec<_> = input.options.iter().map(|s| &**s).collect();

    let mut diagnostics = DiagnosticSink::new(diagnostics);
    let mut context = Context::default();
    context.add_include_path(case);
    context.add_include_path("./include".as_ref());
    let kccgst = context.rmlvo_to_kccgst(
        &mut diagnostics,
        &input.rules,
        &input.model,
        &groups,
        &options,
    );
    let output = RmlvoOutput::from(kccgst);

    let expected_path = case.join("expected.toml");
    let expected = match std::fs::read_to_string(&expected_path) {
        Ok(e) => toml::from_str::<RmlvoOutput>(&e).map_err(ResultError::DeserializeExpectedFailed),
        Err(e) => {
            if e.kind() == ErrorKind::NotFound && WRITE_MISSING {
                return std::fs::write(&expected_path, toml::to_string(&output).unwrap())
                    .map_err(ResultError::WriteExpectedFailed);
            }
            Err(ResultError::ReadExpected(e))
        }
    };

    if Some(&output) != expected.as_ref().ok() {
        let actual = match WRITE_FAILED {
            true => "expected.toml",
            false => "actual.toml",
        };
        let actual = case.join(actual);
        std::fs::write(&actual, toml::to_string(&output).unwrap())
            .map_err(ResultError::WriteActualFailed)?;
        return Err(ResultError::OutputComparisonFailed);
    }

    Ok(())
}
