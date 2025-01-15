use {
    error_reporter::Report,
    integration_test_utils::run,
    isnt::std_1::vec::IsntVecExt,
    kbvm::{
        keysym::Keysym,
        xkb::{compose::FeedResult, diagnostic::Diagnostic, Context},
    },
    std::{
        fmt::Write,
        io::{self, ErrorKind},
        path::Path,
    },
    thiserror::Error,
};

// const SINGLE: Option<&str> = Some("t0019");
const SINGLE: Option<&str> = None;
const WRITE_MISSING: bool = true;
const WRITE_FAILED: bool = false;
const SHOW_ALL_DIAGNOSTICS_IF_SINGLE: bool = true;
const SHOW_ALL_DIAGNOSTICS: bool = false;

fn main() {
    std::env::set_current_dir(env!("CARGO_MANIFEST_DIR")).unwrap();
    let mut results = run(SINGLE, |d, case| test_case2(d, case));
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
        }
    }
    if any_failed {
        std::process::exit(1);
    }
}

#[derive(Debug, Error)]
enum ResultError {
    #[error("could not read input file")]
    ReadInputFailed(#[source] io::Error),
    #[error("could not read expected file")]
    ReadExpectedFailed(#[source] io::Error),
    #[error("unknown keysym `{0}`")]
    UnknownKeysym(String),
    #[error("could not write expected file")]
    WriteExpectedFailed(#[source] io::Error),
    #[error("text comparison failed")]
    TextComparisonFailed,
    #[error("could not write actual file")]
    WriteActualFailed(#[source] io::Error),
    #[error("could not create compose table")]
    CreateComposeTable,
}

fn test_case2(diagnostics: &mut Vec<Diagnostic>, case: &Path) -> Result<(), ResultError> {
    let xcompose = case.join("XCompose");
    let xcompose_exists = xcompose.exists();
    let xcomposefile = match xcompose_exists {
        true => None,
        false => Some(case.join("XCOMPOSEFILE").display().to_string()),
    };
    let mut context = Context::builder();
    context.clear();
    context.enable_environment(true);
    context.environment_accessor(move |s| match s {
        "XLOCALEDIR" => Some("xlocaledir".to_string()),
        "HOME" => Some("home".to_string()),
        "XCOMPOSEFILE" => xcomposefile.clone(),
        _ => None,
    });
    let context = context.build();
    let mut builder = context.compose_table_builder();
    if xcompose_exists {
        builder.file(&xcompose);
    }
    let Some(table) = builder.build(diagnostics) else {
        return Err(ResultError::CreateComposeTable);
    };

    let mut actual = String::new();

    let input_path = case.join("input.txt");
    let input = std::fs::read_to_string(&input_path).map_err(ResultError::ReadInputFailed)?;

    let mut state = table.state();

    for mut line in input.lines() {
        writeln!(actual, "{}", line).unwrap();
        if let Some((pre, _)) = line.split_once("#") {
            line = pre;
        }
        line = line.trim();
        if line.is_empty() {
            continue;
        }
        let Some(keysym) = Keysym::from_str(line) else {
            return Err(ResultError::UnknownKeysym(line.to_string()));
        };
        let Some(res) = table.feed(&mut state, keysym) else {
            continue;
        };
        match res {
            FeedResult::Pending => actual.push_str("    pending\n"),
            FeedResult::Aborted => actual.push_str("    aborted\n"),
            FeedResult::Composed { string, keysym } => {
                actual.push_str("    composed\n");
                actual.push_str("       ");
                if let Some(s) = string {
                    actual.push_str(" ");
                    actual.push_str(s);
                }
                if let Some(k) = keysym {
                    write!(actual, " {:?}", k).unwrap();
                }
                actual.push_str("\n");
            }
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
