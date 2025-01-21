#[rustfmt::skip]
mod generated;
#[path = "../../kbvm/src/phf.rs"]
mod phf;
#[path = "../../kbvm/src/phf_map.rs"]
mod phf_map;

use {
    crate::generated::{CODE_TO_NAME, NAME_TO_CODE},
    error_reporter::Report,
    integration_test_utils::run,
    isnt::std_1::vec::IsntVecExt,
    kbvm::{
        state_machine::{Direction, Event},
        xkb::{diagnostic::Diagnostic, Context},
        GroupIndex, Keycode, ModifierMask,
    },
    phf_map::PhfMap,
    std::{
        fmt::{Display, Formatter, Write},
        io::{self, ErrorKind},
        path::Path,
    },
    thiserror::Error,
};

// const SINGLE: Option<&str> = Some("t0099");
const SINGLE: Option<&str> = None;
const WRITE_MISSING: bool = true;
const WRITE_FAILED: bool = false;
const SHOW_ALL_DIAGNOSTICS_IF_SINGLE: bool = true;
const SHOW_ALL_DIAGNOSTICS: bool = false;

fn main() {
    std::env::set_current_dir(env!("CARGO_MANIFEST_DIR")).unwrap();
    let mut results = run(SINGLE, test_case2);
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
            if let ResultError::ParsingFailed(e) = err {
                write_diagnostic(e);
            }
        }
    }
    if any_failed {
        std::process::exit(1);
    }
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
    #[error("could not parse keymap")]
    ParsingFailed(#[source] Diagnostic),
    #[error("could not write expected file")]
    WriteExpectedFailed(#[source] io::Error),
    #[error("text comparison failed")]
    TextComparisonFailed,
    #[error("could not write actual file")]
    WriteActualFailed(#[source] io::Error),
}

enum NameOrKey {
    Name(&'static str),
    Key(u32),
}

impl Display for NameOrKey {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            NameOrKey::Name(n) => f.write_str(n),
            NameOrKey::Key(k) => write!(f, "0x{k:x}"),
        }
    }
}

fn test_case2(diagnostics: &mut Vec<Diagnostic>, case: &Path) -> Result<(), ResultError> {
    let map_path = case.join("map.xkb");
    let map = std::fs::read(&map_path).map_err(ResultError::ReadMapFailed)?;

    let mut context = Context::builder();
    context.clear();
    context.enable_environment(false);
    context.append_path(case);
    context.append_path(&case.join("extra-includes"));
    context.append_path("./include");
    let context = context.build();
    let map = context
        .keymap_from_bytes(diagnostics, Some(&map_path), &map)
        .map_err(ResultError::ParsingFailed)?;

    // println!("{:#}", map);

    let builder = map.to_builder();
    let state_machine = builder.build_state_machine();
    let lookup_table = builder.build_lookup_table();
    let mut state = state_machine.create_state();

    // println!("{:#?}", state_machine);

    let mut actual = String::new();

    let input_path = case.join("input.txt");
    let input = std::fs::read_to_string(&input_path).map_err(ResultError::ReadInputFailed)?;
    let mut mods = ModifierMask(0);
    let mut group = GroupIndex(0);
    let mut events = vec![];
    let mut repeating_key = None;
    let key_name = |code: Keycode| {
        let (name, kc) = CODE_TO_NAME[&code.raw()];
        if kc != code.raw() {
            return NameOrKey::Key(code.raw());
        }
        NameOrKey::Name(name)
    };
    let handle_down = |actual: &mut String,
                       kc: Keycode,
                       repeating_key: &mut Option<Keycode>,
                       group: GroupIndex,
                       mods: ModifierMask,
                       is_repeat: bool| {
        writeln!(
            actual,
            "    key_{}({})",
            if is_repeat { "repeat" } else { "down" },
            key_name(kc),
        )
        .unwrap();
        let lookup = lookup_table.lookup(group, mods, kc);
        // println!("{:?}", lookup.into_iter());
        if lookup.repeats() {
            *repeating_key = Some(kc);
        }
        for p in lookup {
            write!(actual, "    sym = {:?}", p.keysym()).unwrap();
            if let Some(char) = p.char() {
                write!(actual, ", char = {:?}", char).unwrap();
            }
            writeln!(actual).unwrap();
        }
        Ok(())
    };
    for mut line in input.lines() {
        writeln!(actual, "{}", line).unwrap();
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
            }
            _ => line,
        };
        let command = command.trim();
        let arg = arg.trim();
        let get_key = || {
            let (kn, kc) = NAME_TO_CODE[arg];
            if arg != kn {
                return Err(ResultError::InvalidKeyName(arg.to_string()));
            }
            Ok(Keycode::from_x11(kc))
        };
        events.clear();
        let mut key = |direction: Direction| {
            state_machine.handle_key(&mut state, &mut events, get_key()?, direction);
            Ok(())
        };
        match command {
            "down" => key(Direction::Down)?,
            "up" => key(Direction::Up)?,
            "both" => {
                key(Direction::Down)?;
                key(Direction::Up)?;
            }
            "repeat" => {
                if let Some(kc) = repeating_key {
                    handle_down(&mut actual, kc, &mut repeating_key, group, mods, true)?;
                }
            }
            _ => return Err(ResultError::UnknownCommand(command.to_string())),
        }
        for event in events.drain(..) {
            match event {
                Event::ModsPressed(m) => {
                    writeln!(actual, "    mods_pressed = {m:?}").unwrap();
                }
                Event::ModsLatched(m) => {
                    writeln!(actual, "    mods_latched = {m:?}").unwrap();
                }
                Event::ModsLocked(m) => {
                    writeln!(actual, "    mods_locked = {m:?}").unwrap();
                }
                Event::ModsEffective(m) => {
                    mods = m;
                    writeln!(actual, "    mods_effective = {m:?}").unwrap();
                }
                Event::KeyDown(kc) => {
                    handle_down(&mut actual, kc, &mut repeating_key, group, mods, false)?;
                }
                Event::KeyUp(kc) => {
                    if repeating_key == Some(kc) {
                        repeating_key = None;
                    }
                    writeln!(actual, "    key_up({})", key_name(kc)).unwrap();
                }
                Event::GroupPressed(g) => {
                    writeln!(actual, "    group_pressed = {g:?}").unwrap();
                }
                Event::GroupLatched(g) => {
                    writeln!(actual, "    group_latched = {g:?}").unwrap();
                }
                Event::GroupLocked(g) => {
                    writeln!(actual, "    group_locked = {g:?}").unwrap();
                }
                Event::GroupEffective(g) => {
                    group = g;
                    writeln!(actual, "    group_effective = {g:?}").unwrap();
                }
                Event::Controls(c) => {
                    writeln!(actual, "    controls = {c:?}").unwrap();
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
