use {
    kbvm::xkb::diagnostic::Diagnostic,
    parking_lot::Mutex,
    std::{
        fs::read_dir,
        mem,
        path::{Path, PathBuf},
        sync::atomic::{AtomicUsize, Ordering::Relaxed},
        thread::available_parallelism,
    },
};

struct Results<E> {
    idx: AtomicUsize,
    cases: Vec<PathBuf>,
    results: Mutex<Vec<TestResult<E>>>,
}

pub struct TestResult<E> {
    pub case: PathBuf,
    pub diagnostics: Vec<Diagnostic>,
    pub result: Result<(), E>,
}

pub fn run<E, F>(single: Option<&str>, f: F) -> Vec<TestResult<E>>
where
    F: Fn(&mut Vec<Diagnostic>, &Path) -> Result<(), E> + Sync + Send,
    E: Send,
{
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
                if let Some(s) = single {
                    if f.file_name() != s {
                        continue;
                    }
                }
                cases.push(f.path());
            }
        }
    }
    let results = Results {
        idx: Default::default(),
        cases,
        results: Default::default(),
    };
    std::thread::scope(|scope| {
        for _ in 0..available_parallelism().unwrap().get() {
            scope.spawn(|| test_thread(&results, &f));
        }
    });
    let x = mem::take(&mut *results.results.lock());
    x
}

fn test_thread<E, F>(results: &Results<E>, f: F)
where
    F: Fn(&mut Vec<Diagnostic>, &Path) -> Result<(), E>,
{
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
        test_case(results, path, &f)
    }
}

fn test_case<E, F>(results: &Results<E>, case: &Path, f: F)
where
    F: Fn(&mut Vec<Diagnostic>, &Path) -> Result<(), E>,
{
    let mut diagnostics = Vec::new();
    let result = TestResult {
        result: f(&mut diagnostics, case),
        diagnostics,
        case: case.to_path_buf(),
    };
    results.results.lock().push(result);
}
