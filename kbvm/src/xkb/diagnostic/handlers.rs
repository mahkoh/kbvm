use crate::xkb::diagnostic::{Diagnostic, DiagnosticHandler, DiagnosticKind, Severity};

impl DiagnosticHandler for Vec<Diagnostic> {
    fn filter(&self, _kind: DiagnosticKind, _is_fatal: bool) -> bool {
        true
    }

    fn handle(&mut self, diag: Diagnostic) {
        self.push(diag);
    }
}

impl<T> DiagnosticHandler for &'_ mut T
where
    T: ?Sized + DiagnosticHandler,
{
    fn filter(&self, kind: DiagnosticKind, is_fatal: bool) -> bool {
        T::filter(*self, kind, is_fatal)
    }

    fn handle(&mut self, diag: Diagnostic) {
        T::handle(self, diag);
    }
}

impl<T, F> DiagnosticHandler for (T, F)
where
    T: DiagnosticHandler,
    F: Fn(DiagnosticKind, bool) -> bool,
{
    fn filter(&self, kind: DiagnosticKind, is_fatal: bool) -> bool {
        self.1(kind, is_fatal) && self.0.filter(kind, is_fatal)
    }

    fn handle(&mut self, diag: Diagnostic) {
        self.0.handle(diag);
    }
}

impl<T> DiagnosticHandler for (T, Severity)
where
    T: DiagnosticHandler,
{
    fn filter(&self, kind: DiagnosticKind, is_fatal: bool) -> bool {
        kind.severity() >= self.1 && self.0.filter(kind, is_fatal)
    }

    fn handle(&mut self, diag: Diagnostic) {
        self.0.handle(diag);
    }
}

#[cfg(feature = "log")]
pub(crate) mod log {
    use {
        crate::xkb::diagnostic::{Diagnostic, DiagnosticHandler, DiagnosticKind, Severity},
        log::Level,
    };

    /// A simple [`DiagnosticHandler`] that forwards diagnostics to the `log` crate.
    ///
    /// Messages will be formatted using [`Diagnostic::with_code`] with a level based on
    /// the [`Severity`] and using `"kbvm"` as its target.
    ///
    /// # Example
    ///
    /// ```txt
    /// [2025-01-09T11:29:38.647Z ERROR kbvm] at <anonymous file> 335:8: unknown action:
    /// >>         invalid statement;
    ///            ^~~~~~~~~~~~~~~~~~
    /// ```
    #[derive(Copy, Clone, Debug)]
    pub struct WriteToLog;

    fn level(kind: DiagnosticKind) -> Level {
        match kind.severity() {
            Severity::Debug => Level::Debug,
            Severity::Warning => Level::Warn,
            Severity::Error => Level::Error,
        }
    }

    const TARGET: &str = "kbvm";

    impl DiagnosticHandler for WriteToLog {
        fn filter(&self, kind: DiagnosticKind, _is_fatal: bool) -> bool {
            log::log_enabled!(target: TARGET, level(kind))
        }

        fn handle(&mut self, diag: Diagnostic) {
            log::log!(target: TARGET, level(diag.kind), "{}", diag.with_code());
        }
    }
}

pub(crate) mod stderr {
    use crate::xkb::diagnostic::{Diagnostic, DiagnosticHandler};

    /// A simple [`DiagnosticHandler`] that forwards diagnostics to STDERR.
    ///
    /// Messages will be formatted using [`Diagnostic::with_code`].
    ///
    /// # Example
    ///
    /// ```txt
    /// at <anonymous file> 335:8: unknown action:
    /// >>         invalid statement;
    ///            ^~~~~~~~~~~~~~~~~~
    /// ```
    #[derive(Copy, Clone, Debug)]
    pub struct WriteToStderr;

    impl DiagnosticHandler for WriteToStderr {
        fn handle(&mut self, diag: Diagnostic) {
            eprintln!("{}", diag.with_code());
        }
    }
}
