#[macro_use]
mod macros;
#[cfg(test)]
mod formatter;
pub mod lexer;
pub mod parser;
pub mod resolver;
mod token;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum MergeMode {
    Augment,
    Override,
}

#[derive(Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub struct Element {
    pub merge_mode: MergeMode,
    pub include: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub struct Expanded {
    pub keycodes: Vec<Element>,
    pub types: Vec<Element>,
    pub compat: Vec<Element>,
    pub symbols: Vec<Element>,
    pub geometry: Vec<Element>,
}

mod tests {
    use crate::xkb::{diagnostic::DiagnosticSink, Context, RmlvoGroup};

    #[test]
    fn test() {
        let context = Context::builder().build();
        let mut diag = vec![];
        let out = context.rmlvo_to_kccgst(
            &mut DiagnosticSink::new(&mut diag),
            "evdev",
            "",
            &[RmlvoGroup {
                layout: "de",
                variant: "neo",
            }],
            &[],
        );
        println!("{:#}", out);
    }
}
