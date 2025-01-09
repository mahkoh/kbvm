#[macro_use]
mod macros;
#[cfg(test)]
mod formatter;
pub mod lexer;
pub mod parser;
pub mod resolver;
mod token;

pub struct Group<'a> {
    pub layout: &'a str,
    pub variant: &'a str,
}

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

#[cfg(test)]
mod tests {
    use crate::xkb::{rmlvo, Context};

    #[test]
    fn test() {
        let context = Context::builder().build();
        let mut diag = vec![];
        let out = context.expand_names(
            &mut diag,
            None,
            None,
            Some(&[rmlvo::Group {
                layout: "de",
                variant: "neo",
            }]),
            None,
        );
        println!("{:#}", out);
    }
}
