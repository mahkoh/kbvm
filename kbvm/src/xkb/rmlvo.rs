//! RMLVO helpers and types.

#[macro_use]
mod macros;
#[cfg(test)]
mod formatter;
pub(crate) mod lexer;
pub(crate) mod parser;
pub(crate) mod resolver;
mod token;

/// An RMLVO group consisting of a layout and a variant.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct Group<'a> {
    /// The layout of the group.
    pub layout: &'a str,
    /// The variant of the group.
    pub variant: &'a str,
}

impl<'a> Group<'a> {
    /// Creates a Group iterator from comma-separated lists of layouts and variants.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::xkb::rmlvo::Group;
    /// let layout = "us,il,ru,de,jp";
    /// let variant = ",,phonetic,neo";
    /// let groups: Vec<_> = Group::from_layouts_and_variants(layout, variant).collect();
    /// assert_eq!(
    ///     groups,
    ///     [
    ///         Group { layout: "us", variant: "" },
    ///         Group { layout: "il", variant: "" },
    ///         Group { layout: "ru", variant: "phonetic" },
    ///         Group { layout: "de", variant: "neo" },
    ///         Group { layout: "jp", variant: "" },
    ///     ],
    /// );
    /// ```
    pub fn from_layouts_and_variants(
        layout: &'a str,
        variant: &'a str,
    ) -> impl Iterator<Item = Self> {
        let layouts = layout.split(',');
        let mut variants = variant.split(',');
        layouts.map(move |layout| Group {
            layout: layout.trim(),
            variant: variants.next().unwrap_or_default().trim(),
        })
    }
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
