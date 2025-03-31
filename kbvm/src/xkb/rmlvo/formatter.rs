use {
    crate::xkb::{
        interner::{Interned, Interner},
        rmlvo::parser::{Line, MappingKey, MappingKeyIndex, MappingValue, RuleKey},
    },
    std::io::{self, Write},
};

pub(crate) struct Formatter<'a, W> {
    interner: &'a Interner,
    out: W,
}

pub(crate) trait Format {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write;
}

impl<'a, W> Formatter<'a, W>
where
    W: Write,
{
    pub(crate) fn new(interner: &'a Interner, out: W) -> Self {
        Self { interner, out }
    }

    fn write_all(&mut self, s: &str) -> io::Result<()> {
        self.out.write_all(s.as_bytes())
    }

    fn write_interned(&mut self, interned: Interned) -> io::Result<()> {
        let s = self.interner.get(interned);
        self.out.write_all(s.as_bytes())?;
        Ok(())
    }
}

impl Format for Line<'_> {
    fn format<W>(&self, f: &mut Formatter<'_, W>) -> io::Result<()>
    where
        W: Write,
    {
        match self {
            Line::Macro(k, v) => {
                f.write_all("! $")?;
                f.write_interned(*k)?;
                f.write_all(" = ")?;
                for v in *v {
                    f.write_all(" ")?;
                    f.write_interned(*v)?;
                }
            }
            Line::Include(i) => {
                f.write_all("! include ")?;
                f.write_interned(*i)?;
            }
            Line::Mapping(k, v) => {
                f.write_all("!")?;
                for k in *k {
                    f.write_all(" ")?;
                    let write_index = |f: &mut Formatter<'_, W>, idx: &MappingKeyIndex| {
                        f.write_all("[")?;
                        match idx {
                            MappingKeyIndex::Single => f.write_all("single")?,
                            MappingKeyIndex::First => f.write_all("first")?,
                            MappingKeyIndex::Later => f.write_all("later")?,
                            MappingKeyIndex::Any => f.write_all("any")?,
                            MappingKeyIndex::Value(v) => write!(f.out, "{}", v.raw())?,
                        }
                        f.write_all("]")
                    };
                    match k {
                        MappingKey::Model => f.write_all("model")?,
                        MappingKey::Option => f.write_all("option")?,
                        MappingKey::Layout(l) => {
                            f.write_all("layout")?;
                            if let Some(idx) = l {
                                write_index(f, idx)?;
                            }
                        }
                        MappingKey::Variant(l) => {
                            f.write_all("variant")?;
                            if let Some(idx) = l {
                                write_index(f, idx)?;
                            }
                        }
                    }
                }
                f.write_all(" =")?;
                for v in *v {
                    f.write_all(" ")?;
                    let n = match v {
                        MappingValue::Keycodes => "keycodes",
                        MappingValue::Symbols => "symbols",
                        MappingValue::Types => "types",
                        MappingValue::Compat => "compat",
                        MappingValue::Geometry => "geometry",
                    };
                    f.write_all(n)?;
                }
            }
            Line::Rule(k, v) => {
                f.write_all(" ")?;
                for k in *k {
                    f.write_all(" ")?;
                    match k {
                        RuleKey::Star => f.write_all("*")?,
                        RuleKey::Macro(l) => {
                            f.write_all("$")?;
                            f.write_interned(*l)?;
                        }
                        RuleKey::Ident(l) => f.write_interned(*l)?,
                        RuleKey::Any => f.write_all("<any>")?,
                        RuleKey::Some => f.write_all("<some>")?,
                        RuleKey::None => f.write_all("<none>")?,
                    }
                }
                f.write_all(" =")?;
                for v in *v {
                    f.write_all(" ")?;
                    f.write_interned(v.ident.val)?;
                }
            }
        }
        f.write_all("\n")
    }
}
