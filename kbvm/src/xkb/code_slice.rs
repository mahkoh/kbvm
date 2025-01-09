use {
    crate::xkb::code::Code,
    bstr::ByteSlice,
    std::{
        borrow::Borrow,
        fmt::{Debug, Formatter},
        hash::{Hash, Hasher},
        ops::{Deref, Range},
    },
};

#[derive(Clone)]
enum OwnedOrBorrowed<'a> {
    Owned(Code),
    Borrowed(&'a Code),
}

#[derive(Clone)]
pub(crate) struct CodeSlice<'a> {
    code: OwnedOrBorrowed<'a>,
    range: Range<usize>,
}

impl OwnedOrBorrowed<'_> {
    fn to_owned(&self) -> OwnedOrBorrowed<'static> {
        let c = match self {
            OwnedOrBorrowed::Owned(c) => c.clone(),
            OwnedOrBorrowed::Borrowed(c) => (*c).clone(),
        };
        OwnedOrBorrowed::Owned(c)
    }
}

impl<'a> CodeSlice<'a> {
    pub(crate) fn new_ref(code: &'a Code, range: Range<usize>) -> Self {
        #[cfg(debug_assertions)]
        let _ = code[range.clone()];
        Self {
            code: OwnedOrBorrowed::Borrowed(code),
            range,
        }
    }

    pub(crate) fn new_owned(code: &Code, range: Range<usize>) -> Self {
        CodeSlice::new_ref(code, range).to_owned()
    }

    pub(crate) fn code(&self) -> &Code {
        match &self.code {
            OwnedOrBorrowed::Owned(c) => c,
            OwnedOrBorrowed::Borrowed(c) => c,
        }
    }

    pub(crate) fn as_bytes(&self) -> &[u8] {
        &self.code()[self.range.clone()]
    }

    pub(crate) fn to_owned(&self) -> CodeSlice<'static> {
        CodeSlice {
            code: self.code.to_owned(),
            range: self.range.clone(),
        }
    }

    // pub(crate) fn borrow(&self) -> CodeSlice<'_> {
    //     CodeSlice::new_ref(self.code(), self.range.clone())
    // }

    pub(crate) fn slice(&self, range: Range<usize>) -> CodeSlice<'_> {
        CodeSlice::new_ref(
            self.code(),
            Range {
                start: self.range.start + range.start,
                end: self.range.start + range.end,
            },
        )
    }
}

impl AsRef<[u8]> for CodeSlice<'_> {
    fn as_ref(&self) -> &[u8] {
        self
    }
}

impl Deref for CodeSlice<'_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.as_bytes()
    }
}

impl Borrow<[u8]> for CodeSlice<'_> {
    fn borrow(&self) -> &[u8] {
        self.as_bytes()
    }
}

impl Debug for CodeSlice<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        Debug::fmt(self.as_bstr(), f)
    }
}

impl PartialEq for CodeSlice<'_> {
    fn eq(&self, other: &Self) -> bool {
        self.as_bytes() == other.as_bytes()
    }
}

impl Eq for CodeSlice<'_> {}

impl Hash for CodeSlice<'_> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.as_bytes().hash(state);
    }
}
