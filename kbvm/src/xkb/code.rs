use {
    crate::xkb::code_slice::CodeSlice,
    bstr::ByteSlice,
    std::{
        fmt::{Debug, Formatter},
        ops::Deref,
        sync::Arc,
    },
};

#[derive(Clone, Eq, PartialEq)]
pub struct Code {
    code: Arc<Vec<u8>>,
}

impl Code {
    pub fn new(code: &Arc<Vec<u8>>) -> Self {
        Self { code: code.clone() }
    }

    pub fn as_bytes(&self) -> &[u8] {
        self.code.as_slice()
    }

    pub fn to_slice(&self) -> CodeSlice<'_> {
        CodeSlice::new_ref(self, 0..self.len())
    }
}

impl Deref for Code {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.as_bytes()
    }
}

impl Debug for Code {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        Debug::fmt(self.as_bstr(), f)
    }
}
