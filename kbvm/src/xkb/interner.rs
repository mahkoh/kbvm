use {
    crate::xkb::code_slice::CodeSlice,
    hashbrown::{Equivalent, HashMap, hash_map::EntryRef},
    kbvm_proc::CloneWithDelta,
    std::fmt::{Debug, Formatter},
};

#[derive(Default)]
pub(crate) struct Interner {
    id_to_string: Vec<CodeSlice<'static>>,
    string_to_id: HashMap<CodeSlice<'static>, Interned>,
}

#[derive(Copy, Clone, Eq, PartialEq, Hash, CloneWithDelta)]
pub(crate) struct Interned(usize);

impl Debug for Interned {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        Debug::fmt(&self.0, f)
    }
}

impl Debug for Interner {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let mut map = f.debug_map();
        for (k, v) in self.id_to_string.iter().enumerate() {
            map.entry(&k, v);
        }
        map.finish()
    }
}

impl Interner {
    pub(crate) fn intern(&mut self, code: &CodeSlice<'_>) -> Interned {
        #[derive(Hash)]
        struct Ref<'a, 'b>(&'a CodeSlice<'b>);
        impl Equivalent<CodeSlice<'static>> for Ref<'_, '_> {
            fn equivalent(&self, key: &CodeSlice<'static>) -> bool {
                self.0.as_bytes() == key.as_bytes()
            }
        }
        impl From<&Ref<'_, '_>> for CodeSlice<'static> {
            fn from(value: &Ref<'_, '_>) -> Self {
                value.0.to_owned()
            }
        }

        match self.string_to_id.entry_ref(&Ref(code)) {
            EntryRef::Occupied(e) => *e.get(),
            EntryRef::Vacant(v) => {
                let id = Interned(self.id_to_string.len());
                let key = v.insert_entry(id).key().clone();
                self.id_to_string.push(key);
                id
            }
        }
    }

    pub(crate) fn get(&self, id: Interned) -> &CodeSlice<'static> {
        &self.id_to_string[id.0]
    }

    pub(crate) fn get_existing(&self, name: &[u8]) -> Option<Interned> {
        self.string_to_id.get(name).copied()
    }
}
