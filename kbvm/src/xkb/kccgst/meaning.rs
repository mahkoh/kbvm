pub(crate) use generated::Meaning;
use {
    crate::{
        index_map::IndexMap,
        xkb::{
            interner::{Interned, Interner},
            kccgst::meaning::generated::LONGEST,
        },
    },
    arrayvec::ArrayVec,
    bstr::ByteSlice,
    hashbrown::{hash_map::Entry, HashMap},
};

mod generated;

#[derive(Default)]
pub struct MeaningCache {
    sensitive: HashMap<Interned, Meaning>,
    insensitive: HashMap<Interned, Meaning>,
}

impl MeaningCache {
    pub fn get_case_sensitive(&mut self, interner: &Interner, name: Interned) -> Meaning {
        match self.sensitive.entry(name) {
            Entry::Occupied(e) => *e.get(),
            Entry::Vacant(e) => {
                let val = interner.get(name);
                let meaning = generated::STRING_TO_MEANING[val.as_bytes()];
                let res = match meaning.name().as_bytes() == val.as_bytes() {
                    true => meaning,
                    _ => Meaning::__Unknown,
                };
                e.insert(res);
                res
            }
        }
    }

    pub fn get_case_insensitive(&mut self, interner: &Interner, name: Interned) -> Meaning {
        match self.sensitive.entry(name) {
            Entry::Occupied(e) => *e.get(),
            Entry::Vacant(e) => {
                let val = interner.get(name);
                let res = if val.len() > LONGEST {
                    Meaning::__Unknown
                } else {
                    let mut buffer = ArrayVec::<u8, LONGEST>::new();
                    for &c in val.as_bytes() {
                        buffer.push(c.to_ascii_lowercase());
                    }
                    let meaning = generated::LOWERCASE_TO_MEANING[buffer.as_bytes()];
                    match meaning.lowercase().as_bytes() == buffer.as_bytes() {
                        true => meaning,
                        _ => Meaning::__Unknown,
                    }
                };
                e.insert(res);
                res
            }
        }
    }
}
