use {
    crate::Keycode,
    hashbrown::HashMap,
    std::{collections::VecDeque, iter::Enumerate},
};

#[derive(Debug, Clone)]
pub(crate) struct KeyStorage<T> {
    storage: Storage<T>,
}

#[derive(Debug, Clone)]
enum Storage<T> {
    Vec(VecStorage<T>),
    Map(HashMap<Keycode, T>),
}

impl<T> KeyStorage<T> {
    pub(crate) fn new(mut map: HashMap<Keycode, T>) -> Self {
        let max = map.keys().copied().max().unwrap_or_default().0;
        if max > 0xfff {
            map.shrink_to_fit();
            return Self {
                storage: Storage::Map(map),
            };
        }
        let mut data: Vec<_> = std::iter::repeat_with(|| None)
            .take(max as usize + 1)
            .collect();
        let len = map.len();
        for (k, v) in map {
            data[k.0 as usize] = Some(Box::new(v));
        }
        Self {
            storage: Storage::Vec(VecStorage { len, data }),
        }
    }

    #[inline]
    pub(crate) fn get(&self, key: Keycode) -> Option<&T> {
        match &self.storage {
            Storage::Vec(v) => {
                if let Some(t) = v.data.get(key.0 as usize) {
                    return t.as_deref();
                }
                None
            }
            Storage::Map(m) => m.get(&key),
        }
    }

    pub(crate) fn len(&self) -> usize {
        match &self.storage {
            Storage::Vec(v) => v.len,
            Storage::Map(m) => m.len(),
        }
    }
}

#[derive(Debug, Clone)]
struct VecStorage<T> {
    len: usize,
    data: Vec<Option<Box<T>>>,
}

pub(crate) enum Iter<'a, T> {
    Vec {
        iter: Enumerate<std::slice::Iter<'a, Option<Box<T>>>>,
    },
    Map {
        iter: VecDeque<(Keycode, &'a T)>,
    },
}

impl<'a, T> Iterator for Iter<'a, T> {
    type Item = (Keycode, &'a T);

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            Iter::Vec { iter } => loop {
                if let (keycode, Some(el)) = iter.next()? {
                    let keycode = Keycode(keycode as u32);
                    return Some((keycode, el));
                }
            },
            Iter::Map { iter } => iter.pop_front(),
        }
    }
}

impl<'a, T> IntoIterator for &'a KeyStorage<T> {
    type Item = (Keycode, &'a T);
    type IntoIter = Iter<'a, T>;

    fn into_iter(self) -> Self::IntoIter {
        match &self.storage {
            Storage::Vec(a) => Iter::Vec {
                iter: a.data.iter().enumerate(),
            },
            Storage::Map(a) => {
                let mut iter: Vec<_> = a.iter().map(|k| (*k.0, k.1)).collect();
                iter.sort_unstable_by_key(|k| k.0);
                Iter::Map { iter: iter.into() }
            }
        }
    }
}
