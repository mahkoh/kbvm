//! [`Keysym`] helpers.

#[rustfmt::skip]
#[allow(clippy::identity_op)]
pub(crate) mod generated;
#[cfg(test)]
mod tests;

use {
    crate::{
        from_bytes::FromBytes,
        keysym::{
            generated::{
                CHAR_TO_BESPOKE_IDX, DATAS, KEYSYM_TO_CHAR, KEYSYM_TO_IDX, KEYSYM_TO_LOWER_KEYSYM,
                KEYSYM_TO_UPPER_KEYSYM, LONGEST_NAME, LOWER_NAME_TO_IDX, NAMES,
            },
            hidden::Keysym,
        },
        phf_map::PhfMap,
        syms,
    },
    arrayvec::ArrayVec,
    generated::NAME_TO_IDX,
    std::{
        fmt::{Debug, Display, Formatter},
        ops::Range,
        str::FromStr,
    },
    thiserror::Error,
};

pub(crate) mod hidden {
    #[allow(unused_imports)]
    use std::fmt::{Debug, Display};

    /// A keysym.
    ///
    /// Keysyms are usually generated when a physical key is pressed. In KBVM, the process
    /// of converting key presses to keysyms is handled by the
    /// [`LookupTable`](crate::lookup::LookupTable) type.
    ///
    /// A keysym represents the logical output of a key press. For example,
    ///
    /// - [`syms::a`](crate::syms::a) - the lowercase letter `a`, usually generated when
    ///   pressing the physical `A` key.
    /// - [`syms::A`](crate::syms::A) - the uppercase letter `A`, usually generated when
    ///   pressing the physical `A` key while holding the physical `Shift` key.
    /// - [`syms::BackSpace`](crate::syms::BackSpace)
    /// - [`syms::Return`](crate::syms::Return)
    ///
    /// The [`syms`](crate::syms) module contains pre-defined constants for all well-known
    /// keysyms. These keysyms all have a name assigned that can be accessed by calling
    /// [`Keysym::name`]. This name is identical to the name of the constant (except that
    /// constants for keysyms that start with a digit have a `_` prefix).
    ///
    /// In addition to the keysyms from the `syms` module, all `char`s can be encoded as
    /// keysyms by calling [`Keysym::from_char`]. Most of these keysyms do not have names
    /// assigned and [`Keysym::name`] returns `None`.
    ///
    /// # Formatting and Parsing
    ///
    /// Keysyms can be created from strings by calling [`Keysym::from_str`] or
    /// [`Keysym::from_str_insensitive`]. The keysym can be turned back into a string
    /// by using the [`Display`] implementation.
    ///
    /// The `Display` implementation guarantees that the output can once again be parsed
    /// by calling [`Keysym::from_str`]. It produces 3 kinds of outputs:
    ///
    /// - If the keysym has a name, that name is printed.
    /// - Otherwise, if the keysym encodes a Unicode code point, it prints `Uxxxx` where
    ///   `xxxx` is the hexadecimal representation of the code point and not necessarily
    ///   4 characters long.
    /// - Otherwise, it prints `0xXXXXXXXX` where `XXXXXXXX` is simply the hexadecimal
    ///   representation of the `u32`.
    ///
    /// This type additionally implements [`Debug`]. It produces 3 kinds of outputs:
    ///
    /// - If the keysym has a name, that name is printed.
    /// - Otherwise, if the keysym can be converted to a char via [`Keysym::char`], that
    ///   char is printed with `Debug::format`.
    /// - Otherwise, it prints `0xXXXXXXXX` as for the `Display` implementation.
    #[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Default)]
    pub struct Keysym(pub u32);
}

const HAS_CHAR: u8 = 1 << 0;
const KEYSYM_IS_CHAR: u8 = 1 << 1;
const IS_LOWER: u8 = 1 << 2;
const IS_UPPER: u8 = 1 << 3;
const IS_SECONDARY_IDX: u8 = 1 << 4;

#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
#[repr(packed)]
struct KeysymData {
    keysym_or_definitive_idx: u32,
    name_start: u16,
    name_len: u8,
    flags: u8,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
#[repr(packed)]
struct KeysymChar {
    keysym: u16,
    char: char,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
#[repr(packed)]
struct KeysymCaseMapping {
    keysym: u16,
    other: u32,
}

/// The error returned by [`Keysym::from_str`](FromStr::from_str).
#[derive(Debug, Error)]
#[error("The keysym name is unknown")]
pub struct UnknownKeysymName;

/// An iterator over all well-known keysyms.
///
/// You can create this iterator via [`Keysym::all`].
///
/// This iterator returns those keysyms that have an assigned name. This does not include
/// all valid keysyms. In particular, all 150 thousand Unicode code points have valid
/// keysym representations but this iterator only returns around 2500 keysyms.
#[derive(Clone)]
pub struct Keysyms {
    idx: Range<u16>,
}

impl KeysymData {
    fn name(&self) -> &'static str {
        let start = self.name_start as usize;
        let end = start + self.name_len as usize;
        &NAMES[start..end]
    }
}

impl Iterator for Keysyms {
    type Item = Keysym;

    fn next(&mut self) -> Option<Self::Item> {
        for idx in self.idx.by_ref() {
            let data = &DATAS[idx as usize];
            if data.flags & IS_SECONDARY_IDX == 0 {
                return Some(Keysym(data.keysym_or_definitive_idx));
            }
        }
        None
    }
}

impl DoubleEndedIterator for Keysyms {
    fn next_back(&mut self) -> Option<Self::Item> {
        while let Some(idx) = self.idx.next_back() {
            let data = &DATAS[idx as usize];
            if data.flags & IS_SECONDARY_IDX == 0 {
                return Some(Keysym(data.keysym_or_definitive_idx));
            }
        }
        None
    }
}

macro_rules! case_change {
    ($slf:ident, $ascii:ident, $unicode:ident, $map:ident) => {
        if $slf.0 <= 0xffff {
            if $slf.0 < 0x80 {
                return Keysym(($slf.0 as u8).$ascii() as u32);
            }
            let mapping = $map[&$slf.0];
            if mapping.keysym as u32 == $slf.0 {
                Self(mapping.other)
            } else {
                $slf
            }
        } else if $slf.0 >> 24 == 0x01 {
            #[cold]
            fn convert_unicode(sym: Keysym) -> Keysym {
                let Some(c) = char::from_u32(sym.0 & 0xff_ff_ff) else {
                    return sym;
                };
                // None of the case conversion functions are inline. Therefore there is
                // at least one function call here. By using rfold, we avoid additional
                // function calls.
                let mut count = 0;
                let cased = c.$unicode().rfold(c, |_, y| {
                    count += 1;
                    y
                });
                if count != 1 {
                    return sym;
                }
                let cased = cased as u32;
                if cased < 0x100 {
                    return Keysym(cased);
                }
                Keysym(0x01_00_00_00 | cased)
            }
            convert_unicode($slf)
        } else {
            $slf
        }
    };
}

impl Keysym {
    /// Returns an iterator over all well-known keysyms.
    ///
    /// See the documentation of [`Keysyms`] for more details.
    pub fn all() -> Keysyms {
        Keysyms {
            idx: 0..DATAS.len() as u16,
        }
    }

    /// Creates a keysym from a `char`.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::Keysym;
    /// let keysym = Keysym::from_char('ァ');
    /// assert_eq!(keysym.name().unwrap(), "kana_a");
    /// assert_eq!(keysym.char().unwrap(), 'ァ');
    /// ```
    pub fn from_char(char: char) -> Self {
        let c = char as u32;
        if matches!(c, 0x20..=0x7e | 0xa0..=0xff) {
            return Self(c);
        }
        if matches!(c, 0x08..=0x0b | 0x0d | 0x1b) {
            return Self(c | 0xff00);
        }
        if matches!(c, 0x7f) {
            return syms::Delete;
        }
        if matches!(c, 0xfdd0..=0xfdef) {
            return syms::NoSymbol;
        }
        if matches!(c & 0xffff, 0xfffe..=0xffff) {
            return syms::NoSymbol;
        }
        let idx = CHAR_TO_BESPOKE_IDX[&char];
        let data = &DATAS[idx as usize];
        if data.flags & HAS_CHAR != 0 {
            let sym = Self(data.keysym_or_definitive_idx);
            if data.flags & KEYSYM_IS_CHAR != 0 && sym.0 == char as u32 {
                return sym;
            }
            let sym16 = sym.0 as u16;
            let pos = KEYSYM_TO_CHAR.binary_search_by(|k| {
                let keysym = k.keysym;
                keysym.cmp(&sym16)
            });
            if let Ok(pos) = pos {
                if KEYSYM_TO_CHAR[pos].char == char {
                    return Self(data.keysym_or_definitive_idx);
                }
            }
        }
        Self(0x01_00_00_00 | c)
    }

    fn data(self) -> Option<&'static KeysymData> {
        let idx = KEYSYM_TO_IDX[&self.0];
        let data = &DATAS[idx as usize];
        (data.keysym_or_definitive_idx == self.0).then_some(data)
    }

    /// Returns the name of the keysym if the keysym is a well-known keysym.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::syms;
    /// assert_eq!(syms::kana_a.name().unwrap(), "kana_a");
    /// ```
    pub fn name(self) -> Option<&'static str> {
        self.data().map(|i| i.name())
    }

    /// Returns the `char` corresponding to this keysym.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::syms;
    /// assert_eq!(syms::kana_a.char().unwrap(), 'ァ');
    /// ```
    pub fn char(self) -> Option<char> {
        let c = self.0;
        if matches!(c, 0x20..=0x7e | 0xa0..=0xff) {
            return Some(c as u8 as char);
        }
        if c >> 8 == 0xff {
            let c = c & 0xff;
            if matches!(c, 0x80) {
                return Some(' ');
            }
            if matches!(c, 0x08..=0x0b | 0x0d | 0x1b | 0x89 | 0x8d | 0xaa..=0xb9 | 0xbd | 0xff) {
                return Some((c & 0x7f) as u8 as char);
            }
        }
        if c > 0xffff {
            if c >> 24 == 0x01 {
                return char::from_u32(c & 0xff_ff_ff);
            }
            return None;
        }
        let c = c as u16;
        let pos = KEYSYM_TO_CHAR
            .binary_search_by(|k| {
                let keysym = k.keysym;
                keysym.cmp(&c)
            })
            .ok()?;
        Some(KEYSYM_TO_CHAR[pos].char)
    }

    /// Creates a keysym from a case-sensitive string.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::{syms, Keysym};
    /// assert_eq!(Keysym::from_str("kana_a").unwrap(), syms::kana_a);
    /// ```
    pub fn from_str(name: &(impl AsRef<[u8]> + ?Sized)) -> Option<Self> {
        from_str::<false>(name.as_ref())
    }

    /// Creates a keysym from a case-insensitive string.
    ///
    /// If the well-known names of two keysyms differ only by casing, the keysym with the
    /// larger name is returned. Here, *larger* is defined in terms of `str::cmp`.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::{syms, Keysym};
    /// assert_eq!(Keysym::from_str_insensitive("A").unwrap(), syms::a);
    /// assert_eq!(Keysym::from_str("A").unwrap(), syms::A);
    /// ```
    pub fn from_str_insensitive(name: &(impl AsRef<[u8]> + ?Sized)) -> Option<Self> {
        from_str::<true>(name.as_ref())
    }

    /// Returns the uppercase variant of this keysym.
    ///
    /// If this keysym does not have an uppercase variant or if it already uppercase,
    /// `self` is returned.
    ///
    /// Note that some Unicode code points have uppercase variants that cannot be
    /// represented as a single Unicode code point. In these cases this function also
    /// returns `self`. If you only about the text representation, consider first
    /// converting to `char` via [`Self::char`] and then using the standard library
    /// functions to do the case conversion.
    ///
    /// # Examples
    ///
    /// ```
    /// # use kbvm::{syms, Keysym};
    /// assert_eq!(syms::a.to_uppercase(), syms::A);
    /// ```
    ///
    /// ```
    /// # use kbvm::{syms, Keysym};
    /// let u1f80 = '\u{1f80}';
    /// assert_eq!(u1f80.to_uppercase().collect::<String>(), "\u{1f08}\u{399}");
    /// let sym = Keysym::from_char(u1f80);
    /// assert_eq!(sym.to_uppercase(), sym);
    /// ```
    pub fn to_uppercase(self) -> Self {
        case_change!(
            self,
            to_ascii_uppercase,
            to_uppercase,
            KEYSYM_TO_UPPER_KEYSYM
        )
    }

    /// Returns the lowercase variant of this keysym.
    ///
    /// If this keysym does not have an lowercase variant or if it already lowercase,
    /// `self` is returned.
    ///
    /// The warning about Unicode code points from [`Self::to_uppercase`] also applies to
    /// this function.
    ///
    /// # Examples
    ///
    /// ```
    /// # use kbvm::{syms, Keysym};
    /// assert_eq!(syms::A.to_lowercase(), syms::a);
    /// ```
    pub fn to_lowercase(self) -> Self {
        case_change!(
            self,
            to_ascii_lowercase,
            to_lowercase,
            KEYSYM_TO_LOWER_KEYSYM
        )
    }

    fn unicode_char(self) -> Option<char> {
        char::from_u32(self.0 & 0xff_ff_ff)
    }

    /// Returns whether this keysym is in the Unicode range.
    ///
    /// Keysyms encode Unicode code points in the range by setting the most significant
    /// byte to `0x01`. This function returns whether the most significant byte is `0x01`.
    ///
    /// Note that this function does not always return `true` if the keysym was created
    /// with [`Self::from_char`]. Many Unicode code points are represented by well-known
    /// keysyms outside of the Unicode range.
    ///
    /// Even if this function returns true, that does not mean that [`Self::char`] returns
    /// `Some`. For example `Keysym(0x01_ff_ff_ff).is_in_unicode_range()` returns true but
    /// `0xff_ff_ff` is not a valid Unicode code point.
    ///
    /// # Examples
    ///
    /// ```
    /// # use kbvm::{syms, Keysym};
    /// assert!(!syms::A.is_in_unicode_range());
    /// assert!(Keysym(0x01_00_00_41).is_in_unicode_range());
    /// ```
    pub fn is_in_unicode_range(self) -> bool {
        self.0 >> 24 == 0x01
    }

    /// Returns the negation of [`Self::is_in_unicode_range`].
    pub fn is_not_in_unicode_range(self) -> bool {
        !self.is_in_unicode_range()
    }

    /// Returns whether this keysym is well-known.
    ///
    /// This is equivalent to [`Self::name`] returning `Some`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use kbvm::{syms, Keysym};
    /// assert!(syms::A.is_well_known());
    /// ```
    pub fn is_well_known(self) -> bool {
        self.data().is_some()
    }

    /// Returns the negation of [`Self::is_well_known`].
    pub fn is_not_well_known(self) -> bool {
        !self.is_well_known()
    }

    /// Returns whether this keysym is valid.
    ///
    /// A keysym is valid if it is well-known or if is in the Unicode range and represents
    /// a valid Unicode code point.
    ///
    /// # Examples
    ///
    /// ```
    /// # use kbvm::{syms, Keysym};
    /// assert!(syms::A.is_valid());
    /// assert!(!Keysym(0x01_ff_ff_ff).is_valid());
    /// ```
    pub fn is_valid(self) -> bool {
        if self.is_in_unicode_range() {
            self.unicode_char().is_some()
        } else {
            self.is_well_known()
        }
    }

    /// Returns the negation of [`Self::is_valid`].
    pub fn is_invalid(self) -> bool {
        !self.is_valid()
    }

    /// Returns whether this is a lowercase keysym.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::syms;
    /// assert!(syms::a.is_lowercase());
    /// assert!(!syms::A.is_lowercase());
    /// assert!(!syms::Return.is_lowercase());
    /// ```
    pub fn is_lowercase(self) -> bool {
        if self.is_in_unicode_range() {
            match self.unicode_char() {
                None => false,
                Some(c) => c.is_lowercase(),
            }
        } else {
            match self.data() {
                None => false,
                Some(d) => d.flags & IS_LOWER != 0,
            }
        }
    }

    /// Returns whether this is an uppercase keysym.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::syms;
    /// assert!(syms::A.is_uppercase());
    /// assert!(!syms::a.is_uppercase());
    /// assert!(!syms::Return.is_uppercase());
    /// ```
    pub fn is_uppercase(self) -> bool {
        if self.is_in_unicode_range() {
            match self.unicode_char() {
                None => false,
                Some(c) => c.is_uppercase(),
            }
        } else {
            match self.data() {
                None => false,
                Some(d) => d.flags & IS_UPPER != 0,
            }
        }
    }

    /// Returns whether this is a keypad keysym.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::syms;
    /// assert!(syms::KP_0.is_keypad());
    /// assert!(!syms::a.is_keypad());
    /// ```
    pub fn is_keypad(self) -> bool {
        self >= syms::KP_Space && self <= syms::KP_Equal
    }

    /// Returns whether this is a modifier keysym.
    ///
    /// # Example
    ///
    /// ```
    /// # use kbvm::syms;
    /// assert!(syms::Shift_L.is_modifier());
    /// assert!(!syms::a.is_modifier());
    /// ```
    pub fn is_modifier(self) -> bool {
        (self >= syms::Shift_L && self <= syms::Hyper_R)
            || (self >= syms::ISO_Lock && self <= syms::ISO_Level5_Lock)
            || self == syms::Mode_switch
            || self == syms::Num_Lock
    }
}

fn from_str<const CASE_INSENSITIVE: bool>(s: &[u8]) -> Option<Keysym> {
    // This ensures that every valid string of the form Uxxxx or 0xXXXXXXXX is also
    // processed.
    debug_assert!(LONGEST_NAME >= 10);
    // +1 to handle the XF86_ case.
    if s.len() > LONGEST_NAME + 1 {
        return None;
    }
    let mut buf;
    let mut b = s;
    let mut map = &NAME_TO_IDX;
    if CASE_INSENSITIVE {
        buf = [0u8; LONGEST_NAME + 1];
        let bi = &mut buf[..b.len()];
        bi.copy_from_slice(b);
        bi.make_ascii_lowercase();
        b = bi;
        map = &LOWER_NAME_TO_IDX;
    }
    {
        let idx = map[b];
        let mut data = &DATAS[idx as usize];
        let name = data.name().as_bytes();
        let good = match CASE_INSENSITIVE {
            true => name.eq_ignore_ascii_case(s),
            false => name == s,
        };
        if good {
            if data.flags & IS_SECONDARY_IDX != 0 {
                data = &DATAS[data.keysym_or_definitive_idx as usize];
            }
            return Some(Keysym(data.keysym_or_definitive_idx));
        }
    }
    if s.len() > 1 && (b[0] == b'U' || (CASE_INSENSITIVE && b[0] == b'u')) {
        if let Some(cp) = u32::from_bytes_hex(&s[1..]) {
            let v = match cp {
                0x000000..=0x00001f => None,
                0x000020..=0x00007e => Some(cp),
                0x00007f..=0x00009f => None,
                0x0000a0..=0x0000ff => Some(cp),
                0x000100..=0x10ffff => Some(cp | 0x01_00_00_00),
                0x110000.. => None,
            };
            return v.map(Keysym);
        }
        return None;
    }
    if s.len() > 2 && &b[0..2] == b"0x" {
        if let Some(cp) = u32::from_bytes_hex(&s[2..]) {
            return Some(Keysym(cp));
        }
    }
    {
        const XF86_PREFIX: &[u8] = b"XF86_";
        let is_critical = s.len() > XF86_PREFIX.len()
            && match CASE_INSENSITIVE {
                true => s[..XF86_PREFIX.len()].eq_ignore_ascii_case(XF86_PREFIX),
                false => &s[..XF86_PREFIX.len()] == XF86_PREFIX,
            };
        if is_critical {
            let mut fixed = ArrayVec::<u8, LONGEST_NAME>::new();
            let _ = fixed.try_extend_from_slice(b"XF86");
            let _ = fixed.try_extend_from_slice(&s[XF86_PREFIX.len()..]);
            return from_str::<CASE_INSENSITIVE>(&fixed);
        }
    }
    None
}

impl FromStr for Keysym {
    type Err = UnknownKeysymName;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        from_str::<false>(s.as_bytes()).ok_or(UnknownKeysymName)
    }
}

impl Debug for Keysym {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        if let Some(name) = self.name() {
            return f.write_str(name);
        }
        if let Some(char) = self.char() {
            return Debug::fmt(&char, f);
        }
        write!(f, "0x{:08x}", self.0)
    }
}

impl Display for Keysym {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        if let Some(name) = self.name() {
            return f.write_str(name);
        }
        if self.is_in_unicode_range() {
            let d = self.0 & 0xff_ff_ff;
            return write!(f, "U{d:x}");
        }
        write!(f, "0x{:08x}", self.0)
    }
}
