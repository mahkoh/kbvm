macro_rules! keyed_bitfield {
    (
        $name:ident:
        $($idx:expr => $ident:ident => $s:ident $(| $so:ident)*,)*
    ) => {
        const _: () = {
            use std::fmt::{self, Display, Formatter, Debug};
            use std::ops::{BitOr, Not, BitAnd, BitOrAssign};
            use crate::xkb::meaning::Meaning;

            impl $name {
                pub(crate) const NONE: Self = Self(0);
                pub(crate) const ALL: Self = Self($((1 << $idx))|*);
                $(
                    pub(crate) const $ident: Self = Self(1 << $idx);
                )*

                pub(crate) fn from_meaning(meaning: Meaning) -> Option<Self> {
                    let res = match meaning {
                        Meaning::None => Self::NONE,
                        Meaning::All => Self::ALL,
                        $(
                            Meaning::$s $(| Meaning::$so)* => Self::$ident,
                        )*
                        _ => return None,
                    };
                    Some(res)
                }

                #[allow(dead_code)]
                pub(crate) fn contains(self, other: Self) -> bool {
                    (self & other) == other
                }
            }

            impl Display for $name {
                fn fmt(&self, f: &mut Formatter) -> fmt::Result {
                    if self.0 == 0 {
                        return f.write_str("none");
                    }
                    let mut first = true;
                    $(
                        if self.0 & (1 << $idx) != 0 {
                            if !first {
                                f.write_str("+")?;
                            }
                            first = false;
                            f.write_str(stringify!($s))?;
                        }
                    )*
                    let _ = first;
                    Ok(())
                }
            }

            impl Debug for $name {
                fn fmt(&self, f: &mut Formatter) -> fmt::Result {
                    Display::fmt(self, f)
                }
            }

            impl BitOr for $name {
                type Output = Self;

                fn bitor(self, rhs: Self) -> Self::Output {
                    Self(self.0 | rhs.0)
                }
            }

            impl BitOrAssign for $name {
                fn bitor_assign(&mut self, rhs: Self) {
                    self.0 |= rhs.0;
                }
            }

            impl BitAnd for $name {
                type Output = Self;

                fn bitand(self, rhs: Self) -> Self::Output {
                    Self(self.0 & rhs.0)
                }
            }

            impl Not for $name {
                type Output = Self;

                fn not(self) -> Self::Output {
                    Self(Self::ALL.0 & !self.0)
                }
            }
        };

    };
}
