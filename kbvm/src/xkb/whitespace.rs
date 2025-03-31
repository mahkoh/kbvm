pub(crate) fn consume_whitespace(pos: &mut usize, c: &[u8], significant_newline: bool) {
    let mut p = *pos;
    while p < c.len() {
        let b0 = c[p];
        match b0 {
            0x0020 | 0x0009 => {
                // U+0020 - space
                // U+0009 - tab
                p += 1;
                continue;
            }
            0x000a if significant_newline => {
                // U+000a - line feed
                break;
            }
            0x000a..=0x000d => {
                // U+000a - line feed
                // U+000b - vertical tab
                // U+000c - form feed
                // U+000d - carriage return
                p += 1;
                continue;
            }
            0xc2 if p + 1 < c.len() => {
                let b1 = c[p + 1];
                if b1 == 0x85 {
                    // U+0085 - next line
                    p += 2;
                    continue;
                }
            }
            0xe2 if p + 2 < c.len() => {
                let b1 = c[p + 1];
                if b1 == 0x80 {
                    let b2 = c[p + 2];
                    if matches!(b2, 0x8e | 0x8f | 0xa8 | 0xa9) {
                        // U+200e - left-to-right mark
                        // U+200f - right-to-left mark
                        // U+2028 - line separator
                        // U+2029 - paragraph separator
                        p += 3;
                        continue;
                    }
                }
            }
            _ => {}
        }
        break;
    }
    *pos = p;
}
