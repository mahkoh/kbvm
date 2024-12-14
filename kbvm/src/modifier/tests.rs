use crate::ModifierMask;

#[test]
fn xor() {
    let xor = |l, r, d| {
        assert_eq!(ModifierMask(l) ^ ModifierMask(r), ModifierMask(d));
        let mut x = ModifierMask(l);
        x ^= ModifierMask(r);
        assert_eq!(x, ModifierMask(d));
    };
    xor(0, 0, 0);
    xor(0, 1, 1);
    xor(1, 0, 1);
    xor(1, 1, 0);
}
