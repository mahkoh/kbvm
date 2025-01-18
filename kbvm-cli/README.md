# kbvm-cli

This crate provides the `kbvm` binary that can be used to compile and test XKB keymaps
and XCompose files without installing them.

## Usage

### `kbvm compile-xkb`

The `compile-xkb` sub-command can be used to compile an existing XKB keymap to a
self-contained, fully-resolved XKB keymap.

```console
$ cat map.xkb
xkb_keymap {
    xkb_keycodes {
        include "evdev"
    };
    xkb_compat {
        include "complete"
    };
    xkb_symbols {
        key <AC01> { [ a, A ] };
        key <LFSH> { [ Shift_L ] };
    };
};
$ kbvm compile-xkb map.xkb
xkb_keymap {
    xkb_keycodes {
        minimum = 8;
        maximum = 255;

        // ...

        <AC01> = 38;
        <LFSH> = 50;
    };

    xkb_types {
        // ...

        type "ALPHABETIC" {
            modifiers = Shift+Lock;
            level_name[Level1] = "Base";
            level_name[Level2] = "Caps";
            map[Shift] = Level2;
            map[Lock] = Level2;
        };

        type "ONE_LEVEL" {
            modifiers = None;
            level_name[Level1] = "Any";
            map[None] = Level1;
        };
    };

    xkb_compat {
        // ...
    };

    xkb_symbols {
        key.repeat = true;

        key <AC01> {
            type[Group1] = "ALPHABETIC",
            symbols[Group1] = [ a, A ]
        };
        key <LFSH> {
            repeat = false,
            type[Group1] = "ONE_LEVEL",
            symbols[Group1] = [ Shift_L ],
            actions[Group1] = [ SetMods(modifiers = Shift) ]
        };
    };
};
```
