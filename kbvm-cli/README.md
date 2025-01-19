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

#### Include Paths

The following include paths are used by default:

- `$HOME/.config/xkb`
- `$HOME/.xkb`
- `/etc/xkb`
- `/usr/share/X11/xkb`

This can be disabled by using the `--no-default-includes` flag. You can add additional
include paths by using the `--prepend-include <PATH>` and `--append-include <PATH>`
parameters.

### `kbvm compile-rmlvo`

The `compile-rmlvo` subcommand can be used to turn a list of RMLVO names into a
self-contained, fully-resolved XKB keymap.

```console
$ kbvm compile-rmlvo --layout us,de --variant ,neo --options grp:ctrl_space_toggle
xkb_keymap {
    xkb_keycodes {
        // ...
    };

    xkb_types {
        // ...
    };

    xkb_compat {
        // ...
    };

    xkb_symbols {
        // ...

        name[Group1] = "English (US)";
        name[Group2] = "German (Neo 2)";

        key.repeat = true;

        key <AB01> {
            type[Group1] = "ALPHABETIC",
            type[Group2] = "EIGHT_LEVEL_ALPHABETIC_WITH_LEVEL5_LOCK",
            symbols[Group1] = [ z, Z ],
            symbols[Group2] = [ udiaeresis, Udiaeresis, numbersign, NoSymbol, Escape, Escape, union ]
        };
        // ...
        key <SPCE> {
            type[Group1] = "PC_CONTROL_LEVEL2",
            type[Group2] = "PC_CONTROL_LEVEL2",
            symbols[Group1] = [ space, ISO_Next_Group ],
            actions[Group1] = [ NoAction(), LockGroup(group = +1) ],
            symbols[Group2] = [ space, ISO_Next_Group, space, nobreakspace, KP_0, KP_0, U202f ],
            actions[Group2] = [ NoAction(), LockGroup(group = +1), NoAction(), NoAction(), NoAction(), NoAction(), NoAction() ]
        };
        // ...
    };
};
```

#### Include Paths

See the same section for the `compile-xkb` command.

### `kbvm expand-rmlvo`

The `expand-rmlvo` subcommand can be used to expand a list of RMLVO names to an XKB keymap
that is not yet self contained.

```console
~$ kbvm expand-rmlvo --layout us,de --variant ,neo --options grp:ctrl_space_toggle
xkb_keymap {
    xkb_keycodes {
        override "evdev"
        override "aliases(qwerty)"
    };
    xkb_types {
        override "complete"
    };
    xkb_compat {
        override "complete"
        override "caps(caps_lock):2"
        override "misc(assign_shift_left_action):2"
        override "level5(level5_lock):2"
    };
    xkb_symbols {
        override "pc"
        override "us"
        override "de(neo):2"
        override "inet(evdev)"
        override "group(ctrl_space_toggle):1"
        override "group(ctrl_space_toggle):2"
    };
    xkb_geometry {
        override "pc(pc105)"
    };
};
```

#### Include Paths

See the same section for the `compile-xkb` command.

### `kbvm test-wayland`

The `test-wayland` subcommand can be used to test either the keymap sent by the compositor
or a completely different keymap without install it.

```console
~$ kbvm test-wayland
[0279709.670] key down A
[0279709.670]   sym a 'a'
[0279709.813] key up   A
[0279710.210] ------------
[0279710.210] key down LEFTSHIFT
[0279710.210]   sym Shift_L
[0279710.210] mods_pressed = 0x00000001
[0279710.210] mods         = 0x00000001
[0279710.373] key down O
[0279710.373]   sym O 'O'
[0279710.476] key up   O
[0279710.946] ------------
[0279710.946] key down E
[0279710.946]   sym E 'E'
[0279711.052] key up   E
[0279711.601] ------------
[0279711.601] key down U
[0279711.601]   sym U 'U'
[0279711.669] key up   U
[0279711.851] key up   LEFTSHIFT
[0279711.851] mods_pressed = 0x00000000
[0279711.851] mods         = 0x00000000
```

#### Testing Custom Keymaps

You can use the `--keymap <FILE>` parameter to use a custom keymap instead of the one
configured in your compositor.

```console
~$ kbvm expand-rmlvo --layout us,de --variant ,neo --options grp:ctrl_space_toggle | kbvm test-wayland --keymap -
[0279870.220] key down A
[0279870.220]   sym a 'a'
[0279870.306] key up   A
[0279870.451] key down B
[0279870.451]   sym b 'b'
[0279870.535] key up   B
[0279870.710] key down C
[0279870.710]   sym c 'c'
[0279870.774] key up   C
[0279871.610] ------------
[0279871.610] key down LEFTCTRL
[0279871.610]   sym Control_L
[0279871.610] mods_pressed = 0x00000004
[0279871.610] mods         = 0x00000004
[0279871.898] ------------
[0279871.898] key down SPACE
[0279871.898]   sym ISO_Next_Group
[0279871.898] group_locked  = 1
[0279871.898] group         = 1
[0279871.997] key up   SPACE
[0279872.539] ------------
[0279872.539] key up   LEFTCTRL
[0279872.539] mods_pressed = 0x00000000
[0279872.539] mods         = 0x00000000
[0279877.090] ------------
[0279877.090] key down BACKSLASH
[0279877.090]   sym ISO_Level3_Shift
[0279877.090] mods_pressed = 0x00000080
[0279877.090] mods         = 0x00000080
[0279877.551] ------------
[0279877.551] key down LEFTSHIFT
[0279877.551]   sym Shift_L
[0279877.551] mods_pressed = 0x00000081
[0279877.551] mods         = 0x00000081
[0279878.210] ------------
[0279878.210] key down R
[0279878.210]   sym Greek_chi 'χ'
[0279878.311] key up   R
[0279879.965] ------------
[0279879.965] key up   BACKSLASH
[0279879.965] mods_pressed = 0x00000001
[0279879.965] mods         = 0x00000001
[0279879.989] key up   LEFTSHIFT
[0279879.989] mods_pressed = 0x00000000
[0279879.989] mods         = 0x00000000
```

## License

kbvm-cli is free software licensed under the GNU General Public License v3.0.
