use crate::{
    state_machine::{Direction, Event, Keycode},
    xkb::Context,
    GroupIndex, ModifierMask,
};

const KEY_CONTROL_L: Keycode = Keycode(37);
const KEY_Q: Keycode = Keycode(24);

#[test]
fn test() {
    let mut sink = vec![];
    let mut context = Context::builder();
    context.clear();
    context.enable_environment(false);
    let keymap = context
        .build()
        .keymap_from_bytes(&mut sink, None, MAP.as_bytes())
        .unwrap();
    let builder = keymap.to_builder();
    let state_machine = builder.build_state_machine();
    let lookup = builder.build_lookup_table();
    let mut state = state_machine.create_state();
    let mut effective_mods = ModifierMask::default();
    let mut key = |key: Keycode, down: bool| {
        let mut events = vec![];
        state_machine.handle_key(
            &mut state,
            &mut events,
            key,
            match down {
                false => Direction::Up,
                true => Direction::Down,
            },
        );
        for event in events {
            println!("{:#?}", event);
            match event {
                Event::KeyDown(kc) => {
                    let lookup = lookup.lookup(GroupIndex(0), effective_mods, kc);
                    println!("  {:?}", lookup);
                    for sym in lookup {
                        println!("  {:?}", sym);
                    }
                }
                Event::ModsEffective(mm) => effective_mods = mm,
                _ => {}
            }
        }
    };
    // key(KEY_CAPS, true);
    // key(KEY_CAPS, false);
    key(KEY_CONTROL_L, true);
    // key(KEY_CONTROL_L, false);
    key(KEY_Q, true);
    key(KEY_Q, false);
    // key(KEY_SHIFT_L, true);
    // key(KEY_Q, true);
    // key(KEY_Q, false);
    // key(KEY_SHIFT_L, false);
    // key(KEY_CONTROL_L, true);
    // key(KEY_CONTROL_L, false);
    // key(KEY_CONTROL_L, true);
    // key(KEY_CONTROL_L, false);
    // key(KEY_CONTROL_L, true);
    // key(KEY_CONTROL_L, false);
}

const MAP: &str = r#"
xkb_keymap {

    xkb_keycodes {

          <1> =   9; # ESC
          <2> =  10; # 1
          <3> =  11; # 2
          <4> =  12; # 3
          <5> =  13; # 4
          <6> =  14; # 5
          <7> =  15; # 6
          <8> =  16; # 7
          <9> =  17; # 8
         <10> =  18; # 9
         <11> =  19; # 0
         <12> =  20; # MINUS
         <13> =  21; # EQUAL
         <14> =  22; # BACKSPACE
         <15> =  23; # TAB
         <16> =  24; # Q
         <17> =  25; # W
         <18> =  26; # E
         <19> =  27; # R
         <20> =  28; # T
         <21> =  29; # Y
         <22> =  30; # U
         <23> =  31; # I
         <24> =  32; # O
         <25> =  33; # P
         <26> =  34; # LEFTBRACE
         <27> =  35; # RIGHTBRACE
         <28> =  36; # ENTER
         <29> =  37; # LEFTCTRL
         <30> =  38; # A
         <31> =  39; # S
         <32> =  40; # D
         <33> =  41; # F
         <34> =  42; # G
         <35> =  43; # H
         <36> =  44; # J
         <37> =  45; # K
         <38> =  46; # L
         <39> =  47; # SEMICOLON
         <40> =  48; # APOSTROPHE
         <41> =  49; # GRAVE
         <42> =  50; # LEFTSHIFT
         <43> =  51; # BACKSLASH
         <44> =  52; # Z
         <45> =  53; # X
         <46> =  54; # C
         <47> =  55; # V
         <48> =  56; # B
         <49> =  57; # N
         <50> =  58; # M
         <51> =  59; # COMMA
         <52> =  60; # DOT
         <53> =  61; # SLASH
         <54> =  62; # RIGHTSHIFT
         <55> =  63; # KPASTERISK
         <56> =  64; # LEFTALT
         <57> =  65; # SPACE
         <58> =  66; # CAPSLOCK
         <59> =  67; # F1
         <60> =  68; # F2
         <61> =  69; # F3
         <62> =  70; # F4
         <63> =  71; # F5
         <64> =  72; # F6
         <65> =  73; # F7
         <66> =  74; # F8
         <67> =  75; # F9
         <68> =  76; # F10
         <69> =  77; # NUMLOCK
         <70> =  78; # SCROLLLOCK
         <71> =  79; # KP7
         <72> =  80; # KP8
         <73> =  81; # KP9
         <74> =  82; # KPMINUS
         <75> =  83; # KP4
         <76> =  84; # KP5
         <77> =  85; # KP6
         <78> =  86; # KPPLUS
         <79> =  87; # KP1
         <80> =  88; # KP2
         <81> =  89; # KP3
         <82> =  90; # KP0
         <83> =  91; # KPDOT
         <87> =  95; # F11
         <88> =  96; # F12
         <89> =  97; # RO
         <92> = 100; # HENKAN
         <93> = 101; # KATAKANAHIRAGANA
         <94> = 102; # MUHENKAN
         <96> = 104; # KPENTER
         <97> = 105; # RIGHTCTRL
         <98> = 106; # KPSLASH
        <100> = 108; # RIGHTALT
        <102> = 110; # HOME
        <103> = 111; # UP
        <104> = 112; # PAGEUP
        <105> = 113; # LEFT
        <106> = 114; # RIGHT
        <107> = 115; # END
        <108> = 116; # DOWN
        <109> = 117; # PAGEDOWN
        <110> = 118; # INSERT
        <111> = 119; # DELETE
        <117> = 125; # KPEQUAL
        <119> = 127; # PAUSE
        <120> = 128; # SCALE
        <124> = 132; # YEN
        <125> = 133; # LEFTMETA
        <126> = 134; # RIGHTMETA
        <139> = 147; # MENU
        <183> = 191; # F13
        <184> = 192; # F14
        <185> = 193; # F15
        <186> = 194; # F16
        <187> = 195; # F17
        <188> = 196; # F18
        <189> = 197; # F19
        <190> = 198; # F20
        <191> = 199; # F21
        <192> = 200; # F22
        <193> = 201; # F23
        <194> = 202; # F24
        <210> = 218; # PRINT

        # We must include at least one indicator here. Otherwise Xwayland segfaults.
        indicator 1 = "DUMMY";
    };

    xkb_types {

        # We must include at least one virtual modifier.
        # Otherwise Xwayland rejects our keymap.
        virtual_modifiers Dummy;

        type "ONE_LEVEL" {
            modifiers = none;
            level_name[Level1] = "Base";
        };
        type "TWO_LEVEL" {
            modifiers  = Shift;
            map[Shift] = Level2;
            level_name[Level1] = "Base";
            level_name[Level2] = "Shift";
        };
        type "ALPHABETIC" {
            modifiers  = Shift+Lock;
            map[Shift] = Level2;
            map[Lock]  = Level2;
            level_name[Level1] = "Base";
            level_name[Level2] = "Shift";
        };
        type "KEYPAD" {
            modifiers = Shift+Mod2;
            map[Mod2] = Level2;
            level_name[Level1] = "Base";
            level_name[Level2] = "Shift";
        };
    };

    xkb_compatibility {

        interpret.repeat  = False;
        interpret.locking = False;
        interpret Shift_L {
            action = SetMods(modifiers=Shift);
        };
        interpret Shift_R {
            action = SetMods(modifiers=Shift);
        };
        interpret Caps_Lock {
            action = LockMods(modifiers=Lock);
        };
        interpret Control_L {
            action = LatchMods(modifiers=Control, latchToLock, clearLocks);
        };
        interpret Control_R {
            action = SetMods(modifiers=Control);
        };
        interpret Alt_L {
            action = SetMods(modifiers=Mod1);
        };
        interpret Alt_R {
            action = SetMods(modifiers=Mod1);
        };
        interpret Num_Lock {
            action = LockMods(modifiers=Mod2);
        };
        interpret Super_L {
            action = SetMods(modifiers=Mod4);
        };
        interpret Super_R {
            action = SetMods(modifiers=Mod4);
        };
    };

    xkb_symbols {

        key   <1> { [ Escape      ] };
        key  <59> { [ F1          ] };
        key  <60> { [ F2          ] };
        key  <61> { [ F3          ] };
        key  <62> { [ F4          ] };
        key  <63> { [ F5          ] };
        key  <64> { [ F6          ] };
        key  <65> { [ F7          ] };
        key  <66> { [ F8          ] };
        key  <67> { [ F9          ] };
        key  <68> { [ F10         ] };
        key  <87> { [ F11         ] };
        key  <88> { [ F12         ] };
        key <183> { [ F13         ] };
        key <184> { [ F14         ] };
        key <185> { [ F15         ] };
        key <186> { [ F16         ] };
        key <187> { [ F17         ] };
        key <188> { [ F18         ] };
        key <189> { [ F19         ] };
        key <190> { [ F20         ] };
        key <191> { [ F21         ] };
        key <192> { [ F22         ] };
        key <193> { [ F23         ] };
        key <194> { [ F24         ] };
        key <210> { [ Print       ] };
        key  <70> { [ Scroll_Lock ] };
        key <119> { [ Pause       ] };

        key  <69> { [ Num_Lock    ]          };
        key  <96> { [ KP_Enter    ]          };
        key  <98> { [ KP_Divide   ]          };
        key  <74> { [ KP_Subtract ]          };
        key  <55> { [ KP_Multiply ]          };
        key  <78> { [ KP_Add      ]          };
        key <117> { [ KP_Equal    ]          };
        key  <83> { [ KP_Delete,  KP_Decimal ]  };
        key  <71> { [ KP_Home,    KP_7       ]  };
        key  <72> { [ KP_Up,      KP_8       ]  };
        key  <73> { [ KP_Prior,   KP_9       ]  };
        key  <75> { [ KP_Left,    KP_4       ]  };
        key  <76> { [ KP_Begin,   KP_5       ]  };
        key  <77> { [ KP_Right,   KP_6       ]  };
        key  <79> { [ KP_End,     KP_1       ]  };
        key  <80> { [ KP_Down,    KP_2       ]  };
        key  <81> { [ KP_Next,    KP_3       ]  };
        key  <82> { [ KP_Insert,  KP_0       ]  };

        key <103> { [ Up    ] };
        key <105> { [ Left  ] };
        key <106> { [ Right ] };
        key <108> { [ Down  ] };

        key <102> { [ Home   ] };
        key <104> { [ Prior  ] };
        key <107> { [ End    ] };
        key <109> { [ Next   ] };
        key <110> { [ Insert ] };
        key <111> { [ Delete ] };

        key  <14> { [ BackSpace ]            };
        key  <15> { [ Tab,      ISO_Left_Tab ]  };
        key  <58> { [ Caps_Lock ]            };
        key  <28> { [ Return    ]            };
        key  <42> { [ Shift_L   ]            };
        key  <54> { [ Shift_R   ]            };
        // key  <29> { [ Control_L ]            };
        key  <29> {
            [ Control_L ],
            [ { SetMods(mods = Control), LatchMods(mods = Control, latchToLock, clearLocks) } ]
            };
        key <125> { [ Super_L   ]            };
        key  <56> { [ Alt_L     ]            };
        key  <57> { [ space     ]            };
        key <100> { [ Alt_R     ]            };
        key <126> { [ Super_R   ]            };
        key <139> { [ Menu      ]            };
        key  <97> { [ Control_R ]            };

        key <41> { [ grave,        asciitilde ] };
        key <12> { [ minus,        underscore ] };
        key <13> { [ equal,        plus       ] };
        key <26> { [ bracketleft,  braceleft  ] };
        key <27> { [ bracketright, braceright ] };
        key <43> { [ backslash,    bar        ] };
        key <39> { [ semicolon,    colon      ] };
        key <40> { [ apostrophe,   quotedbl   ] };
        key <51> { [ comma,        less       ] };
        key <52> { [ period,       greater    ] };
        key <53> { [ slash,        question   ] };

        key <16> { [ U8A9E ], [ q ] };
        key <17> { [ w, W ] };
        key <18> { [ e, E ] };
        key <19> { [ r, R ] };
        key <20> { [ t, T ] };
        key <21> { [ y, Y ] };
        key <22> { [ u, U ] };
        key <23> { [ i, I ] };
        key <24> { [ o, O ] };
        key <25> { [ p, P ] };
        key <30> { [ a, A ] };
        key <31> { [ s, S ] };
        key <32> { [ d, D ] };
        key <33> { [ f, F ] };
        key <34> { [ g, G ] };
        key <35> { [ h, H ] };
        key <36> { [ j, J ] };
        key <37> { [ k, K ] };
        key <38> { [ l, L ] };
        key <44> { [ z, Z ] };
        key <45> { [ x, X ] };
        key <46> { [ c, C ] };
        key <47> { [ v, V ] };
        key <48> { [ b, B ] };
        key <49> { [ n, N ] };
        key <50> { [ m, M ] };

        key  <2> { [ 1, exclam      ] };
        key  <3> { [ 2, at          ] };
        key  <4> { [ 3, numbersign  ] };
        key  <5> { [ 4, dollar      ] };
        key  <6> { [ 5, percent     ] };
        key  <7> { [ 6, asciicircum ] };
        key  <8> { [ 7, ampersand   ] };
        key  <9> { [ 8, asterisk    ] };
        key <10> { [ 9, parenleft   ] };
        key <11> { [ 0, parenright  ] };

        key <89> { [ dollar     , asciitilde  ] };
        key <93> { [ ampersand  , percent     ] };
        key <94> { [ equal   , asterisk   ] };
        key <124> { [ at         , asciicircum ] };
        key <92> { [ numbersign , grave       ] };
        key <120> { [ XF86Macro1       ] };

        modifier_map Mod1 { Alt_L, Alt_R };
    };

};
"#;
