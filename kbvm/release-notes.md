# Unreleased

# 0.1.4 (2025-04-21)

- Fixed the following scenario:

  ```xkb
  xkb_types {
      virtual_modifiers a = 1;
      virtual_modifiers a;
  };
  ```
  
  This is now the same as

  ```xkb
  xkb_types {
      virtual_modifiers a = 1;
  };
  ```
  
  whereas previously it was the same as

  ```xkb
  xkb_types {
      virtual_modifiers a;
  };
  ```
- Fixed the following scenario:

  ```xkb
  xkb_keycodes {
      indicator 1 = "A";
      indicator 2 = "B";
      indicator 1 = "B";
  };
  ```
  
  Previously this would produce

  ```xkb
  xkb_keycodes {
      indicator 1 = "B";
      indicator 2 = "B";
  };
  ```
  
  Instead, this now produces

  ```xkb
  xkb_keycodes {
      indicator 1 = "B";
  };
  ```
- Fixed an arithmetic underflow when printing diagnostic messages containing
  non-ascii text.
- The following characters are now always treated as insignificant whitespace
  between tokens

  - U+0009 - tab
  - U+000b - vertical tab
  - U+000c - form feed
  - U+000d - carriage return
  - U+0020 - space
  - U+0085 - next line
  - U+200e - left-to-right mark
  - U+200f - right-to-left mark
  - U+2028 - line separator
  - U+2029 - paragraph separator

  In xkb files, the following character is also insignificant whitespace

  - U+000a - line feed

  In RMLVO and compose files, line feeds are significant.
- The character `^` can now be used to signify the `replace` merge mode
  wherever `+` and `|` could already be used.
- The expressions `<any>`, `<none>`, and `<some>` can now be used in RMLVO files
  in the same places where the wildcard `*` could already be used. They match
  always, if the needle is empty, and if the needle is non-empty, respectively.
- Strings can now contain unicode escape sequences of the form `\u{N}` where `N`
  should be a hexadecimal number. The escape sequence will be expanded to the
  UTF-8 representation of the unicode code point `U+N`.
- Fixed an error where certain keysyms would be formatted in a way that could
  not be parsed back. For example, the keysym `0x01000000` will now be formatted
  as `0x01000000` instead of `U0`.
- The `action` field in an `interpret` statement can now contain multiple
  actions written as a brace list. For example

  ```xkb
  xkb_compat {
      interpret a {
          action = { SetMods(mods = Mod1), SetGroup(group = 1) };
      };
  };
  ```
- Keysyms can now be specified as strings in all positions.

  ```xkb
  xkb_compat {
      interpret "日" {
          action = SetMods(mods = Mod1);
      };
  };
  xkb_symbols {
      key <a> { [ "日本語" ] };
      key <b> { [ { "日本語", Control_R } ] };
      modmap Mod1 { "日本語" };
  };
  ```

  Such strings must be UTF-8 encoded. The mapping from codepoints to keysyms uses
  `Keysym::from_char`.

  In an `interpret` statement, the string must contain 0 or 1 codepoint. 0 codepoints
  behaves like `any`. 1 codepoint behaves as if the keysym had been written using standard
  XKB notation.

  In `key` statements, they expand to a list of keysyms. For example, the declaration of
  `key <a>` above is the same as `key <a> { [ { U65e5, U672c, U8a9e } ] }`. If the string
  occurs within an existing list, such as in the declaration of `key <b>` above, the inner
  list is embedded into the outer list.

  In a `modmap` statement, the behavior is as if the keysyms had been written individually
  as a comma-separated list.

# 0.1.3 (2025-02-13)

- Reduce memory usage when keymap contains large keycodes.
- Fixed the following scenario:

  ```xkb
  xkb_compat {
      virtual_modifiers LevelThree = Mod5;

      interpret ISO_Level3_Shift + AnyOfOrNone(all) {
          virtualModifier = LevelThree;
          action = SetMods(mods = LevelThree);
      };
  };

  xkb_symbols {
      key <rightalt> {
          [ Alt_R, Meta_R ],
      };
      key <rightalt> {
          type = "ONE_LEVEL",
          [ ISO_Level3_Shift ],
      };
      modmap Mod1 { Meta_R };
  };
  ```
  
  Previously this would, effectively, compile to

  ```xkb
  xkb_types {
      virtual_modifiers LevelThree = Mod1+Mod5;
  };

  xkb_symbols {
      key <rightalt> {
          type[Group1] = "ONE_LEVEL",
          symbols[Group1] = [ ISO_Level3_Shift, Meta_R ],
          actions[Group1] = [ SetMods(modifiers = Mod1+Mod5), NoAction() ]
      };
  };
  ```
  
  Which would cause applications to interpret the AltGr key as the Alt key.
  
  The behavior has been changed so that definitely-unreachable levels are ignored when
  interpreting `modmap` statements. In this case the key type is `ONE_LEVEL`, which means
  that the `Meta_R` keysym is unreachable:
  
  ```xkb
  xkb_types {
      virtual_modifiers LevelThree = Mod5;
  };

  xkb_symbols {
      key <rightalt> {
          type[Group1] = "ONE_LEVEL",
          symbols[Group1] = [ ISO_Level3_Shift, Meta_R ],
          actions[Group1] = [ SetMods(modifiers = Mod5), NoAction() ]
      };
  };
  ```

# 0.1.2 (2025-01-26)

- Reduced CPU and memory usage when parsing XKB maps.

# 0.1.1 (2025-01-23)

- Levels with multiple actions are no longer formatted by default. For example,

  ```xkb
  xkb_symbols {
    key <a> {
      [
        {
          SetMods(mods = Mod2),
          SetGroup(group = +1),
        }
      ]
    };
  };
  ```
  
  will be formatted as

  ```xkb
  xkb_symbols {
    key <a> { [ NoAction() ] };
  };
  ```
  
  This is because xkbcommon and Xwayland will reject the entire keymap if there are
  multiple actions per level.

  This can be reverted with the `multiple_actions_per_level` flag.
