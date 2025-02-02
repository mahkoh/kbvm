# Unreleased

- Reduce memory usage when keymap contains large keycodes.
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
