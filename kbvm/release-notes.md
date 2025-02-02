# Unreleased

- Reduce memory usage when keymap contains large keycodes.

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
