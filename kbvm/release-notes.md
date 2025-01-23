# Unreleased

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
