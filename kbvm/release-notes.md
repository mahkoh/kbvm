# Unreleased

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
