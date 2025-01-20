# Compatibility

KBVM aims to be mostly compatible with the X server and libxkbcommon. Some of the
differences are described in this file.

## XKB

### Syntax

KBVM supports a number of syntax extensions:

- All non-empty list can have trailing commas.
- All lists can be empty.
- All config items (`xkb_keycodes`, `xkb_symbols`, etc.) can be omitted from keymaps.
- If a keymap contains multiple items of the same type, they are concatenated.
- Symbol and action lists are expressions that can appear in expression positions.

### Semantics

- `xkb_geometry` items are ignored.
- Only the following behaviors are supported:
  - Locking
- Only the following actions are supported:
  - `SetMods`
  - `LatchMods`
  - `LockMods`
  - `SetGroup`
  - `LatchGroup`
  - `LockGroup`
  - `RedirectKey`
- Modifiers are not truncated to the lower 8 bits.
- Each key can have up to 32 groups.
- Each key level can have an arbitrary number of symbols and actions.
- Merge modes are not retained after they have been applied. This should have no effect
  on typical RMLVO setups.

## XCompose

- Modifier requirements are ignored.
