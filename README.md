# KBVM

[![crates.io](https://img.shields.io/crates/v/kbvm.svg)](http://crates.io/crates/kbvm)
[![docs.rs](https://docs.rs/kbvm/badge.svg)](http://docs.rs/kbvm)
![MSRV](https://img.shields.io/crates/msrv/kbvm)

KBVM is a rust implementation of the XKB specification and associated protocols. It
supports

- creating keymaps from XKB files,
- creating keymaps from RMLVO names,
- creating keymaps from X11 connections,
- creating a composition state machine from XCompose files, and
- loading the RMLVO registry.

A keymap can be turned into a compositor-side state machine or a client-side lookup table.

## Compatibility

See [compatibility.md](./docs/compatibility.md).

## CLI

The [kbvm-cli](./kbvm-cli/README.md) crate provides a binary that can be used to compile
and test keymaps without installing them.

## License

This kbvm crate is licensed under either of

- Apache License, Version 2.0
- MIT License

at your option.
