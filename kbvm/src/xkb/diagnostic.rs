//! Diagnostic messages.
//!
//! This module contains types for diagnostic messages emitted by the XKB code.

mod handlers;

#[cfg(feature = "log")]
pub use handlers::log::WriteToLog;
pub use handlers::stderr::WriteToStderr;
use {
    crate::xkb::{
        code_map::CodeMap,
        code_slice::CodeSlice,
        span::{Span, Spanned},
    },
    bstr::ByteSlice,
    debug_fn::debug_fn,
    kbvm_proc::diagnostic_kind,
    std::{
        error::Error,
        fmt::{Debug, Display, Formatter, Write},
        ops::Deref,
        path::{Path, PathBuf},
        sync::Arc,
    },
    unicode_width::UnicodeWidthChar,
};

/// A handler for diagnostic messages.
///
/// # Example
///
/// The following example forwards all diagnostic messages to the `log` crate.
///
/// ```
/// # use kbvm::xkb::Context;
/// # use kbvm::xkb::diagnostic::WriteToLog;
/// let context = Context::default();
/// let handler = WriteToLog;
/// let _ = context.keymap_from_names(handler, None, None, None, None);
/// ```
///
/// # Pre-defined handlers
///
/// This crate implements the following handlers:
///
/// - `Vec<Diagnostic>`: The messages are appended to the vector.
/// - [`WriteToStderr`]: This handler writes messages to STDERR.
/// - [`WriteToLog`]: If the `log` feature is enabled, this handler passes messages to the
///   `log` crate.
/// - `(T, Severity) where T: DiagnosticHandler`: The messages are passed to `T` but only
///   if their [`Severity`] is at least the given severity.
/// - `(T, F) where T: DiagnosticHandler, F: Fn(DiagnosticKind, bool) -> bool`: The
///   messages are passed to `T` but only if the filter, `F`, permits them.
pub trait DiagnosticHandler {
    /// A filter for diagnostic messages.
    ///
    /// This filter can be used to skip messages. By default, all messages are passed
    /// through.
    ///
    /// The function should return `true` if the message should be passed to the handler.
    ///
    /// The `is_fatal` argument is `true` if this message is a fatal error that will also
    /// be returned to the application via `Result::Err`. In this case the application
    /// might want to only receive the diagnostic via one of the two mechanisms.
    fn filter(&self, kind: DiagnosticKind, is_fatal: bool) -> bool {
        let _ = kind;
        let _ = is_fatal;
        true
    }

    /// Asks the handler to handle a diagnostic message.
    fn handle(&mut self, diag: Diagnostic);
}

pub(crate) struct DiagnosticSink<'a, 'b> {
    handler: &'a mut (dyn DiagnosticHandler + 'b),
}

/// The severity of a diagnostic message.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
#[non_exhaustive]
pub enum Severity {
    /// Debug severity.
    Debug,
    /// Warning severity.
    Warning,
    /// Error severity.
    ///
    /// Note that such errors are usually not fatal.
    Error,
}

/// The type of a diagnostic message.
#[diagnostic_kind]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
#[non_exhaustive]
pub enum DiagnosticKind {
    /// An octal string escape sequence has overflowed.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keymap "\777" { };
    /// ```
    #[severity = Error]
    OctalStringEscapeOverflow,
    /// A string contains an unknown escape sequence.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keymap "\u" { };
    /// ```
    #[severity = Error]
    UnknownEscapeSequence,
    /// An error occurred while opening a file.
    ///
    /// This diagnostic is not emitted for missing files.
    #[severity = Error]
    FileOpenFailed,
    /// An error occurred while deserializing a registry file.
    #[severity = Error]
    DeserializeRegistryFailed,
    /// An error occurred while reading a file.
    ///
    /// This diagnostic is not emitted for missing files.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keycodes {
    ///     include "file"
    /// };
    /// ```
    #[severity = Error]
    FileReadFailed,
    /// An included file could not be found.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keycodes {
    ///     include "file"
    /// };
    /// ```
    #[severity = Warning]
    FileNotFound,
    /// An included file contains an item with an unexpected type.
    ///
    /// # Example
    ///
    /// ```xkb
    /// // In symbols/file:
    /// xkb_keycodes { };
    /// ```
    ///
    /// Files in the symbols include directory should only contain `xkb_symbols` items.
    #[severity = Warning]
    UnexpectedItemType,
    /// An included file contains multiple default items.
    ///
    /// # Example
    ///
    /// ```xkb
    /// default xkb_keycodes { };
    ///
    /// default xkb_keycodes { };
    /// ```
    #[severity = Warning]
    MultipleDefaultItems,
    /// The same item name occurs multiple times in an include file.
    ///
    /// Note that this diagnostic might not be detected because files are parse
    /// incrementally.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keycodes "X" { };
    ///
    /// xkb_keycodes "X" { };
    /// ```
    #[severity = Warning]
    DuplicateItemName,
    /// An include statement does not contain a file name.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keycodes {
    ///     include "(abc)"
    /// };
    /// ```
    #[severity = Error]
    MissingIncludeFileName,
    /// An include statement contains an unterminated map name.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keycodes {
    ///     include "file(abc"
    /// };
    /// ```
    #[severity = Error]
    UnterminatedIncludeMapName,
    /// An include statement component does not start with a merge mode.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keycodes {
    ///     include "a(b)c(d)"
    /// };
    /// ```
    #[severity = Error]
    MissingIncludeMergeMode,
    /// An include statement component has an invalid group index.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keycodes {
    ///     include "a(b):not_valid"
    /// };
    /// ```
    #[severity = Error]
    InvalidIncludeGroupIndex,
    /// A key name is not terminated with a `>`.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keycodes {
    ///     <a = 1;
    /// };
    /// ```
    #[severity = Error]
    UnterminatedKeyName,
    /// A string is not terminated with a `"`.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keycodes "abc {
    /// };
    /// ```
    #[severity = Error]
    UnterminatedString,
    /// A float literal could not be parsed.
    #[severity = Error]
    InvalidFloatLiteral,
    /// An integer literal could not be parsed.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keycodes {
    ///     <a> = 999999999999999999999999999999999999999999;
    /// };
    /// ```
    #[severity = Error]
    InvalidIntegerLiteral,
    /// The source contains a byte that the lexer did not recognize.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keycodes {
    ///     %
    /// };
    /// ```
    #[severity = Error]
    UnlexableByte,
    /// The maximum depth of an include tree has been reached. Deeper includes are
    /// ignored.
    #[severity = Error]
    MaxIncludeDepthReached,
    /// The maximum number of includes has been reached. Further includes are ignored.
    #[severity = Error]
    MaxIncludesReached,
    /// A rules file contains an empty macro name.
    ///
    /// # Example
    ///
    /// ```rmlvo
    /// ! $ = abc
    /// ```
    #[severity = Error]
    EmptyMacroName,
    /// Expected a token but encountered EOF.
    ///
    /// # Example
    ///
    /// ```rmlvo
    /// ! $
    /// ```
    #[severity = Error]
    UnexpectedEof,
    /// Expected one or more tokens but encountered a different token.
    ///
    /// # Example
    ///
    /// ```rmlvo
    /// ! $ !
    /// ```
    #[severity = Error]
    UnexpectedToken,
    /// Expected a line terminator but encountered another token.
    ///
    /// # Example
    ///
    /// ```rmlvo
    /// ! include abc def
    /// ```
    #[severity = Error]
    ExpectedEol,
    /// Expected a matcher index opener but found a different byte.
    ///
    /// # Example
    ///
    /// ```rmlvo
    /// ! layout^ = symbols
    /// ```
    #[severity = Error]
    ExpectedIndexStart,
    /// Expected a matcher index terminator but found a different byte.
    ///
    /// # Example
    ///
    /// ```rmlvo
    /// ! layout[1^ = symbols
    /// ```
    #[severity = Error]
    ExpectedIndexEnd,
    /// Expected a matcher index terminator but found an unexpected string.
    ///
    /// # Example
    ///
    /// ```rmlvo
    /// ! layout[not_valid] = symbols
    /// ```
    #[severity = Error]
    InvalidMatcherIndex,
    /// An expression is too deply nested.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     name[((((((((((((((((((((((((1))))))))))))))))))))))))] = "name";
    /// };
    /// ```
    #[severity = Error]
    TooDeeplyNested,
    /// An item contains an unexpected declaration.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     <a> = 1;
    /// };
    /// ```
    #[severity = Error]
    UnexpectedDeclaration,
    /// A number overflowed u32.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keycodes {
    ///     indicator 999999999999999999 = "A";
    /// };
    /// ```
    #[severity = Error]
    U32Overflow,
    /// The source contains an unknown keysym.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> { [ Invalid ] };
    /// };
    /// ```
    #[severity = Error]
    UnknownKeysym,
    /// Source refers to an unknown group.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     name[abcd] = "abcd";
    /// };
    /// ```
    #[severity = Error]
    UnknownGroup,
    /// Source refers to an unknown level.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     type "X" {
    ///         level_name[abcd] = "abcd";
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownLevel,
    /// Source refers to an unknown radio group.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         allownone[abcd],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownRadioGroup,
    /// Encountered a recursive include statement.
    ///
    /// # Example
    ///
    /// ```xkb
    /// // In types/a.xkb
    /// default xkb_types {
    ///     include "b.xkb"
    /// };
    ///
    /// // In types/b.xkb
    /// default xkb_types {
    ///     include "a.xkb"
    /// };
    /// ```
    #[severity = Error]
    RecursiveInclude,
    /// RMLVO rule does not have the same number of keys as its mapping.
    ///
    /// # Example
    ///
    /// ```rmlvo
    /// ! model option = symbols
    ///   *            = pc
    /// ```
    #[severity = Error]
    InvalidNumberOfRuleKeys,
    /// RMLVO rule appears without a previous mapping.
    ///
    /// # Example
    ///
    /// ```rmlvo
    ///   * = pc
    /// ```
    #[severity = Error]
    UnexpectedRule,
    /// RMLVO rule does not have the same number of values as its mapping.
    ///
    /// # Example
    ///
    /// ```rmlvo
    /// ! model = symbols keycodes
    ///   *     = pc
    /// ```
    #[severity = Error]
    InvalidNumberOfRuleValues,
    /// RMLVO rule contains an invalid percent encoding.
    ///
    /// # Example
    ///
    /// ```rmlvo
    ///   *     = %u
    /// ```
    #[severity = Error]
    InvalidPercentEncoding,
    /// Indicator index is out of bounds.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keycodes {
    ///     indicator 99 = "A";
    /// };
    /// ```
    #[severity = Error]
    InvalidIndicatorIndex,
    /// Map contains too many indicators.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "1" { };
    ///     // ...
    ///     indicator "32" { };
    ///     indicator "33" { };
    /// };
    /// ```
    #[severity = Error]
    TooManyIndicators,
    /// Source contains an unknown keycodes variable.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keycodes {
    ///     abcd = 1;
    /// };
    /// ```
    #[severity = Error]
    UnknownKeycodesVariable,
    /// Source contains an unknown types variable.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     abcd = 1;
    /// };
    /// ```
    #[severity = Error]
    UnknownTypesVariable,
    /// Source contains an unknown compat variable.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     abcd = 1;
    /// };
    /// ```
    #[severity = Error]
    UnknownCompatVariable,
    /// Key alias does not refer to a known key.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_keycodes {
    ///     alias <a> = <unknown>;
    /// };
    /// ```
    #[severity = Error]
    UnknownKeyAlias,
    /// Duplicate key type definition has been ignored.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     type "A" { };
    ///     augment type "A" { };
    /// };
    /// ```
    #[severity = Warning]
    DuplicateKeyTypeDefinition,
    /// Source refers to an unknown action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> { [ UnknownAction() ] };
    /// };
    /// ```
    #[severity = Error]
    UnknownAction,
    /// Duplicate interpret field has been ignored.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     interpret A { repeats = true; };
    ///     augment interpret A { repeats = false; };
    /// };
    /// ```
    #[severity = Warning]
    IgnoredInterpretField,
    /// Duplicate indicator field has been ignored.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "A" { mods = Mod1; };
    ///     augment indicator "A" { mods = Mod2; };
    /// };
    /// ```
    #[severity = Warning]
    IgnoredIndicatorField,
    /// Modifier is unknown.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     modmap Unknown { A };
    /// };
    /// ```
    #[severity = Error]
    UnknownModifier,
    /// Duplicate modmap entry has been ignored.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     modmap Mod1 { A };
    ///     augment modmap Mod2 { A };
    /// };
    /// ```
    #[severity = Warning]
    IgnoredModMapEntry,
    /// Source refers to an unknown key.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <unknown> { };
    /// };
    /// ```
    #[severity = Error]
    UnknownKey,
    /// Source contains an unknown symbols variable.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     abcd = 1;
    /// };
    /// ```
    #[severity = Error]
    UnknownSymbolsVariable,
    /// Duplicate key field has been ignored.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> { repeats = true };
    ///     augment key <a> { repeats = false };
    /// };
    /// ```
    #[severity = Warning]
    IgnoredKeyField,
    /// Included symbols contain declarations for more than the first group but the
    /// include statement specifies an explicit group index. All but the first group are
    /// discarded.
    ///
    /// # Example
    ///
    /// ```xkb
    /// // In main.xkb
    /// xkb_symbols {
    ///     include "a:2"
    /// };
    ///
    /// // In symbols/a.xkb
    /// default xkb_symbols {
    ///     key <a> { [ A ], [ B ] };
    /// };
    /// ```
    #[severity = Error]
    DiscardingGroup,
    /// Group name declaration does not specify a name.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     name[Group1];
    /// };
    /// ```
    #[severity = Warning]
    MissingGroupName,
    /// Source contains too many virtual modifiers.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     virtual_modifiers m01;
    ///     // ...
    ///     virtual_modifiers m23;
    ///     virtual_modifiers m24;
    /// };
    /// ```
    #[severity = Error]
    TooManyVirtualModifiers,
    /// Duplicate virtual modifier definition has been ignored.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     virtual_modifiers X = 1;
    ///     augment virtual_modifiers X = 2;
    /// };
    /// ```
    #[severity = Warning]
    IgnoringVmodRedefinition,
    /// The `$HOME` environment variable is not set or has not been enabled.
    ///
    /// # Example
    ///
    /// ```rmlvo
    /// ! include %H/abcd
    /// ```
    #[severity = Error]
    HomeNotSet,
    /// RMLVO include contains an unknown escape sequence.
    ///
    /// # Example
    ///
    /// ```rmlvo
    /// ! include %U
    /// ```
    #[severity = Error]
    UnknownRulesEscapeSequence,
    /// Included file contains no default item. Using the first item instead.
    ///
    /// # Example
    ///
    /// ```xkb
    /// // In main.xkb
    /// xkb_symbols {
    ///     include "a.xkb"
    /// };
    ///
    /// // In symbols/a.xkb
    /// xkb_symbols {
    /// };
    /// ```
    #[severity = Warning]
    UsingFirstInsteadOfDefault,
    /// A modifier calculation overflowed.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     virtual_modifiers A = 0xffffffffffff;
    /// };
    /// ```
    #[severity = Error]
    ModsCalculationOverflow,
    /// A keysym calculation overflowed.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> { [ 0xffffffffffff ] };
    /// };
    /// ```
    #[severity = Error]
    KeysymCalculationOverflow,
    /// A level calculation overflowed.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     type "A" {
    ///         map[Mod1] = 0xffffffffffff;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    LevelCalculationOverflow,
    /// A radio group calculation overflowed.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         allownone = 0xfffffffffff;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    RadioGroupCalculationOverflow,
    /// An expression in group position has an unexpected form.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     name["Group1"] = "name";
    /// };
    /// ```
    #[severity = Error]
    UnsupportedExpressionForGroup,
    /// An expression in group-change position has an unexpected form.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> = { [ SetGroup(group = "Group1") ] };
    /// };
    /// ```
    #[severity = Error]
    UnsupportedExpressionForGroupChange,
    /// A group calculation overflowed.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     name[0xffffffffffff] = "name";
    /// };
    /// ```
    #[severity = Error]
    GroupCalculationOverflow,
    /// An expression in level position has an unexpected form.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     type "A" {
    ///         map[Mod1] = "Level2";
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnsupportedExpressionForLevel,
    /// An expression in radio-group position has an unexpected form.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         allownone[1 | 2],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnsupportedExpressionForRadioGroup,
    /// An expression in string position has an unexpected form.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     name[Group1] = 123;
    /// };
    /// ```
    #[severity = Error]
    UnsupportedExpressionForString,
    /// An expression in boolean position has an unexpected form.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     interpret A {
    ///         repeats = 1;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnsupportedExpressionForBoolean,
    /// An expression in keysym position has an unexpected form.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> = { [ "A" ] };
    /// };
    /// ```
    #[severity = Error]
    UnsupportedExpressionForKeysym,
    /// An expression in mod-mask position has an unexpected form.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     virtual_modifiers A = 2 * 2;
    /// };
    /// ```
    #[severity = Error]
    UnsupportedExpressionForModMask,
    /// An expression in key-repeat position has an unexpected form.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> { repeats = 1 };
    /// };
    /// ```
    #[severity = Error]
    UnsupportedExpressionForKeyRepeats,
    /// An expression in LockModifiers(affect) position has an unexpected form.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> { [ LockModifiers(affect = 1) ] };
    /// };
    /// ```
    #[severity = Error]
    UnsupportedExpressionForLockModsAffect,
    /// An expression in action position has an unexpected form.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         actions[Group1] = [ A ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnsupportedExpressionForAction,
    /// An expression in symbols or actions position has an unexpected form.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         actions[Group1] = 1,
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnsupportedExpressionForSymbolsOrActions,
    /// An expression in keycode position has an unexpected form.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ RedirectKey(key = "abcd") ]
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnsupportedExpressionForKeycode,
    /// An expression in boolean position has an unexpected value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         repeats = Yep,
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownBooleanValue,
    /// The source refers to an unimplemented action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ MovePtr() ],
    ///     };
    /// };
    /// ```
    #[severity = Debug]
    UnimplementedAction,
    /// An unknown parameter appears in a `NoAction` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ NoAction(abcd = 1) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownParameterForNoAction,
    /// An unknown parameter appears in a `SetMods` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ SetMods(abcd = 1) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownParameterForSetMods,
    /// An unknown parameter appears in a `LatchMods` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ LatchMods(abcd = 1) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownParameterForLatchMods,
    /// An unknown parameter appears in a `LockMods` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ LockMods(abcd = 1) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownParameterForLockMods,
    /// An unknown parameter appears in a `SetGroup` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ SetGroup(abcd = 1) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownParameterForSetGroup,
    /// An unknown parameter appears in a `LatchGroup` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ LatchGroup(abcd = 1) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownParameterForLatchGroup,
    /// An unknown parameter appears in a `LockGroups` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ LockGroups(abcd = 1) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownParameterForLockGroup,
    /// The `mods` argument in a `SetMods` action does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ SetMods(mods) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingValueForSetModsMods,
    /// The `mods` argument in a `LatchMods` action does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ LatchMods(mods) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingValueForLatchModsMods,
    /// The `mods` argument in a `LockMods` action does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ LockMods(mods) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingValueForLockModsMods,
    /// The `affect` argument in a `LockMods` action does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ LockMods(affect) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingValueForLockModsAffect,
    /// The `group` argument in a `SetGroup` action does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ SetGroup(group) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingValueForSetGroupGroup,
    /// The `group` argument in a `LatchGroup` action does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ LatchGroup(group) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingValueForLatchGroupGroup,
    /// The `group` argument in a `LockGroup` action does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ LockGroup(group) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingValueForLockGroupGroup,
    /// The `key` argument in a `RedirectKey` action does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ RedirectKey(key) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingValueForRedirectKeyKey,
    /// The `clearMods` argument in a `RedirectKey` action does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ RedirectKey(clearMods) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingValueForRedirectKeyClearmods,
    /// The `mods` argument in a `RedirectKey` action does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ RedirectKey(mods) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingValueForRedirectKeyMods,
    /// The `controls` argument in a `SetControls` action does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ SetControls(controls) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingValueForSetControlsControls,
    /// The `controls` argument in a `LockControls` action does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ LockControls(controls) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingValueForLockControlsControls,
    /// The `affect` argument in a `LockControls` action does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ LockControls(affect) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingValueForLockControlsAffect,
    /// The `affect` argument in a `LockMods` action has an unexpected value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ LockMods(affect = unknown) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownLockModsAffect,
    /// An interpret filter has more than one argument.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     interpret A + AnyOf(Mod1, Mod2) {
    ///     };
    /// };
    /// ```
    #[severity = Error]
    NotOneInterpretFilterArgument,
    /// An interpret filter has a named argument.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     interpret A + AnyOf(mods = Mod1) {
    ///     };
    /// };
    /// ```
    #[severity = Error]
    NamedFilterArgArgument,
    /// An interpret filter has an unknown predicate.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     interpret A + Unknown(Mod1) {
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownFilterPredicate,
    /// An interpret statement contains an unknown field.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     interpret A {
    ///         unknown = 1;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownInterpretField,
    /// An interpret statement contains an unimplemented field.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     interpret A {
    ///         locking = true;
    ///     };
    /// };
    /// ```
    #[severity = Debug]
    UnimplementedInterpretField,
    /// The `action` field in an interpret statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     interpret A {
    ///         action;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingInterpretActionValue,
    /// The `virtualmodifier` field in an interpret statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     interpret A {
    ///         virtualmodifier;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingInterpretVirtualmodValue,
    /// The `virtualmodifier` field in an interpret field refers to an unknown virtual modifier.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     interpret A {
    ///         virtualmodifier = Unknown;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownInterpretVirtualModifier,
    /// An type statement contains an unknown field.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     type "A" {
    ///         unknown = 1;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownTypeField,
    /// The `modifiers` field in an type statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     type "A" {
    ///         modifiers;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingTypeModifiersValue,
    /// The `map` field in an type statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     type "A" {
    ///         map[Mod1];
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingTypeMapValue,
    /// The `preserve` field in an type statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     type "A" {
    ///         preserve[Mod1];
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingTypePreserveValue,
    /// The `level_name` field in an type statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     type "A" {
    ///         level_name[Mod1];
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingTypeLevelNameValue,
    /// The `useModMap` field in an interpret statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     interpret A {
    ///         useModMap;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingInterpretUseModMapModValue,
    /// The `useModMap` field in an interpret statement has an unknown value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     interpret A {
    ///         useModMap = Unknown;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    InvalidInterpretUseModMapModValue,
    /// A indicator statement contains an unimplemented field.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "A" {
    ///         !allowExplicit;
    ///     };
    /// };
    /// ```
    #[severity = Debug]
    UnimplementedIndicatorField,
    /// A indicator statement contains an unknown field.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "A" {
    ///         unknown = 1;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownIndicatorField,
    /// The `modifiers` field in an indicator statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "A" {
    ///         modifiers;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingIndicatorModifiersValue,
    /// An expression in group position refers to an out-of-bounds group.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     name[Group33] = "name";
    /// };
    /// ```
    #[severity = Error]
    GroupOutOfBounds,
    /// An expression in level position refers to an out-of-bounds level.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_types {
    ///     type "A" {
    ///         map[Mod1] = Level33;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    LevelOutOfBounds,
    /// An expression in radio-group position refers to an out-of-bounds radio group.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         allownone[group33],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    RadioGroupOutOfBounds,
    /// The `groups` field in an indicator statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "A" {
    ///         groups;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingIndicatorGroupsValue,
    /// The `controls` field in an indicator statement refers to an unknown value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "A" {
    ///         controls = Unknown;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownControl,
    /// The `controls` field in an indicator statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "A" {
    ///         controls;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingIndicatorControlValue,
    /// An expression in mod-component position refers to an unknown mod component.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "A" {
    ///         whichModState = Unknown;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownModComponent,
    /// An expression in mod-component position has an unexpected form.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "A" {
    ///         whichModState = Latched * Locked;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnsupportedExpressionForModComponent,
    /// An expression in controls position has an unexpected form.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "A" {
    ///         controls = SlowKeys * SlowKeys;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnsupportedExpressionForControl,
    /// The `whichModState` field in an indicator statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "A" {
    ///         whichModState;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingIndicatorWhichModStateValue,
    /// An expression in group-component position refers to an unknown group component.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "A" {
    ///         whichGroupState = Unknown;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownGroupComponent,
    /// An expression in group-component position has an unexpected form.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "A" {
    ///         whichGroupState = Latched * Locked;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnsupportedExpressionForGroupComponent,
    /// The `whichGroupState` field in an indicator statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_compat {
    ///     indicator "A" {
    ///         whichGroupState;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingIndicatorWhichGroupStateValue,
    /// An expression in keycode position refers to an unknown keycode.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     modmap Mod1 { <unknown> };
    /// };
    /// ```
    #[severity = Error]
    UnknownKeycode,
    /// A key statement contains an unknown field.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         unknown = 1
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownKeyField,
    /// The `groupsRedirect` field in a key statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         groupsRedirect,
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingKeyGroupsRedirectValue,
    /// The `symbols` field in a key statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         symbols[Group1],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingKeySymbolsValue,
    /// The `actions` field in a key statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         actions[Group1],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingKeyActionsValue,
    /// The `virtualModifiers` field in a key statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         virtual_modifiers,
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingKeyVirtualModifiersValue,
    /// The `repeats` field in a key statement refers to an unknown value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         repeats = Yep,
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownKeyRepeatingValue,
    /// The `type` field in a key statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         type[Group1];
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingKeyTypeValue,
    /// The `type` field in a key refers to an unknown value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         type[Group1] = "Unknown";
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownKeyType,
    /// A virtual-modifier declaration uses the name of a real modifier.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     virtual_modifiers Shift;
    /// };
    /// ```
    #[severity = Error]
    VirtualModifierHasRealName,
    /// A keysym is not terminated with a `>`.
    ///
    /// # Example
    ///
    /// ```compose
    /// <Multi_key> <a : "@"
    /// ```
    #[severity = Error]
    UnterminatedKeysym,
    /// Compose include contains an unknown escape sequence.
    ///
    /// # Example
    ///
    /// ```rmlvo
    /// include "%U"
    /// ```
    #[severity = Error]
    UnknownComposeIncludeEscape,
    /// The locale compose file, %L, could not be resolved.
    ///
    /// # Example
    ///
    /// ```rmlvo
    /// include "%L"
    /// ```
    #[severity = Error]
    LocaleComposeFileNotResolved,
    /// A compose file contains duplicate entries.
    ///
    /// # Example
    ///
    /// ```rmlvo
    /// <a>: "A"
    /// <a>: "B"
    /// ```
    #[severity = Debug]
    IgnoringDuplicateComposeEntry,
    /// A compose file contains an entry that is a prefix of another entry.
    ///
    /// # Example
    ///
    /// ```rmlvo
    /// <a>: "A"
    /// <a> <b>: "B"
    /// ```
    #[severity = Debug]
    IgnoringComposePrefix,
    /// A file path is not valid UTF-8.
    ///
    /// # Example
    ///
    /// ```compose
    /// include "\xFF"
    /// ```
    #[severity = Error]
    NonUTF8Path,
    /// A compose rule has no conditions.
    ///
    /// # Example
    ///
    /// ```compose
    /// : "a"
    /// ```
    #[severity = Error]
    ComposeRuleWithoutConditions,
    /// The maximum runtime has been reached. Further code.
    #[severity = Error]
    MaxRuntimeReached,
    /// The maximum number of compose rules has been reached. Further rules are ignored.
    #[severity = Error]
    MaxComposeRulesReached,
    /// An unknown parameter appears in a `RedirectKey` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ RedirectKey(abcd = 1) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownParameterForRedirectKey,
    /// An unknown parameter appears in a `SetControls` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ SetControls(abcd = 1) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownParameterForSetControls,
    /// An unknown parameter appears in a `LockControls` action.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         [ LockControls(abcd = 1) ],
    ///     };
    /// };
    /// ```
    #[severity = Error]
    UnknownParameterForLockControls,
    /// The `overlay1`/`overlay2` field in a key statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         overlay1;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingKeyOverlayValue,
    /// The `radiogroup` field in a key statement does not have a value.
    ///
    /// # Example
    ///
    /// ```xkb
    /// xkb_symbols {
    ///     key <a> {
    ///         radiogroup;
    ///     };
    /// };
    /// ```
    #[severity = Error]
    MissingKeyRadiogroupValue,
}

impl DiagnosticKind {
    /// Returns the severity of the message type.
    pub fn severity(&self) -> Severity {
        self.severity_()
    }
}

/// A diagnostic message.
///
/// Diagnostic messages are emitted when creating keymaps and usually indicate a
/// less-than-ideal situation.
///
/// Each message has an associated type called [`DiagnosticKind`] that can be accessed
/// with the [`Diagnostic::kind`] function.
///
/// Each message is associated with a chain of source-code locations. The first element of
/// this chain can be accessed with the [`Diagnostic::location`] function.
///
/// This type implements the [`Error`] and [`Display`] traits that can be used to format
/// this message as expected. If you also want to display code excerpts showing exactly
/// where an error occurred, you can use the [`Diagnostic::with_code`] function.
pub struct Diagnostic {
    kind: DiagnosticKind,
    location: DiagnosticLocation,
}

/// A location of a diagnostic message.
///
/// This location is part of a chain of locations. The next location can be accessed with
/// the [`DiagnosticLocation::next`] function.
///
/// The message associated with this location can be accessed with the [`Display`]
/// implementation of this type.
pub struct DiagnosticLocation {
    message: Option<Box<dyn Display + Send + Sync>>,
    source_file: Option<Arc<PathBuf>>,
    line: CodeSlice<'static>,
    line_num: usize,
    in_line_offset: usize,
    in_line_len: usize,
    inner: Option<Box<DiagnosticLocation>>,
}

struct WithCode<'a> {
    diagnostic: &'a Diagnostic,
}

impl<'a, 'b> DiagnosticSink<'a, 'b> {
    pub(crate) fn new(handler: &'a mut (dyn DiagnosticHandler + 'b)) -> Self {
        Self { handler }
    }

    fn push_(
        &mut self,
        map: &mut CodeMap,
        kind: DiagnosticKind,
        message: Spanned<impl Display + Send + Sync + 'static>,
        is_fatal: bool,
    ) {
        if !self.handler.filter(kind, is_fatal) {
            return;
        }
        let diagnostic = Diagnostic::new(map, kind, message.val, message.span);
        self.handler.handle(diagnostic);
    }

    pub(crate) fn push(
        &mut self,
        map: &mut CodeMap,
        kind: DiagnosticKind,
        message: Spanned<impl Display + Send + Sync + 'static>,
    ) {
        self.push_(map, kind, message, false)
    }

    pub(crate) fn push_fatal(
        &mut self,
        map: &mut CodeMap,
        kind: DiagnosticKind,
        message: Spanned<impl Display + Send + Sync + 'static>,
    ) -> Diagnostic {
        let message = message.map(Arc::new);
        self.push_(map, kind, message.clone(), true);
        Diagnostic::new(map, kind, message.val, message.span)
    }
}

impl DiagnosticLocation {
    fn new(
        map: &mut CodeMap,
        message: Option<Box<dyn Display + Send + Sync + 'static>>,
        span: Span,
        inner: Option<Box<DiagnosticLocation>>,
    ) -> Self {
        let info = map.get(span);
        let lo = span.lo.max(info.span.lo) - info.lines_offset;
        let hi = span.hi.min(info.span.hi) - info.lines_offset;
        let line_idx = info
            .lines
            .binary_search_by(|r| r.cmp(&lo))
            .unwrap_or_else(|i| i - 1);
        let line_lo = info.lines[line_idx];
        let line_hi = info
            .lines
            .get(line_idx + 1)
            .map(|l| *l - 1)
            .unwrap_or(info.span.hi - info.lines_offset);
        let in_line_offset = (lo - line_lo) as usize;
        let in_line_len = (hi.min(line_hi).saturating_sub(lo)) as usize;
        let line_lo = (line_lo + info.lines_offset - info.span.lo) as usize;
        let line_hi = (line_hi + info.lines_offset - info.span.lo) as usize;
        let slice = info.code.to_slice().slice(line_lo..line_hi).to_owned();
        let mut res = Self {
            message,
            source_file: info.file.cloned(),
            line: slice,
            line_num: line_idx + 1,
            in_line_offset,
            in_line_len,
            inner,
        };
        if let Some(span) = info.include_span {
            res = Self::new(map, None, span, Some(Box::new(res)))
        }
        res
    }

    fn fmt(&self, f: &mut Formatter<'_>, with_code: bool) -> std::fmt::Result {
        write!(
            f,
            "at {} {}:{}: {}",
            debug_fn(|f| match &self.source_file {
                Some(p) => Display::fmt(&p.display(), f),
                None => f.write_str("<anonymous file>"),
            }),
            self.line_num,
            self.in_line_offset,
            debug_fn(|f| match &self.message {
                Some(m) => m.fmt(f),
                None => f.write_str("while processing include"),
            }),
        )?;
        if with_code {
            f.write_str(":\n")?;
            write!(f, ">> ")?;
            let (prefix, suffix) = self.line.split_at(self.in_line_offset);
            let (content, suffix) = suffix.split_at(self.in_line_len);
            let mut send = |s: &[u8]| {
                let mut counter = s.len();
                if s.iter().any(|c| !matches!(*c, 0x20..=0x7E)) {
                    for c in s.as_bstr().chars() {
                        if c == '\t' {
                            counter += 3;
                            f.write_str("    ")?;
                        } else {
                            counter -= c.len_utf8();
                            if let Some(w) = c.width() {
                                counter += w;
                                f.write_char(c)?;
                            }
                        }
                    }
                } else {
                    write!(f, "{}", s.as_bstr())?;
                }
                Ok(counter)
            };
            let offset = send(prefix)?;
            let len = send(content)?;
            write!(f, "{}\n   ", suffix.as_bstr())?;
            for _ in 0..offset {
                f.write_str(" ")?
            }
            f.write_str("^")?;
            for _ in 1..len {
                f.write_str("~")?;
            }
            if let Some(inner) = &self.inner {
                f.write_str("\n")?;
                inner.deref().fmt(f, true)?;
            }
        }
        Ok(())
    }

    /// Returns the next location in this chain of locations.
    ///
    /// Locations are chained from outermost to innermost. For example, if file `A`
    /// includes file `B` and file `B` contains an error, then the location of the include
    /// statement in `A` will appear first, with `next` returning the location in `B`.
    pub fn next(&self) -> Option<&DiagnosticLocation> {
        self.inner.as_deref()
    }

    /// Returns the path of the source file that this location points to.
    ///
    /// Returns `None` if this location is not associated with a source file.
    pub fn source_file(&self) -> Option<&Path> {
        self.source_file.as_deref().map(|v| &**v)
    }

    /// Returns the contents (excluding the terminating newline) of the line that this
    /// location points to.
    pub fn line_contents(&self) -> &[u8] {
        &self.line
    }

    /// Return the entire contents of the file that this location points to.
    pub fn file_contents(&self) -> &[u8] {
        self.line.code()
    }

    /// Returns the 1-based line number that this location points to.
    pub fn line(&self) -> usize {
        self.line_num
    }

    /// Returns the 0-based column number that this location points to.
    ///
    /// This column is in terms of bytes. If the line contains non-ascii characters or
    /// tabs, then it is up to the application to convert this value to something usable.
    pub fn column(&self) -> usize {
        self.in_line_offset
    }

    /// Returns the length, in bytes, of the contents that this location points to.
    pub fn length(&self) -> usize {
        self.in_line_len
    }
}

impl Diagnostic {
    pub(crate) fn new(
        map: &mut CodeMap,
        kind: DiagnosticKind,
        message: impl Display + Send + Sync + 'static,
        span: Span,
    ) -> Self {
        Self {
            kind,
            location: DiagnosticLocation::new(map, Some(Box::new(message)), span, None),
        }
    }

    /// Returns a [`Display`] type that can be used to format the diagnostic with code
    /// excerpts.
    ///
    /// # Example
    ///
    /// ```
    /// use kbvm::xkb::diagnostic::Diagnostic;
    ///
    /// fn format_diagnostic(diag: &Diagnostic) {
    ///     println!("{}", diag.with_code());
    /// }
    /// ```
    ///
    /// This might print something like
    ///
    /// ```txt
    /// at <anonymous file> 4:32: while processing include:
    /// >>         xkb_compat { include "complete" };
    ///                                  ^~~~~~~~
    /// at /usr/share/X11/xkb/compat/complete 2:13: while processing include:
    /// >>     include "basic"
    ///                 ^~~~~
    /// at /usr/share/X11/xkb/compat/basic 40:13: while processing include:
    /// >>     include "ledcaps"
    ///                 ^~~~~~~
    /// at /usr/share/X11/xkb/compat/ledcaps 6:2: unknown `indicator` field:
    /// >>     !allowExplicit;
    ///         ^~~~~~~~~~~~~
    /// ```
    pub fn with_code(&self) -> impl Display + use<'_> {
        WithCode { diagnostic: self }
    }

    /// Returns the type of this diagnostic message.
    pub fn kind(&self) -> DiagnosticKind {
        self.kind
    }

    /// Returns the source location of this diagnostic message.
    pub fn location(&self) -> Option<&DiagnosticLocation> {
        Some(&self.location)
    }
}

impl Display for WithCode<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        self.diagnostic.location.fmt(f, true)
    }
}

impl Debug for Diagnostic {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        self.location.fmt(f, false)
    }
}

impl Display for Diagnostic {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        self.location.fmt(f, false)
    }
}

impl Error for Diagnostic {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        self.location.source()
    }
}

impl Debug for DiagnosticLocation {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        self.fmt(f, false)
    }
}

impl Display for DiagnosticLocation {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        self.fmt(f, false)
    }
}

impl Error for DiagnosticLocation {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        self.inner.as_deref().map(|d| d as &dyn Error)
    }
}
