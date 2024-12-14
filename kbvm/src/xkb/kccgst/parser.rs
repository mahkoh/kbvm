pub(crate) mod error;
#[cfg(test)]
mod tests;

use {
    crate::{
        state_machine::Keycode,
        xkb::{
            code_loader::CodeType,
            interner::{Interned, Interner},
            kccgst::{
                ast::{
                    ArrayElement, ArrayInit, Call, CallArg, Compatmap, CompatmapDecl, CompositeMap,
                    ConfigItem, ConfigItemType, Coord, CoordAssignment, Decl, Decls,
                    DirectOrIncluded, DoodadDecl, DoodadType, Expr, ExprAssignment, Flag,
                    FlagWrapper, Flags, Geometry, GeometryDecl, GroupCompatDecl, Include,
                    InterpretDecl, InterpretMatch, Item, ItemType, Key, KeyAliasDecl, KeyExprs,
                    KeyNameDecl, KeySymbolsDecl, KeyTypeDecl, KeycodeDecl, Keycodes, Keys,
                    LedMapDecl, LedNameDecl, MergeMode, ModMapDecl, NamedParam, NestedConfigItem,
                    Outline, OverlayDecl, OverlayItem, Path, PathComponent, PathIndex, RowBody,
                    RowBodyItem, SectionDecl, SectionItem, ShapeDecl, ShapeDeclType, Symbols,
                    SymbolsArrayAssign, SymbolsDecl, SymbolsExprAssign, SymbolsIdent,
                    SymbolsVarDecl, Types, TypesDecl, VModDecl, VModDef, VarAssign, VarDecl,
                    VarDeclIdent,
                },
                meaning::{Meaning, MeaningCache},
                parser::error::{
                    CompatmapDeclExpectation, Expected, GeometryDeclExpectation,
                    KeycodeDeclExpectation, ParseDeclExpectation, ParserError,
                    SymbolsDeclExpectation, TypesDeclExpectation, ARRAY_ELEMENT_EXPECTED,
                    CONFIG_ITEM_EXPECTED, EXPR_TOKENS, FLAGS_EXPECTED, KEY_EXPECTED,
                    OUTLINE_TOKENS, SECTION_ITEM_EXPECTED, SYMBOLS_VAR_DECL_EXPECTED,
                },
                token::{
                    Punctuation::{self, Cbracket, Cparen, Oparen},
                    Token,
                },
            },
            span::{Span, SpanExt, SpanMap, Spanned},
        },
    },
    std::fmt::Debug,
    Punctuation::{Cbrace, Obrace, Obracket},
};

struct Parser<'a> {
    tokens: &'a [Spanned<Token>],
    interner: &'a Interner,
    meaning_cache: &'a mut MeaningCache,
    pos: usize,
    diagnostic_delta: u64,
}

#[derive(Debug, Copy, Clone)]
enum DeclCandidate {
    None,
    Doodad(Meaning),
    GroupCompat,
    Include(Spanned<MergeMode>, Interned),
    Interpret,
    KeyAlias,
    KeyName(Interned),
    Keys,
    KeyType,
    LedMap,
    LedName(bool),
    ModMap,
    Overlay,
    Row,
    Section,
    Shape,
    Symbols,
    Var(Token),
    Vmod,
}

macro_rules! parse_inline_list {
    ($self:ident, $lo:expr, $terminator:expr, $expected:expr, $f:expr $(,)?) => {
        $self.parse_inline_list(
            $lo,
            $terminator,
            &[Expected::Punctuation($terminator), Expected::Nested($expected)],
            &[Expected::Punctuation($terminator), Expected::Punctuation(punctuation![,])],
            $f,
        )
    };
}

pub fn snoop_ty_and_name(
    tokens: &[Spanned<Token>],
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
) -> Result<(Option<Span>, Spanned<CodeType>, Option<Spanned<Interned>>), Spanned<ParserError>> {
    let mut parser = Parser {
        tokens,
        interner,
        meaning_cache,
        pos: 0,
        diagnostic_delta: 0,
    };
    let flags = parser.parse_flags();
    let default = flags
        .flags
        .iter()
        .find(|f| f.flag == Flag::Default)
        .map(|f| f.name.span);
    let i = parser.parse_ident()?;
    let meaning = parser.meaning_cache.get_case_insensitive(interner, i.val);
    let ty = match meaning {
        Meaning::XkbKeymap | Meaning::XkbSemantics | Meaning::XkbLayout => CodeType::Keymap,
        Meaning::XkbKeycodes => CodeType::Keycodes,
        Meaning::XkbTypes => CodeType::Types,
        Meaning::XkbCompatibilityMap
        | Meaning::XkbCompatibility
        | Meaning::XkbCompatMap
        | Meaning::XkbCompat => CodeType::Compat,
        Meaning::XkbSymbols => CodeType::Symbols,
        Meaning::XkbGeometry => CodeType::Geometry,
        _ => {
            const EXPECTED: &[Expected] = &[
                Expected::Nested(FLAGS_EXPECTED),
                Expected::Ident(Meaning::XkbKeymap),
                Expected::Ident(Meaning::XkbSemantics),
                Expected::Ident(Meaning::XkbLayout),
                Expected::Nested(CONFIG_ITEM_EXPECTED),
            ];
            return Err(parser.unexpected_token(EXPECTED, i.map(Token::Ident)));
        }
    };
    let (name, _) = parser.parse_map_name()?;
    Ok((default, ty.spanned2(i.span), name))
}

pub fn parse_item(
    interner: &Interner,
    meaning_cache: &mut MeaningCache,
    tokens: &[Spanned<Token>],
    diagnostic_delta: u64,
) -> Result<Spanned<Item>, Spanned<ParserError>> {
    Parser {
        tokens,
        interner,
        meaning_cache,
        pos: 0,
        diagnostic_delta,
    }
    .parse_item()
}

impl<'a> Parser<'a> {
    fn next(
        &mut self,
        expected: &'static [Expected],
    ) -> Result<Spanned<Token>, Spanned<ParserError>> {
        if self.pos >= self.tokens.len() {
            return Err(self.expected_but_eof(self.tokens.last().unwrap().span, expected));
        }
        let token = self.tokens[self.pos];
        self.pos += 1;
        Ok(token)
    }

    fn try_peek(&mut self) -> Option<Spanned<Token>> {
        self.tokens.get(self.pos).copied()
    }

    fn try_peek2(&mut self) -> Option<Spanned<Token>> {
        self.tokens.get(self.pos + 1).copied()
    }

    fn peek(
        &mut self,
        expected: &'static [Expected],
    ) -> Result<Spanned<Token>, Spanned<ParserError>> {
        if self.pos >= self.tokens.len() {
            return Err(self.expected_but_eof(self.tokens.last().unwrap().span, expected));
        }
        Ok(self.tokens[self.pos])
    }

    fn parse_item(&mut self) -> Result<Spanned<Item>, Spanned<ParserError>> {
        let flags = self.parse_flags();
        let i = self.parse_ident()?;
        let meaning = self
            .meaning_cache
            .get_case_insensitive(self.interner, i.val);
        let map = match meaning {
            Meaning::XkbKeymap | Meaning::XkbSemantics | Meaning::XkbLayout => {
                self.parse_composite_map(i)?.map(ItemType::Composite)
            }
            Meaning::XkbKeycodes
            | Meaning::XkbTypes
            | Meaning::XkbCompatibilityMap
            | Meaning::XkbCompatibility
            | Meaning::XkbCompatMap
            | Meaning::XkbCompat
            | Meaning::XkbSymbols
            | Meaning::XkbGeometry => self
                .parse_config_item_type(i, meaning)?
                .map(ItemType::Config),
            _ => {
                const EXPECTED: &[Expected] = &[
                    Expected::Nested(FLAGS_EXPECTED),
                    Expected::Ident(Meaning::XkbKeymap),
                    Expected::Ident(Meaning::XkbSemantics),
                    Expected::Ident(Meaning::XkbLayout),
                    Expected::Nested(CONFIG_ITEM_EXPECTED),
                ];
                return Err(self.unexpected_token(EXPECTED, i.map(Token::Ident)));
            }
        };
        let lo = match flags.flags.first() {
            Some(f) => f.name.span.lo,
            None => i.span.lo,
        };
        let item = Item { flags, ty: map.val };
        Ok(item.spanned(lo, map.span.hi))
    }

    fn parse_config_item_type(
        &mut self,
        ty: Spanned<Interned>,
        meaning: Meaning,
    ) -> Result<Spanned<ConfigItemType>, Spanned<ParserError>> {
        match meaning {
            Meaning::XkbKeycodes => self.parse_config_item(
                &KeycodeDeclExpectation,
                ty.span.lo,
                |slf| slf.parse_keycode_decl(),
                |name, decls| ConfigItemType::Keycodes(Keycodes { name, decls }),
            ),
            Meaning::XkbTypes => self.parse_config_item(
                &TypesDeclExpectation,
                ty.span.lo,
                |slf| slf.parse_types_decl(),
                |name, decls| ConfigItemType::Types(Types { name, decls }),
            ),
            Meaning::XkbCompatibilityMap
            | Meaning::XkbCompatibility
            | Meaning::XkbCompatMap
            | Meaning::XkbCompat => self.parse_config_item(
                &CompatmapDeclExpectation,
                ty.span.lo,
                |slf| slf.parse_compatmap_decl(),
                |name, decls| ConfigItemType::Compatmap(Compatmap { name, decls }),
            ),
            Meaning::XkbSymbols => self.parse_config_item(
                &SymbolsDeclExpectation,
                ty.span.lo,
                |slf| slf.parse_symbols_decl(),
                |name, decls| ConfigItemType::Symbols(Symbols { name, decls }),
            ),
            Meaning::XkbGeometry => self.parse_config_item(
                &GeometryDeclExpectation,
                ty.span.lo,
                |slf| slf.parse_geometry_decl(),
                |name, decls| ConfigItemType::Geometry(Geometry { name, decls }),
            ),
            _ => {
                const EXPECTED: &[Expected] = &[
                    Expected::Nested(FLAGS_EXPECTED),
                    Expected::Nested(CONFIG_ITEM_EXPECTED),
                ];
                Err(self.unexpected_token(EXPECTED, ty.map(Token::Ident)))
            }
        }
    }

    fn parse_composite_map(
        &mut self,
        ty: Spanned<Interned>,
    ) -> Result<Spanned<CompositeMap>, Spanned<ParserError>> {
        const EXPECTED: &[Expected] = &[Expected::Punctuation(Cbrace), Expected::AnyIdent];
        let (name, obrace) = self.parse_map_name()?;
        let items =
            self.parse_item_list(obrace.lo, Cbrace, EXPECTED, |slf| slf.parse_map_config())?;
        let hi = self.consume_token(punctuation![;])?.hi;
        let map = CompositeMap {
            ty,
            name,
            config_items: items.val,
        };
        Ok(map.spanned(ty.span.lo, hi))
    }

    fn parse_map_config(&mut self) -> Result<Spanned<NestedConfigItem>, Spanned<ParserError>> {
        let flags = self.parse_flags();
        let ty = self.parse_ident()?;
        let meaning = self
            .meaning_cache
            .get_case_insensitive(self.interner, ty.val);
        let ct = self.parse_config_item_type(ty, meaning)?;
        let lo = match flags.flags.first() {
            Some(f) => f.name.span.lo,
            None => ty.span.lo,
        };
        let item = NestedConfigItem {
            flags,
            item: ConfigItem {
                ty,
                specific: ct.val,
            },
        };
        Ok(item.spanned(lo, ct.span.hi))
    }

    fn parse_keycode_decl(&mut self) -> Result<Spanned<Decl<KeycodeDecl>>, Spanned<ParserError>> {
        self.parse_decl(
            &KeycodeDeclExpectation,
            |slf, candidate, span, expected| match candidate {
                DeclCandidate::Include(mm, i) => slf
                    .parse_include(mm, i.spanned2(span))
                    .span_map(KeycodeDecl::Include),
                DeclCandidate::KeyName(i) => slf
                    .parse_keyname_decl(i.spanned2(span))
                    .span_map(KeycodeDecl::KeyName),
                DeclCandidate::KeyAlias => slf
                    .parse_key_alias_decl(span.lo)
                    .span_map(KeycodeDecl::KeyAlias),
                DeclCandidate::Var(t) => slf
                    .parse_var_decl(Some(t.spanned2(span)))
                    .span_map(KeycodeDecl::Var),
                DeclCandidate::LedName(virt) => slf
                    .parse_led_name_decl(virt, span.lo)
                    .span_map(KeycodeDecl::LedName),
                _ => Err(slf.unexpected_decl_candidate(expected, candidate, span)),
            },
        )
    }

    fn parse_types_decl(&mut self) -> Result<Spanned<Decl<TypesDecl>>, Spanned<ParserError>> {
        self.parse_decl(
            &TypesDeclExpectation,
            |slf, candidate, span, expected| match candidate {
                DeclCandidate::Include(mm, i) => slf
                    .parse_include(mm, i.spanned2(span))
                    .span_map(TypesDecl::Include),
                DeclCandidate::KeyType => slf
                    .parse_key_type_decl(span.lo)
                    .span_map(TypesDecl::KeyType),
                DeclCandidate::Var(t) => slf
                    .parse_var_decl(Some(t.spanned2(span)))
                    .span_map(TypesDecl::Var),
                DeclCandidate::Vmod => slf.parse_vmod_decl(span.lo).span_map(TypesDecl::VMod),
                _ => Err(slf.unexpected_decl_candidate(expected, candidate, span)),
            },
        )
    }

    fn parse_compatmap_decl(
        &mut self,
    ) -> Result<Spanned<Decl<CompatmapDecl>>, Spanned<ParserError>> {
        self.parse_decl(
            &CompatmapDeclExpectation,
            |slf, candidate, span, expected| match candidate {
                DeclCandidate::Include(mm, i) => slf
                    .parse_include(mm, i.spanned2(span))
                    .span_map(CompatmapDecl::Include),
                DeclCandidate::Interpret => slf
                    .parse_interpret_decl(span.lo)
                    .span_map(CompatmapDecl::Interpret),
                DeclCandidate::GroupCompat => slf
                    .parse_group_compat_decl(span.lo)
                    .span_map(CompatmapDecl::GroupCompat),
                DeclCandidate::LedMap => slf
                    .parse_led_map_decl(span.lo)
                    .span_map(CompatmapDecl::LedMap),
                DeclCandidate::Var(t) => slf
                    .parse_var_decl(Some(t.spanned2(span)))
                    .span_map(CompatmapDecl::Var),
                DeclCandidate::Vmod => slf.parse_vmod_decl(span.lo).span_map(CompatmapDecl::VMod),
                _ => Err(slf.unexpected_decl_candidate(expected, candidate, span)),
            },
        )
    }

    fn parse_symbols_decl(&mut self) -> Result<Spanned<Decl<SymbolsDecl>>, Spanned<ParserError>> {
        self.parse_decl(
            &SymbolsDeclExpectation,
            |slf, candidate, span, expected| match candidate {
                DeclCandidate::Include(mm, i) => slf
                    .parse_include(mm, i.spanned2(span))
                    .span_map(SymbolsDecl::Include),
                DeclCandidate::Symbols => slf
                    .parse_key_symbols_decl(span.lo)
                    .span_map(SymbolsDecl::Symbols),
                DeclCandidate::Var(t) => slf
                    .parse_var_decl(Some(t.spanned2(span)))
                    .span_map(SymbolsDecl::Var),
                DeclCandidate::Vmod => slf.parse_vmod_decl(span.lo).span_map(SymbolsDecl::VMod),
                DeclCandidate::ModMap => slf
                    .parse_mod_map_decl(span.lo)
                    .span_map(SymbolsDecl::ModMap),
                _ => Err(slf.unexpected_decl_candidate(expected, candidate, span)),
            },
        )
    }

    fn parse_geometry_decl(&mut self) -> Result<Spanned<Decl<GeometryDecl>>, Spanned<ParserError>> {
        self.parse_decl(
            &GeometryDeclExpectation,
            |slf, candidate, span, expected| match candidate {
                DeclCandidate::Include(mm, i) => slf
                    .parse_include(mm, i.spanned2(span))
                    .span_map(GeometryDecl::Include),
                DeclCandidate::KeyAlias => slf
                    .parse_key_alias_decl(span.lo)
                    .span_map(GeometryDecl::KeyAlias),
                DeclCandidate::Var(t) => slf
                    .parse_var_decl(Some(t.spanned2(span)))
                    .span_map(GeometryDecl::Var),
                DeclCandidate::Shape => slf.parse_shape_decl(span.lo).span_map(GeometryDecl::Shape),
                DeclCandidate::Section => slf
                    .parse_section_decl(span.lo)
                    .span_map(GeometryDecl::Section),
                DeclCandidate::LedMap => slf
                    .parse_led_map_decl(span.lo)
                    .span_map(GeometryDecl::LedMap),
                DeclCandidate::Doodad(meaning) => slf
                    .parse_doodad_decl(meaning.spanned2(span))
                    .span_map(GeometryDecl::Doodad),
                _ => Err(slf.unexpected_decl_candidate(expected, candidate, span)),
            },
        )
    }

    fn parse_include(
        &mut self,
        mm: Spanned<MergeMode>,
        path: Spanned<Interned>,
    ) -> Result<Spanned<Include>, Spanned<ParserError>> {
        let ty = Include {
            mm,
            path,
            resolved: None,
        };
        Ok(ty.spanned2(path.span))
    }

    fn parse_config_item<T: Debug, U>(
        &mut self,
        e: &impl ParseDeclExpectation,
        lo: u64,
        f: impl FnMut(&mut Self) -> Result<Spanned<Decl<T>>, Spanned<ParserError>>,
        g: impl FnOnce(Option<Spanned<Interned>>, Decls<T>) -> U,
    ) -> Result<Spanned<U>, Spanned<ParserError>> {
        let (name, obrace) = self.parse_map_name()?;
        let items = self.parse_item_list(
            obrace.lo,
            Cbrace,
            e.expected_plus_merge_mode_plus_cbrace(),
            f,
        )?;
        let hi = self.consume_token(punctuation![;])?.hi;
        let decls = Decls { decls: items.val };
        let codes = g(name, decls);
        Ok(codes.spanned(lo, hi))
    }

    fn parse_decl<T>(
        &mut self,
        e: &impl ParseDeclExpectation,
        f: impl FnOnce(
            &mut Self,
            DeclCandidate,
            Span,
            &'static [Expected],
        ) -> Result<Spanned<T>, Spanned<ParserError>>,
    ) -> Result<Spanned<Decl<T>>, Spanned<ParserError>> {
        let merge_mode = self.parse_merge_mode();
        let expected = match merge_mode.is_some() {
            true => e.expected_plus_include(),
            false => e.expected_plus_merge_mode(),
        };
        let t = self.next(expected)?;
        let candidate = self.detect_decl_candidate(merge_mode, t);
        let ty = f(self, candidate, t.span, expected)?;
        let mut span = ty.span;
        if let Some(mm) = merge_mode {
            span.lo = mm.span.lo;
        }
        let decl = Decl {
            merge_mode,
            ty: DirectOrIncluded::Direct(ty.val),
        };
        Ok(decl.spanned2(span))
    }

    fn parse_map_name(
        &mut self,
    ) -> Result<(Option<Spanned<Interned>>, Span), Spanned<ParserError>> {
        let mut map_name = None;
        if let Some(t) = self.try_peek() {
            if let Token::String(i) = t.val {
                self.pos += 1;
                map_name = Some(i.spanned2(t.span));
            }
        }
        let expected = match map_name.is_some() {
            true => &[Expected::Punctuation(Obrace)][..],
            false => &[Expected::String, Expected::Punctuation(Obrace)],
        };
        let t = self.next(expected)?;
        if t.val != Obrace {
            return Err(self.unexpected_token(expected, t));
        }
        Ok((map_name, t.span))
    }

    fn parse_flags(&mut self) -> Flags {
        let mut flags = vec![];
        while let Some(t) = self.try_peek() {
            let Token::Ident(i) = t.val else {
                break;
            };
            macro_rules! map {
                ($($ident:ident,)*) => {
                    match self.meaning_cache.get_case_insensitive(self.interner, i) {
                        $(Meaning::$ident => Flag::$ident,)*
                        _ => break,
                    }
                };
            }
            let flag = map! {
                Partial,
                Default,
                Hidden,
                AlphanumericKeys,
                ModifierKeys,
                KeypadKeys,
                FunctionKeys,
                AlternateGroup,
            };
            flags.push(FlagWrapper {
                name: i.spanned2(t.span),
                flag,
            });
            self.pos += 1;
        }
        Flags { flags }
    }

    fn parse_interpret_decl(
        &mut self,
        lo: u64,
    ) -> Result<Spanned<InterpretDecl>, Spanned<ParserError>> {
        let match_ = self.parse_interpret_match()?;
        let obrace = self.consume_token(Obrace)?;
        let vars = self.parse_item_list(
            obrace.lo,
            Cbrace,
            &[Expected::Punctuation(Cbrace), Expected::VarDecl],
            |slf| slf.parse_var_decl(None),
        )?;
        let decl = InterpretDecl {
            match_,
            vars: vars.val,
        };
        let hi = self.consume_token(punctuation![;])?.hi;
        Ok(decl.spanned(lo, hi))
    }

    fn parse_interpret_match(&mut self) -> Result<Spanned<InterpretMatch>, Spanned<ParserError>> {
        let sym = self.parse_ident()?;
        let mut span = sym.span;
        let mut filter = None;
        if let Some(t) = self.try_peek() {
            if t.val == token![+] {
                self.pos += 1;
                let e = self.parse_expr(EXPR_TOKENS)?;
                span.hi = e.span.hi;
                filter = Some(e);
            }
        }
        let m = InterpretMatch { sym, filter };
        Ok(m.spanned2(span))
    }

    fn parse_key_alias_decl(
        &mut self,
        lo: u64,
    ) -> Result<Spanned<KeyAliasDecl>, Spanned<ParserError>> {
        let name = self.parse_keyname()?;
        self.consume_token(punctuation![=])?;
        let alias_for = self.parse_keyname()?;
        let hi = self.consume_token(punctuation![;])?.hi;
        let ty = KeyAliasDecl { name, alias_for };
        Ok(ty.spanned(lo, hi))
    }

    fn parse_key_type_decl(
        &mut self,
        lo: u64,
    ) -> Result<Spanned<KeyTypeDecl>, Spanned<ParserError>> {
        let name = self.parse_string()?;
        let obrace = self.consume_token(Obrace)?;
        let decls = self.parse_item_list(
            obrace.lo,
            Cbrace,
            &[Expected::Punctuation(Cbrace), Expected::VarDecl],
            |slf| slf.parse_var_decl(None),
        )?;
        let hi = self.consume_token(punctuation![;])?.hi;
        let decl = KeyTypeDecl {
            name,
            decls: decls.val,
        };
        Ok(decl.spanned(lo, hi))
    }

    fn parse_key_symbols_decl(
        &mut self,
        lo: u64,
    ) -> Result<Spanned<KeySymbolsDecl>, Spanned<ParserError>> {
        let key = self.parse_keyname()?;
        let obrace = self.consume_token(Obrace)?;
        let vars = parse_inline_list!(self, obrace.lo, Cbrace, SYMBOLS_VAR_DECL_EXPECTED, |slf| {
            slf.parse_symbols_var_decl()
        })?;
        let hi = self.consume_token(punctuation![;])?.hi;
        let decl = KeySymbolsDecl {
            key,
            vars: vars.val,
        };
        Ok(decl.spanned(lo, hi))
    }

    fn parse_symbols_var_decl(&mut self) -> Result<Spanned<SymbolsVarDecl>, Spanned<ParserError>> {
        let t = self.peek(SYMBOLS_VAR_DECL_EXPECTED)?;
        match t.val {
            Token::Punctuation(Obracket) => self.parse_array_init().span_map(SymbolsVarDecl::Array),
            token![!] => {
                let lo = self.next(SYMBOLS_VAR_DECL_EXPECTED)?.span.lo;
                let ident = self.parse_ident()?;
                Ok(SymbolsVarDecl::Ident(SymbolsIdent { not: true, ident })
                    .spanned(lo, ident.span.hi))
            }
            Token::Ident(_) => {
                let path = self.parse_path(None)?;
                if let Some(next) = self.try_peek() {
                    if next.val == token![=] {
                        self.pos += 1;
                        const EXPECTED: &[Expected] = &[
                            Expected::Punctuation(Obracket),
                            Expected::Nested(EXPR_TOKENS),
                        ];
                        return match self.peek(EXPECTED)?.val {
                            Token::Punctuation(Obracket) => {
                                let array = self.parse_array_init()?;
                                let lo = path.span.lo;
                                let hi = array.span.hi;
                                let decl = SymbolsVarDecl::ArrayAssign(SymbolsArrayAssign {
                                    lhs: path,
                                    array,
                                });
                                Ok(decl.spanned(lo, hi))
                            }
                            _ => {
                                let expr = self.parse_expr(EXPECTED)?;
                                let lo = path.span.lo;
                                let hi = expr.span.hi;
                                let decl = SymbolsVarDecl::ExprAssign(SymbolsExprAssign {
                                    lhs: path,
                                    expr,
                                });
                                Ok(decl.spanned(lo, hi))
                            }
                        };
                    }
                }
                let components = &path.val.components;
                let mut bad_token = components[0].index.as_ref().map(|i| i.obracket);
                if bad_token.is_none() {
                    bad_token = components.get(1).map(|c| c.ident.map(Token::Ident));
                }
                if let Some(bt) = bad_token {
                    const EXPECTED: &[Expected] = &[
                        Expected::Punctuation(punctuation![=]),
                        Expected::Punctuation(punctuation![,]),
                        Expected::Punctuation(Cbrace),
                    ];
                    return Err(self.unexpected_token(EXPECTED, bt));
                }
                Ok(SymbolsVarDecl::Ident(SymbolsIdent {
                    not: false,
                    ident: components[0].ident,
                })
                .spanned2(path.span))
            }
            _ => Err(self.unexpected_token(SYMBOLS_VAR_DECL_EXPECTED, t)),
        }
    }

    fn parse_array_init(&mut self) -> Result<Spanned<ArrayInit>, Spanned<ParserError>> {
        let obracket = self.consume_token(Obracket)?;
        let elements =
            parse_inline_list!(self, obracket.lo, Cbracket, ARRAY_ELEMENT_EXPECTED, |slf| {
                slf.parse_array_element()
            })?;
        Ok(ArrayInit {
            elements: elements.val,
        }
        .spanned2(elements.span))
    }

    fn parse_array_element(&mut self) -> Result<Spanned<ArrayElement>, Spanned<ParserError>> {
        if self.peek(ARRAY_ELEMENT_EXPECTED)?.val == Token::Punctuation(Obrace) {
            let obrace = self.consume_token(Obrace)?;
            self.parse_expr_list(obrace.lo)
                .span_map(ArrayElement::Braced)
        } else {
            self.parse_expr(ARRAY_ELEMENT_EXPECTED)
                .span_map(ArrayElement::Expr)
        }
    }

    fn parse_mod_map_decl(&mut self, lo: u64) -> Result<Spanned<ModMapDecl>, Spanned<ParserError>> {
        let modifier = self.parse_ident()?;
        let obrace = self.consume_token(Obrace)?;
        let keys = self.parse_expr_list(obrace.lo)?;
        let hi = self.consume_token(punctuation![;])?.hi;
        let ty = ModMapDecl {
            modifier,
            keys: keys.val,
        };
        Ok(ty.spanned(lo, hi))
    }

    fn parse_expr_list(
        &mut self,
        lo: u64,
    ) -> Result<Spanned<Vec<Spanned<Expr>>>, Spanned<ParserError>> {
        parse_inline_list!(self, lo, Cbrace, EXPR_TOKENS, |slf| slf
            .parse_expr(EXPR_TOKENS))
    }

    fn parse_group_compat_decl(
        &mut self,
        lo: u64,
    ) -> Result<Spanned<GroupCompatDecl>, Spanned<ParserError>> {
        let (group_name, group) = self.parse_u32()?;
        self.consume_token(punctuation![=])?;
        let val = self.parse_expr(EXPR_TOKENS)?;
        let hi = self.consume_token(punctuation![;])?.hi;
        let ty = GroupCompatDecl {
            group_name,
            group,
            val,
        };
        Ok(ty.spanned(lo, hi))
    }

    fn parse_led_map_decl(&mut self, lo: u64) -> Result<Spanned<LedMapDecl>, Spanned<ParserError>> {
        let name = self.parse_string()?;
        let obrace = self.consume_token(Obrace)?;
        let decls = self.parse_item_list(
            obrace.lo,
            Cbrace,
            &[Expected::Punctuation(Cbrace), Expected::VarDecl],
            |slf| slf.parse_var_decl(None),
        )?;
        let hi = self.consume_token(punctuation![;])?.hi;
        let decl = LedMapDecl {
            name,
            decls: decls.val,
        };
        Ok(decl.spanned(lo, hi))
    }

    fn parse_led_name_decl(
        &mut self,
        virt: bool,
        lo: u64,
    ) -> Result<Spanned<LedNameDecl>, Spanned<ParserError>> {
        if virt {
            let indicator = self.parse_ident()?;
            if self
                .meaning_cache
                .get_case_insensitive(&self.interner, indicator.val)
                != Meaning::Indicator
            {
                return Err(self.unexpected_token(
                    &[Expected::Ident(Meaning::Indicator)],
                    indicator.map(Token::Ident),
                ));
            }
        }
        let (idx_name, idx) = self.parse_u32()?;
        self.consume_token(punctuation![=])?;
        let val = self.parse_expr(EXPR_TOKENS)?;
        let hi = self.consume_token(punctuation![;])?.hi;
        let decl = LedNameDecl {
            virt,
            idx_name,
            idx,
            val,
        };
        Ok(decl.spanned(lo, hi))
    }

    fn parse_shape_decl(&mut self, lo: u64) -> Result<Spanned<ShapeDecl>, Spanned<ParserError>> {
        let name = self.parse_string()?;
        let ob = self.consume_token(Obrace)?;
        const EXPECTED: &[Expected] = &[
            Expected::Punctuation(Obracket),
            Expected::Nested(OUTLINE_TOKENS),
        ];
        let ty = if self.peek(EXPECTED)?.val == Obracket {
            self.parse_coord_list(ob.lo)?.map(ShapeDeclType::CoordList)
        } else {
            let outlines = parse_inline_list!(self, ob.lo, Cbrace, OUTLINE_TOKENS, |slf| slf
                .parse_outline())?;
            outlines.map(ShapeDeclType::OutlineList)
        };
        let hi = self.consume_token(punctuation![;])?.hi;
        let shape = ShapeDecl { name, ty };
        Ok(shape.spanned(lo, hi))
    }

    fn parse_coord_list(
        &mut self,
        lo: u64,
    ) -> Result<Spanned<Vec<Spanned<Coord>>>, Spanned<ParserError>> {
        parse_inline_list!(
            self,
            lo,
            Cbrace,
            &[Expected::Punctuation(Obracket)],
            |slf| slf.parse_coord(),
        )
    }

    fn parse_outline(&mut self) -> Result<Spanned<Outline>, Spanned<ParserError>> {
        let t = self.next(OUTLINE_TOKENS)?;
        if t.val == Obrace {
            return self
                .parse_coord_list(t.span.lo)
                .span_map(Outline::CoordList);
        }
        let Token::Ident(name) = t.val else {
            return Err(self.unexpected_token(OUTLINE_TOKENS, t));
        };
        self.consume_token(punctuation![=])?;
        const EXPECTED2: &[Expected] =
            &[Expected::Punctuation(Obrace), Expected::Nested(EXPR_TOKENS)];
        let maybe_brace = self.peek(EXPECTED2)?;
        if maybe_brace.val == Obrace {
            let obrace = self.consume_token(Obrace)?;
            let el = self.parse_coord_list(obrace.lo)?;
            let span = Span {
                lo: t.span.lo,
                hi: el.span.hi,
            };
            return Ok(Outline::CoordAssignment(CoordAssignment {
                name: name.spanned2(t.span),
                coords: el,
            })
            .spanned2(span));
        }
        let expr = self.parse_expr(EXPECTED2)?;
        let span = Span {
            lo: t.span.lo,
            hi: expr.span.hi,
        };
        Ok(Outline::ExprAssignment(ExprAssignment {
            name: name.spanned2(t.span),
            expr,
        })
        .spanned2(span))
    }

    fn parse_coord(&mut self) -> Result<Spanned<Coord>, Spanned<ParserError>> {
        let lo = self.consume_token(Obracket)?.lo;
        let x = self.parse_expr(EXPR_TOKENS)?;
        self.consume_token(punctuation![,])?;
        let y = self.parse_expr(EXPR_TOKENS)?;
        let hi = self.consume_token(Cbracket)?.lo;
        let ty = Coord { x, y };
        Ok(ty.spanned(lo, hi))
    }

    fn parse_section_decl(
        &mut self,
        lo: u64,
    ) -> Result<Spanned<SectionDecl>, Spanned<ParserError>> {
        let name = self.parse_string()?;
        let obrace = self.consume_token(Obrace)?;
        let items = self.parse_item_list(
            obrace.lo,
            Cbrace,
            &[
                Expected::Punctuation(Cbrace),
                Expected::Nested(SECTION_ITEM_EXPECTED),
            ],
            |slf| slf.parse_section_item(),
        )?;
        let hi = self.consume_token(punctuation![;])?.hi;
        let ty = SectionDecl {
            name,
            items: items.val,
        };
        Ok(ty.spanned(lo, hi))
    }

    fn parse_section_item(&mut self) -> Result<Spanned<SectionItem>, Spanned<ParserError>> {
        let t = self.next(SECTION_ITEM_EXPECTED)?;
        match self.detect_decl_candidate(None, t) {
            DeclCandidate::Row => self.parse_row_decl(t.span.lo).span_map(SectionItem::Row),
            DeclCandidate::Overlay => self
                .parse_overlay_decl(t.span.lo)
                .span_map(SectionItem::Overlay),
            DeclCandidate::LedMap => self
                .parse_led_map_decl(t.span.lo)
                .span_map(SectionItem::LedMap),
            DeclCandidate::Doodad(m) => self
                .parse_doodad_decl(m.spanned2(t.span))
                .span_map(SectionItem::Doodad),
            DeclCandidate::Var(_) => self.parse_var_decl(Some(t)).span_map(SectionItem::Var),
            c => Err(self.unexpected_decl_candidate(SECTION_ITEM_EXPECTED, c, t.span)),
        }
    }

    fn parse_row_decl(&mut self, lo: u64) -> Result<Spanned<RowBody>, Spanned<ParserError>> {
        let obrace = self.consume_token(Obrace)?;
        let items =
            self.parse_item_list(obrace.lo, Cbrace, &[Expected::Punctuation(Cbrace)], |slf| {
                slf.parse_row_body_item()
            })?;
        let hi = self.consume_token(punctuation![;])?.hi;
        Ok(RowBody { items: items.val }.spanned(lo, hi))
    }

    fn parse_row_body_item(&mut self) -> Result<Spanned<RowBodyItem>, Spanned<ParserError>> {
        const EXPECTED: &[Expected] = &[Expected::Ident(Meaning::Keys), Expected::VarDecl];
        let t = self.next(EXPECTED)?;
        match self.detect_decl_candidate(None, t) {
            DeclCandidate::Keys => self.parse_keys(t.span.lo).span_map(RowBodyItem::Keys),
            DeclCandidate::Var(_) => self.parse_var_decl(Some(t)).span_map(RowBodyItem::Var),
            c => Err(self.unexpected_decl_candidate(EXPECTED, c, t.span)),
        }
    }

    fn parse_keys(&mut self, lo: u64) -> Result<Spanned<Keys>, Spanned<ParserError>> {
        let obrace = self.consume_token(Obrace)?;
        let keys =
            parse_inline_list!(self, obrace.lo, Cbrace, KEY_EXPECTED, |slf| slf.parse_key(),)?;
        let hi = self.consume_token(punctuation![;])?.hi;
        let k = Keys { keys: keys.val };
        Ok(k.spanned(lo, hi))
    }

    fn parse_key(&mut self) -> Result<Spanned<Key>, Spanned<ParserError>> {
        let t = self.next(KEY_EXPECTED)?;
        let res = match t.val {
            Token::KeyName(i) => Key::Name(i).spanned2(t.span),
            Token::Punctuation(Obrace) => {
                let expr = self.parse_expr_list(t.span.lo)?;
                Key::Exprs(KeyExprs { exprs: expr.val }).spanned(t.span.lo, expr.span.hi)
            }
            _ => return Err(self.unexpected_token(KEY_EXPECTED, t)),
        };
        Ok(res)
    }

    fn parse_overlay_decl(
        &mut self,
        lo: u64,
    ) -> Result<Spanned<OverlayDecl>, Spanned<ParserError>> {
        let name = self.parse_string()?;
        let obrace = self.consume_token(Obrace)?;
        let items = parse_inline_list!(self, obrace.lo, Cbrace, &[Expected::KeyName], |slf| {
            let name = slf.parse_keyname()?;
            slf.consume_token(punctuation![=])?;
            let alias = slf.parse_keyname()?;
            Ok(OverlayItem { name, alias }.spanned(name.span.lo, alias.span.hi))
        },)?;
        let hi = self.consume_token(punctuation![;])?.hi;
        let item = OverlayDecl {
            name,
            items: items.val,
        };
        Ok(item.spanned(lo, hi))
    }

    fn detect_decl_candidate(
        &mut self,
        mm: Option<Spanned<MergeMode>>,
        t: Spanned<Token>,
    ) -> DeclCandidate {
        // println!("{:?}", t);
        match t.val {
            Token::KeyName(i) => DeclCandidate::KeyName(i),
            token![!] => DeclCandidate::Var(t.val),
            Token::String(i) => match mm {
                Some(mm) => DeclCandidate::Include(mm, i),
                _ => DeclCandidate::None,
            },
            Token::Ident(i) => {
                if let Some(next) = self.try_peek() {
                    if matches!(
                        next.val,
                        token![.] | token![=] | token![;] | Token::Punctuation(Obracket)
                    ) {
                        return DeclCandidate::Var(t.val);
                    }
                }
                let meaning = self.meaning_cache.get_case_insensitive(self.interner, i);
                match meaning {
                    Meaning::Interpret => DeclCandidate::Interpret,
                    Meaning::Alias => DeclCandidate::KeyAlias,
                    Meaning::Type => DeclCandidate::KeyType,
                    Meaning::Key => DeclCandidate::Symbols,
                    Meaning::Keys => DeclCandidate::Keys,
                    Meaning::ModifierMap | Meaning::ModMap | Meaning::Modmap => {
                        DeclCandidate::ModMap
                    }
                    Meaning::Group => DeclCandidate::GroupCompat,
                    Meaning::Indicator => {
                        if let Some(next) = self.try_peek() {
                            if matches!(next.val, Token::String(_)) {
                                return DeclCandidate::LedMap;
                            }
                        }
                        DeclCandidate::LedName(false)
                    }
                    Meaning::Virtual => DeclCandidate::LedName(true),
                    Meaning::Shape => DeclCandidate::Shape,
                    Meaning::Section => DeclCandidate::Section,
                    Meaning::Text | Meaning::Outline | Meaning::Solid | Meaning::Logo => {
                        DeclCandidate::Doodad(meaning)
                    }
                    Meaning::VirtualModifiers => DeclCandidate::Vmod,
                    Meaning::Row => DeclCandidate::Row,
                    Meaning::Overlay => DeclCandidate::Overlay,
                    _ => DeclCandidate::None,
                }
            }
            _ => DeclCandidate::None,
        }
    }

    fn parse_doodad_decl(
        &mut self,
        meaning: Spanned<Meaning>,
    ) -> Result<Spanned<DoodadDecl>, Spanned<ParserError>> {
        let ty = match meaning.val {
            Meaning::Text => DoodadType::Text,
            Meaning::Outline => DoodadType::Outline,
            Meaning::Solid => DoodadType::Solid,
            Meaning::Logo => DoodadType::Logo,
            _ => unreachable!(),
        };
        let name = self.parse_string()?;
        let obrace = self.consume_token(Obrace)?;
        let decls = self.parse_item_list(
            obrace.lo,
            Cbrace,
            &[Expected::Punctuation(Cbrace), Expected::VarDecl],
            |slf| slf.parse_var_decl(None),
        )?;
        let hi = self.consume_token(punctuation![;])?.hi;
        let ty = DoodadDecl {
            ty: ty.spanned2(meaning.span),
            name,
            decls: decls.val,
        };
        Ok(ty.spanned(meaning.span.lo, hi))
    }

    fn parse_item_list<E: Debug>(
        &mut self,
        lo: u64,
        terminator: Punctuation,
        expected: &'static [Expected],
        mut f: impl FnMut(&mut Self) -> Result<E, Spanned<ParserError>>,
    ) -> Result<Spanned<Vec<E>>, Spanned<ParserError>> {
        let mut res = vec![];
        loop {
            if self.peek(expected)?.val == Token::Punctuation(terminator) {
                break;
            }
            res.push(f(self)?);
            // println!("{:?}", res.last());
        }
        let hi = self.consume_token(terminator)?.hi;
        Ok(res.spanned(lo, hi))
    }

    fn parse_inline_list<T>(
        &mut self,
        lo: u64,
        terminator: Punctuation,
        expected: &'static [Expected],
        terminator_plus_comma: &'static [Expected],
        mut f: impl FnMut(&mut Self) -> Result<T, Spanned<ParserError>>,
    ) -> Result<Spanned<Vec<T>>, Spanned<ParserError>> {
        let mut res = vec![];
        let mut consume_comma = false;
        loop {
            let expected = match consume_comma {
                true => terminator_plus_comma,
                false => expected,
            };
            let t = self.peek(expected)?;
            if t.val == Token::Punctuation(terminator) {
                break;
            }
            if consume_comma {
                if t.val == token![,] {
                    consume_comma = false;
                    self.pos += 1;
                    continue;
                } else {
                    return Err(self.unexpected_token(expected, t));
                }
            }
            res.push(f(self)?);
            consume_comma = true;
        }
        let hi = self.consume_token(terminator)?.hi;
        Ok(res.spanned(lo, hi))
    }

    fn parse_string(&mut self) -> Result<Spanned<Interned>, Spanned<ParserError>> {
        let t = self.next(&[Expected::String])?;
        match t.val {
            Token::String(i) => Ok(i.spanned2(t.span)),
            _ => Err(self.unexpected_token(&[Expected::String], t)),
        }
    }

    fn parse_keyname(&mut self) -> Result<Spanned<Interned>, Spanned<ParserError>> {
        let t = self.next(&[Expected::KeyName])?;
        match t.val {
            Token::KeyName(i) => Ok(i.spanned2(t.span)),
            _ => Err(self.unexpected_token(&[Expected::KeyName], t)),
        }
    }

    fn parse_vmod_decl(&mut self, lo: u64) -> Result<Spanned<VModDecl>, Spanned<ParserError>> {
        let defs = parse_inline_list!(self, lo, punctuation![;], &[Expected::AnyIdent], |slf| {
            let name = slf.parse_ident()?;
            let mut span = name.span;
            let mut val = None;
            if let Some(t) = slf.try_peek() {
                if t.val == token![=] {
                    slf.pos += 1;
                    let e = slf.parse_expr(EXPR_TOKENS)?;
                    span.hi = e.span.hi;
                    val = Some(e);
                }
            }
            Ok(VModDef { name, val }.spanned2(span))
        })?;
        let ty = VModDecl { defs: defs.val };
        Ok(ty.spanned(lo, defs.span.hi))
    }

    fn parse_var_decl(
        &mut self,
        first: Option<Spanned<Token>>,
    ) -> Result<Spanned<VarDecl>, Spanned<ParserError>> {
        let t = match first {
            Some(first) => first,
            _ => self.next(&[Expected::AnyIdent, Expected::Punctuation(punctuation![!])])?,
        };
        if t.val == token![!] {
            let ident = self.parse_ident()?;
            let hi = self.consume_token(punctuation![;])?.hi;
            let ty = VarDecl::Ident(VarDeclIdent { not: true, ident });
            return Ok(ty.spanned(t.span.lo, hi));
        }
        let Token::Ident(i) = t.val else {
            const EXPECTED: &[Expected] =
                &[Expected::Punctuation(punctuation![!]), Expected::AnyIdent];
            return Err(self.unexpected_token(EXPECTED, t));
        };
        let maybe_semicolon =
            self.peek(&[Expected::AnyIdent, Expected::Punctuation(punctuation![;])])?;
        if maybe_semicolon.val == token![;] {
            self.pos += 1;
            let ty = VarDecl::Ident(VarDeclIdent {
                not: false,
                ident: i.spanned2(t.span),
            });
            return Ok(ty.spanned(t.span.lo, maybe_semicolon.span.hi));
        }
        let path = self.parse_path(Some(i.spanned2(t.span)))?;
        self.consume_token(punctuation![=])?;
        let expr = self.parse_expr(EXPR_TOKENS)?;
        let hi = self.consume_token(punctuation![;])?.hi;
        let ty = VarDecl::VarAssign(VarAssign { path, expr });
        Ok(ty.spanned(t.span.lo, hi))
    }

    fn parse_keyname_decl(
        &mut self,
        keyname: Spanned<Interned>,
    ) -> Result<Spanned<KeyNameDecl>, Spanned<ParserError>> {
        let _ = self.consume_token(punctuation![=])?;
        let (code_str, code) = self.parse_u32()?;
        let hi = self.consume_token(punctuation![;])?.hi;
        let ty = KeyNameDecl {
            name: keyname,
            code_str,
            code: Keycode(code),
        };
        Ok(ty.spanned(keyname.span.lo, hi))
    }

    fn parse_u32(&mut self) -> Result<(Spanned<Interned>, u32), Spanned<ParserError>> {
        let t = self.next(&[Expected::Integer])?;
        let Token::Integer(i, v) = t.val else {
            return Err(self.unexpected_token(&[Expected::Integer], t));
        };
        let Ok(v) = u32::try_from(v) else {
            return Err(self.invalid_u32(i.spanned2(t.span)));
        };
        Ok((i.spanned2(t.span), v))
    }

    fn parse_merge_mode(&mut self) -> Option<Spanned<MergeMode>> {
        let t = self.try_peek()?;
        let Token::Ident(i) = t.val else {
            return None;
        };
        let meaning = self.meaning_cache.get_case_insensitive(self.interner, i);
        let mode = match meaning {
            Meaning::Include => MergeMode::Include,
            Meaning::Augment => MergeMode::Augment,
            Meaning::Override => MergeMode::Override,
            Meaning::Replace => MergeMode::Replace,
            Meaning::Alternate => MergeMode::Alternate,
            _ => return None,
        };
        self.pos += 1;
        Some(mode.spanned2(t.span))
    }

    fn parse_path(
        &mut self,
        first: Option<Spanned<Interned>>,
    ) -> Result<Spanned<Path>, Spanned<ParserError>> {
        let first = match first {
            None => self.parse_ident()?,
            Some(first) => first,
        };
        let mut components = vec![];
        let first = self.parse_path_component(Some(first))?;
        let mut span = first.span;
        components.push(first.val);
        while let Some(t) = self.try_peek() {
            if t.val != token![.] {
                break;
            }
            self.pos += 1;
            let last = self.parse_path_component(None)?;
            span.hi = last.span.hi;
            components.push(last.val);
        }
        Ok(Path { components }.spanned2(span))
    }

    fn parse_path_component(
        &mut self,
        first: Option<Spanned<Interned>>,
    ) -> Result<Spanned<PathComponent>, Spanned<ParserError>> {
        let ident = match first {
            Some(f) => f,
            None => self.parse_ident()?,
        };
        let mut index = None;
        let mut span = ident.span;
        if let Some(t) = self.try_peek() {
            if t.val == Obracket {
                self.pos += 1;
                index = Some(PathIndex {
                    obracket: t,
                    index: self.parse_expr(EXPR_TOKENS)?,
                });
                span.hi = self.consume_token(Cbracket)?.hi;
            }
        }
        Ok(PathComponent { ident, index }.spanned2(span))
    }

    fn parse_ident(&mut self) -> Result<Spanned<Interned>, Spanned<ParserError>> {
        let t = self.next(&[Expected::AnyIdent])?;
        match t.val {
            Token::Ident(i) => Ok(i.spanned2(t.span)),
            _ => Err(self.unexpected_token(&[Expected::AnyIdent], t)),
        }
    }

    fn parse_call_args(&mut self) -> Result<Spanned<Vec<Spanned<CallArg>>>, Spanned<ParserError>> {
        let lo = self.consume_token(Oparen)?.lo;
        const EXPECTED: &[Expected] =
            &[Expected::Punctuation(Cparen), Expected::Nested(EXPR_TOKENS)];
        parse_inline_list!(self, lo, Cparen, EXPR_TOKENS, |slf| {
            if let Some(t) = slf.try_peek() {
                if let Token::Ident(_) = t.val {
                    if let Some(t) = slf.try_peek2() {
                        if matches!(t.val, token![.] | token![=] | Token::Punctuation(Obracket)) {
                            let name = slf.parse_path(None)?;
                            slf.consume_token(punctuation![=])?;
                            let expr = slf.parse_expr(EXPR_TOKENS)?;
                            let span = Span {
                                lo: name.span.lo,
                                hi: expr.span.hi,
                            };
                            return Ok(
                                CallArg::NamedParam(NamedParam { name, expr }).spanned2(span)
                            );
                        }
                    }
                }
            }
            slf.parse_expr(EXPR_TOKENS).span_map(CallArg::Expr)
        })
    }

    fn consume_token(&mut self, token: Punctuation) -> Result<Span, Spanned<ParserError>> {
        let t = self.next(token.expected())?;
        if t.val != Token::Punctuation(token) {
            return Err(self.unexpected_token(token.expected(), t));
        }
        Ok(t.span)
    }

    fn parse_expr(
        &mut self,
        expected: &'static [Expected],
    ) -> Result<Spanned<Expr>, Spanned<ParserError>> {
        self.parse_add_sub_expr(expected)
    }

    fn parse_add_sub_expr(
        &mut self,
        expected: &'static [Expected],
    ) -> Result<Spanned<Expr>, Spanned<ParserError>> {
        let mut lhs = self.parse_mul_div_expr(expected)?;
        while let Some(t) = self.try_peek() {
            let sub = match t.val {
                token![+] => false,
                token![-] => true,
                _ => break,
            };
            self.pos += 1;
            let rhs = self.parse_mul_div_expr(EXPR_TOKENS)?;
            let span = Span {
                lo: lhs.span.lo,
                hi: rhs.span.hi,
            };
            let expr = match sub {
                false => Expr::Add(Box::new(lhs), Box::new(rhs)),
                true => Expr::Sub(Box::new(lhs), Box::new(rhs)),
            };
            lhs = expr.spanned2(span);
        }
        Ok(lhs)
    }

    fn parse_mul_div_expr(
        &mut self,
        expected: &'static [Expected],
    ) -> Result<Spanned<Expr>, Spanned<ParserError>> {
        let mut lhs = self.parse_term_expr(expected)?;
        while let Some(t) = self.try_peek() {
            let div = match t.val {
                token![*] => false,
                token![/] => true,
                _ => break,
            };
            self.pos += 1;
            let rhs = self.parse_term_expr(EXPR_TOKENS)?;
            let span = Span {
                lo: lhs.span.lo,
                hi: rhs.span.hi,
            };
            let expr = match div {
                false => Expr::Mul(Box::new(lhs), Box::new(rhs)),
                true => Expr::Div(Box::new(lhs), Box::new(rhs)),
            };
            lhs = expr.spanned2(span);
        }
        Ok(lhs)
    }

    fn parse_term_expr(
        &mut self,
        expected: &'static [Expected],
    ) -> Result<Spanned<Expr>, Spanned<ParserError>> {
        let t = self.next(expected)?;
        macro_rules! un_expr {
            ($ident:ident) => {{
                let arg = self.parse_term_expr(EXPR_TOKENS)?;
                let span = Span {
                    lo: t.span.lo,
                    hi: arg.span.hi,
                };
                Ok(Expr::$ident(Box::new(arg)).spanned2(span))
            }};
        }
        match t.val {
            Token::Ident(i) => {
                let path = self.parse_path(Some(i.spanned2(t.span)))?;
                if let Some(t) = self.try_peek() {
                    if t.val == Oparen {
                        let args = self.parse_call_args()?;
                        let span = Span {
                            lo: path.span.lo,
                            hi: args.span.hi,
                        };
                        let call = Call {
                            path,
                            args: args.val,
                        };
                        return Ok(Expr::Call(call).spanned2(span));
                    }
                }
                Ok(Expr::Path(path.val).spanned2(path.span))
            }
            Token::String(s) => Ok(Expr::String(s).spanned2(t.span)),
            Token::KeyName(k) => Ok(Expr::KeyName(k).spanned2(t.span)),
            Token::Integer(i, v) => Ok(Expr::Integer(i, v).spanned2(t.span)),
            Token::Float(i, v) => Ok(Expr::Float(i, v).spanned2(t.span)),
            Token::Punctuation(Oparen) => {
                let expr = self.parse_expr(EXPR_TOKENS)?;
                let hi = self.consume_token(Cparen)?.hi;
                Ok(Expr::Parenthesized(Box::new(expr)).spanned(t.span.lo, hi))
            }
            token![+] => {
                un_expr!(UnPlus)
            }
            token![-] => {
                un_expr!(UnMinus)
            }
            token![!] => {
                un_expr!(UnNot)
            }
            token![~] => {
                un_expr!(UnInverse)
            }
            _ => Err(self.unexpected_token(expected, t)),
        }
    }
}
