use {
    crate::xkb::{
        code::Code,
        code_loader::CodeLoader,
        code_map::CodeMap,
        diagnostic::DiagnosticSink,
        interner::Interner,
        kccgst::{
            ast_cache::AstCache, embedder::embed, includer::resolve_includes, lexer::Lexer,
            parser::parse_item, resolver::resolve,
        },
        keymap::Keymap,
        meaning::MeaningCache,
        string_cooker::StringCooker,
    },
    std::{path::Path, sync::Arc},
};

#[test]
fn test() {
    let mut map = CodeMap::default();
    let mut interner = Interner::default();
    let mut diag = vec![];
    let mut diagnostics = DiagnosticSink::new(&mut diag);
    let mut cooker = StringCooker::default();
    let mut ast_cache = AstCache::default();
    let mut loader = CodeLoader::new(&[Arc::new(
        Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../xkeyboard-config")).to_path_buf(),
    )]);
    let mut meaning_cache = MeaningCache::default();
    let input = r#"
        // xkb_keymap "hol up" {
        //     xkb_keycodes {
        //         <x> = 19;
        //         alias <y> = <x>;
        //         <z> = 20;

        //         virtual indicator 32 = "X";
        //     };
        //     xkb_types {
        //         virtual_modifiers TOP, X = 0x1000, Y, NumLock = Mod2;

        //         type "abc" {
        //             modifiers = X;
        //         };

        //         type "hello" {
        //             modifiers = mod1 + TOP;
        //             level_name[Level4] = "a\nyo";
        //             map[mod1 + TOP] = Level3;
        //             preserve[mod1] = mod2;
        //             preserve[mod5] = mod5;
        //         };
        //     };
        //     xkb_compat {
        //         indicator "X" {
        //             modifiers = mod1;
        //             groups = all;
        //             controls = repeat;
        //             whichmodstate = base;
        //             whichgroupstate = any + latched;
        //         };

        //         interpret A + AnyOfOrNone(mod1+mod2) {
        //             virtualmodifier = TOP;
        //             action = SetMods(mods = modmapmods);
        //         };
        //     };
        //     xkb_symbols {
        //         name[1] = "hello world";
        //         key.repeat = true;
        //         key <x> {
        //             [ { A, A } ],
        //         };
        //         key <z> {
        //             [ KP_Space, KP_Tab ],
        //             [ x, Y ],
        //             [ u, V ],
        //         };
        //         modmap mod2 { A };
        //         modmap mod2 { <x> };
        //     };
        // };
        xkb_keymap {
            xkb_keycodes  { include "evdev+aliases(qwerty)"	};
            xkb_types     { include "complete"	};
            xkb_compat    { include "complete+grp_led(caps)"	};
            // xkb_symbols   { include "pc+de"	};
            xkb_symbols   { include "pc+us(altgr-intl)+sk(qwerty):2+de:3+inet(evdev)+group(menu_toggle)"	};
            xkb_symbols   {
                key <TLDE> {
                    groupsClamp = true
                };
            };
            xkb_geometry  { include "thinkpad(60)"	};
        };
    "#;
    let code = Code::new(&Arc::new(input.as_bytes().to_vec()));
    let span = map.add(None, None, &code);
    let mut lexer = Lexer::new(None, &code, span.lo);
    let mut tokens = vec![];
    loop {
        if let Err(e) = lexer.lex_item(&mut interner, &mut tokens) {
            diagnostics.push(&mut map, e.val.diagnostic_kind(), e);
            break;
        }
        if tokens.is_empty() {
            break;
        }
        let parsed = parse_item(
            &mut map,
            &mut diagnostics,
            &interner,
            &mut meaning_cache,
            &tokens,
            0,
        );
        tokens.clear();
        let mut parsed = match parsed {
            Ok(p) => p,
            Err(e) => {
                diagnostics.push(&mut map, e.val.diagnostic_kind(), e);
                continue;
            }
        };
        resolve_includes(
            &mut diagnostics,
            &mut map,
            &mut ast_cache,
            &mut loader,
            &mut interner,
            &mut meaning_cache,
            &mut parsed.val,
            1024,
            128,
        );
        embed(&mut parsed.val);
        // let mut out = vec![];
        // parsed
        //     .val
        //     .format(&mut Formatter::new(&interner, &mut out))
        //     .unwrap();
        // println!("{}", out.as_bstr());
        let resolved = resolve(
            &mut map,
            &mut diagnostics,
            &mut interner,
            &mut meaning_cache,
            &mut cooker,
            &parsed.val,
        );
        // println!("{:#?}", resolved);
        println!("{:#}", Keymap::from_resolved(&interner, &resolved));
    }
    for d in diag {
        println!("{}", d.with_code());
        println!();
    }
}
