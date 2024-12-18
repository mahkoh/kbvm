use {
    crate::xkb::{
        code::Code,
        code_loader::CodeLoader,
        code_map::CodeMap,
        diagnostic::{Diagnostic, DiagnosticSink},
        interner::Interner,
        kccgst::{
            ast_cache::AstCache, embedder::embed, includer::resolve_includes, lexer::Lexer,
            parser::parse_item, resolver::resolve,
        },
        keymap::Keymap,
        meaning::MeaningCache,
        string_cooker::StringCooker,
    },
    std::{
        path::{Path, PathBuf},
        sync::Arc,
    },
};

#[derive(Default)]
pub struct Context {
    paths: Vec<Arc<PathBuf>>,
}

impl Context {
    pub fn add_include_path(&mut self, path: &Path) {
        self.paths.push(Arc::new(path.to_path_buf()));
    }

    pub fn parse_keymap(
        &self,
        diagnostics: &mut DiagnosticSink,
        path: Option<&Path>,
        kccgst: &[u8],
    ) -> Result<Keymap, Diagnostic> {
        let mut map = CodeMap::default();
        let mut interner = Interner::default();
        let mut cooker = StringCooker::default();
        let mut ast_cache = AstCache::default();
        let mut loader = CodeLoader::new(&self.paths);
        let mut meaning_cache = MeaningCache::default();
        let code = Code::new(&Arc::new(kccgst.to_vec()));
        let span = map.add(
            path.map(|p| Arc::new(p.to_path_buf())).as_ref(),
            None,
            &code,
        );
        let mut lexer = Lexer::new(None, &code, span.lo);
        let mut tokens = vec![];
        if let Err(e) = lexer.lex_item(&mut interner, &mut tokens) {
            return Err(Diagnostic::new(
                &mut map,
                e.val.diagnostic_kind(),
                e.val,
                e.span,
            ));
        }
        let parsed = parse_item(
            &mut map,
            diagnostics,
            &interner,
            &mut meaning_cache,
            &tokens,
            0,
        );
        tokens.clear();
        let mut parsed = match parsed {
            Ok(p) => p,
            Err(e) => {
                return Err(Diagnostic::new(
                    &mut map,
                    e.val.diagnostic_kind(),
                    e.val,
                    e.span,
                ));
            }
        };
        resolve_includes(
            diagnostics,
            &mut map,
            &mut ast_cache,
            &mut loader,
            &mut interner,
            &mut meaning_cache,
            &mut parsed.val,
        );
        embed(&mut parsed.val);
        let resolved = resolve(
            &mut map,
            diagnostics,
            &mut interner,
            &mut meaning_cache,
            &mut cooker,
            &parsed.val,
        );
        Ok(Keymap::from_resolved(&interner, &resolved))
    }
}
