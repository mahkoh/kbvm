use {
    crate::xkb::{
        Context,
        diagnostic::WriteToStderr,
        registry::{
            Layout, Model, Opt, OptGroup,
            Popularity::{Exotic, Standard},
            Registry, Variant,
        },
    },
    std::path::Path,
};

#[test]
fn without_extra() {
    let mut context = Context::builder();
    context.clear();
    context.enable_environment(false);
    let root = env!("CARGO_MANIFEST_DIR").to_string() + "/src/xkb/registry";
    context.append_path(&Path::new(&root).join("dir1"));
    context.append_path(&Path::new(&root).join("dir2"));
    let registry = context.build().default_registry(WriteToStderr);
    assert_eq!(
        registry,
        Registry {
            models: vec![
                Model {
                    name: "ijkl".to_string(),
                    description: None,
                    vendor: None,
                    popularity: Standard,
                },
                Model {
                    name: "abcd".to_string(),
                    description: None,
                    vendor: None,
                    popularity: Standard,
                },
            ],
            layouts: vec![],
            options: vec![],
        }
    );
}

#[test]
fn with_extra() {
    let mut context = Context::builder();
    context.clear();
    context.enable_environment(false);
    context.load_extra_rules(true);
    let root = env!("CARGO_MANIFEST_DIR").to_string() + "/src/xkb/registry";
    context.append_path(&Path::new(&root).join("dir1"));
    context.append_path(&Path::new(&root).join("dir2"));
    let registry = context.build().default_registry(WriteToStderr);
    assert_eq!(
        registry,
        Registry {
            models: vec![
                Model {
                    name: "ijkl".to_string(),
                    description: None,
                    vendor: None,
                    popularity: Standard
                },
                Model {
                    name: "abcd".to_string(),
                    description: None,
                    vendor: None,
                    popularity: Standard
                },
                Model {
                    name: "pc86".to_string(),
                    description: Some("Generic 86-key PC".to_string()),
                    vendor: Some("Generic".to_string()),
                    popularity: Exotic
                }
            ],
            layouts: vec![Layout {
                name: "al".to_string(),
                short_description: Some("sq".to_string()),
                description: Some("Albanian".to_string()),
                popularity: Standard,
                languages: vec!["sqi".to_string(), "yyy".to_string()],
                countries: vec!["AL".to_string(), "XX".to_string()],
                variants: vec![
                    Variant {
                        name: "plisi".to_string(),
                        short_description: Some("aoeu".to_string()),
                        description: Some("Albanian (Plisi)".to_string()),
                        popularity: Standard,
                        languages: vec![],
                        countries: vec![]
                    },
                    Variant {
                        name: "veqilharxhi".to_string(),
                        short_description: None,
                        description: Some("Albanian (Veqilharxhi)".to_string()),
                        popularity: Exotic,
                        languages: vec!["a".to_string()],
                        countries: vec!["b".to_string()]
                    }
                ]
            }],
            options: vec![OptGroup {
                allow_multiple: true,
                name: "grp".to_string(),
                description: Some("Switching to another layout".to_string()),
                popularity: Standard,
                options: vec![Opt {
                    name: "grp:switch".to_string(),
                    short_description: None,
                    description: Some("Right Alt (while pressed)".to_string()),
                    popularity: Standard
                }]
            }]
        }
    );
}
