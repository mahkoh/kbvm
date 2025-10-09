use {
    crate::{config::DEFAULT_INCLUDE_DIR, xkb::Context},
    std::{env::set_var, sync::Once},
};

fn init_env() {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| unsafe {
        set_var("HOME", "a");
        set_var("XDG_CONFIG_HOME", "b");
        set_var("XKB_CONFIG_EXTRA_PATH", "c");
        set_var("XKB_CONFIG_ROOT", "d");
        set_var("XLOCALEDIR", "e");
        set_var("XCOMPOSEFILE", "f");
        set_var("XKB_DEFAULT_RULES", "g");
        set_var("XKB_DEFAULT_MODEL", "h");
        set_var("XKB_DEFAULT_LAYOUT", "i");
        set_var("XKB_DEFAULT_VARIANT", "j");
        set_var("XKB_DEFAULT_OPTIONS", "k");
    })
}

#[test]
fn default_env() {
    let mut context = Context::builder();
    context.clear();
    context.enable_environment(false);
    let context = context.build();
    let e = &context.env;
    assert_eq!(e.home, None);
    assert_eq!(e.xlocaledir, "/usr/share/X11/locale");
    assert_eq!(e.xcomposefile, None);
    assert_eq!(e.xkb_default_rules, "evdev");
    assert_eq!(e.xkb_default_model, "pc105");
    assert_eq!(e.xkb_default_layout, "us");
    assert_eq!(e.xkb_default_variant, "");
    assert_eq!(e.xkb_default_options, "");
    assert_eq!(e.xkb_config_extra_path, "/etc/xkb");
    assert_eq!(e.xkb_config_root, DEFAULT_INCLUDE_DIR);
}

#[test]
fn env_env() {
    init_env();
    let mut context = Context::builder();
    context.clear();
    context.enable_environment(true);
    let context = context.build();
    let e = &context.env;
    assert_eq!(e.home.as_deref(), Some("a"));
    assert_eq!(e.xkb_config_extra_path, "c");
    assert_eq!(e.xkb_config_root, "d");
    assert_eq!(e.xlocaledir, "e");
    assert_eq!(e.xcomposefile.as_deref(), Some("f"));
    assert_eq!(e.xkb_default_rules, "g");
    assert_eq!(e.xkb_default_model, "h");
    assert_eq!(e.xkb_default_layout, "i");
    assert_eq!(e.xkb_default_variant, "j");
    assert_eq!(e.xkb_default_options, "k");
}

#[test]
fn system_paths() {
    let mut context = Context::builder();
    context.clear();
    context.enable_environment(false);
    context.enable_default_includes(true);
    let c = context.build();
    assert_eq!(c.paths.len(), 2);
    assert_eq!(c.paths[0].as_os_str(), "/etc/xkb");
    assert_eq!(c.paths[1].as_os_str(), DEFAULT_INCLUDE_DIR);
}

#[test]
fn system_and_env_paths() {
    init_env();
    let mut context = Context::builder();
    context.clear();
    context.enable_default_includes(true);
    context.enable_environment(true);
    let c = context.build();
    assert_eq!(c.paths.len(), 4);
    assert_eq!(c.paths[0].as_os_str(), "b/xkb");
    assert_eq!(c.paths[1].as_os_str(), "a/.xkb");
    assert_eq!(c.paths[2].as_os_str(), "c");
    assert_eq!(c.paths[3].as_os_str(), "d");
}

#[test]
fn prefix_and_suffix() {
    let mut context = Context::builder();
    context.clear();
    context.enable_environment(false);
    context.enable_default_includes(true);
    context.prepend_path("prefix1");
    context.prepend_path("prefix2");
    context.append_path("suffix1");
    context.append_path("suffix2");
    let c = context.build();
    assert_eq!(c.paths.len(), 6);
    assert_eq!(c.paths[0].as_os_str(), "prefix2");
    assert_eq!(c.paths[1].as_os_str(), "prefix1");
    assert_eq!(c.paths[2].as_os_str(), "/etc/xkb");
    assert_eq!(c.paths[3].as_os_str(), DEFAULT_INCLUDE_DIR);
    assert_eq!(c.paths[4].as_os_str(), "suffix1");
    assert_eq!(c.paths[5].as_os_str(), "suffix2");
}

#[test]
fn default_extra_paths() {
    let mut context = Context::builder();
    context.clear();
    context.enable_environment(false);
    let c = context.build();
    assert_eq!(c.load_extra_rules, false);
}

#[test]
fn enable_extra_paths() {
    let mut context = Context::builder();
    context.clear();
    context.enable_environment(false);
    context.load_extra_rules(true);
    let c = context.build();
    assert_eq!(c.load_extra_rules, true);
}

#[test]
fn default_limits() {
    let mut context = Context::builder();
    context.clear();
    context.enable_environment(false);
    let c = context.build();
    assert_eq!(c.max_includes, 1024);
    assert_eq!(c.max_include_depth, 128);
    assert_eq!(c.max_runtime, 500_000);
    assert_eq!(c.max_compose_rules, 100_000);
}

#[test]
fn custom_limits() {
    let mut context = Context::builder();
    context.clear();
    context.enable_environment(false);
    context.max_includes(123);
    context.max_include_depth(456);
    context.max_runtime(789);
    context.max_compose_rules(12);
    let c = context.build();
    assert_eq!(c.max_includes, 123);
    assert_eq!(c.max_include_depth, 456);
    assert_eq!(c.max_runtime, 789);
    assert_eq!(c.max_compose_rules, 12);
}
