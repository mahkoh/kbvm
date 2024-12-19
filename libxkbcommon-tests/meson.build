project(
    'libxkbcommon',
    'c',
    version: '1.7.0',
    default_options: [
        'c_std=c11',
        'warning_level=2',
        'b_lundef=true',
    ],
    meson_version : '>= 0.58.0', # Released on May 2021
)

cc = meson.get_compiler('c')

libdir = meson.current_source_dir() + '/../target/debug'
message(libdir)
libxkbcommon = cc.find_library('kbvm_c', dirs: libdir)

# config.h.
configh_data = configuration_data()
configh_data.set('EXIT_INVALID_USAGE', '2')
configh_data.set('_GNU_SOURCE', '1')
configh_data.set('DFLT_XKB_CONFIG_ROOT', 'NULL')
configh_data.set_quoted('LIBXKBCOMMON_VERSION', meson.project_version())
configure_file(output: 'config.h', configuration: configh_data)

# Tests
test_env = environment()
test_env.set('XKB_LOG_LEVEL', 'debug')
test_env.set('XKB_LOG_VERBOSITY', '10')
test_env.set('top_srcdir', meson.current_source_dir())
test_env.set('top_builddir', meson.current_build_dir())

test_configh_data = configuration_data()
test_configh_data.set_quoted('TEST_XKB_CONFIG_ROOT', meson.current_source_dir()/'test'/'data')
configure_file(output: 'test-config.h', configuration: test_configh_data)

libxkbcommon_test_internal = static_library(
    'xkbcommon-test-internal',
    'test/common.c',
    'test/test.h',
    'test/evdev-scancodes.h',
    dependencies: [libxkbcommon],
    include_directories: include_directories('src', 'include'),
    c_args: ['-DENABLE_PRIVATE_APIS'],
)
test_dep = declare_dependency(
    link_with: libxkbcommon_test_internal,
    dependencies: [libxkbcommon],
    include_directories: include_directories('src', 'include'),
)
test(
    'keysym',
    executable('test-keysym', 'test/keysym.c', 'test/keysym.h',
               dependencies: test_dep,
               ),
    env: test_env,
)
# test(
#     'keymap',
#     executable('test-keymap', 'test/keymap.c', 'test/keysym.h',
#                dependencies: test_dep),
#     env: test_env,
# )
# test(
#     'filecomp',
#     executable('test-filecomp', 'test/filecomp.c', dependencies: test_dep),
#     env: test_env,
# )
# test(
#     'context',
#     executable('test-context', 'test/context.c', dependencies: test_dep),
#     env: test_env,
# )
# test(
#     'rules-file',
#     executable('test-rules-file', 'test/rules-file.c', dependencies: test_dep),
#     env: test_env,
# )
# test(
#     'rules-file-includes',
#     executable('test-rules-file-includes', 'test/rules-file-includes.c', dependencies: test_dep),
#     env: test_env,
# )
# test(
#     'stringcomp',
#     executable(
#         'test-stringcomp',
#         'test/stringcomp.c',
#         'test/utils-text.c',
#         'test/utils-text.h',
#         dependencies: test_dep
#     ),
#     env: test_env,
# )
# test(
#     'buffercomp',
#     executable('test-buffercomp', 'test/buffercomp.c', dependencies: test_dep),
#     env: test_env,
# )
# test(
#     'log',
#     executable('test-log', 'test/log.c', dependencies: test_dep),
#     env: test_env,
# )
# test(
#     'atom',
#     executable('test-atom', 'test/atom.c', dependencies: test_dep),
#     env: test_env,
# )
# test(
#     'state',
#     executable('test-state', 'test/state.c', dependencies: test_dep),
#     env: test_env,
# )
# test(
#     'keyseq',
#     executable('test-keyseq', 'test/keyseq.c', dependencies: test_dep),
#     env: test_env,
# )
# test(
#     'rulescomp',
#     executable('test-rulescomp', 'test/rulescomp.c', dependencies: test_dep),
#     env: test_env,
# )
# test(
#     'compose',
#     executable(
#         'test-compose',
#         'test/compose.c',
#         'test/compose-iter.c',
#         'test/compose-iter.h',
#         'test/utils-text.c',
#         'test/utils-text.h',
#         dependencies: test_dep
#     ),
#     env: test_env,
# )
# test(
#     'utils',
#     executable(
#         'test-utils',
#         'test/utils.c',
#         'test/utils-text.c',
#         'test/utils-text.h',
#         dependencies: test_dep
#     ),
#     env: test_env,
# )
# test(
#     'symbols-leak-test',
#     find_program('test/symbols-leak-test.py'),
#     env: test_env,
#     suite: ['python-tests'],
# )
# test(
#     'modifiers',
#     executable('test-modifiers', 'test/modifiers.c', dependencies: test_dep),
#     env: test_env,
# )
