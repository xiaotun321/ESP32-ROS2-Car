"""Reject unsupported distributions before a mirror can reuse an old archive.

The ohhuo mirror at 1ae444b6 ships libmicroros and skips rebuilding when that
directory exists, even for a distro absent from its repository definitions.
Inspect definitions without importing/executing dependency code.
"""
import ast
import os
from pathlib import Path


def supported_distros(source):
    tree = ast.parse(source)
    mappings = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign) or not isinstance(node.value, ast.Dict):
            continue
        names = {t.id for t in node.targets if isinstance(t, ast.Name)}
        if names & {'dev_environments', 'mcu_environments', 'ignore_packages'}:
            mappings.append({key.value for key in node.value.keys
                             if isinstance(key, ast.Constant) and isinstance(key.value, str)})
    if not mappings:
        raise RuntimeError('Cannot verify micro-ROS distro support: repository definitions changed.')
    return set.intersection(*mappings)


def check(env):
    requested = env.GetProjectOption('board_microros_distro')
    library = Path(env.subst('$PROJECT_LIBDEPS_DIR')) / env.subst('$PIOENV') / 'micro_ros_platformio'
    definitions = library / 'microros_utils' / 'repositories.py'
    if not definitions.is_file():
        raise RuntimeError('Install the configured dependency with pio pkg install, then build again.')
    supported = supported_distros(definitions.read_text(encoding='utf-8-sig'))
    if requested not in supported:
        raise RuntimeError(
            'Configured micro_ros_platformio mirror does not support ROS 2 ' + requested +
            '. Supported: ' + ', '.join(sorted(supported)) +
            '. Its bundled archive is not evidence of Jazzy compatibility. '
            'Update the mirror or select a Jazzy-capable revision; do not relabel an older distro.')
    if os.name == 'nt':
        raise RuntimeError(
            'This micro_ros_platformio source builder requires a POSIX/Linux environment '
            '(shell tools and penv/bin/activate). Build from Linux or VS Code Remote WSL; '
            'native Windows is not supported by this configured build path.')


if 'Import' in globals():
    Import('env')
    check(env)
