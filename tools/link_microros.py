"""Link the pinned official Jazzy Arduino archive in Windows PlatformIO.

Upstream v2.0.8-jazzy uses the same esp32_toolchain.cmake and colcon.meta
for ESP32 and ESP32-S3. This integration is scoped to Arduino 2.0.17
and espressif32 6.11.0; do not silently select an archive of another distro.
"""
from pathlib import Path

Import('env')

library = (Path(env.subst('$PROJECT_LIBDEPS_DIR')) / env.subst('$PIOENV') /
           'micro_ros_arduino')
properties = library / 'library.properties'
if not properties.is_file():
    raise RuntimeError('Missing micro_ros_arduino dependency; run pio pkg install first.')
values = dict(line.split('=', 1) for line in properties.read_text(encoding='utf-8').splitlines()
              if '=' in line and not line.startswith('#'))
if values.get('version') != '2.0.8-jazzy':
    raise RuntimeError('Expected micro_ros_arduino 2.0.8-jazzy; reinstall the configured dependency.')
archive_dir = library / 'src' / 'esp32'
if not (archive_dir / 'libmicroros.a').is_file():
    raise RuntimeError('Missing Jazzy libmicroros.a; run pio pkg install.')
env.Append(LIBPATH=[str(archive_dir)], LIBS=['microros'])
