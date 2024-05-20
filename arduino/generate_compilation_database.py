# From: https://github.com/platformio/platformio-core/issues/4711#issuecomment-1912763259
import subprocess

if "compiledb" not in COMMAND_LINE_TARGETS:  # avoids infinite recursion
    subprocess.run(["pio", "run", "-t", "compiledb"])