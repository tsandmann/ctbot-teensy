from platformio import util
from platformio.util import get_systype


Import("env")

if "darwin_x86_64" in util.get_systype():
    env.Append(CCFLAGS=["-I/usr/local/include"])
    env.Append(LINKFLAGS=["-L/usr/local/lib"])

if "darwin_arm64" in util.get_systype():
    env.Append(CCFLAGS=["-I/opt/homebrew/include"])
    env.Append(LINKFLAGS=["-L/opt/homebrew/lib"])
