Import("env")

env.Append(CXXFLAGS=["-Wno-volatile", "-Wno-overloaded-virtual"])
