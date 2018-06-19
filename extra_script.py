Import("env")

#
# Dump build environment (for debug)
# print env.Dump()
#

env.Append(
  CFLAGS=[
      "-Wno-old-style-declaration"
  ]
)

env.Append(
  LINKFLAGS=[
      "--specs=nano.specs"
  ]
)
