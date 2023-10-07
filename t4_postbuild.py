from os.path import join
Import("env", "projenv")

env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", env.VerboseAction(" ".join(["$OBJCOPY", "-O", "binary", "$TARGET", "$BUILD_DIR/${PROGNAME}.bin"]), "Building $TARGET"))
