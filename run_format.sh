#!/bin/sh
find src lib/arduino lib/pprintpp lib/rc5 -iname *.h -o -iname *.c -o -iname *.cpp | xargs clang-format -i
