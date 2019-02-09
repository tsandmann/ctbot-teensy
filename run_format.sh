#!/bin/sh
find src lib/arduino lib/arduino_native lib/ctsim_native lib/freertos_native lib/lcd_native lib/pprintpp lib/rc5 -iname *.h -o -iname *.c -o -iname *.cpp | xargs clang-format -i
