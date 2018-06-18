# c't-Bot Teensy framework

This is a very basic framework for the [c't-Bot][ctBot] with a [Teensy 35][Teensy] or [Teensy 36][Teensy] controller. It is licensed under the terms of the [GPLv3 license](LICENSE.md).

## Summary

This code was created for testing a new and simplified design of the c't-Bot software framework. The basic idea is to have a modular structured framework implemented in C++ built on modern language features.

### Notice

Consider this as experimental code. **If it breaks, you get to keep both pieces.**

## Setup

1. install PlatformIO core as described [here][PIOInstall]
    * can be skipped, if using VS Code IDE with [PlatformIO extension][PlatformIOVSC]
    * if you don't want to use PlatformIO core, see [manual build](#manual-build) for setup
1. clone this git repository: `git clone https://github.com/tsandmann/ctbot-teensy`
1. change to cloned repo: `cd ctbot-teensy`
1. initialize build system for...
    * commandline build: `platformio init`
    * [VS Code][VSCode] project: `platformio init --ide vscode`
    * [Eclipse CDT][EclipseCDT] project: `platformio init --ide eclipse`
    * any other environment supported by [PlatformIO][PlatformIOIDE]

## Usage

1. build project
    * commandline: `plaformio run`
    * VS Code: use “Build” button on the PlatformIO toolbar or shortcut (`ctrl/cmd+alt+b`)
    * Eclipse CDT: `Project` -> `Build Project` or shortcut (`ctrl/cmd+b`)
1. upload firmware image
    * connect USB cable to teensy board
    * commandline: `platformio run -v -e teensy36 -t upload`
    * VS Code: use “Upload” button on the PlatformIO toolbar or shortcut (`ctrl/cmd+alt+t`) and select "PlatformIO: Upload"
1. use a terminal program (e.g. minicom) to connect to the USB serial device
    * if you use minicom:
      * goto `Serial port setup` settings and set `Serial Device` to your serial device (typically sth. like `/dev/cu.usbmodemXXXXXXX` or `/dev/tty.usbmodemXXXXXXX`), `Bps/Par/Bits` to `4000000 8N1` and `Hardware Flow Control` to `No` as well as `Software Flow Control` to `No`
      * goto `Screen and keyboard` settings and set `Add carriage return` to `Yes`
1. have fun with the command line interface
    * type `help` to get a list and description of available commands
1. cruise around with the c't-Bot using
    * your remote control: **arrow keys** for *forward*, *backward*, *left*, *right* and **power button** for *stop*
    * the command line interface: `set speed 30 30` for 30% of max speed on both wheels, `set speed 0 0` (or just `set speed`) to stop
1. press **play** on remote control get a little easter egg on the command line interface :) or **I/II** to shutdown the bot :(

## Notices

* to build the documentation with Doxygen: `doxygen Doxyfile`
  * [PlantUML] has to be installed, to build the UML diagrams
  * documentation is located here: [doc/html/index.html](doc/html/index.html)
  * documentation is highly incomplete
* conventions:
  * indentation is done by 4 (four) spaces for each level, **never** ever use tabs (`\t` | `HT`)
  * follow the [C++ Core Guidelines]. There are two really worth seeing talks about it: [Bjarne Stroustrup "Writing Good C++14"][CppCon2015Stroustrup] and [Herb Sutter "Writing Good C++14... By Default"][CppCon2015Sutter]
  * documentation is done with Doxygen, use [Doxygen style 1]
    * at least all public members should be documented
    * every task's implementation (mainly its `run()` method) should be modeled by an UML sequence diagram, e.g. as for [CtBot::run()](doc/html/CtBot_run.png)
  * more to come soon
* ...

## Manual Build

* this is currently untested
* to compile for a teensy board
    1. install a C++ compiler for the arm-none-eabi architecture, e.g. arm-none-eabi-g++ with newlib C standard library and libstdc++ C++ runtime library. The compiler has to be capable to compile at least C++14.
    1. download and compile [Teensy Core Libraries for Arduino](https://github.com/PaulStoffregen/cores/tree/master/teensy3) into a static library, e.g. `libFrameworkArduino.a`
    1. download and compile [Teensy Wire Library](https://github.com/PaulStoffregen/Wire.git) into a static library, e.g. `libWire.a`
    1. compile the following files from these subdirectories of the project:
        * `src/*.cpp`
        * `lib/lcd/*.cpp`
        * `lib/PCF8574/*.cpp`
        * `lib/pid/*.cpp`
        * `lib/rc5/*.cpp`
    1. add the following subdirectories to your compiler include path:
        * `src/`
        * `lib/arduino/`
        * `lib/lcd/`
        * `lib/PCF8574/`
        * `lib/pid/`
        * `lib/rc5/`
    1. additional linker flags (beside others): `-Wl,--start-group libWire.a libFrameworkArduino.a -lm -lstdc++ -Wl,--end-group`

[ctBot]: https://www.heise.de/ct/artikel/c-t-Bot-und-c-t-Sim-284119.html
[Teensy]: https://www.pjrc.com/teensy/index.html
[PlatformIO]: https://platformio.org
[PIOGithub]: https://github.com/platformio/platformio-core
[PIOInstall]: http://docs.platformio.org/en/latest/installation.html
[PlatformIOVSC]: http://docs.platformio.org/en/latest/faq.html#faq-install-shell-commands
[VSCode]: https://github.com/Microsoft/vscode
[EclipseCDT]: https://eclipse.org
[PlatformIOIDE]: http://docs.platformio.org/en/latest/ide.html#ide
[PlantUML]: http://plantuml.com
[C++ Core Guidelines]: https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md
[CppCon2015Stroustrup]: https://youtu.be/1OEu9C51K2A
[CppCon2015Sutter]: https://youtu.be/hEx5DNLWGgA
[Doxygen style 1]: https://www.stack.nl/~dimitri/doxygen/manual/docblocks.html#cppblock
