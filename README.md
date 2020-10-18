# c't-Bot Teensy framework

This is a very basic framework for the [c't-Bot][ctBot] with a [Teensy 3.5][Teensy], [Teensy 3.6][Teensy], [Teensy 4.0][Teensy] or [Teensy 4.1][Teensy] controller. It is licensed under the terms of the [GPLv3 license](LICENSE.md).

## Summary

This code was created for testing a new and simplified design of the c't-Bot software framework. The basic idea is to have a modular structured framework implemented in C++ built on modern language features.

### Notice

Consider this as experimental code. **If it breaks, you get to keep both pieces.**

## Setup

_Note for Atom IDE users:_ When installed via Atom package, Bash (Shell) used in the terminal currently does not seem to recognise the command `platformio`; use `pio` instead. (State as of May 2019)

1. Install PlatformIO core as described [here][PIOInstall]
    * can be skipped, if using VS Code IDE with [PlatformIO extension][PlatformIOVSC]
    * can be skipped, if using Atom IDE with package [platformio-ide][PlatformIOAtom]
    * if you don't want to use PlatformIO core, see [manual build](#manual-build) for setup
1. Clone this git repository: `git clone https://github.com/tsandmann/ctbot-teensy`
1. Change to cloned repo: `cd ctbot-teensy`
1. Initialize build system for...
    * Commandline build: `platformio init`
    * [VS Code][VSCode] project: `platformio init --ide vscode`
    * [Eclipse CDT][EclipseCDT] project: `platformio init --ide eclipse`
    * any other environment supported by [PlatformIO][PlatformIOIDE]

## Usage

1. Build project
    * Commandline: `platformio run -e teensy36`
    * VS Code: use “Build” button on the PlatformIO toolbar or shortcut (`ctrl/cmd+alt+b`)
    * Eclipse CDT: `Project` -> `Build Project` or shortcut (`ctrl/cmd+b`)
    * Atom: Select project folder in sidebar -> Menu `PlatformIO` -> `Build` or shortcut (`Alt+Ctrl+B`)
1. Upload firmware image
    * Connect USB cable to teensy board
    * Commandline: `platformio run -v -e teensy36 -t upload`
    * VS Code: use “Upload” button on the PlatformIO toolbar or shortcut (`ctrl/cmd+alt+t`) and select "PlatformIO: Upload"
    * Atom: Select project folder in sidebar -> Menu `PlatformIO` -> `Upload` or shortcut (`Alt+Ctrl+U`)
1. Use a terminal program (e.g. minicom) to connect to the USB serial device
    * If you use minicom:
      * goto `Serial port setup` settings and set `Serial Device` to your serial device (typically sth. like `/dev/cu.usbmodemXXXXXXX` or `/dev/tty.usbmodemXXXXXXX`), `Bps/Par/Bits` to `4000000 8N1` and `Hardware Flow Control` to `No` as well as `Software Flow Control` to `No`
      * goto `Screen and keyboard` settings and set `Add carriage return` to `Yes`
1. Have fun with the command line interface
    * type `help` to get a list and description of available commands
1. Cruise around with the c't-Bot using
    * your remote control: **arrow keys** for *forward*, *backward*, *left*, *right* and **power button** for *stop*
    * the command line interface: `set speed 30 30` for 30% of max speed on both wheels, `set speed 0 0` (or just `set speed`) to stop
1. Press **play** on remote control to show a little easter egg on the command line interface :) or **I/II** to shutdown the bot :(

## Notices

* To build the documentation with Doxygen: `doxygen Doxyfile`
  * [PlantUML] has to be installed, to build the UML diagrams
  * documentation is located here: [doc/html/index.html](doc/html/index.html)
  * documentation is highly incomplete
* Conventions:
  * Indentation is done by 4 (four) spaces for each level, **never** ever use tabs (`\t` | `HT`)
  * Source code formatting is done with [clang-format], use [.clang-format](.clang-format) for style settings
  * Follow the [C++ Core Guidelines]. There are two really worth seeing talks about it: [Bjarne Stroustrup "Writing Good C++14"][CppCon2015Stroustrup] and [Herb Sutter "Writing Good C++14... By Default"][CppCon2015Sutter]
  * Documentation is done with Doxygen, use [Doxygen style 1]
    * At least all public members should be documented
    * Every task's implementation (mainly its `run()` method) should be modeled by an UML sequence diagram, e.g. as for [CtBot::run()](doc/html/CtBot_run.png)
  * More to come soon
* ...

[ctBot]: https://www.heise.de/ct/artikel/c-t-Bot-und-c-t-Sim-284119.html
[Teensy]: https://www.pjrc.com/teensy/index.html
[PlatformIO]: https://platformio.org
[PIOGithub]: https://github.com/platformio/platformio-core
[PIOInstall]: http://docs.platformio.org/en/latest/installation.html
[PlatformIOVSC]: http://docs.platformio.org/en/latest/faq.html#faq-install-shell-commands
[PlatformIOAtom]: https://atom.io/packages/platformio-ide
[VSCode]: https://github.com/Microsoft/vscode
[EclipseCDT]: https://eclipse.org
[PlatformIOIDE]: http://docs.platformio.org/en/latest/ide.html#ide
[PlantUML]: http://plantuml.com
[clang-format]: https://clang.llvm.org/docs/ClangFormat.html
[C++ Core Guidelines]: https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md
[CppCon2015Stroustrup]: https://youtu.be/1OEu9C51K2A
[CppCon2015Sutter]: https://youtu.be/hEx5DNLWGgA
[Doxygen style 1]: https://www.stack.nl/~dimitri/doxygen/manual/docblocks.html#cppblock
[TeensyLibCore]: https://github.com/tsandmann/teensy-cores.git
[TeensyLibWire]: https://github.com/tsandmann/teensy-wire.git
