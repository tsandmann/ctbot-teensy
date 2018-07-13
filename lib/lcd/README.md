Basic liquid crystal library
============================

This is a basic implementation of the LiquidCrystal library of the
Arduino SDK. This library is a refactored version of the one supplied
in the Arduino SDK in such a way that it simplifies its extension
to support other mechanism to communicate to LCDs such as I2C, Serial, SR,
The original library has been reworked in such a way that this will be
the base class implementing all generic methods to command an LCD based
on the Hitachi HD44780 and compatible chipsets.

It also implements a basic liquid crystal library that comes as standard
in the Arduino SDK but using an I2C IO extension board.

(c) F. Malpartida - fmalpartida@gmail.com
