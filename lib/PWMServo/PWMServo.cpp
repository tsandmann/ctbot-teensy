/*
  PWMServo.cpp - Hardware Servo Timer Library
  http://arduiniana.org/libraries/pwmservo/
  Author: Jim Studt, jim@federated.com
  Copyright (c) 2007 David A. Mellis.  All right reserved.
  renamed to PWMServo by Mikal Hart
  ported to other chips by Paul Stoffregen

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "PWMServo.h"
#include <Arduino.h>


uint32_t PWMServo::attachedpins[(NUM_DIGITAL_PINS + 31) / 32]; // 1 bit per digital pin

PWMServo::PWMServo() : pin(255U), angle(NO_ANGLE) {}

bool PWMServo::attach(const uint8_t pinArg, const uint16_t min, const uint16_t max) {
    if (pinArg >= NUM_DIGITAL_PINS) {
        return false;
    }
    if (! digitalPinHasPWM(pinArg)) {
        return false;
    }

    pin = pinArg;
    analogWriteFrequency(pin, 50.f);
    min16 = min >> 4; // FIXME: improve resolution?
    max16 = max >> 4;
    angle = NO_ANGLE;
    digitalWriteFast(pin, false);
    pinMode(pin, OUTPUT);
    attachedpins[pin >> 5] |= (1 << (pin & 31));
    return true;
}

void PWMServo::write(const int angleArg) {
    if (pin >= NUM_DIGITAL_PINS) {
        return;
    }
    if (angleArg < 0) {
        angle = 0;
    } else if (angleArg > 180) {
        angle = 180;
    } else {
        angle = angleArg;
    }
    // float usec = (float)((max16 - min16) << 4) * ((float)angle / 180.f) + (float)(min16 << 4);
    // uint32_t duty = (int)(usec / 20000.f * 4096.f);
    const uint32_t us = (((max16 - min16) * 46603 * angle) >> 11) + (min16 << 12); // us * 256
    const uint32_t duty = (us * 3355) >> 22;

#if TEENSYDUINO >= 137
    // noInterrupts(); // FIXME: why?
    const uint32_t oldres = analogWriteResolution(12);
    analogWrite(pin, duty);
    analogWriteResolution(oldres);
    // interrupts();
#else
    analogWriteResolution(12);
    analogWrite(pin, duty);
#endif
}

void PWMServo::disable() const {
    analogWrite(pin, 0);
}

uint8_t PWMServo::attached() const {
    if (pin >= NUM_DIGITAL_PINS) {
        return 0;
    }
    return (attachedpins[pin >> 5] & (1 << (pin & 31))) ? 1 : 0;
}

// #endif
