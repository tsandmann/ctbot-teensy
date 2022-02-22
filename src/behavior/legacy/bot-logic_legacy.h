/*
 * This file is part of the ct-Bot teensy framework.
 * Copyright (c) 2019 Timo Sandmann
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    bot-logic_legacy.h
 * @brief   Support layer for legacy behaviors
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#pragma once

#include "avr/pgmspace.h"

#ifdef __cplusplus
#include <cstddef>
#include <cstdint>
namespace ctbot {
namespace legacy {
#else
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#endif // __cplusplus


#if defined(__GNUC__) && __GNUC__ >= 7
#define CASE_NO_BREAK __attribute__((fallthrough))
#else
#define CASE_NO_BREAK ((void) 0)
#endif /* __GNUC__ >= 7 */


/** 2D-Position. Ist effizienter, als Zeiger auf X- und Y-Anteil */
typedef struct {
    int16_t x; /**< X-Anteil der Position */
    int16_t y; /**< Y-Anteil der Position */
} position_t;

#ifndef __cplusplus
#define True 1
#define False 0
#define EEPROM

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 (M_PI / 2.)
#endif

static const uint8_t BEHAVIOUR_INACTIVE = 0; /**< Verhalten ist aus */
static const uint8_t BEHAVIOUR_ACTIVE = 1; /**< Verhalten ist an */

static const uint8_t BEHAVIOUR_NOOVERRIDE = 0; /**< Konstanten, wenn Verhalten beim Aufruf alte Wuensche nicht ueberschreiben sollen */
static const uint8_t BEHAVIOUR_OVERRIDE = 1; /**< Konstante, wenn Verhalten beim Aufruf alte Wuensche ueberschreiben sollen */
static const uint8_t BEHAVIOUR_FOREGROUND = 0; /**< Konstante, wenn Verhalten im Vordergrund laufen sollen (default), also z.B. die Motoren beeinflussen */
static const uint8_t BEHAVIOUR_BACKGROUND = 2; /**< Konstante, wenn Verhalten im Hintergrund laufen sollen */

static const uint8_t BEHAVIOUR_SUBFAIL = 0; /**< Konstante fuer Behaviour_t->subResult: Aufgabe nicht abgeschlossen */
static const uint8_t BEHAVIOUR_SUBSUCCESS = 1; /**< Konstante fuer Behaviour_t->subResult: Aufgabe erfolgreich abgeschlossen */
static const uint8_t BEHAVIOUR_SUBRUNNING = 2; /**< Konstante fuer Behaviour_t->subResult: Aufgabe wird noch bearbeitet */
static const uint8_t BEHAVIOUR_SUBCANCEL = 3; /**< Konstante fuer Behaviour_t->subResult: Aufgabe wurde unterbrochen */
static const uint8_t BEHAVIOUR_SUBBACKGR = 4; /**< Konstange fuer Behaviour_t->subResult: Aufgabe wird im Hintergrund bearbeitet */


/** Aufteilung eines Words in zwei Bytes */
typedef union {
    uint16_t word;
    struct {
        uint8_t byte_0;
        uint8_t byte_1;
    } bytes;
} eeprom_word_t;

/** Aufteilung eines DWords in vier Bytes */
typedef union {
    uint32_t dword;
    struct {
        uint8_t byte_0;
        uint8_t byte_1;
        uint8_t byte_2;
        uint8_t byte_3;
    } bytes;
} eeprom_dword_t;

/**
 * Schreibt ein Byte in das EEPROM
 * @param *address  Adresse des Bytes im EEPROM
 * @param value     Das zu schreibende Byte
 */
static inline void ctbot_eeprom_write_byte(uint8_t* address, uint8_t value) {
    *address = value;
}

/**
 * Liest ein Byte aus dem EEPROM
 * @param *address  Adresse des zu lesenden Bytes im EEPROM
 * @return          Das zu lesende Byte
 */
static inline uint8_t ctbot_eeprom_read_byte(const uint8_t* address) {
    return *address;
}

/**
 * Liest die zwei Bytes eines Words aus dem EEPROM.
 * @param *address  Adresse des Words im EEPROM
 * @return          Das zu lesende Word
 */
static inline uint16_t ctbot_eeprom_read_word(const uint16_t* address) {
    eeprom_word_t data;
    size_t eeprom_addr = (size_t) address;

    data.bytes.byte_0 = ctbot_eeprom_read_byte((const uint8_t*) eeprom_addr++);
    data.bytes.byte_1 = ctbot_eeprom_read_byte((const uint8_t*) eeprom_addr);
    return data.word;
}

/**
 * aktualisiert ein Byte im EEPROM.
 * @param *address  Adresse des Bytes im EEPROM
 * @param value     Neuer Wert des Bytes
 */
static inline void ctbot_eeprom_update_byte(uint8_t* address, const uint8_t value) {
    if (ctbot_eeprom_read_byte(address) != value) {
        ctbot_eeprom_write_byte(address, value);
    }
}

/**
 * aktualisiert die zwei Bytes eines Words im EEPROM.
 * @param *address  Adresse des Words im EEPROM
 * @param value     Neuer Wert des Words
 */
static inline void ctbot_eeprom_update_word(uint16_t* address, const uint16_t value) {
    eeprom_word_t data;
    data.word = value;
    size_t eeprom_addr = (size_t) address;

    if (ctbot_eeprom_read_byte((uint8_t*) eeprom_addr) != data.bytes.byte_0) {
        ctbot_eeprom_write_byte((uint8_t*) eeprom_addr, data.bytes.byte_0);
    }
    eeprom_addr++;
    if (ctbot_eeprom_read_byte((uint8_t*) eeprom_addr) != data.bytes.byte_1) {
        ctbot_eeprom_write_byte((uint8_t*) eeprom_addr, data.bytes.byte_1);
    }
}

#endif // __cplusplus

/** Verwaltungsstruktur fuer die Verhaltensroutinen */
typedef struct _Behaviour_t {
    void (*work)(struct _Behaviour_t*); /**< Zeiger auf die Funktion, die das Verhalten bearbeitet */
    uint8_t priority; /**< Prioritaet */
    struct _Behaviour_t* caller; /**< aufrufendes Verhalten */
    unsigned active : 1; /**< Ist das Verhalten aktiv */
    unsigned subResult : 3; /**< War das aufgerufene Unterverhalten erfolgreich (== 1)? */
    struct _Behaviour_t* next; /**< Naechster Eintrag in der Liste */
} Behaviour_t;

/** Dieser Typ definiert eine Funktion die das eigentliche Verhalten ausfuehrt */
typedef void (*BehaviourFunc_t)(Behaviour_t*);

typedef struct {
    unsigned override : 1; /**< 0 wenn Verhalten beim Aufruf alte Wuensche nicht ueberschreiben sollen; 1 sonst */
    unsigned background : 1; /**< 0 wenn Verhalten im Vordergrund laufen sollen (default), also z.B. die Motoren beeinflussen; 1 sonst */
} behaviour_mode_t;

/** Repraesentation eines Bits, dem ein Byte-Wert zugewiesen werden kann */
typedef union {
    uint8_t byte;
    unsigned bit : 1;
} bit_t;

/** In diesem Typ steht die Drehrichtung, auch wenn die Speed-Variablen bereits wieder auf Null sind */
typedef union {
    struct {
        unsigned left : 1;
        unsigned right : 1;
    };
    uint8_t raw;
} direction_t;

/** Datenfeld fuer den Zugriff auf die LEDs */
typedef struct {
    unsigned rechts : 1; /**< LED in Fahrichtung rechts */
    unsigned links : 1; /**< LED in Fahrichtung links */
    unsigned rot : 1; /**< LED Rot */
    unsigned orange : 1; /**< LED Orange */
    unsigned gelb : 1; /**< LED Gelb */
    unsigned gruen : 1; /**< LED Gruen */
    unsigned tuerkis : 1; /**< LED Tuerkis */
    unsigned weiss : 1; /**< LED Weiss */
} led_t;

#ifdef __cplusplus
extern "C" {
#endif

extern int16_t speedWishLeft; /**< Puffervariable fuer die Verhaltensfunktionen absolute Geschwindigkeit links */
extern int16_t speedWishRight; /**< Puffervariable fuer die Verhaltensfunktionen absolute Geschwindigkeit rechts */
extern float factorWishLeft; /**< Puffervariable fuer die Verhaltensfunktionen Modifikationsfaktor links */
extern float factorWishRight; /**< Puffervariable fuer die Verhaltensfunktionen Modifikationsfaktor rechts */

extern int16_t sensEncL;
extern int16_t sensEncR;
extern int16_t sensMouseX;
extern int16_t sensMouseY;
extern int16_t sensDistL;
extern int16_t sensDistR;
extern int16_t sensDistLRaw;
extern int16_t sensDistRRaw;
extern uint8_t sensDistLToggle;
extern uint8_t sensDistRToggle;
extern int16_t sensBorderL;
extern int16_t sensBorderR;
extern int16_t sensLineL;
extern int16_t sensLineR;
extern int16_t sensLDRL;
extern int16_t sensLDRR;

extern uint8_t sensTrans;
extern uint8_t sensDoor;
extern uint8_t sensError;

extern int16_t x_pos;
extern int16_t y_pos;
extern float heading;
extern float heading_mou;
extern int16_t heading_int;
extern int16_t heading_10_int;
extern float heading_sin;
extern float heading_cos;
extern int16_t v_enc_left;
extern int16_t v_enc_right;
extern int16_t v_mou_left;
extern int16_t v_mou_right;

Behaviour_t* switch_to_behaviour(Behaviour_t* from, void (*to)(Behaviour_t*), uint8_t mode);
void exit_behaviour(Behaviour_t* data, uint8_t state);
void return_from_behaviour(Behaviour_t* data);
uint8_t behaviour_is_activated(BehaviourFunc_t function);
Behaviour_t* activateBehaviour(Behaviour_t* from, void (*to)(Behaviour_t*));
void deactivate_behaviour(Behaviour_t* beh);
void deactivate_called_behaviours(Behaviour_t* caller);
Behaviour_t* get_behaviour(BehaviourFunc_t function);

static inline void deactivateBehaviour(BehaviourFunc_t function) {
    deactivate_behaviour(get_behaviour(function));
}

void legacy_caller_behaviour(Behaviour_t* data);
void legacy_abort(Behaviour_t* caller);
void legacy_abort_helper(Behaviour_t*);

Behaviour_t* bot_servo_wrapper(Behaviour_t* caller, uint8_t servo, uint8_t pos);
Behaviour_t* bot_turn_speed_wrapper(Behaviour_t* caller, int16_t degrees, int16_t minspeed, int16_t maxspeed);
Behaviour_t* bot_turn_maxspeed_wrapper(Behaviour_t* caller, int16_t degrees, int16_t speed);
Behaviour_t* bot_turn_wrapper(Behaviour_t* caller, int16_t degrees);
Behaviour_t* bot_drive_wrapper(Behaviour_t* caller, int16_t distance);
Behaviour_t* bot_square_wrapper(Behaviour_t* caller);
Behaviour_t* bot_follow_line_wrapper(Behaviour_t* caller, uint8_t search);

int8_t register_emergency_proc(void (*function)(void));
void start_registered_emergency_procs(void);

uint32_t timer_get_tickCount32(void);

/**
 * Berechnet die Differenz eines Winkels zur aktuellen Botausrichtung
 * @param angle Winkel [Grad] zum Vergleich mit heading
 * @return      Winkeldifferenz [Grad] in Richtung der derzeitigen Botdrehung, -1, falls Bot geradeaus faehrt oder steht
 */
int16_t turned_angle(int16_t angle);

void LED_set(uint8_t led);

void servo_set(uint8_t servo, uint8_t pos);
uint8_t servo_get_pos(uint8_t servo);
uint8_t servo_get_active(uint8_t servo);

void display_clear(void);
void display_cursor(int16_t row, int16_t column);
uint8_t display_printf(const char* format, ...) __attribute__((format(printf, 1, 2)));
uint8_t display_puts(const char* text);

/**
 * Allgemeines Debugging (Methode DiesUndDas wurde mit Parameter SoUndSo aufgerufen ...)
 */
#define LOG_DEBUG(format, ...)                      \
    {                                               \
        static const char _data[] PROGMEM = format; \
        log_debug(_data, ##__VA_ARGS__);            \
    }

/**
 * Allgemeine Informationen (Programm gestartet, Programm beendet, Verbindung zu Host Foo aufgebaut, Verarbeitung dauerte SoUndSoviel Sekunden ...)
 */
#define LOG_INFO(format, ...)                       \
    {                                               \
        static const char _data[] PROGMEM = format; \
        log_info(_data, ##__VA_ARGS__);             \
    }

/**
 * Fehler aufgetreten, Bearbeitung wurde alternativ fortgesetzt.
 */
#define LOG_ERROR(format, ...)                      \
    {                                               \
        static const char _data[] PROGMEM = format; \
        log_error(_data, ##__VA_ARGS__);            \
    }

#define LOG_WARN LOG_INFO
#define LOG_FATAL LOG_ERROR

FLASHMEM size_t log_error(const char* format, ...) __attribute__((format(printf, 1, 2)));
FLASHMEM size_t log_info(const char* format, ...) __attribute__((format(printf, 1, 2)));
FLASHMEM size_t log_debug(const char* format, ...) __attribute__((format(printf, 1, 2)));

/**
 * Makro zur Umrechnung von Ticks in ms
 */
#define TICKS_TO_MS(ticks) ((ticks) * (176 / 8) / (1000 / 8))

/**
 * Makro zur Umrechnung von ms in Ticks
 */
#define MS_TO_TICKS(ms) ((ms) * (1000 / 8) / (176 / 8))

#define TIMER_GET_TICKCOUNT_8 (uint8_t) timer_get_tickCount32() /**< Systemzeit [176 us] in 8 Bit */
#define TIMER_GET_TICKCOUNT_16 (uint16_t) timer_get_tickCount32() /**< Systemzeit [176 us] in 16 Bit */
#define TIMER_GET_TICKCOUNT_32 timer_get_tickCount32() /**< Systemzeit [176 us] in 32 Bit */

/**
 * Prueft, ob seit dem letzten Aufruf mindestens ms Millisekunden vergangen sind.
 * 32-Bit Version, fuer Code, der (teilweise) seltener als alle 11 s aufgerufen wird.
 * @param old_ticks Zeiger auf eine Variable, die einen Timestamp speichern kann
 * @param ms        Zeit in ms, die vergangen sein muss, damit True geliefert wird
 * @return          True oder False
 */
static inline uint8_t timer_ms_passed_32(uint32_t* old_ticks, uint32_t ms) {
    uint32_t ticks = timer_get_tickCount32();
    if ((uint32_t) (ticks - *old_ticks) > MS_TO_TICKS(ms)) {
        *old_ticks = ticks;
        return 1;
    }
    return 0;
}

/**
 * Prueft, ob seit dem letzten Aufruf mindestens ms Millisekunden vergangen sind.
 * Siehe auch timer_ms_passed_32()
 * 16-Bit Version, fuer Code, der alle 11 s oder oefter ausgefuehrt werden soll.
 * @param old_ticks Zeiger auf eine Variable, die einen Timestamp speichern kann
 * @param ms        Zeit in ms, die vergangen sein muss, damit True geliefert wird
 * @return          True oder False
 */
static inline uint8_t timer_ms_passed_16(uint16_t* old_ticks, uint32_t ms) {
    uint16_t ticks = (uint16_t) timer_get_tickCount32();
    if ((uint16_t) (ticks - *old_ticks) > MS_TO_TICKS(ms)) {
        *old_ticks = ticks;
        return 1;
    }
    return 0;
}

/**
 * Prueft, ob seit dem letzten Aufruf mindestens ms Millisekunden vergangen sind.
 * Siehe auch timer_ms_passed_32()
 * 8-Bit Version, fuer Code, der alle 40 ms oder oefter ausgefuehrt werden soll.
 * @param old_ticks Zeiger auf eine Variable, die einen Timestamp speichern kann
 * @param ms        Zeit in ms, die vergangen sein muss, damit True geliefert wird
 * @return          True oder False
 */
static inline uint8_t timer_ms_passed_8(uint8_t* old_ticks, uint16_t ms) {
    uint8_t ticks = (uint8_t) timer_get_tickCount32();
    if ((uint8_t) (ticks - *old_ticks) > MS_TO_TICKS(ms)) {
        *old_ticks = ticks;
        return 1;
    }
    return 0;
}

/**
 * Die Funktion gibt aus, ob sich innerhalb einer gewissen Entfernung ein Objekt-Hindernis befindet.
 * @param distance  Entfernung in mm, bis zu welcher ein Objekt gesichtet wird.
 * @return          Gibt False (0) zurueck, wenn kein Objekt innerhalb von distance gesichtet wird. Ansonsten die Differenz
 *                  zwischen dem linken und rechten Sensor. Negative Werte besagen, dass das Objekt naeher am linken, positive, dass
 *                  es naeher am rechten Sensor ist. Sollten beide Sensoren den gleichen Wert haben, gibt die Funktion 1 zurueck, um
 *                  von False unterscheiden zu koennen.
 */
static inline int16_t is_obstacle_ahead(int16_t distance) {
    if (sensDistL > distance && sensDistR > distance) {
        return 0;
    }
    if (sensDistL - sensDistR == 0) {
        return 1;
    }
    return (int16_t) (sensDistL - sensDistR);
}

#ifndef __cplusplus
/**
 * Rundet float und gibt das Ergebnis als int zurueck.
 * Selbst implementiert, weil es kein roundf() in der avr-libc gibt.
 * @param x Eingabewert
 * @return  roundf(x)
 */
static inline int16_t iroundf(float x) {
    if (x >= 0.f) {
        return (int16_t) (x + 0.5f);
    }
    return (int16_t) (x - 0.5f);
}

/**
 * Wandelt einen Winkel von Grad in Bogenmass um
 * @param degree    Winkel [Grad]
 * @return          Winkel [Bogenmass]
 */
static inline float rad(float degree) {
    return degree * (M_PI / 180.f);
}

/**
 * Wandelt einen Winkel von Bogenmass in Grad um
 * @param radian    Winkel [Bogenmass]
 * @return          Winkel [Grad]
 */
static inline float deg(float radian) {
    return radian / (M_PI / 180.f);
}

/**
 * Berechnung einer Winkeldifferenz zwischen dem aktuellen Standpunkt und einem anderen Ort
 * @param xDiff x-Differenz
 * @param yDiff y-Differenz
 * @return      Berechnete Winkeldifferenz [Bogenmass]
 */
static inline float calc_angle_diff_rad(int16_t xDiff, int16_t yDiff) {
    const float newHeading = atan2((float) yDiff, (float) xDiff);

    float toTurn = newHeading - rad(heading);
    if (toTurn > M_PI) {
        toTurn -= 2.f * M_PI;
    }
    if (toTurn < -M_PI) {
        toTurn += 2.f * M_PI;
    }

    return toTurn;
}

/**
 * Berechnung einer Winkeldifferenz zwischen dem aktuellen Standpunkt und einem anderen Ort
 * @param xDiff x-Differenz
 * @param yDiff y-Differenz
 * @return      Berechnete Winkeldifferenz [Grad]
 */
static inline float calc_angle_diff(int16_t xDiff, int16_t yDiff) {
    return deg(calc_angle_diff_rad(xDiff, yDiff));
}

/**
 * Ermittlung des Quadrat-Abstands zwischen zwei Punkten
 * @param x1    X-Koordinate des ersten Punktes
 * @param y1    y-Koordinate des ersten Punktes
 * @param x2    X-Koordinate des zweiten Punktes
 * @param y2    Y-Koordiante des zweiten Punktes
 * @return      liefert Quadrat-Abstand zwischen den zwei Punkten
 */
static inline int32_t get_dist(int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
    int16_t xt = x2 - x1;
    int16_t yt = y2 - y1;

    /* Abstandsermittlung nach dem guten alten Pythagoras ohne Ziehen der Wurzel */
    return (int32_t) ((int32_t) xt * (int32_t) xt) + (int32_t) ((int32_t) yt * (int32_t) yt);
}

/**
 * Ermittelt die Koordinaten eines Punktes, der um dx mm in x- und
 * dy mm in y-Richtung gegenueber der aktuellen Bot-Position verschoben ist.
 * \param alpha	Winkel zum Punkt [Grad]
 * \param dx	x-Komponente des Verschiebungsvektors [mm]
 * \param dy	y-Komponente des Verschiebungsvektors [mm]
 * \return		Gesuchter Punkt
 */
static inline position_t calc_point_in_distance(float alpha, int16_t dx, int16_t dy) {
    float h = rad(alpha);
    float cos_h = cosf(h);
    float sin_h = sinf(h);

    position_t dest;
    dest.x = x_pos + (int16_t) ((dx * cos_h) - (dy * sin_h));
    dest.y = y_pos + (int16_t) ((dy * cos_h) + (dx * sin_h));
    return dest;
}

#endif // __cplusplus

extern const float WHEEL_TO_WHEEL_DIAMETER;
extern const float WHEEL_DIAMETER;
extern const int ENCODER_MARKS;
extern const float WHEEL_PERIMETER;

extern const int DISTSENSOR_POS_FW;
extern const int DISTSENSOR_POS_SW;
extern const int BORDERSENSOR_POS_FW;
extern const int BORDERSENSOR_POS_SW;

extern const int SENS_IR_MIN_DIST;
extern const int SENS_IR_MAX_DIST;
extern const int SENS_IR_INFINITE;
extern const int SENS_IR_SAFE_DIST;

extern const int BORDER_DANGEROUS;
extern const int LINE_SENSE;
extern const int MAX_PILLAR_DISTANCE;

extern const int16_t BOT_SPEED_IGNORE; /**< wird verwendet um einen Eintrag zu ignorieren */
extern const int16_t BOT_SPEED_STOP; /**< Motor aus */
extern const int16_t BOT_SPEED_MIN; /**< langsamste Fahrt in mm/s */
extern const int16_t BOT_SPEED_SLOW; /**< langsame Fahrt in mm/s */
extern const int16_t BOT_SPEED_FOLLOW; /**< vorsichtige Fahrt, fuer Folgeverhalten in mm/s */
extern const int16_t BOT_SPEED_MEDIUM; /**< mittlere Fahrt in mm/s */
extern const int16_t BOT_SPEED_NORMAL; /**< normale Fahrt in mm/s  */
extern const int16_t BOT_SPEED_FAST; /**< schnelle Fahrt in mm/s */
extern const int16_t BOT_SPEED_MAX; /**< maximale Fahrt in mm/s */

extern const int LED_RECHTS;
extern const int LED_LINKS;
extern const int LED_ROT;
extern const int LED_ORANGE;
extern const int LED_GELB;
extern const int LED_GRUEN;
extern const int LED_TUERKIS;
extern const int LED_WEISS;
extern const int LED_ALL;

extern const int SERVO_1;
extern const int SERVO_2;
extern const int DOOR_CLOSE; /**< Rechter Anschlag des Servos */
extern const int DOOR_OPEN; /**< Linker Anschlag des Servos */
extern const int SERVO_OFF;

#ifdef __cplusplus
} // extern C
} // namespace legacy
} // namespace ctbot
#endif // __cplusplus
