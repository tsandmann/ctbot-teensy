/*
 * This file is part of the c't-Bot teensy framework.
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
 * @file    behavior_legacy.h
 * @brief   Support layer for ct-Bot legacy behaviors
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#pragma once

#include "../behavior.h"

#include "pprintpp.hpp"

#include <thread>
#include <memory>
#include <vector>
#include <map>
#include <tuple>
#include <cstdarg>


namespace ctbot {

class Sensors;
class Pose;
class Speed;
class Leds;
class Lcd;

namespace legacy {

#include "bot-logic_legacy.h"

} /* namespace legacy */

class BehaviorLegacy : public Behavior {
    static constexpr bool DEBUG_ { true };
    static constexpr uint32_t STACK_SIZE { 2048 };

    static Sensors* p_sensors_;
    static const Pose* p_pose_;
    static const Speed* p_speed_;

protected:
    std::map<legacy::Behaviour_t* /*caller*/, std::unique_ptr<Behavior> /*behavior*/> called_behaviors_;
    bool running_;
    uint8_t max_active_priority_;
    legacy::Behaviour_t* behavior_; /**< Liste mit allen Verhalten */
    std::vector<void (*)()> emerg_functions_; /**< hier liegen die Zeiger auf die auszufuehrenden Notfall-Funktionen */

    FLASHMEM BehaviorLegacy(const std::string& name);

    virtual void run() override;

    /**
     * Zentrale Verhaltens-Routine, wird regelmaessig aufgerufen.
     */
    uint8_t bot_behave() const noexcept;

    /**
     * Initialisiert alle Verhalten
     */
    FLASHMEM void bot_behave_init() noexcept;

    /**
     * Fuegt ein Verhalten der Verhaltenliste anhand der Prioritaet ein.
     * @param *behave   Zeiger auf einzufuegendes Verhalten
     */
    FLASHMEM void insert_behavior_to_list(legacy::Behaviour_t* behave) noexcept;

    /**
     * Erzeugt ein neues Verhalten
     * @param priority  Die Prioritaet
     * @param *work     Die Funktion, die sich drum kuemmert
     * @param active    Boolean, ob das Verhalten aktiv oder inaktiv erstellt wird
     * @return          Zeiger auf erzeugten Verhaltensdatensatz, oder NULL im Fehlerfall
     */
    FLASHMEM legacy::Behaviour_t* new_behavior(uint8_t priority, void (*work)(legacy::Behaviour_t*), uint8_t active) const noexcept;

    /**
     * Gibt das naechste Verhalten der Liste zurueck
     * @param *beh	Zeiger auf Verhalten, dessen Nachfolger gewuenscht ist, NULL fuer Listenanfang
     * @return		Zeiger auf Nachfolger von beh
     */
    FLASHMEM legacy::Behaviour_t* get_next_behavior(legacy::Behaviour_t* beh) const noexcept;

    /**
     * liefert !=0 zurueck, wenn function ueber eine beliebige Kette (job->caller->caller ....) von anderen Verhalten job aufgerufen hat
     * @param *job          Zeiger auf den Datensatz des aufgerufenen Verhaltens
     * @param *caller_beh   Das Verhalten, das urspruenglich aufgerufen hat
     * @return              0 wenn keine Call-Abhaengigkeit besteht, ansonsten die Anzahl der Stufen
     */
    FLASHMEM uint8_t is_in_call_hierarchy(legacy::Behaviour_t* job, legacy::Behaviour_t* caller_beh) const noexcept;

    /**
     * @brief Direkter Zugriff auf die Motoren
     * @param left  Geschwindigkeit fuer den linken Motor
     * @param right Geschwindigkeit fuer den rechten Motor
     *
     * Geschwindigkeit liegt zwischen -450 und +450. 0 bedeutet Stillstand, 450 volle Kraft voraus, -450 volle Kraft zurueck.
     * Sinnvoll ist die Verwendung der Konstanten: BOT_SPEED_XXX, also z.B. motor_set(BOT_SPEED_SLOW, -BOT_SPEED_SLOW) fuer eine langsame Drehung
     */
    void motor_set(int16_t left, int16_t right, uint8_t prio) const noexcept;

    /**
     * Display-Screen Registrierung
     * @param function Zeiger auf eine Funktion, die den Display-Screen anzeigt
     *
     * Legt einen neuen Display-Screen an und haengt eine Anzeigefunktion ein.
     * Diese Funktion kann auch RC5-Kommandos behandeln. Wurde eine Taste ausgewertet, setzt man RC5_Code auf 0.
     */
    FLASHMEM int8_t register_screen(void (*function)(), void (*keyhandler)(int16_t* const)) const noexcept;

public:
    static constexpr auto BEHAVIOUR_INACTIVE { 0 }; /**< Verhalten ist aus */
    static constexpr auto BEHAVIOUR_ACTIVE { 1 }; /**< Verhalten ist an */

    static constexpr auto BEHAVIOUR_NOOVERRIDE { 0 }; /**< Konstanten, wenn Verhalten beim Aufruf alte Wuensche nicht ueberschreiben sollen */
    static constexpr auto BEHAVIOUR_OVERRIDE { 1 }; /**< Konstante, wenn Verhalten beim Aufruf alte Wuensche ueberschreiben sollen */
    static constexpr auto BEHAVIOUR_FOREGROUND { 0 }; /**< Konstante, wenn Verhalten im Vordergrund laufen sollen, also z.B. die Motoren beeinflussen */
    static constexpr auto BEHAVIOUR_BACKGROUND { 2 }; /**< Konstante, wenn Verhalten im Hintergrund laufen sollen */

    static constexpr auto BEHAVIOUR_SUBFAIL { 0 }; /**< Konstante fuer Behaviour_t->subResult: Aufgabe nicht abgeschlossen */
    static constexpr auto BEHAVIOUR_SUBSUCCESS { 1 }; /**< Konstante fuer Behaviour_t->subResult: Aufgabe erfolgreich abgeschlossen */
    static constexpr auto BEHAVIOUR_SUBRUNNING { 2 }; /**< Konstante fuer Behaviour_t->subResult: Aufgabe wird noch bearbeitet */
    static constexpr auto BEHAVIOUR_SUBCANCEL { 3 }; /**< Konstante fuer Behaviour_t->subResult: Aufgabe wurde unterbrochen */
    static constexpr auto BEHAVIOUR_SUBBACKGR { 4 }; /**< Konstange fuer Behaviour_t->subResult: Aufgabe wird im Hintergrund bearbeitet */

    static constexpr bool GET_DEBUG() {
        return DEBUG_;
    }


    BehaviorLegacy(const BehaviorLegacy&) = delete;
    BehaviorLegacy& operator=(const BehaviorLegacy&) = delete;
    FLASHMEM virtual ~BehaviorLegacy();

    static auto get_instance() {
        static BehaviorLegacy* p_instance { new BehaviorLegacy("legacy") };
        return p_instance;
    }

    struct name_view {
        char const* data;
        size_t size;
    };

    template <class T>
    static constexpr name_view get_name_helper() {
        auto p { __PRETTY_FUNCTION__ };
        while (*p++ != '=') {
        }
        for (; *p == ' '; ++p) {
        }
        auto p2 { p };
        size_t count { 1 };
        for (;; ++p2) {
            switch (*p2) {
                case '[': {
                    ++count;
                    break;
                }
                case ']': {
                    --count;
                    if (!count) {
                        return { p, static_cast<size_t>(p2 - p) };
                    }
                }
            }
        }
        return {};
    }

    template <size_t N>
    static constexpr size_t length_helper(char const (&)[N]) {
        return N - 1;
    }

    FLASHMEM static void init() noexcept;

    static void update_global_data() noexcept;

    FLASHMEM static size_t print_log(const char* type, const size_t type_len, const char* format, va_list vlist);

    void abort_behavior(legacy::Behaviour_t* caller);

    /**
     * Ruft ein anderes Verhalten auf und merkt sich den Ruecksprung
     * return_from_behaviour() kehrt dann spaeter wieder zum aufrufenden Verhalten zurueck
     * @param *from aufrufendes Verhalten
     * @param *to   aufgerufenes Verhalten
     * @param mode  Hier sind vier Werte moeglich:
     *      1. BEHAVIOUR_OVERRIDE: Das Zielverhalten to wird aktiviert, auch wenn es noch aktiv ist.
     *          Das Verhalten, das es zuletzt aufgerufen hat wird dadurch automatisch
     *          wieder aktiv und muss selbst sein eigenes Feld subResult auswerten, um zu pruefen, ob das
     *          gewuenschte Ziel erreicht wurde, oder vorher ein Abbruch stattgefunden hat.
     *      2. BEHAVIOUR_NOOVERRIDE: Das Zielverhalten wird nur aktiviert, wenn es gerade nichts zu tun hat.
     *          In diesem Fall kann der Aufrufer aus seinem eigenen subResult auslesen, ob seinem Wunsch Folge geleistet wurde.
     *      3. BEHAVIOUR_FOREGROUND	Das Verhalten laeuft im Fordergrund (Aufrufer wird solange deaktiviert)
     *      4. BEHAVIOUR_BACKGROUND	Das Verhalten laeuft im Hintergrund (Aufrufer bleibt aktiv)
     * @return      Zeiger auf Verhaltensdatensatz des aufgerufenen Verhaltens, oder NULL im Fehlerfall
     */
    legacy::Behaviour_t* switch_to_behavior(legacy::Behaviour_t* from, void (*to)(legacy::Behaviour_t*), uint8_t mode) noexcept;

    /**
     * Kehrt zum aufrufenden Verhalten zurueck und setzt den Status auf Erfolg oder Misserfolg
     * @param *data laufendes Verhalten
     * @param state Abschlussstatus des Verhaltens (BEHAVIOUR_SUBSUCCESS oder BEHAVIOUR_SUBFAIL)
     */
    void exit_behavior(legacy::Behaviour_t* data, uint8_t state) const noexcept;

    /**
     * Kehrt zum aufrufenden Verhalten zurueck
     * @param *data Derzeit laufendes Verhalten
     */
    void return_from_behavior(legacy::Behaviour_t* data) const noexcept;

    /**
     * Rueckgabe von True, wenn das Verhalten gerade laeuft (aktiv ist), sonst False
     * @param function  Die Funktion, die das Verhalten realisiert.
     * @return          true wenn Verhalten aktiv, sonst false
     */
    bool behavior_is_activated(legacy::BehaviourFunc_t function) const noexcept;

    /**
     * Aktiviert eine Regel mit gegebener Funktion, impliziert BEHAVIOUR_NOOVERRIDE.
     * Im Gegensatz zu switch_to_behaviour() wird der Aufrufer jedoch nicht deaktiviert (Hintergrundausfuehrung).
     * @param *from aufrufendes Verhalten
     * @param *to   aufgerufendes Verhalten
     * @return      Zeiger auf Verhaltensdatensatz des aufgerufenen Verhaltens, oder NULL im Fehlerfall
     */
    legacy::Behaviour_t* activateBehavior(legacy::Behaviour_t* from, void (*to)(legacy::Behaviour_t*)) noexcept;

    /**
     * Deaktiviert ein Verhalten
     * @param beh Das zu deaktivierende Verhalten
     */
    void deactivate_behavior(legacy::Behaviour_t* beh) const noexcept;

    /**
     * Deaktiviert alle von diesem Verhalten aufgerufenen Verhalten.
     * Das Verhalten selbst bleibt aktiv und bekommt ein BEHAVIOUR_SUBCANCEL in seine Datanestruktur eingetragen.
     * @param *caller Zeiger auf den Aufrufer
     */
    void deactivate_called_behaviors(legacy::Behaviour_t* caller) const noexcept;

    /**
     * Deaktiviert alle Verhalten
     */
    void deactivate_all_behaviors() const noexcept;

    /**
     * Liefert das Verhalten zurueck, welches durch function implementiert ist
     * @param function  Die Funktion, die das Verhalten realisiert
     * @return          Zeiger auf Verhaltensdatensatz oder NULL
     */
    legacy::Behaviour_t* get_behavior(legacy::BehaviourFunc_t function) const noexcept;

    /**
     * Routine zum Registrieren einer Notfallfunktion, die beim Ausloesen eines Abgrundsensors
     * aufgerufen wird; hierdurch kann ein Verhalten vom Abgrund benachrichtigt werden und
     * entsprechend dem Verhalten reagieren
     * @param *func  Die zu registrierende Routine, welche aufzurufen ist
     * @return       Index, den die Routine im Array einnimmt, bei -1 ist alles voll
     */
    FLASHMEM int8_t register_emergency_proc(void (*function)()) noexcept;

    /**
     * Beim Ausloesen eines Notfalls wird diese Routine angesprungen
     * und ruft alle registrierten Prozeduren der Reihe nach auf
     */
    void start_registered_emergency_procs() const noexcept;

    /**
     * @brief Wrapper to start a C++-behavior from a (legacy) C-behavior
     *
     * @tparam Beh Behavior to start
     * @tparam Args Types of parameters for the behavior to start
     * @param[in] caller Behavior dataset of the caller
     * @param args Parameter for the behavior to start
     * @return Behavior dataset of callee
     */
    template <class Beh, typename... Args>
    legacy::Behaviour_t* call(legacy::Behaviour_t* caller, Args&&... args) {
        if (running_ || !caller) {
            caller->subResult = BehaviorLegacy::BEHAVIOUR_SUBFAIL;
            return nullptr; // behavior is already running, abort
        }

        running_ = true;
        caller->active = BehaviorLegacy::BEHAVIOUR_INACTIVE;
        caller->subResult = BehaviorLegacy::BEHAVIOUR_SUBRUNNING;

        constexpr auto callee { get_name_helper<Beh>() };
        const std::string callee_name { callee.data, callee.size };

        called_behaviors_[caller] = behavior_factory<Beh>(std::forward<Args>(args)...);
        debug_printf<DEBUG_>(PP_ARGS("BehaviorLegacy::call(): created behavior \"{s}\"\r\n", callee_name.c_str()));

        auto callee_data { switch_to_behavior(caller, legacy::legacy_caller_behaviour, BEHAVIOUR_OVERRIDE) };

        if (!callee_data) {
            debug_print<DEBUG_>(PSTR("BehaviorLegacy::call(): \"legacy::legacy_caller_behaviour\" could not be activated.\r\n"));
            caller->subResult = BehaviorLegacy::BEHAVIOUR_SUBFAIL;
            caller->active = BehaviorLegacy::BEHAVIOUR_ACTIVE;
            return nullptr;
        }

        debug_print<DEBUG_>(PSTR("BehaviorLegacy::call(): \"legacy::legacy_caller_behaviour\" activated.\r\n"));

        return callee_data;
    }

    /**
     * @brief Wrapper-behavior to support starting C++-behaviors from (legacy) C-behaviors
     *
     * @param[in] data Behavior dataset of the caller
     */
    void legacy_caller_behaviour(legacy::Behaviour_t* data);

    /**
     * liefert Ticks in 32 Bit seit Systemstart [176 us]
     */
    uint32_t timer_get_tickCount32() const noexcept;

    /**
     * Zeigt eine 8-Bit Variable mit den LEDs an
     * @param LED Wert der gezeigt werden soll
     */
    void led_set(uint8_t led) const;

    /**
     * @brief Stellt die Servos
     * @param servo Nummer des Servos
     * @param pos   Zielwert
     *
     * Sinnvolle Werte liegen zwischen DOOR_CLOSE / CAM_LEFT und DOOR_OPEN / CAM_RIGHT oder SERVO_OFF fuer Servo aus
     */
    void servo_set(uint8_t servo, uint8_t pos) noexcept;

    /**
     * @brief Gibt die Sollposition der Servos zurueck
     * @param servo Servo ID
     * @return Sollposition
     */
    uint8_t servo_get_pos(uint8_t servo) const noexcept;

    /**
     * @brief Gibt die Aktivitaet der Servos zurueck
     * @param servo Servo ID
     * @return 1, falls Servo aktiv, 0 sonst
     */
    uint8_t servo_get_active(uint8_t servo) const noexcept;

    /**
     * Loescht das ganze Display
     */
    FLASHMEM void display_clear() const noexcept;

    /**
     * Positioniert den Cursor
     * @param row    Zeile
     * @param column Spalte
     */
    FLASHMEM void display_cursor(int16_t row, int16_t column) const noexcept;

    /**
     * Schreibt einen String auf das Display.
     * @param *format    Format, wie beim printf
     * @param args       Variable Argumentenliste, wie beim printf
     * @return           Anzahl der geschriebenen Zeichen
     */
    FLASHMEM uint8_t display_printf(const char* format, va_list args) const noexcept;

    /**
     * Gibt einen String auf dem Display aus
     * @param *text  Zeiger auf den auszugebenden String
     * @return       Anzahl der geschriebenen Zeichen
     */
    FLASHMEM uint8_t display_puts(const char* text) const noexcept;
};

} /* namespace ctbot */
