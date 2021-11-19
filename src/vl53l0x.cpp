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
 * @file    vl53l0x.cpp
 * @brief   VL53L0X sensor driver
 * @author  Timo Sandmann
 * @date    07.12.2019
 */

#include "vl53l0x.h"
#include "ctbot.h"
#include "ctbot_config.h"

#include "pprintpp.hpp"

#include <array>
#include <chrono>
#include <thread>


namespace ctbot {
VL53L0X::VL53L0X(const uint8_t i2c_bus, const uint32_t i2c_freq, const uint8_t i2c_addr)
    : i2c_ { i2c_bus, i2c_freq, CtBotConfig::I2C0_PIN_SDA, CtBotConfig::I2C0_PIN_SCL }, i2c_addr_ { i2c_addr }, timing_budget_us_ {}, stop_variable_ {} {}

FLASHMEM bool VL53L0X::init() {
    /* check model ID */
    uint8_t id { 0xcc };
    if (read_reg8(MODEL_ID_REG, id) || id != 0xee) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_printf<true>(PP_ARGS("VL53L0X::init(): read(MODEL_ID_REG) failed, id={}.\r\n", id));
        }
        return false;
    }

    /* switch to 2.8V mode */
    if (set_bit(VOLTAGE_MODE_REG, 0, 1)) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::init(): set_bit(VOLTAGE_MODE_REG) failed.\r\n"), true);
        }
        return false;
    }

    uint8_t ret {};
    /* set i2c standard mode */
    ret |= write_reg8(static_cast<uint8_t>(0x88), 0);

    ret |= write_reg8(static_cast<uint8_t>(0x80), 1);
    ret |= write_reg8(static_cast<uint8_t>(0xff), 1);
    ret |= write_reg8(static_cast<uint8_t>(0), 0);
    ret |= read_reg8(static_cast<uint8_t>(0x91), stop_variable_);
    ret |= write_reg8(static_cast<uint8_t>(0), 1);
    ret |= write_reg8(static_cast<uint8_t>(0xff), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x80), 0);

    if (ret) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::init(): set i2c standard mode failed.\r\n"), true);
        }
        return false;
    }

    /* disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks */
    ret |= set_bit(MSRC_CONFIG_REG, 1, 1);
    ret |= set_bit(MSRC_CONFIG_REG, 4, 1);

    if (ret) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(
                PSTR("VL53L0X::init(): disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks failed.\r\n"), true);
        }
        return false;
    }

    /* set final range signal rate limit to 0.25 MCPS (million counts per second) */
    if (!set_signal_rate_limit(0.25f)) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::init(): set_signal_rate_limit() failed.\r\n"), true);
        }
        return false;
    }

    if (write_reg8(static_cast<uint8_t>(SEQUENCE_CONFIG_REG), 0xff)) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::init(): write(SYSTEM_SEQUENCE_REG) failed.\r\n"), true);
        }
        return false;
    }

    uint8_t spad_count {};
    bool spad_type_is_aperture {};
    if (!get_spad_info(spad_count, spad_type_is_aperture)) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::init(): get_spad_info() failed.\r\n"), true);
        }
        return false;
    }

    std::array<uint8_t, 6> ref_spad_map;
    if (read_bytes(SPAD_ENABLES_REF_0_REG, ref_spad_map.data(), ref_spad_map.size())) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::init(): read_bytes(SPAD_ENABLES_REF_0_REG) failed.\r\n"), true);
        }
        return false;
    }

    ret |= write_reg8(static_cast<uint8_t>(0xff), 1);
    ret |= write_reg8(SPAD_REF_EN_START_OFFSET_REG, 0);
    ret |= write_reg8(SPAD_NUM_REQUESTED_REF_SPAD_REG, 0x2c);
    ret |= write_reg8(static_cast<uint8_t>(0xff), 0);
    ret |= write_reg8(REF_EN_START_SELECT_REG, 0xb4);

    uint8_t first_spad_to_enable { static_cast<uint8_t>(spad_type_is_aperture ? 12U : 0U) }; // 12 is the first aperture spad
    uint8_t spads_enabled {};

    for (uint8_t i {}; i < 48; ++i) {
        if (i < first_spad_to_enable || spads_enabled == spad_count) {
            // This bit is lower than the first one that should be enabled, or (reference_spad_count) bits have already been enabled, so zero this bit
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 1) {
            ++spads_enabled;
        }
    }

    if (write_bytes(SPAD_ENABLES_REF_0_REG, ref_spad_map.data(), ref_spad_map.size())) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::init(): write_bytes(SPAD_ENABLES_REF_0_REG) failed.\r\n"), true);
        }
        return false;
    }

    /* DefaultTuningSettings from vl53l0x_tuning.h */
    ret |= write_reg8(static_cast<uint8_t>(0xff), 1);
    ret |= write_reg8(static_cast<uint8_t>(0), 0);

    ret |= write_reg8(static_cast<uint8_t>(0xff), 0);
    ret |= write_reg8(static_cast<uint8_t>(9), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x10), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x11), 0);

    ret |= write_reg8(static_cast<uint8_t>(0x24), 1);
    ret |= write_reg8(static_cast<uint8_t>(0x25), 0xff);
    ret |= write_reg8(static_cast<uint8_t>(0x75), 0);

    ret |= write_reg8(static_cast<uint8_t>(0xff), 1);
    ret |= write_reg8(static_cast<uint8_t>(0x4e), 0x2c);
    ret |= write_reg8(static_cast<uint8_t>(0x48), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x30), 0x20);

    ret |= write_reg8(static_cast<uint8_t>(0xff), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x30), 9);
    ret |= write_reg8(static_cast<uint8_t>(0x54), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x31), 4);
    ret |= write_reg8(static_cast<uint8_t>(0x32), 3);
    ret |= write_reg8(static_cast<uint8_t>(0x40), 0x83);
    ret |= write_reg8(static_cast<uint8_t>(0x46), 0x25);
    ret |= write_reg8(static_cast<uint8_t>(0x60), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x27), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x50), 6);
    ret |= write_reg8(static_cast<uint8_t>(0x51), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x52), 0x96);
    ret |= write_reg8(static_cast<uint8_t>(0x56), 8);
    ret |= write_reg8(static_cast<uint8_t>(0x57), 0x30);
    ret |= write_reg8(static_cast<uint8_t>(0x61), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x62), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x64), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x65), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x66), 0xa0);

    ret |= write_reg8(static_cast<uint8_t>(0xff), 1);
    ret |= write_reg8(static_cast<uint8_t>(0x22), 0x32);
    ret |= write_reg8(static_cast<uint8_t>(0x47), 0x14);
    ret |= write_reg8(static_cast<uint8_t>(0x49), 0xff);
    ret |= write_reg8(static_cast<uint8_t>(0x4a), 0);

    ret |= write_reg8(static_cast<uint8_t>(0xff), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x7a), 0xa);
    ret |= write_reg8(static_cast<uint8_t>(0x7b), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x78), 0x21);

    ret |= write_reg8(static_cast<uint8_t>(0xff), 1);
    ret |= write_reg8(static_cast<uint8_t>(0x23), 0x34);
    ret |= write_reg8(static_cast<uint8_t>(0x42), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x44), 0xff);
    ret |= write_reg8(static_cast<uint8_t>(0x45), 0x26);
    ret |= write_reg8(static_cast<uint8_t>(0x46), 5);
    ret |= write_reg8(static_cast<uint8_t>(0x40), 0x40);
    ret |= write_reg8(static_cast<uint8_t>(0xe), 6);
    ret |= write_reg8(static_cast<uint8_t>(0x20), 0x1a);
    ret |= write_reg8(static_cast<uint8_t>(0x43), 0x40);

    ret |= write_reg8(static_cast<uint8_t>(0xff), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x34), 3);
    ret |= write_reg8(static_cast<uint8_t>(0x35), 0x44);

    ret |= write_reg8(static_cast<uint8_t>(0xff), 1);
    ret |= write_reg8(static_cast<uint8_t>(0x31), 4);
    ret |= write_reg8(static_cast<uint8_t>(0x4b), 9);
    ret |= write_reg8(static_cast<uint8_t>(0x4c), 5);
    ret |= write_reg8(static_cast<uint8_t>(0x4d), 4);

    ret |= write_reg8(static_cast<uint8_t>(0xff), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x44), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x45), 0x20);
    ret |= write_reg8(static_cast<uint8_t>(0x47), 8);
    ret |= write_reg8(static_cast<uint8_t>(0x48), 0x28);
    ret |= write_reg8(static_cast<uint8_t>(0x67), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x70), 4);
    ret |= write_reg8(static_cast<uint8_t>(0x71), 1);
    ret |= write_reg8(static_cast<uint8_t>(0x72), 0xfe);
    ret |= write_reg8(static_cast<uint8_t>(0x76), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x77), 0);

    ret |= write_reg8(static_cast<uint8_t>(0xff), 1);
    ret |= write_reg8(static_cast<uint8_t>(0xd), 1);

    ret |= write_reg8(static_cast<uint8_t>(0xff), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x80), 1);
    ret |= write_reg8(static_cast<uint8_t>(1), 0xf8);

    ret |= write_reg8(static_cast<uint8_t>(0xff), 1);
    ret |= write_reg8(static_cast<uint8_t>(0x8e), 1);
    ret |= write_reg8(static_cast<uint8_t>(0), 1);
    ret |= write_reg8(static_cast<uint8_t>(0xff), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x80), 0);

    /* set interrupt config to new sample ready */
    ret |= write_reg8(INTERRUPT_CONFIG_GPIO_REG, 4);
    ret |= set_bit(MUX_ACTIVE_HIGH_REG, 4, 0); // active low
    ret |= write_reg8(INTERRUPT_CLEAR_REG, 1);


    timing_budget_us_ = get_measurement_timing_budget();

    /* Disable MSRC and TCC by default
     * MSRC = Minimum Signal Rate Check
     * TCC = Target CentreCheck */
    ret |= write_reg8(SEQUENCE_CONFIG_REG, 0xe8);

    /* Recalculate timing budget */
    if (!set_measurement_timing_budget(timing_budget_us_)) {
        return false;
    }


    ret |= write_reg8(SEQUENCE_CONFIG_REG, 1);
    if (!perform_single_ref_calibration(0x40)) {
        return false;
    }


    ret |= write_reg8(SEQUENCE_CONFIG_REG, 2);
    if (!perform_single_ref_calibration(0)) {
        return false;
    }

    /* restore the previous Sequence Config */
    ret |= write_reg8(SEQUENCE_CONFIG_REG, 0xe8);

    if (DEBUG_) {
        if (ret) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::init(): failed.\r\n"), true);
        } else {
            CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::init(): done.\r\n"), true);
        }
    }

    return ret == 0;
}

FLASHMEM bool VL53L0X::get_spad_info(uint8_t& count, bool& type_is_aperture) const {
#if defined __x86_64__ || defined __i386__ || defined __APPLE__ || defined __linux__
    return true; // FIXME: workaround for simulator
#endif

    uint8_t ret {};
    ret |= write_reg8(static_cast<uint8_t>(0x80), 1);
    ret |= write_reg8(static_cast<uint8_t>(0xff), 1);
    ret |= write_reg8(static_cast<uint8_t>(0), 0);
    ret |= write_reg8(static_cast<uint8_t>(0xff), 6);
    ret |= set_bit(static_cast<uint8_t>(0x83), 2, 1);
    ret |= write_reg8(static_cast<uint8_t>(0xff), 7);
    ret |= write_reg8(static_cast<uint8_t>(0x81), 1);
    ret |= write_reg8(static_cast<uint8_t>(0x80), 1);
    ret |= write_reg8(static_cast<uint8_t>(0x94), 0x6b);
    ret |= write_reg8(static_cast<uint8_t>(0x83), 0);
    if (ret) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::get_spad_info(): write() 1 failed.\r\n"), true);
        }
        return false;
    }

    elapsedMillis ms;
    uint8_t tmp1 {};
    while (!read_reg8(static_cast<uint8_t>(0x83), tmp1) && tmp1 == 0) {
        if (ms > 2'000) { // FIXME: timeout value?
            break;
        }
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(100ms);
    }
    if (!tmp1) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::get_spad_info(): read(0x83) failed.\r\n"), true);
        }
        return false;
    }
    ret |= write_reg8(static_cast<uint8_t>(0x83), 1);

    uint8_t tmp2;
    ret |= read_reg8(static_cast<uint8_t>(0x92), tmp2);
    if (ret) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::get_spad_info(): read(0x92) failed.\r\n"), true);
        }
        return false;
    }
    count = tmp2 & 0x7f;
    type_is_aperture = (tmp2 >> 7) & 1;

    ret |= write_reg8(static_cast<uint8_t>(0x81), 0);
    ret |= write_reg8(static_cast<uint8_t>(0xff), 6);
    ret |= set_bit(static_cast<uint8_t>(0x83), 2, 0);
    ret |= write_reg8(static_cast<uint8_t>(0xff), 1);
    ret |= write_reg8(static_cast<uint8_t>(0), 1);
    ret |= write_reg8(static_cast<uint8_t>(0xff), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x80), 0);
    if (ret) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::get_spad_info(): write() 2 failed.\r\n"), true);
        }
        return false;
    }

    return true;
}

FLASHMEM uint32_t VL53L0X::get_measurement_timing_budget() const {
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    const uint16_t StartOverhead { 1910 };
    const uint16_t EndOverhead { 960 };
    const uint16_t MsrcOverhead { 660 };
    const uint16_t TccOverhead { 590 };
    const uint16_t DssOverhead { 690 };
    const uint16_t PreRangeOverhead { 660 };
    const uint16_t FinalRangeOverhead { 550 };

    // "Start and end overhead times always present"
    uint32_t budget_us { StartOverhead + EndOverhead };

    if (!get_sequence_step_enables(enables)) {
        return 0;
    }
    if (!get_sequence_step_timeouts(enables, timeouts)) {
        return 0;
    }

    if (enables.tcc) {
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss) {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    } else if (enables.msrc) {
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range) {
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range) {
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    return budget_us;
}

FLASHMEM bool VL53L0X::set_measurement_timing_budget(const uint32_t budget_us) const {
    const uint16_t StartOverhead { 1910 };
    const uint16_t EndOverhead { 960 };
    const uint16_t MsrcOverhead { 660 };
    const uint16_t TccOverhead { 590 };
    const uint16_t DssOverhead { 690 };
    const uint16_t PreRangeOverhead { 660 };
    const uint16_t FinalRangeOverhead { 550 };
    const uint32_t MinTimingBudget { 20000 };

    if (budget_us < MinTimingBudget) {
        return false;
    }

    uint32_t used_budget_us { StartOverhead + EndOverhead };

    SequenceStepEnables enables;
    if (!get_sequence_step_enables(enables)) {
        return false;
    }
    SequenceStepTimeouts timeouts;
    if (!get_sequence_step_timeouts(enables, timeouts)) {
        return false;
    }

    if (enables.tcc) {
        used_budget_us += timeouts.msrc_dss_tcc_us + TccOverhead;
    }

    if (enables.dss) {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    } else if (enables.msrc) {
        used_budget_us += timeouts.msrc_dss_tcc_us + MsrcOverhead;
    }

    if (enables.pre_range) {
        used_budget_us += timeouts.pre_range_us + PreRangeOverhead;
    }

    if (enables.final_range) {
        used_budget_us += FinalRangeOverhead;

        /* Note that the final range timeout is determined by the timing budget and the sum of all other timeouts within the sequence.
         * If there is no room for the final range timeout, then an error will be set. Otherwise the remaining time will be applied to
         * the final range. */
        if (used_budget_us > budget_us) {
            /* requested timeout is too big */
            return false;
        }

        const uint32_t final_range_timeout_us { budget_us - used_budget_us };

        /* For the final range timeout, the pre-range timeout must be added. To do this both final and pre-range
         * timeouts must be expressed in macro periods MClks because they have different vcsel periods. */
        uint32_t final_range_timeout_mclks { timeout_us_to_mclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks) };

        if (enables.pre_range) {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        if (write_reg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_REG_HI, timeout_encode(final_range_timeout_mclks))) {
            if (DEBUG_) {
                CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::set_measurement_timing_budget(): write16() failed.\r\n"), true);
            }

            return false;
        }
    }
    return true;
}

FLASHMEM bool VL53L0X::get_sequence_step_enables(SequenceStepEnables& enables) const {
    uint8_t sequence_config;
    if (read_reg8(SEQUENCE_CONFIG_REG, sequence_config)) {
        return false;
    }

    enables.tcc = (sequence_config >> 4) & 1;
    enables.dss = (sequence_config >> 3) & 1;
    enables.msrc = (sequence_config >> 2) & 1;
    enables.pre_range = (sequence_config >> 6) & 1;
    enables.final_range = (sequence_config >> 7) & 1;

    return true;
}

FLASHMEM bool VL53L0X::get_sequence_step_timeouts(const SequenceStepEnables& enables, SequenceStepTimeouts& timeouts) const {
    timeouts.pre_range_vcsel_period_pclks = get_vcsel_pulse_period(false);

    uint8_t tmp8;
    if (read_reg8(TIMEOUT_MACROP_REG, tmp8)) {
        return false;
    }
    timeouts.msrc_dss_tcc_mclks = tmp8 + 1U;

    timeouts.msrc_dss_tcc_us = timeout_mclks_to_us(timeouts.msrc_dss_tcc_mclks, timeouts.pre_range_vcsel_period_pclks);

    uint16_t tmp16;
    if (read_reg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_REG_HI, tmp16)) {
        return false;
    }
    timeouts.pre_range_mclks = timeout_decode(tmp16);
    timeouts.pre_range_us = timeout_mclks_to_us(timeouts.pre_range_mclks, timeouts.pre_range_vcsel_period_pclks);

    timeouts.final_range_vcsel_period_pclks = get_vcsel_pulse_period(true);

    if (read_reg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_REG_HI, tmp16)) {
        return false;
    }
    timeouts.final_range_mclks = timeout_decode(tmp16);

    if (enables.pre_range) {
        timeouts.final_range_mclks -= timeouts.pre_range_mclks;
    }

    timeouts.final_range_us = timeout_mclks_to_us(timeouts.final_range_mclks, timeouts.final_range_vcsel_period_pclks);

    return true;
}

FLASHMEM uint8_t VL53L0X::get_vcsel_pulse_period(const bool final) const {
    uint8_t tmp;
    if (read_reg8(final ? FINAL_RANGE_CONFIG_VCSEL_PERIOD_REG : PRE_RANGE_CONFIG_VCSEL_PERIOD_REG, tmp)) {
        return 255;
    }
    return vcsel_period_decode(tmp);
}

FLASHMEM bool VL53L0X::perform_single_ref_calibration(const uint8_t vhv_init_byte) const {
    if (write_reg8(SYSRANGE_START_REG, 1 | vhv_init_byte)) {
        return false;
    }

    uint8_t tmp;
    do {
        if (read_reg8(RESULT_INTERRUPT_STATUS_REG, tmp)) {
            return false;
        }
    } while ((tmp & 7) == 0); // FIXME: timeout?

    if (write_reg8(INTERRUPT_CLEAR_REG, 1)) {
        return false;
    }
    if (write_reg8(SYSRANGE_START_REG, 0)) {
        return false;
    }

    return true;
}

FLASHMEM bool VL53L0X::set_signal_rate_limit(const float limit_mcps) const {
    if (limit_mcps < 0.f || limit_mcps > 511.99f) {
        return false;
    }

    // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
    if (write_reg16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REG, limit_mcps * (1 << 7))) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::set_signal_rate_limit(): write16() failed.\r\n"), true);
        }
        return false;
    }

    return true;
}

FLASHMEM bool VL53L0X::set_address(const uint8_t addr) {
    if (addr > 127) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::set_address(): invalid address.\r\n"), true);
        }
        return false;
    }
    if (write_reg8(I2C_ADDRESS_REG, addr)) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::set_address(): write() failed.\r\n"), true);
        }
        return false;
    }
    // i2c_.set_address(addr);
    i2c_addr_ = addr;

    return true;
}

FLASHMEM bool VL53L0X::start_continuous(const uint32_t period_ms) const {
    uint8_t ret {};
    ret |= write_reg8(static_cast<uint8_t>(0x80), 1);
    ret |= write_reg8(static_cast<uint8_t>(0xff), 1);
    ret |= write_reg8(static_cast<uint8_t>(0), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x91), stop_variable_);
    ret |= write_reg8(static_cast<uint8_t>(0), 1);
    ret |= write_reg8(static_cast<uint8_t>(0xff), 0);
    ret |= write_reg8(static_cast<uint8_t>(0x80), 0);

    if (ret) {
        return false;
    }

    if (period_ms != 0) {
        /* continuous timed mode */
        uint16_t osc_calibrate_val;
        if (read_reg16(OSC_CALIBRATE_VAL_REG, osc_calibrate_val)) {
            return false;
        }

        uint32_t period { period_ms };
        if (osc_calibrate_val != 0) {
            period *= osc_calibrate_val;
        }

        if (write_reg32(INTERMEASUREMENT_PERIOD_REG, period)) {
            return false;
        }

        if (write_reg8(SYSRANGE_START_REG, 4)) {
            return false;
        }
    } else {
        /* continuous back-to-back mode */
        if (write_reg8(SYSRANGE_START_REG, 2)) {
            return false;
        }
    }

    return true;
}

bool VL53L0X::get_dist_range(uint16_t& range_mm) const {
    uint8_t data;
    if (!read_reg8(RESULT_INTERRUPT_STATUS_REG, data)) {
        if (data & 7) {
            uint16_t mm;
            if (!read_reg16(static_cast<uint8_t>(RESULT_RANGE_STATUS_REG + 10), mm)) {
                range_mm = mm >= MIN_DISTANCE_ && mm <= MAX_DISTANCE_ ? mm : 9999;
            } else if (DEBUG_) {
                CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::get_dist_range(): i2c error 2\r\n"), true);
            }
            if (write_reg8(INTERRUPT_CLEAR_REG, 1) && DEBUG_) {
                CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::get_dist_range(): i2c error 3\r\n"), true);
            }
            return true;
        }
    } else if (DEBUG_) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("VL53L0X::get_dist_range(): i2c error 1\r\n"), true);
    }

    return false;
}

} // namespace ctbot
