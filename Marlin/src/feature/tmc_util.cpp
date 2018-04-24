/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../inc/MarlinConfig.h"

#if HAS_TRINAMIC

#include "tmc_util.h"
#include "../Marlin.h"

#include "../module/stepper_indirection.h"
#include "../module/printcounter.h"
#include "../libs/duration_t.h"
#include "../gcode/gcode.h"

#if ENABLED(TMC_DEBUG)
  #include "../module/planner.h"
#endif

#if ENABLED(ULTIPANEL)
  #include "../module/stepper.h"
#endif

bool report_tmc_status = false;

/**
 * Check for over temperature or short to ground error flags.
 * Report and log warning of overtemperature condition.
 * Reduce driver current in a persistent otpw condition.
 * Keep track of otpw counter so we don't reduce current on a single instance,
 * and so we don't repeatedly report warning before the condition is cleared.
 * Update status data if user has an LCD.
 */
#if ENABLED(MONITOR_DRIVER_STATUS)
  struct TMC_driver_data {
    uint32_t drv_status;
    bool is_otpw,
         is_ot,
         is_s2ga,
         is_s2gb,
         is_error;
  };
  #if ENABLED(HAVE_TMC2130)
    static uint32_t get_pwm_scale(TMC2130Stepper &st) { return st.PWM_SCALE(); }
    static uint8_t get_status_response(TMC2130Stepper &st, uint32_t) { return st.status_response & 0xF; }
    static TMC_driver_data get_driver_data(TMC2130Stepper &st) {
      constexpr uint32_t OTPW_bm = 0x4000000UL;
      constexpr uint8_t OTPW_bp = 26;
      constexpr uint32_t OT_bm = 0x2000000UL;
      constexpr uint8_t OT_bp = 25;
      constexpr uint8_t S2GA_bp = 27;
      constexpr uint8_t S2GB_bp = 28;
      constexpr uint8_t DRIVER_ERROR_bm = 0x2UL;
      constexpr uint8_t DRIVER_ERROR_bp = 1;
      TMC_driver_data data;
      data.drv_status = st.DRV_STATUS();
      data.is_otpw = (data.drv_status & OTPW_bm) >> OTPW_bp;
      data.is_ot = (data.drv_status & OT_bm) >> OT_bp;
      data.is_s2ga = (data.drv_status >> S2GA_bp) & 0b1;
      data.is_s2gb = (data.drv_status >> S2GB_bp) & 0b1;
      data.is_error = data.is_otpw | data.is_ot | data.is_s2ga | data.is_s2gb;
      return data;
    }
    void update_lcd_data(TMC2130Stepper &st, uint32_t drvstatus) {
      #if ENABLED(SENSORLESS_HOMING)
        st.stored.sg_result = drvstatus & 0x3FF;
      #endif
      st.stored.cs_actual = (drvstatus>>16) & 0x1F;
    }
  #endif
  #if ENABLED(HAVE_TMC2208)
    static uint32_t get_pwm_scale(TMC2208Stepper &st) { return st.pwm_scale_sum(); }
    static uint8_t get_status_response(TMC2208Stepper &st, uint32_t drv_status) {
      uint8_t gstat = st.GSTAT();
      uint8_t response = 0;
      response |= (drv_status >> (31-3)) & 0b1000;
      response |= gstat & 0b11;
      return response;
    }
    static TMC_driver_data get_driver_data(TMC2208Stepper &st) {
      constexpr uint32_t OTPW_bm = 0b1ul;
      constexpr uint8_t OTPW_bp = 0;
      constexpr uint32_t OT_bm = 0b10ul;
      constexpr uint8_t OT_bp = 1;
      constexpr uint8_t S2GA_bp = 2;
      constexpr uint8_t S2GB_bp = 3;
      TMC_driver_data data;
      data.drv_status = st.DRV_STATUS();
      data.is_otpw = (data.drv_status & OTPW_bm) >> OTPW_bp;
      data.is_ot = (data.drv_status & OT_bm) >> OT_bp;
      data.is_s2ga = (data.drv_status >> S2GA_bp) & 0b1;
      data.is_s2gb = (data.drv_status >> S2GB_bp) & 0b1;
      data.is_error = data.is_otpw | data.is_ot | data.is_s2ga | data.is_s2gb;
      return data;
    }
    void update_lcd_data(TMC2208Stepper &st, uint32_t drvstatus) {
      st.stored.cs_actual = (drvstatus>>16) & 0x1F;
    }
  #endif
  #if ENABLED(HAVE_TMC2660)
    static uint32_t get_pwm_scale(TMC2660Stepper) { return 0; }
    static uint8_t get_status_response(TMC2660Stepper) { return 0; }
    static TMC_driver_data get_driver_data(TMC2660Stepper &st) {
      constexpr uint32_t OTPW_bm = 0x4UL;
      constexpr uint8_t OTPW_bp = 2;
      constexpr uint32_t OT_bm = 0x2UL;
      constexpr uint8_t OT_bp = 1;
      constexpr uint8_t DRIVER_ERROR_bm = 0x1EUL;
      TMC_driver_data data;
      data.drv_status = st.DRVSTATUS();
      data.is_otpw = (data.drv_status & OTPW_bm) >> OTPW_bp;
      data.is_ot = (data.drv_status & OT_bm) >> OT_bp;
      data.is_error = data.drv_status & DRIVER_ERROR_bm;
      return data;
    }
  #endif

  template<typename TMC>
  void monitor_tmc_driver(TMC &st, const TMC_AxisEnum axis, uint8_t &otpw_cnt) {
    TMC_driver_data data = get_driver_data(st);

    // Store retrieved data to be shown in menu
    #if ENABLED(ULTIPANEL)
      update_lcd_data(st, data.drv_status);
    #endif

    #if ENABLED(STOP_ON_ERROR)
      if (data.is_error) {
        SERIAL_EOL();
        _tmc_say_axis(axis);
        SERIAL_ECHOLNPGM(" driver error detected:");
        if (data.is_ot) SERIAL_ECHOLNPGM("overtemperature");
        if (data.is_s2ga) SERIAL_ECHOLNPGM("short to ground (coil A)");
        if (data.is_s2gb) SERIAL_ECHOLNPGM("short to ground (coil B)");
        #if ENABLED(TMC_DEBUG)
          tmc_report_all();
        #endif
        kill(PSTR("Driver error"));
      }
    #endif

    // Report if a warning was triggered
    if (data.is_otpw && otpw_cnt == 0) {
      char timestamp[10];
      duration_t elapsed = print_job_timer.duration();
      const bool has_days = (elapsed.value > 60*60*24L);
      (void)elapsed.toDigital(timestamp, has_days);
      SERIAL_EOL();
      SERIAL_ECHO(timestamp);
      SERIAL_ECHOPGM(": ");
      _tmc_say_axis(axis);
      SERIAL_ECHOPGM(" driver overtemperature warning! (");
      SERIAL_ECHO(st.getMilliAmps());
      SERIAL_ECHOLNPGM("mA)");
    }
    #if CURRENT_STEP_DOWN > 0
      // Decrease current if is_otpw is true and driver is enabled and there's been more than 4 warnings
      if (data.is_otpw && st.isEnabled() && otpw_cnt > 4) {
        st.rms_current(st.getMilliAmps() - CURRENT_STEP_DOWN);
        #if ENABLED(REPORT_CURRENT_CHANGE)
          _tmc_say_axis(axis);
          SERIAL_ECHOLNPAIR(" current decreased to ", st.getMilliAmps());
        #endif
      }
    #endif

    if (data.is_otpw) {
      otpw_cnt++;
      st.flag_otpw = true;
    }
    else if (otpw_cnt > 0) otpw_cnt = 0;

    if (report_tmc_status) {
      const uint32_t pwm_scale = get_pwm_scale(st);
      _tmc_say_axis(axis);
      SERIAL_ECHOPAIR(":", pwm_scale);
      SERIAL_ECHOPGM(" |0b"); SERIAL_PRINT(get_status_response(st, data.drv_status), BIN);
      SERIAL_ECHOPGM("| ");
      if (data.is_error) SERIAL_CHAR('E');
      else if (data.is_ot) SERIAL_CHAR('O');
      else if (data.is_otpw) SERIAL_CHAR('W');
      else if (otpw_cnt > 0) SERIAL_PRINT(otpw_cnt, DEC);
      else if (st.flag_otpw) SERIAL_CHAR('F');
      SERIAL_CHAR('\t');
    }
  }

  #define HAS_HW_COMMS(ST) ENABLED(ST##_IS_TMC2130)|| (ENABLED(ST##_IS_TMC2208) && defined(ST##_HARDWARE_SERIAL))

  void monitor_tmc_driver() {
    static millis_t next_cOT = 0;
    if (ELAPSED(millis(), next_cOT)) {
      next_cOT = millis() + 500;
      #if HAS_HW_COMMS(X) || ENABLED(IS_TRAMS)
        static uint8_t x_otpw_cnt = 0;
        monitor_tmc_driver(stepperX, TMC_X, x_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(Y) || ENABLED(IS_TRAMS)
        static uint8_t y_otpw_cnt = 0;
        monitor_tmc_driver(stepperY, TMC_Y, y_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(Z) || ENABLED(IS_TRAMS)
        static uint8_t z_otpw_cnt = 0;
        monitor_tmc_driver(stepperZ, TMC_Z, z_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(X2)
        static uint8_t x2_otpw_cnt = 0;
        monitor_tmc_driver(stepperX2, TMC_X, x2_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(Y2)
        static uint8_t y2_otpw_cnt = 0;
        monitor_tmc_driver(stepperY2, TMC_Y, y2_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(Z2)
        static uint8_t z2_otpw_cnt = 0;
        monitor_tmc_driver(stepperZ2, TMC_Z, z2_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(E0) || ENABLED(IS_TRAMS)
        static uint8_t e0_otpw_cnt = 0;
        monitor_tmc_driver(stepperE0, TMC_E0, e0_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(E1)
        static uint8_t e1_otpw_cnt = 0;
        monitor_tmc_driver(stepperE1, TMC_E1, e1_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(E2)
        static uint8_t e2_otpw_cnt = 0;
        monitor_tmc_driver(stepperE2, TMC_E2, e2_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(E3)
        static uint8_t e3_otpw_cnt = 0;
        monitor_tmc_driver(stepperE3, TMC_E3, e3_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(E4)
        static uint8_t e4_otpw_cnt = 0;
        monitor_tmc_driver(stepperE4, TMC_E4, e4_otpw_cnt);
      #endif

      if (report_tmc_status) SERIAL_EOL();
    }
  }

#endif // MONITOR_DRIVER_STATUS

void _tmc_say_axis(const TMC_AxisEnum axis) {
  static const char ext_X[]  PROGMEM = "X",  ext_Y[]  PROGMEM = "Y",  ext_Z[]  PROGMEM = "Z",
                    ext_X2[] PROGMEM = "X2", ext_Y2[] PROGMEM = "Y2", ext_Z2[] PROGMEM = "Z2",
                    ext_E0[] PROGMEM = "E0", ext_E1[] PROGMEM = "E1",
                    ext_E2[] PROGMEM = "E2", ext_E3[] PROGMEM = "E3",
                    ext_E4[] PROGMEM = "E4";
  static const char* const tmc_axes[] PROGMEM = { ext_X, ext_Y, ext_Z, ext_X2, ext_Y2, ext_Z2, ext_E0, ext_E1, ext_E2, ext_E3, ext_E4 };
  serialprintPGM((char*)pgm_read_ptr(&tmc_axes[axis]));
}

void _tmc_say_current(const TMC_AxisEnum axis, const uint16_t curr) {
  _tmc_say_axis(axis);
  SERIAL_ECHOLNPAIR(" driver current: ", curr);
}
void _tmc_say_otpw(const TMC_AxisEnum axis, const bool otpw) {
  _tmc_say_axis(axis);
  SERIAL_ECHOPGM(" temperature prewarn triggered: ");
  serialprintPGM(otpw ? PSTR("true") : PSTR("false"));
  SERIAL_EOL();
}
void _tmc_say_otpw_cleared(const TMC_AxisEnum axis) {
  _tmc_say_axis(axis);
  SERIAL_ECHOLNPGM(" prewarn flag cleared");
}
void _tmc_say_pwmthrs(const TMC_AxisEnum axis, const uint32_t thrs) {
  _tmc_say_axis(axis);
  SERIAL_ECHOLNPAIR(" stealthChop max speed: ", thrs);
}
void _tmc_say_sgt(const TMC_AxisEnum axis, const int8_t sgt) {
  _tmc_say_axis(axis);
  SERIAL_ECHOPGM(" homing sensitivity: ");
  SERIAL_PRINTLN(sgt, DEC);
}

#if ENABLED(TMC_DEBUG)

  enum TMC_debug_enum : char {
    TMC_CODES,
    TMC_ENABLED,
    TMC_CURRENT,
    TMC_RMS_CURRENT,
    TMC_MAX_CURRENT,
    TMC_IRUN,
    TMC_IHOLD,
    TMC_CS_ACTUAL,
    TMC_PWM_SCALE,
    TMC_VSENSE,
    TMC_STEALTHCHOP,
    TMC_MICROSTEPS,
    TMC_TSTEP,
    TMC_TPWMTHRS,
    TMC_TPWMTHRS_MMS,
    TMC_OTPW,
    TMC_OTPW_TRIGGERED,
    TMC_TOFF,
    TMC_TBL,
    TMC_HEND,
    TMC_HSTRT,
    TMC_SGT
  };
  enum TMC_drv_status_enum : char {
    TMC_DRV_CODES,
    TMC_STST,
    TMC_OLB,
    TMC_OLA,
    TMC_S2GB,
    TMC_S2GA,
    TMC_DRV_OTPW,
    TMC_OT,
    TMC_STALLGUARD,
    TMC_DRV_CS_ACTUAL,
    TMC_FSACTIVE,
    TMC_SG_RESULT,
    TMC_DRV_STATUS_HEX,
    TMC_T157,
    TMC_T150,
    TMC_T143,
    TMC_T120,
    TMC_STEALTH,
    TMC_S2VSB,
    TMC_S2VSA
  };
  enum TMC_get_registers_enum {
    TMC_AXIS_CODES,
    TMC_GET_GCONF,
    TMC_GET_IHOLD_IRUN,
    TMC_GET_GSTAT,
    TMC_GET_IOIN,
    TMC_GET_TPOWERDOWN,
    TMC_GET_TSTEP,
    TMC_GET_TPWMTHRS,
    TMC_GET_TCOOLTHRS,
    TMC_GET_THIGH,
    TMC_GET_CHOPCONF,
    TMC_GET_COOLCONF,
    TMC_GET_PWMCONF,
    TMC_GET_PWM_SCALE,
    TMC_GET_DRV_STATUS
  };
  static void print_32b_hex(const uint32_t drv_status) {
    for (int B = 24; B >= 8; B -= 8){
      SERIAL_PRINT((uint8_t)(drv_status >> (B + 4)) & 0xF, HEX);
      SERIAL_PRINT((uint8_t)(drv_status >> B) & 0xF, HEX);
      SERIAL_CHAR(':');
    }
    SERIAL_PRINT((uint8_t)(drv_status >> 4) & 0xF, HEX);
    SERIAL_PRINT((uint8_t)(drv_status) & 0xF, HEX);
    SERIAL_EOL();
  }

  #if ENABLED(HAVE_TMC2130)
    static void tmc_status(TMC2130Stepper &st, const TMC_debug_enum i) {
      switch (i) {
        case TMC_PWM_SCALE: SERIAL_PRINT(st.PWM_SCALE(), DEC); break;
        case TMC_SGT: SERIAL_PRINT(st.sgt(), DEC); break;
        case TMC_STEALTHCHOP: serialprintPGM(st.en_pwm_mode() ? PSTR("true") : PSTR("false")); break;
        default: break;
      }
    }
    static void tmc_parse_drv_status(TMC2130Stepper &st, const TMC_drv_status_enum i) {
      switch (i) {
        case TMC_STALLGUARD: if (st.stallguard()) SERIAL_CHAR('X'); break;
        case TMC_SG_RESULT:  SERIAL_PRINT(st.sg_result(), DEC);   break;
        case TMC_FSACTIVE:   if (st.fsactive())   SERIAL_CHAR('X'); break;
        case TMC_DRV_CS_ACTUAL: SERIAL_PRINT(st.cs_actual(), DEC); break;
        default: break;
      }
    }
  #endif

  #if ENABLED(HAVE_TMC2208)
    static void tmc_status(TMC2208Stepper &st, const TMC_debug_enum i) {
      switch (i) {
        case TMC_PWM_SCALE: SERIAL_PRINT(st.pwm_scale_sum(), DEC); break;
        case TMC_STEALTHCHOP: serialprintPGM(st.stealth() ? PSTR("true") : PSTR("false")); break;
        case TMC_S2VSA: if (st.s2vsa()) SERIAL_CHAR('X'); break;
        case TMC_S2VSB: if (st.s2vsb()) SERIAL_CHAR('X'); break;
        default: break;
      }
    }
    static void tmc_parse_drv_status(TMC2208Stepper &st, const TMC_drv_status_enum i) {
      switch (i) {
        case TMC_T157: if (st.t157()) SERIAL_CHAR('X'); break;
        case TMC_T150: if (st.t150()) SERIAL_CHAR('X'); break;
        case TMC_T143: if (st.t143()) SERIAL_CHAR('X'); break;
        case TMC_T120: if (st.t120()) SERIAL_CHAR('X'); break;
        case TMC_DRV_CS_ACTUAL: SERIAL_PRINT(st.cs_actual(), DEC); break;
        default: break;
      }
    }
  #endif

  #if ENABLED(HAVE_TMC2660)
    static void tmc_parse_drv_status(TMC2660Stepper, const TMC_drv_status_enum) { }
  #endif

  template <typename TMC>
  static void tmc_status(TMC &st, const TMC_AxisEnum axis, const TMC_debug_enum i, const float spmm) {
    SERIAL_ECHO('\t');
    switch (i) {
      case TMC_CODES: _tmc_say_axis(axis); break;
      case TMC_ENABLED: serialprintPGM(st.isEnabled() ? PSTR("true") : PSTR("false")); break;
      case TMC_CURRENT: SERIAL_ECHO(st.getMilliAmps()); break;
      case TMC_RMS_CURRENT: SERIAL_PROTOCOL(st.rms_current()); break;
      case TMC_MAX_CURRENT: SERIAL_PRINT((float)st.rms_current() * 1.41, 0); break;
      case TMC_IRUN:
        SERIAL_PRINT(st.irun(), DEC);
        SERIAL_ECHOPGM("/31");
        break;
      case TMC_IHOLD:
        SERIAL_PRINT(st.ihold(), DEC);
        SERIAL_ECHOPGM("/31");
        break;
      case TMC_CS_ACTUAL:
        SERIAL_PRINT(st.cs_actual(), DEC);
        SERIAL_ECHOPGM("/31");
        break;

      case TMC_VSENSE: serialprintPGM(st.vsense() ? PSTR("1=.18") : PSTR("0=.325")); break;

      case TMC_MICROSTEPS: SERIAL_ECHO(st.microsteps()); break;
      case TMC_TSTEP: SERIAL_ECHO(st.TSTEP()); break;
      case TMC_TPWMTHRS: {
          uint32_t tpwmthrs_val = st.TPWMTHRS();
          SERIAL_ECHO(tpwmthrs_val);
        }
        break;
      case TMC_TPWMTHRS_MMS: {
          uint32_t tpwmthrs_val = st.TPWMTHRS();
          if (tpwmthrs_val)
            SERIAL_ECHO(12650000UL * st.microsteps() / (256 * tpwmthrs_val * spmm));
          else
            SERIAL_CHAR('-');
        }
        break;
      case TMC_OTPW: serialprintPGM(st.otpw() ? PSTR("true") : PSTR("false")); break;
      case TMC_OTPW_TRIGGERED: serialprintPGM(st.getOTPW() ? PSTR("true") : PSTR("false")); break;
      case TMC_TOFF: SERIAL_PRINT(st.toff(), DEC); break;
      case TMC_TBL: SERIAL_PRINT(st.blank_time(), DEC); break;
      case TMC_HEND: SERIAL_PRINT(st.hysteresis_end(), DEC); break;
      case TMC_HSTRT: SERIAL_PRINT(st.hysteresis_start(), DEC); break;
      default: tmc_status(st, i); break;
    }
  }

  template<>
  void tmc_status(TMC2660Stepper &st, const TMC_AxisEnum axis, const TMC_debug_enum i, const float) {
    SERIAL_ECHO('\t');
    switch (i) {
      case TMC_CODES: _tmc_say_axis(axis); break;
      case TMC_ENABLED: serialprintPGM(st.isEnabled() ? PSTR("true") : PSTR("false")); break;
      case TMC_CURRENT: SERIAL_ECHO(st.getMilliAmps()); break;
      case TMC_RMS_CURRENT: SERIAL_PROTOCOL(st.rms_current()); break;
      case TMC_MAX_CURRENT: SERIAL_PRINT((float)st.rms_current() * 1.41, 0); break;
      case TMC_IRUN:
        SERIAL_PRINT(st.cs(), DEC);
        SERIAL_ECHOPGM("/31");
        break;
      case TMC_VSENSE: serialprintPGM(st.vsense() ? PSTR("1=.18") : PSTR("0=.325")); break;
      case TMC_MICROSTEPS: SERIAL_ECHO(st.microsteps()); break;
      //case TMC_OTPW: serialprintPGM(st.otpw() ? PSTR("true") : PSTR("false")); break;
      //case TMC_OTPW_TRIGGERED: serialprintPGM(st.getOTPW() ? PSTR("true") : PSTR("false")); break;
      case TMC_SGT: SERIAL_PRINT(st.sgt(), DEC); break;
      case TMC_TOFF: SERIAL_PRINT(st.toff(), DEC); break;
      case TMC_TBL: SERIAL_PRINT(st.blank_time(), DEC); break;
      case TMC_HEND: SERIAL_PRINT(st.hysteresis_end(), DEC); break;
      case TMC_HSTRT: SERIAL_PRINT(st.hysteresis_start(), DEC); break;
      default: break;
    }
  }

  template <typename TMC>
  static void tmc_parse_drv_status(TMC &st, const TMC_AxisEnum axis, const TMC_drv_status_enum i) {
    SERIAL_CHAR('\t');
    switch (i) {
      case TMC_DRV_CODES:     _tmc_say_axis(axis);  break;
      case TMC_STST:          if (st.stst())         SERIAL_CHAR('X'); break;
      case TMC_OLB:           if (st.olb())          SERIAL_CHAR('X'); break;
      case TMC_OLA:           if (st.ola())          SERIAL_CHAR('X'); break;
      case TMC_S2GB:          if (st.s2gb())         SERIAL_CHAR('X'); break;
      case TMC_S2GA:          if (st.s2ga())         SERIAL_CHAR('X'); break;
      case TMC_DRV_OTPW:      if (st.otpw())         SERIAL_CHAR('X'); break;
      case TMC_OT:            if (st.ot())           SERIAL_CHAR('X'); break;
      case TMC_DRV_CS_ACTUAL: SERIAL_PRINT(st.cs_actual(), DEC);       break;
      case TMC_DRV_STATUS_HEX: {
        uint32_t drv_status = st.DRV_STATUS();
        SERIAL_ECHOPGM("\t");
        _tmc_say_axis(axis);
        SERIAL_ECHOPGM(" = 0x");
        print_32b_hex(drv_status);
        if (drv_status == 0xFFFFFFFF || drv_status == 0) SERIAL_ECHOPGM("\t Bad response!");
        SERIAL_EOL();
        break;
      }
      default: tmc_parse_drv_status(st, i); break;
    }
  }

  static void tmc_debug_loop(const TMC_debug_enum i) {
    #if X_IS_TRINAMIC
      tmc_status(stepperX, TMC_X, i, planner.axis_steps_per_mm[X_AXIS]);
    #endif
    #if X2_IS_TRINAMIC
      tmc_status(stepperX2, TMC_X2, i, planner.axis_steps_per_mm[X_AXIS]);
    #endif

    #if Y_IS_TRINAMIC
      tmc_status(stepperY, TMC_Y, i, planner.axis_steps_per_mm[Y_AXIS]);
    #endif
    #if Y2_IS_TRINAMIC
      tmc_status(stepperY2, TMC_Y2, i, planner.axis_steps_per_mm[Y_AXIS]);
    #endif

    #if Z_IS_TRINAMIC
      tmc_status(stepperZ, TMC_Z, i, planner.axis_steps_per_mm[Z_AXIS]);
    #endif
    #if Z2_IS_TRINAMIC
      tmc_status(stepperZ2, TMC_Z2, i, planner.axis_steps_per_mm[Z_AXIS]);
    #endif

    #if E0_IS_TRINAMIC
      tmc_status(stepperE0, TMC_E0, i, planner.axis_steps_per_mm[E_AXIS]);
    #endif
    #if E1_IS_TRINAMIC
      tmc_status(stepperE1, TMC_E1, i, planner.axis_steps_per_mm[E_AXIS
        #if ENABLED(DISTINCT_E_FACTORS)
          + 1
        #endif
      ]);
    #endif
    #if E2_IS_TRINAMIC
      tmc_status(stepperE2, TMC_E2, i, planner.axis_steps_per_mm[E_AXIS
        #if ENABLED(DISTINCT_E_FACTORS)
          + 2
        #endif
      ]);
    #endif
    #if E3_IS_TRINAMIC
      tmc_status(stepperE3, TMC_E3, i, planner.axis_steps_per_mm[E_AXIS
        #if ENABLED(DISTINCT_E_FACTORS)
          + 3
        #endif
      ]);
    #endif
    #if E4_IS_TRINAMIC
      tmc_status(stepperE4, TMC_E4, i, planner.axis_steps_per_mm[E_AXIS
        #if ENABLED(DISTINCT_E_FACTORS)
          + 4
        #endif
      ]);
    #endif

    SERIAL_EOL();
  }

  static void drv_status_loop(const TMC_drv_status_enum i) {
    #if X_IS_TRINAMIC
      tmc_parse_drv_status(stepperX, TMC_X, i);
    #endif
    #if X2_IS_TRINAMIC
      tmc_parse_drv_status(stepperX2, TMC_X2, i);
    #endif

    #if Y_IS_TRINAMIC
      tmc_parse_drv_status(stepperY, TMC_Y, i);
    #endif
    #if Y2_IS_TRINAMIC
      tmc_parse_drv_status(stepperY2, TMC_Y2, i);
    #endif

    #if Z_IS_TRINAMIC
      tmc_parse_drv_status(stepperZ, TMC_Z, i);
    #endif
    #if Z2_IS_TRINAMIC
      tmc_parse_drv_status(stepperZ2, TMC_Z2, i);
    #endif

    #if E0_IS_TRINAMIC
      tmc_parse_drv_status(stepperE0, TMC_E0, i);
    #endif
    #if E1_IS_TRINAMIC
      tmc_parse_drv_status(stepperE1, TMC_E1, i);
    #endif
    #if E2_IS_TRINAMIC
      tmc_parse_drv_status(stepperE2, TMC_E2, i);
    #endif
    #if E3_IS_TRINAMIC
      tmc_parse_drv_status(stepperE3, TMC_E3, i);
    #endif
    #if E4_IS_TRINAMIC
      tmc_parse_drv_status(stepperE4, TMC_E4, i);
    #endif

    SERIAL_EOL();
  }

  /**
   * M122 report functions
   */
  void tmc_set_report_status(const bool status) {
    if ((report_tmc_status = status))
      SERIAL_ECHOLNPGM("axis:pwm_scale |status_response|");
  }

  void tmc_report_all() {
    #define TMC_REPORT(LABEL, ITEM) do{ SERIAL_ECHOPGM(LABEL);  tmc_debug_loop(ITEM); }while(0)
    #define DRV_REPORT(LABEL, ITEM) do{ SERIAL_ECHOPGM(LABEL); drv_status_loop(ITEM); }while(0)
    TMC_REPORT("\t",                 TMC_CODES);
    TMC_REPORT("Enabled\t",          TMC_ENABLED);
    TMC_REPORT("Set current",        TMC_CURRENT);
    TMC_REPORT("RMS current",        TMC_RMS_CURRENT);
    TMC_REPORT("MAX current",        TMC_MAX_CURRENT);
    TMC_REPORT("Run current",        TMC_IRUN);
    TMC_REPORT("Hold current",       TMC_IHOLD);
    TMC_REPORT("CS actual\t",        TMC_CS_ACTUAL);
    TMC_REPORT("PWM scale",          TMC_PWM_SCALE);
    TMC_REPORT("vsense\t",           TMC_VSENSE);
    TMC_REPORT("stealthChop",        TMC_STEALTHCHOP);
    TMC_REPORT("msteps\t",           TMC_MICROSTEPS);
    TMC_REPORT("tstep\t",            TMC_TSTEP);
    TMC_REPORT("pwm\nthreshold\t",   TMC_TPWMTHRS);
    TMC_REPORT("[mm/s]\t",           TMC_TPWMTHRS_MMS);
    TMC_REPORT("OT prewarn",         TMC_OTPW);
    TMC_REPORT("OT prewarn has\n"
               "been triggered",     TMC_OTPW_TRIGGERED);
    TMC_REPORT("off time\t",         TMC_TOFF);
    TMC_REPORT("blank time",         TMC_TBL);
    TMC_REPORT("hysteresis\n-end\t", TMC_HEND);
    TMC_REPORT("-start\t",           TMC_HSTRT);
    TMC_REPORT("Stallguard thrs",    TMC_SGT);

    DRV_REPORT("DRVSTATUS",          TMC_DRV_CODES);
    #if ENABLED(HAVE_TMC2130)
      DRV_REPORT("stallguard\t",     TMC_STALLGUARD);
      DRV_REPORT("sg_result\t",      TMC_SG_RESULT);
      DRV_REPORT("fsactive\t",       TMC_FSACTIVE);
    #endif
    DRV_REPORT("stst\t",             TMC_STST);
    DRV_REPORT("olb\t",              TMC_OLB);
    DRV_REPORT("ola\t",              TMC_OLA);
    DRV_REPORT("s2gb\t",             TMC_S2GB);
    DRV_REPORT("s2ga\t",             TMC_S2GA);
    DRV_REPORT("otpw\t",             TMC_DRV_OTPW);
    DRV_REPORT("ot\t",               TMC_OT);
    #if ENABLED(HAVE_TMC2208)
      DRV_REPORT("157C\t",           TMC_T157);
      DRV_REPORT("150C\t",           TMC_T150);
      DRV_REPORT("143C\t",           TMC_T143);
      DRV_REPORT("120C\t",           TMC_T120);
      DRV_REPORT("s2vsa\t",          TMC_S2VSA);
      DRV_REPORT("s2vsb\t",          TMC_S2VSB);
    #endif
    DRV_REPORT("Driver registers:\n",TMC_DRV_STATUS_HEX);
    SERIAL_EOL();
  }

  #if ENABLED(HAVE_TMC2130)
    static void tmc_get_registers(TMC2130Stepper &st, TMC_AxisEnum axis, const TMC_get_registers_enum i) {
      #define PRINT_2130REGISTER(REG_CASE) case TMC_GET_##REG_CASE: SERIAL_ECHOPGM("0x"); print_32b_hex(st.REG_CASE()); break;
      switch(i) {
        case TMC_AXIS_CODES: SERIAL_CHAR('\t'); _tmc_say_axis(axis); break;
        PRINT_2130REGISTER(GCONF)
        PRINT_2130REGISTER(IHOLD_IRUN)
        PRINT_2130REGISTER(GSTAT)
        PRINT_2130REGISTER(IOIN)
        PRINT_2130REGISTER(TPOWERDOWN)
        PRINT_2130REGISTER(TSTEP)
        PRINT_2130REGISTER(TPWMTHRS)
        PRINT_2130REGISTER(TCOOLTHRS)
        PRINT_2130REGISTER(THIGH)
        PRINT_2130REGISTER(CHOPCONF)
        PRINT_2130REGISTER(COOLCONF)
        PRINT_2130REGISTER(PWMCONF)
        PRINT_2130REGISTER(PWM_SCALE)
        PRINT_2130REGISTER(DRV_STATUS)
        default: SERIAL_ECHOPGM("-\t"); break;
      }
      SERIAL_CHAR('\t');
    }
  #endif
  #if ENABLED(HAVE_TMC2208)
    static void tmc_get_registers(TMC2208Stepper &st, TMC_AxisEnum axis, const TMC_get_registers_enum i) {
      #define PRINT_2208REGISTER(REG_CASE) case TMC_GET_##REG_CASE: SERIAL_ECHOPGM("0x"); st.REG_CASE(&data); print_32b_hex(data); break;
      uint32_t data = 0ul;
      switch(i) {
        case TMC_AXIS_CODES: SERIAL_CHAR('\t'); _tmc_say_axis(axis); break;
        PRINT_2208REGISTER(GCONF)
        PRINT_2208REGISTER(IHOLD_IRUN)
        PRINT_2208REGISTER(GSTAT)
        PRINT_2208REGISTER(IOIN)
        PRINT_2208REGISTER(TPOWERDOWN)
        PRINT_2208REGISTER(TSTEP)
        PRINT_2208REGISTER(TPWMTHRS)
        PRINT_2208REGISTER(CHOPCONF)
        PRINT_2208REGISTER(PWMCONF)
        PRINT_2208REGISTER(PWM_SCALE)
        PRINT_2208REGISTER(DRV_STATUS)
        default: SERIAL_ECHOPGM("-\t"); break;
      }
      SERIAL_CHAR('\t');
    }
  #endif

  static void tmc_get_registers(TMC_get_registers_enum i, bool print_x, bool print_y, bool print_z, bool print_e) {
    if (print_x) {
      #if X_IS_TRINAMIC
        tmc_get_registers(stepperX, TMC_X, i);
      #endif
      #if X2_IS_TRINAMIC
        tmc_get_registers(stepperX2, TMC_X2, i);
      #endif
    }

    if (print_y) {
      #if Y_IS_TRINAMIC
        tmc_get_registers(stepperY, TMC_Y, i);
      #endif
      #if Y2_IS_TRINAMIC
        tmc_get_registers(stepperY2, TMC_Y2, i);
      #endif
    }

    if (print_z) {
      #if Z_IS_TRINAMIC
        tmc_get_registers(stepperZ, TMC_Z, i);
      #endif
      #if Z2_IS_TRINAMIC
        tmc_get_registers(stepperZ2, TMC_Z2, i);
      #endif
    }

    if (print_e) {
      #if E0_IS_TRINAMIC
        tmc_get_registers(stepperE0, TMC_E0, i);
      #endif
      #if E1_IS_TRINAMIC
        tmc_get_registers(stepperE1, TMC_E1, i);
      #endif
      #if E2_IS_TRINAMIC
        tmc_get_registers(stepperE2, TMC_E2, i);
      #endif
      #if E3_IS_TRINAMIC
        tmc_get_registers(stepperE3, TMC_E3, i);
      #endif
      #if E4_IS_TRINAMIC
        tmc_get_registers(stepperE4, TMC_E4, i);
      #endif
    }

    SERIAL_EOL();
  }

  void tmc_get_registers() {
    bool print_axis[XYZE],
         print_all = true;
    LOOP_XYZE(i) if (parser.seen(axis_codes[i])) { print_axis[i] = true; print_all = false; }

    #define TMC_GET_REG(LABEL, ITEM) do{ SERIAL_ECHOPGM(LABEL); tmc_get_registers(ITEM, print_axis[X_AXIS]||print_all, print_axis[Y_AXIS]||print_all, print_axis[Z_AXIS]||print_all, print_axis[E_AXIS]||print_all); }while(0)
    TMC_GET_REG("\t",             TMC_AXIS_CODES);
    TMC_GET_REG("GCONF\t\t",      TMC_GET_GCONF);
    TMC_GET_REG("IHOLD_IRUN\t",   TMC_GET_IHOLD_IRUN);
    TMC_GET_REG("GSTAT\t\t",      TMC_GET_GSTAT);
    TMC_GET_REG("IOIN\t\t",       TMC_GET_IOIN);
    TMC_GET_REG("TPOWERDOWN\t",   TMC_GET_TPOWERDOWN);
    TMC_GET_REG("TSTEP\t\t",      TMC_GET_TSTEP);
    TMC_GET_REG("TPWMTHRS\t",     TMC_GET_TPWMTHRS);
    TMC_GET_REG("TCOOLTHRS\t",    TMC_GET_TCOOLTHRS);
    TMC_GET_REG("THIGH\t\t",      TMC_GET_THIGH);
    TMC_GET_REG("CHOPCONF\t",     TMC_GET_CHOPCONF);
    TMC_GET_REG("COOLCONF\t",     TMC_GET_COOLCONF);
    TMC_GET_REG("PWMCONF\t",      TMC_GET_PWMCONF);
    TMC_GET_REG("PWM_SCALE\t",    TMC_GET_PWM_SCALE);
    TMC_GET_REG("DRV_STATUS\t",   TMC_GET_DRV_STATUS);
  }

#endif // TMC_DEBUG

#if ENABLED(SENSORLESS_HOMING)

  void tmc_sensorless_homing(TMC2130Stepper &st, bool enable/*=true*/) {
    #if ENABLED(STEALTHCHOP)
      st.TCOOLTHRS(enable ? 0xFFFFF : 0);
      st.en_pwm_mode(!enable);
    #endif
    st.diag1_stall(enable ? 1 : 0);
  }

#endif // SENSORLESS_HOMING

#if ENABLED(HAVE_TMC2130) || ENABLED(HAVE_TMC2660)
  #define SET_CS_PIN(st) OUT_WRITE(st##_CS_PIN, HIGH)
  void tmc_init_cs_pins() {
    #if ENABLED(X_IS_TMC2130)
      SET_CS_PIN(X);
    #endif
    #if ENABLED(Y_IS_TMC2130)
      SET_CS_PIN(Y);
    #endif
    #if ENABLED(Z_IS_TMC2130)
      SET_CS_PIN(Z);
    #endif
    #if ENABLED(X2_IS_TMC2130)
      SET_CS_PIN(X2);
    #endif
    #if ENABLED(Y2_IS_TMC2130)
      SET_CS_PIN(Y2);
    #endif
    #if ENABLED(Z2_IS_TMC2130)
      SET_CS_PIN(Z2);
    #endif
    #if ENABLED(E0_IS_TMC2130)
      SET_CS_PIN(E0);
    #endif
    #if ENABLED(E1_IS_TMC2130)
      SET_CS_PIN(E1);
    #endif
    #if ENABLED(E2_IS_TMC2130)
      SET_CS_PIN(E2);
    #endif
    #if ENABLED(E3_IS_TMC2130)
      SET_CS_PIN(E3);
    #endif
    #if ENABLED(E4_IS_TMC2130)
      SET_CS_PIN(E4);
    #endif
  }
#endif // HAVE_TMC2130 HAVE_TMC2660

template<typename TMC>
static void test_connection(TMC &st, const TMC_AxisEnum axis) {
  SERIAL_ECHOPGM("Testing ");
  _tmc_say_axis(axis);
  SERIAL_ECHOPGM(" connection...");
  switch(st.test_connection()) {
    case 0: SERIAL_ECHOPGM("OK"); break;
    case 1: SERIAL_ECHOPGM("Error(0xFFFFFFFF)"); break;
    case 2: SERIAL_ECHOPGM("Error(0x0)"); break;
  }
  SERIAL_EOL();
}

void test_tmc_connection() {
  #if ENABLED(X_IS_TMC2130)
    test_connection(stepperX, TMC_X);
  #endif
  #if ENABLED(Y_IS_TMC2130)
    test_connection(stepperY, TMC_Y);
  #endif
  #if ENABLED(Z_IS_TMC2130)
    test_connection(stepperZ, TMC_Z);
  #endif
  #if ENABLED(X2_IS_TMC2130)
    test_connection(stepperX2, TMC_X2);
  #endif
  #if ENABLED(Y2_IS_TMC2130)
    test_connection(stepperY2, TMC_Y2);
  #endif
  #if ENABLED(Z2_IS_TMC2130)
    test_connection(stepperZ2, TMC_Z2);
  #endif
  #if ENABLED(E0_IS_TMC2130)
    test_connection(stepperE0, TMC_E0);
  #endif
  #if ENABLED(E1_IS_TMC2130)
    test_connection(stepperE1, TMC_E1);
  #endif
  #if ENABLED(E2_IS_TMC2130)
    test_connection(stepperE2, TMC_E2);
  #endif
  #if ENABLED(E3_IS_TMC2130)
    test_connection(stepperE3, TMC_E3);
  #endif
  #if ENABLED(E4_IS_TMC2130)
    test_connection(stepperE4, TMC_E4);
  #endif
}

#if ENABLED(ULTIPANEL)
  #if ENABLED(HAVE_TMC2130)
    bool get_stealthChop(TMC2130Stepper &st) { return st.en_pwm_mode(); }
  #endif
  #if ENABLED(HAVE_TMC2208)
    bool get_stealthChop(TMC2208Stepper &st) { return !st.en_spreadCycle(); }
  #endif

  void init_tmc_section() {
    #if X_IS_TRINAMIC
      stepperX.stored.I_rms = stepperX.getCurrent();
    #endif
    #if Y_IS_TRINAMIC
      stepperY.stored.I_rms = stepperY.getCurrent();
    #endif
    #if Z_IS_TRINAMIC
      stepperZ.stored.I_rms = stepperZ.getCurrent();
    #endif
    #if X2_IS_TRINAMIC
      stepperX2.stored.I_rms = stepperX2.getCurrent();
    #endif
    #if Y2_IS_TRINAMIC
      stepperY2.stored.I_rms = stepperY2.getCurrent();
    #endif
    #if Z2_IS_TRINAMIC
      stepperZ2.stored.I_rms = stepperZ2.getCurrent();
    #endif
    #if E0_IS_TRINAMIC
      stepperE0.stored.I_rms = stepperE0.getCurrent();
    #endif
    #if E1_IS_TRINAMIC
      stepperE1.stored.I_rms = stepperE1.getCurrent();
    #endif
    #if E2_IS_TRINAMIC
      stepperE2.stored.I_rms = stepperE2.getCurrent();
    #endif
    #if E3_IS_TRINAMIC
      stepperE3.stored.I_rms = stepperE3.getCurrent();
    #endif
    #if E4_IS_TRINAMIC
      stepperE4.stored.I_rms = stepperE4.getCurrent();
    #endif

    #if ENABLED(HYBRID_THRESHOLD)
      #define GET_HYBRID_THRS(ST, AX) _tmc_thrs(stepper##ST.microsteps(), stepper##ST.TPWMTHRS(), planner.axis_steps_per_mm[AX##_AXIS])
      #define GET_HYBRID_THRS_E(ST) do { const uint8_t extruder = 0; stepper##ST.stored.hybrid_thrs = _tmc_thrs(stepper##ST.microsteps(), stepper##ST.TPWMTHRS(), planner.axis_steps_per_mm[E_AXIS_N]); }while(0)
      #if X_IS_TRINAMIC
        stepperX.stored.hybrid_thrs = GET_HYBRID_THRS(X, X);
      #endif
      #if Y_IS_TRINAMIC
        stepperY.stored.hybrid_thrs = GET_HYBRID_THRS(Y, Y);
      #endif
      #if Z_IS_TRINAMIC
        stepperZ.stored.hybrid_thrs = GET_HYBRID_THRS(Z, Z);
      #endif
      #if X2_IS_TRINAMIC
        stepperX2.stored.hybrid_thrs = GET_HYBRID_THRS(X2, X);
      #endif
      #if Y2_IS_TRINAMIC
        stepperY2.stored.hybrid_thrs = GET_HYBRID_THRS(Y2, Y);
      #endif
      #if Z2_IS_TRINAMIC
        stepperZ2.stored.hybrid_thrs = GET_HYBRID_THRS(Z2, Z);
      #endif
      #if E0_IS_TRINAMIC
        GET_HYBRID_THRS_E(E0);
      #endif
      #if E1_IS_TRINAMIC
        GET_HYBRID_THRS_E(E1);
      #endif
      #if E2_IS_TRINAMIC
        GET_HYBRID_THRS_E(E2);
      #endif
      #if E3_IS_TRINAMIC
        GET_HYBRID_THRS_E(E3);
      #endif
      #if E4_IS_TRINAMIC
        GET_HYBRID_THRS_E(E4);
      #endif
    #endif

    #if ENABLED(SENSORLESS_HOMING)
      #if ENABLED(X_IS_TMC2130)
        stepperX.stored.homing_thrs = stepperX.sgt();
      #endif
      #if ENABLED(Y_IS_TMC2130)
        stepperY.stored.homing_thrs = stepperY.sgt();
      #endif
      #if ENABLED(Z_IS_TMC2130)
        stepperZ.stored.homing_thrs = stepperZ.sgt();
      #endif
    #endif

    #if ENABLED(STEALTHCHOP)
      #if X_IS_TRINAMIC
        stepperX.stored.stealthChop_enabled = get_stealthChop(stepperX);
      #endif
      #if Y_IS_TRINAMIC
        stepperY.stored.stealthChop_enabled = get_stealthChop(stepperY);
      #endif
      #if Z_IS_TRINAMIC
        stepperZ.stored.stealthChop_enabled = get_stealthChop(stepperZ);
      #endif
      #if X2_IS_TRINAMIC
        stepperX2.stored.stealthChop_enabled = get_stealthChop(stepperX2);
      #endif
      #if Y2_IS_TRINAMIC
        stepperY2.stored.stealthChop_enabled = get_stealthChop(stepperY2);
      #endif
      #if Z2_IS_TRINAMIC
        stepperZ2.stored.stealthChop_enabled = get_stealthChop(stepperZ2);
      #endif
      #if E0_IS_TRINAMIC
        stepperE0.stored.stealthChop_enabled = get_stealthChop(stepperE0);
      #endif
      #if E1_IS_TRINAMIC
        stepperE1.stored.stealthChop_enabled = get_stealthChop(stepperE1);
      #endif
      #if E2_IS_TRINAMIC
        stepperE2.stored.stealthChop_enabled = get_stealthChop(stepperE2);
      #endif
      #if E3_IS_TRINAMIC
        stepperE3.stored.stealthChop_enabled = get_stealthChop(stepperE3);
      #endif
      #if E4_IS_TRINAMIC
        stepperE4.stored.stealthChop_enabled = get_stealthChop(stepperE4);
      #endif
    #endif
  }
  void refresh_tmc_driver_current() {
    SERIAL_ECHO("refresh_tmc_driver_current=");
    SERIAL_ECHO_F(stepperX.stored.I_rms, DEC);
    SERIAL_EOL();
    #if X_IS_TRINAMIC
      stepperX.setCurrent(stepperX.stored.I_rms, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if Y_IS_TRINAMIC
      stepperY.setCurrent(stepperY.stored.I_rms, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if Z_IS_TRINAMIC
      stepperZ.setCurrent(stepperZ.stored.I_rms, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if X2_IS_TRINAMIC
      stepperX2.setCurrent(stepperX2.stored.I_rms, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if Y2_IS_TRINAMIC
      stepperY2.setCurrent(stepperY2.stored.I_rms, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if Z2_IS_TRINAMIC
      stepperZ2.setCurrent(stepperZ2.stored.I_rms, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if E0_IS_TRINAMIC
      stepperE0.setCurrent(stepperE0.stored.I_rms, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if E1_IS_TRINAMIC
      stepperE1.setCurrent(stepperE1.stored.I_rms, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if E2_IS_TRINAMIC
      stepperE2.setCurrent(stepperE2.stored.I_rms, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if E3_IS_TRINAMIC
      stepperE3.setCurrent(stepperE3.stored.I_rms, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if E4_IS_TRINAMIC
      stepperE4.setCurrent(stepperE4.stored.I_rms, R_SENSE, HOLD_MULTIPLIER);
    #endif
  }
  #if ENABLED(HAVE_TMC2130)
    void _set_tmc_stepping_mode(TMC2130Stepper &st, bool enable_stealthChop) {
      st.en_pwm_mode(enable_stealthChop);
    }
  #endif
  #if ENABLED(HAVE_TMC2208)
    void _set_tmc_stepping_mode(TMC2208Stepper &st, bool enable_stealthChop) {
      st.en_spreadCycle(!enable_stealthChop);
    }
  #endif
  #if ENABLED(STEALTHCHOP)
    void set_tmc_stepping_mode() {
      SERIAL_ECHO("set_tmc_stepping_mode=");
      SERIAL_ECHO_F(stepperX.stored.stealthChop_enabled, DEC);
      SERIAL_EOL();
      _set_tmc_stepping_mode(stepperX, stepperX.stored.stealthChop_enabled);
    }
  #endif
  #if ENABLED(HYBRID_THRESHOLD)
    void refresh_tmc_hybrid_thrs() {
      SERIAL_ECHO("refresh_tmc_hybrid_thrs=");
      SERIAL_ECHO_F(stepperX.stored.hybrid_thrs, DEC);
      SERIAL_EOL();
      tmc_set_pwmthrs(stepperX, stepperX.stored.hybrid_thrs, planner.axis_steps_per_mm[X_AXIS]);
    }
  #endif
  #if ENABLED(SENSORLESS_HOMING)
    void refresh_tmc_homing_thrs() {
      SERIAL_ECHO("refresh_tmc_homing_thrs=");
      SERIAL_ECHO_F(stepperX.stored.homing_thrs, DEC);
      SERIAL_EOL();
      tmc_set_sgt(stepperX, stepperX.stored.homing_thrs);
    }
  #endif
#endif

#endif // HAS_TRINAMIC
