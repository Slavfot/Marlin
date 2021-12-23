/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * Mbot 3d Mightyboard Rev.C pin assignments
 * It's a copy of Mightyboards Rev.E with some differences:
 * - Thermistors instead of Thermocouplers
 * - An additional FET
 * - No internal Voltage regulator, external ATX-PSU Needed.
 */

/**
 * Rev B  2 JAN 2017
 *
 *  Added pin definitions for:
 *    M3, M4 & M5 spindle control commands
 *    case light
 *
 *  Corrected pin assignment for EX2_HEAT_PIN pin. Changed it from 9 to 11.  The port
 *  number (B5) agrees with the schematic but B5 is assigned to logical pin 11.
 */

//#define ALLOW_MEGA1280
#include "env_validate.h"

#define BOARD_INFO_NAME      "MBOT3D_MIGHTYBOARD_REVC"
#define DEFAULT_MACHINE_NAME "ZYYX Pro"

//
// Servos
//
#define SERVO0_PIN                            36  // C1 (1280-EX1) x
#define SERVO1_PIN                            37  // C0 (1280-EX2) x
#define SERVO2_PIN                            40  // G1 (1280-EX3) x
#define SERVO3_PIN                            41  // G0 (1280-EX4) x

//
// Limit Switches
//
#define X_MIN_PIN                             49  // L0 x
#define X_MAX_PIN                             48  // L1 x
#define Y_MIN_PIN                             47  // L2 x
#define Y_MAX_PIN                             46  // L3 x
#define Z_MIN_PIN                             43  // L6 x
#define Z_MAX_PIN                             42  // L7 x

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     42  // L7 x
#endif

//
// Filament Runout Pins
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      42 // L7 x
#endif
#ifndef FIL_RUNOUT2_PIN
  #define FIL_RUNOUT2_PIN                     37 // C0 x
#endif

//
// Steppers
//
#define X_STEP_PIN                            55  // F1 x
#define X_DIR_PIN                             54  // F0 x
#define X_ENABLE_PIN                          56  // F2 x

#define Y_STEP_PIN                            59  // F5 x
#define Y_DIR_PIN                             58  // F4 x
#define Y_ENABLE_PIN                          60  // F6 x

#define Z_STEP_PIN                            63  // K1 x
#define Z_DIR_PIN                             62  // K0 x
#define Z_ENABLE_PIN                          64  // K2 x

#define E0_STEP_PIN                           25  // A3 x
#define E0_DIR_PIN                            24  // A2 x
#define E0_ENABLE_PIN                         26  // A4 x

#define E1_STEP_PIN                           29  // A7 x
#define E1_DIR_PIN                            28  // A6 x
#define E1_ENABLE_PIN                         39  // G2 x

//
// I2C Digipots - MCP4018
// Address 5E (2F << 1)
// Set from 0 - 127 with stop bit.
// (Ex. 3F << 1 | 1)
//
#define DIGIPOTS_I2C_SCL                      76  // J5 x
#define DIGIPOTS_I2C_SDA_X                    57  // F3 x
#define DIGIPOTS_I2C_SDA_Y                    61  // F7 x
#define DIGIPOTS_I2C_SDA_Z                    65  // K3 x
#define DIGIPOTS_I2C_SDA_E0                   27  // A5 x
#define DIGIPOTS_I2C_SDA_E1                   77  // J6 x

#ifndef DIGIPOT_I2C_ADDRESS_A
  #define DIGIPOT_I2C_ADDRESS_A             0x2F  // unshifted slave address (5E <- 2F << 1)
#endif
#define DIGIPOT_ENABLE_I2C_PULLUPS                // MightyBoard doesn't have hardware I2C pin pull-ups.

//
// Temperature Sensors
//
#ifndef TEMP_0_PIN
  #define TEMP_0_PIN                          13  // Analog Input, TEMP 1 on motherboard
#endif
//#ifndef TEMP_1_PIN
  //#define TEMP_1_PIN                        12  // Analog Input, TEMP 2 on motherboard
#ifndef TEMP_CHAMBER_PIN
  #define TEMP_CHAMBER_PIN                    12  // Analog Input, TEMP 2 on motherboard
#endif
// K7 - 69 / ADC15 - 15
#define TEMP_BED_PIN                          15 // K7 x Analog pin, HBP-THERM on motherboard

// SPI for MAX Thermocouple
// Uses a separate SPI bus
//
//  3 E5 DO (SO)
//  5 E3 CS1
//  2 E4 CS2
// 78 E2 SCK
//
//#define TEMP_0_CS_PIN                          5  // E3 MBOT3D has no thermocoupler like mightyboard
//#define TEMP_0_SCK_PIN                        78  // E2 MBOT3D has no thermocoupler like mightyboard
//#define TEMP_0_MISO_PIN                        3  // E5 MBOT3D has no thermocoupler like mightyboard
//#define TEMP_0_MOSI_PIN                    ...  // For MAX31865

//#define TEMP_1_CS_PIN                          2  // E4 MBOT3D has no thermocoupler like mightyboard
//#define TEMP_1_SCK_PIN            TEMP_0_SCK_PIN
//#define TEMP_1_MISO_PIN          TEMP_0_MISO_PIN
//#define TEMP_1_MOSI_PIN        TEMP_0_MOSI_PIN

//
// Augmentation for auto-assigning plugs
//
// Two thermocouple connectors allows for either
// 2 extruders or 1 extruder and a heated bed.
// With no heated bed, an additional 24V fan is possible.
//

// Labels from the schematic:
#define EX1_HEAT_PIN                           6  // H3 x
#define EX1_FAN_PIN                            7  // H4 x
#define EX2_HEAT_PIN                          11  // B5 x
#define EX2_FAN_PIN                           12  // B6 x
#define HBP_PIN                               45  // L4 x
#define EXTRA_FET_PIN                         44  // L5 x
#define EXTRA_FET2_PIN                        38  // D7 x

/* Delete this?
#if HAS_MULTI_HOTEND
  #if TEMP_SENSOR_BED
    #define IS_EEB
  #else
    #define IS_EEF
  #endif
#elif TEMP_SENSOR_BED
  #define IS_EFB
#else
  #define IS_EFF
#endif
*/

//
// Heaters / Fans (24V)
//
#define HEATER_0_PIN                EX1_HEAT_PIN // HEAT NOZZLE 1
//#define HEATER_1_PIN                EX2_HEAT_PIN // HEAT NOZZLE 2
//#define HEATER_BED_PIN                       HBP // HBP used for chamber defined in configuration_adv.h

//#define CONTROLLER_FAN_PIN                     // Should be defined?
//#define CHAMBER_FAN_PIN                        // don't know if this is neede or if it is defined by chamber fan index in configuration_adv
#define FAN_PIN                   EXTRA_FET2_PIN // FAN NOZZLE
#define FAN1_PIN                     EX2_FAN_PIN // FANS CHAMBER (set to controller fan) contorllerfan.cpp editetd to enable controller fan with chamber
#define FAN2_PIN                   EXTRA_FET_PIN // FAN EXHAUST  (Set to chamber fan) exhausts the hot air from the chamber when too hot
#define FAN3_PIN                     EX1_FAN_PIN // FAN EXTRUDER (Controller fan, wire to the same as Chamber Fans)

/* Delete this?
#if ENABLED(IS_EFB)                               // Hotend, Fan, Bed
  #define HEATER_BED_PIN                 HBP_PIN
#elif ENABLED(IS_EEF)                             // Hotend, Hotend, Fan
  #define HEATER_1_PIN              EX2_HEAT_PIN
#elif ENABLED(IS_EEB)                             // Hotend, Hotend, Bed
  #define HEATER_1_PIN              EX2_HEAT_PIN
  #define HEATER_BED_PIN                 HBP_PIN
#elif ENABLED(IS_EFF)                             // Hotend, Fan, Fan
  #define FAN1_PIN                       HBP_PIN
#endif
*/

/* Delete this?
#ifndef FAN_PIN
  #if EITHER(IS_EFB, IS_EFF)                      // Hotend, Fan, Bed or Hotend, Fan, Fan
    #define FAN_PIN                 EX2_HEAT_PIN
  #elif EITHER(IS_EEF, IS_SF)                     // Hotend, Hotend, Fan or Spindle, Fan
    #define FAN_PIN                      HBP_PIN
  #else
    #define FAN_PIN                EXTRA_FET_PIN
  #endif
#endif
*/

//
// Misc. Functions
//
#define LED_PIN                               13  // B7 x
#define CUTOFF_RESET_PIN                      16  // H1 x
#define CUTOFF_TEST_PIN                       17  // H0 x
#define CUTOFF_SR_CHECK_PIN                   70  // G4 (TOSC1) x

//
// LCD / Controller
//
#if HAS_WIRED_LCD

  #if IS_RRD_FG_SC

    #define LCD_PINS_RS                       33  // C4: LCD-STROBE x
    #define LCD_PINS_ENABLE                   72  // J2: LEFT x
    #define LCD_PINS_D4                       35  // C2: LCD-CLK x
    #define LCD_PINS_D5                       32  // C5: RLED x
    #define LCD_PINS_D6                       34  // C3: LCD-DATA x
    #define LCD_PINS_D7                       31  // C6: GLED x

    #define BTN_EN2                           75  // J4, UP x
    #define BTN_EN1                           73  // J3, DOWN x
    //STOP button connected as KILL_PIN
    #define KILL_PIN                          14  // J1, RIGHT x
    //KILL - not connected

    #define BEEPER_PIN                         8  // H5, SD_WP x

    //on board leds
    #define STAT_LED_RED_LED          SERVO0_PIN  // C1 (1280-EX1, DEBUG2) x
    #define STAT_LED_BLUE_PIN         SERVO1_PIN  // C0 (1280-EX2, DEBUG3) x

  #else
    // Replicator uses a 3-wire SR controller with HD44780
    #define SR_DATA_PIN                       34  // C3 (56) (D34) (A,8) x
    #define SR_CLK_PIN                        35  // C2 x
    #define SR_STROBE_PIN                     33  // C4 x

    #define BTN_UP                            75  // J4 x
    #define BTN_DWN                           73  // J3 x
    #define BTN_LFT                           72  // J2 x
    #define BTN_RT                            14  // J1 x

    // Disable encoder
    #undef BTN_EN1
    #undef BTN_EN2

    #define BEEPER_PIN                         4  // G5 x

    #define STAT_LED_RED_PIN                  32  // C5 x
    #define STAT_LED_BLUE_PIN                 31  // C6 (Actually green) x

  #endif

  #define BTN_CENTER                          15  // J0 (63) x
  #define BTN_ENC                     BTN_CENTER

#endif // HAS_WIRED_LCD

//
// SD Card
//
#define SDSS                                  53  // B0 x
#define SD_DETECT_PIN                          9  // H6 

//
// TMC 220x
//
#if HAS_TMC_UART
  /**
   * TMC220x stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  #define X_HARDWARE_SERIAL Serial2
  #define Y_HARDWARE_SERIAL Serial1

  /**
   * Software serial
   */

  #define X_SERIAL_TX_PIN                     16 // PH1 x
  #define X_SERIAL_RX_PIN                     17 // PH0 x

  #define Y_SERIAL_TX_PIN                     18 // PD3 x
  #define Y_SERIAL_RX_PIN                     19 // PD2 x

  #define Z_SERIAL_TX_PIN                     41 // PG0 x
  #define Z_SERIAL_RX_PIN                     22 // PA0 x

  #define E0_SERIAL_TX_PIN                    40 // PG1 x
  #define E0_SERIAL_RX_PIN                    23 // PA1 x

  #define E1_SERIAL_TX_PIN                    37 // PC0 x
  #define E1_SERIAL_RX_PIN                    68 // PK6 x

#endif
