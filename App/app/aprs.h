/* Copyright 2025 dikei100
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

// APRS TX beacon — on-device AX.25 frame builder + Bell 202 AFSK TX.
//
// Requires ENABLE_KISS_TNC or ENABLE_APRS_TX in App/CMakeLists.txt;
// kiss.c must also be compiled (provides the AFSK TX engine).
//
// Settings are stored in PY25Q16 SPI flash at physical address 0x00B000.
//
// APRS position format (fixed position, no GPS):
//   !DDMM.MMN/DDDMM.MME-comment
//   where N/S and E/W are embedded in the lat/lon strings.
//
// Default AX.25 destination: APK1UV  (APRS tocall for UV-K1 firmware)
// Digipeater path:            WIDE1-1 (single hop)

#ifndef APP_APRS_H
#define APP_APRS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef ENABLE_APRS_TX

// ----------------------------------------------------------------
// APRS configuration (persisted to flash)
// ----------------------------------------------------------------

typedef struct {
    char    callsign[7];    // 6-char callsign + NUL  (e.g. "NOCALL")
    uint8_t ssid;           // 0-15
    char    lat[9];         // pre-formatted DDMM.MMN (8 chars + NUL)
    char    lon[10];        // pre-formatted DDDMM.MME (9 chars + NUL)
    char    symbol_table;   // '/' = primary, '\\' = alternate
    char    symbol_code;    // '-' house, '>' car, '[' jogger, etc.
    char    comment[44];    // up to 43 chars + NUL
    uint8_t beacon_interval;// 0=off, 1=1 min, 2=5 min, 3=10 min, 4=30 min
    uint8_t _pad[2];        // reserved / alignment
} APRS_Config_t;

extern APRS_Config_t gAprsConfig;

// ----------------------------------------------------------------
// Beacon intervals (minutes; 0 = off)
// ----------------------------------------------------------------
#define APRS_INTERVAL_OFF    0u
#define APRS_INTERVAL_1MIN   1u
#define APRS_INTERVAL_5MIN   2u
#define APRS_INTERVAL_10MIN  3u
#define APRS_INTERVAL_30MIN  4u

// ----------------------------------------------------------------
// API
// ----------------------------------------------------------------

// Load APRS config from flash (call once at boot).
void APRS_Init(void);

// Save current gAprsConfig to flash.
void APRS_SaveConfig(void);

// Build and transmit one APRS position beacon immediately.
// Returns false if AFSK TX is already in progress.
bool APRS_TxBeacon(void);

// Call from the 500 ms app tick to handle auto-beacon interval.
void APRS_Tick500ms(void);

// Returns true when the config still holds default "NOCALL" values.
bool APRS_IsDefault(void);

#endif // ENABLE_APRS_TX

#endif // APP_APRS_H
