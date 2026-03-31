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
// See aprs.h for architecture notes.

#ifdef ENABLE_APRS_TX

#include "app/aprs.h"
#include "app/kiss.h"
#include "driver/py25q16.h"
#include <string.h>

// ----------------------------------------------------------------
// APRS settings storage in PY25Q16 flash
//
// Physical address 0x00B000 is an unused 4 KB sector in the
// PY25Q16 2 MB flash (falls between the settings region at
// 0x00A000-0x00A170 and the calibration region at 0x010000).
// Not mapped by the eeprom_compat.c address table, so we access
// it directly via PY25Q16_ReadBuffer / PY25Q16_WriteBuffer.
// ----------------------------------------------------------------

#define APRS_FLASH_ADDR  0x00B000u

// Sentinel stored in _pad[0]: if != APRS_MAGIC, use defaults.
#define APRS_MAGIC       0xA5u

APRS_Config_t gAprsConfig;

// Default configuration — used when flash has never been written.
static const APRS_Config_t APRS_DEFAULT = {
    .callsign       = "NOCALL",
    .ssid           = 0u,
    .lat            = "0000.00N",
    .lon            = "00000.00E",
    .symbol_table   = '/',
    .symbol_code    = '-',         // house symbol
    .comment        = "UV-K1 APRS",
    .beacon_interval = APRS_INTERVAL_OFF,
    ._pad           = { APRS_MAGIC, 0u }
};

// ----------------------------------------------------------------
// Flash persistence
// ----------------------------------------------------------------

void APRS_Init(void)
{
    APRS_Config_t cfg;
    PY25Q16_ReadBuffer(APRS_FLASH_ADDR, &cfg, sizeof(cfg));

    // Validate: check magic byte and that callsign is printable ASCII
    if (cfg._pad[0] == APRS_MAGIC && cfg.callsign[0] >= 'A' && cfg.callsign[0] <= 'Z') {
        // Ensure NUL terminators (flash might have garbage after valid data)
        cfg.callsign[6] = '\0';
        cfg.lat[8]      = '\0';
        cfg.lon[9]      = '\0';
        cfg.comment[43] = '\0';
        gAprsConfig = cfg;
    } else {
        gAprsConfig = APRS_DEFAULT;
    }
}

void APRS_SaveConfig(void)
{
    gAprsConfig._pad[0] = APRS_MAGIC;
    gAprsConfig._pad[1] = 0u;
    PY25Q16_WriteBuffer(APRS_FLASH_ADDR, &gAprsConfig, sizeof(gAprsConfig), false);
}

bool APRS_IsDefault(void)
{
    return (gAprsConfig.callsign[0] == 'N' &&
            gAprsConfig.callsign[1] == 'O' &&
            gAprsConfig.callsign[2] == 'C');
}

// ----------------------------------------------------------------
// AX.25 frame builder
// ----------------------------------------------------------------

// Encode one AX.25 address field (7 bytes) into buf.
// call: 6-char callsign (space-padded); ssid: 0-15; last: true if
//       this is the final address in the header (sets extension bit).
static void encode_addr(uint8_t *buf, const char *call, uint8_t ssid, bool last)
{
    for (int i = 0; i < 6; i++) {
        char c = (call[i] != '\0') ? call[i] : ' ';
        buf[i] = (uint8_t)((unsigned char)c << 1);
    }
    // SSID byte: 0b0110_ssss_e
    //   bits 7-6 : 0,1 (C=0, reserved=1)
    //   bits 5-4 : 1,1 (reserved)  — common practice: 0b0110 prefix
    //   bits 4-1 : SSID
    //   bit 0    : extension (1 = last address)
    buf[6] = (uint8_t)(0x60u | ((ssid & 0x0Fu) << 1) | (last ? 1u : 0u));
}

// Build a complete AX.25 UI frame into buf[].
// Returns number of bytes written (NOT including FCS — AFSK_TxFrame adds FCS).
static uint16_t build_ax25_frame(uint8_t *buf)
{
    uint16_t pos = 0u;

    // Destination: APK1UV-0  (APRS tocall for UV-K1 firmware, not last)
    encode_addr(&buf[pos], "APK1UV", 0u, false);
    pos += 7u;

    // Source: user callsign + SSID  (not last — digipeater follows)
    char src[7];
    memcpy(src, gAprsConfig.callsign, 6u);
    src[6] = '\0';
    encode_addr(&buf[pos], src, gAprsConfig.ssid, false);
    pos += 7u;

    // Digipeater: WIDE1-1  (last address — extension bit set)
    encode_addr(&buf[pos], "WIDE1 ", 1u, true);
    pos += 7u;

    // Control: UI frame (unnumbered information)
    buf[pos++] = 0x03u;

    // PID: no layer 3 protocol
    buf[pos++] = 0xF0u;

    // Info field: APRS position without timestamp, no messaging
    // Format: !DDMM.MMN/DDDMM.MME<symbol><comment>
    buf[pos++] = '!';

    // Latitude (8 chars: DDMM.MMN)
    const char *lat = gAprsConfig.lat;
    for (int i = 0; i < 8 && lat[i]; i++)
        buf[pos++] = (uint8_t)lat[i];

    // Symbol table ID
    buf[pos++] = (uint8_t)gAprsConfig.symbol_table;

    // Longitude (9 chars: DDDMM.MME)
    const char *lon = gAprsConfig.lon;
    for (int i = 0; i < 9 && lon[i]; i++)
        buf[pos++] = (uint8_t)lon[i];

    // Symbol code
    buf[pos++] = (uint8_t)gAprsConfig.symbol_code;

    // Comment (up to 43 chars)
    const char *comment = gAprsConfig.comment;
    uint16_t clen = 0u;
    while (clen < 43u && comment[clen])
        clen++;
    memcpy(&buf[pos], comment, clen);
    pos += clen;

    return pos;
}

// ----------------------------------------------------------------
// APRS TX
// ----------------------------------------------------------------

bool APRS_TxBeacon(void)
{
    // Max AX.25 frame: 3×7 (addresses) + 2 (ctrl+pid) + ~60 (info) ≈ 83 bytes
    static uint8_t frame[128u];

    uint16_t len = build_ax25_frame(frame);
    return AFSK_TxFrame(frame, len);
}

// ----------------------------------------------------------------
// Auto-beacon interval (called from 500 ms app tick)
// ----------------------------------------------------------------

// Beacon intervals in 500 ms ticks
static const uint16_t interval_ticks[] = {
    0u,     // off
    120u,   // 1  min  = 120 × 500 ms
    600u,   // 5  min  = 600 × 500 ms
    1200u,  // 10 min  = 1200 × 500 ms
    3600u   // 30 min  = 3600 × 500 ms
};

static uint16_t beacon_tick_counter = 0u;

void APRS_Tick500ms(void)
{
    uint8_t interval_idx = gAprsConfig.beacon_interval;
    if (interval_idx == APRS_INTERVAL_OFF ||
        interval_idx >= (uint8_t)(sizeof(interval_ticks) / sizeof(interval_ticks[0])))
        return;

    uint16_t period = interval_ticks[interval_idx];
    if (period == 0u)
        return;

    beacon_tick_counter++;
    if (beacon_tick_counter >= period) {
        beacon_tick_counter = 0u;
        APRS_TxBeacon();
    }
}

#endif // ENABLE_APRS_TX
