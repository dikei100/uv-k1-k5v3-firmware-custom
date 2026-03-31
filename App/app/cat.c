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

// Kenwood TS-480 CAT control emulation over USB CDC.
// Text-based protocol: commands are uppercase ASCII terminated by ';'.
// Auto-detected by the UART dispatcher when first byte is A-Z.

#include "app/cat.h"
#include "driver/vcp.h"
#include "driver/bk4819.h"
#include "radio.h"
#include "settings.h"
#include "frequencies.h"
#include "functions.h"
#include "misc.h"
#include <stdio.h>
#include <string.h>

#define CAT_BUF_SIZE 64

static uint8_t  gCAT_Buf[CAT_BUF_SIZE];
static uint8_t  gCAT_BufLen;

// VCP_ReadIndex is defined in uart.c, declared in driver/vcp.h.

bool CAT_IsCATByte(uint8_t byte)
{
    return (byte >= 'A' && byte <= 'Z');
}

// Send a CAT response string over USB CDC.
static void CAT_Send(const char *str)
{
    VCP_SendStr(str);
}

// Map ModulationMode_t to Kenwood MD mode number.
// Kenwood: 1=LSB, 2=USB, 3=CW, 4=FM, 5=AM, 6=FSK, 9=CW-R
static uint8_t CAT_ModToKenwood(ModulationMode_t mod)
{
    switch (mod) {
        case MODULATION_FM:  return 4;
        case MODULATION_AM:  return 5;
        case MODULATION_USB: return 2;
#ifdef ENABLE_MOD_DIG
        case MODULATION_DIG: return 6; // FSK/digital
#endif
        default:             return 4; // default FM
    }
}

// Map Kenwood MD mode number to ModulationMode_t.
static ModulationMode_t CAT_KenwoodToMod(uint8_t kmd)
{
    switch (kmd) {
        case 4:  return MODULATION_FM;
        case 5:  return MODULATION_AM;
        case 1:  // LSB → USB (we don't have LSB)
        case 2:  return MODULATION_USB;
#ifdef ENABLE_MOD_DIG
        case 6:  return MODULATION_DIG;
#endif
        default: return MODULATION_FM;
    }
}

// Format frequency as 11-digit string (Hz) for Kenwood FA/FB commands.
// UV-K1 stores frequency in units of 10 Hz internally.
static void CAT_FormatFreq(char *buf, uint32_t freq)
{
    // freq is in units of 10 Hz, convert to Hz
    uint32_t hz = freq * 10;
    sprintf(buf, "%011lu", (unsigned long)hz);
}

// Parse 11-digit frequency string (Hz) back to internal units (10 Hz).
static uint32_t CAT_ParseFreq(const char *str)
{
    uint32_t hz = 0;
    for (int i = 0; i < 11 && str[i] >= '0' && str[i] <= '9'; i++)
        hz = hz * 10 + (str[i] - '0');
    return hz / 10;
}

// Handle a complete CAT command (without trailing ';').
static void CAT_HandleCommand(const char *cmd, uint8_t len)
{
    char reply[64];

    if (len < 2)
        return;

    // ID — Radio identification
    if (cmd[0] == 'I' && cmd[1] == 'D') {
        CAT_Send("ID020;");  // TS-480
        return;
    }

    // PS — Power status (always on)
    if (cmd[0] == 'P' && cmd[1] == 'S') {
        if (len == 2) {
            CAT_Send("PS1;");
        }
        return;
    }

    // FA — VFO A frequency
    if (cmd[0] == 'F' && cmd[1] == 'A') {
        if (len == 2) {
            // Read
            char freq[12];
            CAT_FormatFreq(freq, gEeprom.VfoInfo[0].freq_config_RX.Frequency);
            sprintf(reply, "FA%s;", freq);
            CAT_Send(reply);
        } else if (len >= 13) {
            // Set — validate against band table
            uint32_t f = CAT_ParseFreq(cmd + 2);
            if (f > 0 && RX_freq_check(f) >= 0) {
                gEeprom.VfoInfo[0].freq_config_RX.Frequency = f;
                gRequestSaveChannel = 1;
                gUpdateDisplay = true;
            }
        }
        return;
    }

    // FB — VFO B frequency
    if (cmd[0] == 'F' && cmd[1] == 'B') {
        if (len == 2) {
            char freq[12];
            CAT_FormatFreq(freq, gEeprom.VfoInfo[1].freq_config_RX.Frequency);
            sprintf(reply, "FB%s;", freq);
            CAT_Send(reply);
        } else if (len >= 13) {
            uint32_t f = CAT_ParseFreq(cmd + 2);
            if (f > 0 && RX_freq_check(f) >= 0) {
                gEeprom.VfoInfo[1].freq_config_RX.Frequency = f;
                gRequestSaveChannel = 1;
                gUpdateDisplay = true;
            }
        }
        return;
    }

    // MD — Mode (modulation)
    if (cmd[0] == 'M' && cmd[1] == 'D') {
        if (len == 2) {
            uint8_t kmod = CAT_ModToKenwood(gRxVfo->Modulation);
            sprintf(reply,"MD%u;", kmod);
            CAT_Send(reply);
        } else if (len >= 3 && cmd[2] >= '0' && cmd[2] <= '9') {
            uint8_t kmod = cmd[2] - '0';
            ModulationMode_t mod = CAT_KenwoodToMod(kmod);
            gRxVfo->Modulation = mod;
            RADIO_SetModulation(mod);
            gUpdateDisplay = true;
        }
        return;
    }

    // TX — Transmit (TX0; = MIC, TX1; = data)
    if (cmd[0] == 'T' && cmd[1] == 'X') {
        if (gCurrentFunction != FUNCTION_TRANSMIT && !gSerialConfigCountDown_500ms) {
            gFlagPrepareTX = true;
        }
        CAT_Send("TX0;");
        return;
    }

    // RX — Receive
    if (cmd[0] == 'R' && cmd[1] == 'X') {
        if (gCurrentFunction == FUNCTION_TRANSMIT) {
            FUNCTION_Select(FUNCTION_FOREGROUND);
        }
        CAT_Send("RX0;");
        return;
    }

    // SM — S-meter reading (0-30 scale)
    if (cmd[0] == 'S' && cmd[1] == 'M') {
        if (len == 2 || (len >= 3 && cmd[2] == '0')) {
            uint16_t rssi = BK4819_ReadRegister(BK4819_REG_67) & 0x01FFu;
            // Scale RSSI (0-511) to Kenwood S-meter (0000-0030)
            uint16_t smeter = (rssi > 240u) ? 30u : (rssi * 30u / 240u);
            sprintf(reply, "SM0%04u;", smeter);
            CAT_Send(reply);
        }
        return;
    }

    // AG — AF gain
    if (cmd[0] == 'A' && cmd[1] == 'G') {
        if (len == 3 && cmd[2] == '0') {
            // Read: scale 0-63 to 0-255
            uint16_t gain = (gEeprom.VOLUME_GAIN * 255) / 63;
            sprintf(reply,"AG0%03u;", gain);
            CAT_Send(reply);
        } else if (len >= 6) {
            // Set: scale 0-255 to 0-63
            uint16_t val = 0;
            for (int i = 3; i < len && cmd[i] >= '0' && cmd[i] <= '9'; i++)
                val = val * 10 + (cmd[i] - '0');
            gEeprom.VOLUME_GAIN = (val * 63) / 255;
            gUpdateDisplay = true;
        }
        return;
    }

    // SQ — Squelch level (0-255)
    if (cmd[0] == 'S' && cmd[1] == 'Q') {
        if (len == 3 && cmd[2] == '0') {
            uint16_t sq = (gEeprom.SQUELCH_LEVEL * 255) / 9;
            sprintf(reply,"SQ0%03u;", sq);
            CAT_Send(reply);
        }
        return;
    }

    // IF — Transceiver status (composite response)
    if (cmd[0] == 'I' && cmd[1] == 'F') {
        char freq[12];
        CAT_FormatFreq(freq, gRxVfo->pRX->Frequency);
        uint8_t kmod = CAT_ModToKenwood(gRxVfo->Modulation);
        uint8_t tx = (gCurrentFunction == FUNCTION_TRANSMIT) ? 1 : 0;
        // IF response: IF + freq(11) + step(4) + rit(6) + flags + mode + ...
        // Total: 2 + 11 + 4 + 6 + 1+1+2+1+1+1+1+1+1+2+1 + 1 = ~36 chars + NUL
        sprintf(reply,
            "IF%s"       // P1: frequency (11 digits)
            "    "       // P2: freq step (4 spaces)
            "+00000"     // P3: RIT/XIT offset
            "0"          // P4: RIT off
            "0"          // P5: XIT off
            "00"         // P6: memory channel
            "%u"         // P7: RX/TX status
            "%u"         // P8: mode
            "0"          // P9: FR/FT
            "0"          // P10: scan
            "0"          // P11: split
            "0"          // P12: tone
            "00"         // P13: tone number
            "0"          // P14: shift
            ";",
            freq, tx, kmod);
        CAT_Send(reply);
        return;
    }

    // Unknown command — send error '?;'
    CAT_Send("?;");
}

bool CAT_ProcessVCP(void)
{
    uint16_t write_ptr = VCP_RxBufPointer;

    while (VCP_ReadIndex != write_ptr) {
        uint8_t b = VCP_RxBuf[VCP_ReadIndex];
        VCP_ReadIndex = (VCP_ReadIndex + 1) % VCP_RX_BUF_SIZE;

        if (b == ';') {
            // End of command
            if (gCAT_BufLen > 0) {
                gCAT_Buf[gCAT_BufLen] = '\0';
                CAT_HandleCommand((const char *)gCAT_Buf, gCAT_BufLen);
                gCAT_BufLen = 0;
                return true;
            }
        } else if (b >= 0x20 && b < 0x7F) {
            // Printable ASCII
            if (gCAT_BufLen < CAT_BUF_SIZE - 1) {
                gCAT_Buf[gCAT_BufLen++] = b;
            } else {
                // Buffer overflow — discard
                gCAT_BufLen = 0;
            }
        }
        // Ignore CR, LF, and other control chars
    }

    return false;
}
