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

// KISS TNC over USB CDC + Bell 202 AFSK TX engine.
// See kiss.h for protocol and architecture notes.

#if defined(ENABLE_KISS_TNC) || defined(ENABLE_APRS_TX)

#include "app/kiss.h"
#include "driver/bk4819.h"
#include "driver/system.h"
#include "functions.h"
#include "radio.h"
#include "misc.h"
#include "py32f0xx.h"
#include <string.h>

// ----------------------------------------------------------------
// Bell 202 AFSK constants
// ----------------------------------------------------------------

// BK4819 REG_71 frequency values.
// Formula (same as scale_freq in bk4829.c):
//   reg71 = (freq_hz * 1353245 + 65536) >> 17
#define AFSK_REG71_MARK  ((uint16_t)(((uint32_t)1200u * 1353245u + 65536u) >> 17u))
#define AFSK_REG71_SPACE ((uint16_t)(((uint32_t)2200u * 1353245u + 65536u) >> 17u))

// TIM16 at 1200 Hz: 48 MHz / 1 / 40000 = 1200 exactly.
// PSC = 0  (timer clock = PCLK2 = 48 MHz)
// ARR = 39999
#define TIM16_ARR_VAL    39999u

// Tone generator gain (0-127).  60 ≈ -3 dBFS, suitable for linear TX path.
#define AFSK_GAIN        60u

// Flag byte and frame framing parameters
#define AFSK_FLAG        0x7Eu
#define N_PREAMBLE       25u   // leading flag bytes (~167 ms at 1200 baud)
#define N_POSTAMBLE       4u   // trailing flag bytes

// CRC-CCITT (AX.25 FCS) polynomial, bit-reflected
#define CRC_POLY         0x8408u

// ----------------------------------------------------------------
// AFSK TX state machine
// ----------------------------------------------------------------

typedef enum {
    AFSK_IDLE,
    AFSK_PREAMBLE,
    AFSK_DATA,
    AFSK_POSTAMBLE,
    AFSK_DONE          // ISR sets done flag, main loop cleans up
} afsk_phase_t;

static struct {
    uint8_t  frame[AFSK_MAX_FRAME + 2u]; // raw frame + 2 FCS bytes
    uint16_t frame_len;  // total length including FCS

    // TX state — read/written by ISR after phase transitions to non-IDLE
    volatile afsk_phase_t phase;
    uint8_t  flag_bit;   // 0-7: bit position within current flag byte
    uint16_t flag_rem;   // flag bytes remaining in current phase
    uint16_t byte_idx;   // current byte index in frame[]
    uint8_t  bit_idx;    // 0-7: bit position within current byte
    uint8_t  ones_count; // consecutive 1 bits (for bit stuffing)
    uint8_t  tone;       // 0 = mark (1200 Hz), 1 = space (2200 Hz)

    volatile bool done;  // set by ISR when TX is complete
} g_afsk;

// ----------------------------------------------------------------
// AX.25 FCS (CRC-CCITT, bit-by-bit, LSB first)
// ----------------------------------------------------------------

static uint16_t ax25_fcs(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc ^ b) & 1u)
                crc = (crc >> 1) ^ CRC_POLY;
            else
                crc >>= 1;
            b >>= 1;
        }
    }
    return ~crc;
}

// ----------------------------------------------------------------
// TIM16 peripheral control
// ----------------------------------------------------------------

static void afsk_tim16_start(void)
{
    RCC->APBENR2 |= RCC_APBENR2_TIM16EN;

    TIM16->CR1  = 0;
    TIM16->PSC  = 0;
    TIM16->ARR  = TIM16_ARR_VAL;
    TIM16->CNT  = 0;
    TIM16->SR   = 0;          // clear pending flags
    TIM16->DIER = 1u;         // UIE: update interrupt enable
    TIM16->EGR  = 1u;         // UG: reload PSC/ARR now

    NVIC_SetPriority(TIM16_IRQn, 1u);
    NVIC_EnableIRQ(TIM16_IRQn);

    TIM16->CR1  = 1u;         // CEN: start counting
}

static void afsk_tim16_stop(void)
{
    TIM16->CR1  = 0;          // CEN=0: stop
    TIM16->SR   = 0;
    NVIC_DisableIRQ(TIM16_IRQn);
    RCC->APBENR2 &= ~RCC_APBENR2_TIM16EN;
}

// ----------------------------------------------------------------
// BK4819 tone generator helpers
// ----------------------------------------------------------------

static void afsk_bk4819_start(void)
{
    // Mute TX audio path while configuring
    BK4819_EnterTxMute();

    // Enable tone1 generator with chosen gain
    BK4819_WriteRegister(BK4819_REG_70,
        BK4819_REG_70_ENABLE_TONE1 |
        ((AFSK_GAIN & 0x7Fu) << BK4819_REG_70_SHIFT_TONE1_TUNING_GAIN));

    // Configure REG_30 for tone TX (TX DSP + AF DAC + PA; no MIC ADC)
    BK4819_EnableTXLink();

    // Allow RF and PLL to settle
    SYSTEM_DelayMs(50u);

    // Write initial mark frequency
    BK4819_WriteRegister(BK4819_REG_71, AFSK_REG71_MARK);

    // Un-mute — transmission begins
    BK4819_ExitTxMute();
}

static void afsk_bk4819_stop(void)
{
    BK4819_EnterTxMute();
    BK4819_WriteRegister(BK4819_REG_70, 0u);
}

// ----------------------------------------------------------------
// AFSK TX public API
// ----------------------------------------------------------------

bool AFSK_TxFrame(const uint8_t *frame, uint16_t len)
{
    if (g_afsk.phase != AFSK_IDLE)
        return false;
    if (len == 0u || len > AFSK_MAX_FRAME)
        return false;

    // Copy frame and compute + append FCS
    memcpy(g_afsk.frame, frame, len);
    uint16_t fcs = ax25_fcs(frame, len);
    g_afsk.frame[len]     = (uint8_t)(fcs & 0xFFu);
    g_afsk.frame[len + 1] = (uint8_t)(fcs >> 8);
    g_afsk.frame_len      = len + 2u;

    // Initialise TX state
    g_afsk.flag_bit   = 0u;
    g_afsk.flag_rem   = N_PREAMBLE;
    g_afsk.byte_idx   = 0u;
    g_afsk.bit_idx    = 0u;
    g_afsk.ones_count = 0u;
    g_afsk.tone       = 0u;  // start at mark (1200 Hz)
    g_afsk.done       = false;

    // Key the transmitter
    RADIO_PrepareTX();

    // Set up BK4819 tone generator (50 ms settling inside)
    afsk_bk4819_start();

    // Set phase last — starts the ISR state machine
    g_afsk.phase = AFSK_PREAMBLE;

    // Start 1200 Hz baud timer
    afsk_tim16_start();

    return true;
}

bool AFSK_IsTxActive(void)
{
    return g_afsk.phase != AFSK_IDLE;
}

// Called from the main loop — performs post-TX cleanup that is unsafe from ISR.
void AFSK_Poll(void)
{
    if (!g_afsk.done)
        return;

    g_afsk.done = false;

    // Stop baud timer before touching BK4819
    afsk_tim16_stop();

    // Disable tone generator
    afsk_bk4819_stop();

    // Release PTT and return to receive
    FUNCTION_Select(FUNCTION_FOREGROUND);

    g_afsk.phase = AFSK_IDLE;
}

// ----------------------------------------------------------------
// TIM16 IRQ — called at 1200 Hz during AFSK TX
//
// NOTE: BK4819_WriteRegister uses bit-banged GPIO SPI.  During TX the
// main loop performs only minimal BK4819 accesses, so the collision
// risk is low.  If issues arise, add a mutex around BK4819 writes.
// ----------------------------------------------------------------

// Inline tone write to minimise ISR latency
static inline void afsk_write_tone(uint8_t space)
{
    BK4819_WriteRegister(BK4819_REG_71,
        space ? AFSK_REG71_SPACE : AFSK_REG71_MARK);
}

void TIM16_IRQHandler(void)
{
    TIM16->SR = 0u; // clear update interrupt flag

    if (g_afsk.phase == AFSK_DONE) {
        // Flag for main-loop cleanup; keep timer running until Poll() stops it
        g_afsk.done = true;
        return;
    }

    uint8_t bit;

    // ---- Preamble / Postamble: send flag bytes (0x7E) without bit stuffing ----
    if (g_afsk.phase == AFSK_PREAMBLE || g_afsk.phase == AFSK_POSTAMBLE) {
        bit = (AFSK_FLAG >> g_afsk.flag_bit) & 1u;
        if (++g_afsk.flag_bit == 8u) {
            g_afsk.flag_bit = 0u;
            if (--g_afsk.flag_rem == 0u) {
                if (g_afsk.phase == AFSK_PREAMBLE) {
                    // Transition to data phase
                    g_afsk.phase     = AFSK_DATA;
                    g_afsk.byte_idx  = 0u;
                    g_afsk.bit_idx   = 0u;
                    g_afsk.ones_count = 0u;
                } else {
                    // Postamble complete
                    g_afsk.phase = AFSK_DONE;
                    g_afsk.done  = true;
                    return;
                }
            }
        }
        // NRZI: 0 = transition, 1 = no transition
        if (bit == 0u) g_afsk.tone ^= 1u;
        afsk_write_tone(g_afsk.tone);
        return;
    }

    // ---- Data phase with bit stuffing ----

    // If 5 consecutive 1s have been sent, insert a stuffed 0 bit
    if (g_afsk.ones_count == 5u) {
        g_afsk.ones_count = 0u;
        g_afsk.tone ^= 1u;  // 0 = transition
        afsk_write_tone(g_afsk.tone);
        return;
    }

    // All frame bytes (including FCS) transmitted?
    if (g_afsk.byte_idx >= g_afsk.frame_len) {
        // Start postamble
        g_afsk.phase    = AFSK_POSTAMBLE;
        g_afsk.flag_bit = 0u;
        g_afsk.flag_rem = N_POSTAMBLE;
        // Emit first bit of first postamble flag immediately
        bit = (AFSK_FLAG >> 0u) & 1u;
        g_afsk.flag_bit = 1u;
        if (bit == 0u) g_afsk.tone ^= 1u;
        afsk_write_tone(g_afsk.tone);
        return;
    }

    // Get next data bit, LSB first
    bit = (g_afsk.frame[g_afsk.byte_idx] >> g_afsk.bit_idx) & 1u;
    if (++g_afsk.bit_idx == 8u) {
        g_afsk.bit_idx = 0u;
        g_afsk.byte_idx++;
    }

    // Update bit-stuffing counter
    if (bit)
        g_afsk.ones_count++;
    else
        g_afsk.ones_count = 0u;

    // NRZI encoding
    if (bit == 0u) g_afsk.tone ^= 1u;
    afsk_write_tone(g_afsk.tone);
}

#endif // ENABLE_KISS_TNC || ENABLE_APRS_TX

// ================================================================
// KISS TNC protocol decoder
// ================================================================

#ifdef ENABLE_KISS_TNC

#include "driver/vcp.h"

#define KISS_FEND   0xC0u
#define KISS_FESC   0xDBu
#define KISS_TFEND  0xDCu
#define KISS_TFESC  0xDDu

#define KISS_MAX_FRAME  340u

typedef enum {
    KISS_IDLE,      // waiting for FEND
    KISS_IN_CMD,    // received FEND, reading command byte
    KISS_IN_DATA,   // accumulating frame data
    KISS_ESCAPE     // received FESC, next byte is de-escaped
} kiss_state_t;

static struct {
    kiss_state_t state;
    uint8_t      frame[KISS_MAX_FRAME];
    uint16_t     len;
} g_kiss;

// VCP_ReadIndex is owned by uart.c; made non-static so KISS and CAT
// can share the same read pointer into the VCP RX ring buffer.
extern uint16_t VCP_ReadIndex;

bool KISS_IsInFrame(void)
{
    return g_kiss.state != KISS_IDLE;
}

bool KISS_ProcessVCP(void)
{
    uint16_t wp = VCP_RxBufPointer;

    while (VCP_ReadIndex != wp) {
        uint8_t b = VCP_RxBuf[VCP_ReadIndex];
        VCP_ReadIndex = (VCP_ReadIndex + 1u) % VCP_RX_BUF_SIZE;

        switch (g_kiss.state) {

            case KISS_IDLE:
                if (b == KISS_FEND) {
                    g_kiss.state = KISS_IN_CMD;
                    g_kiss.len   = 0u;
                }
                break;

            case KISS_IN_CMD:
                if (b == KISS_FEND) {
                    // Back-to-back FENDs — stay in command-read state,
                    // frame buffer already cleared.
                    break;
                }
                // Low nibble: 0x0 = data frame.  High nibble: port (ignored).
                if ((b & 0x0Fu) == 0x00u) {
                    g_kiss.state = KISS_IN_DATA;
                } else {
                    // Non-data commands (TXDELAY, persistence, etc.) ignored.
                    g_kiss.state = KISS_IDLE;
                }
                break;

            case KISS_IN_DATA:
                if (b == KISS_FEND) {
                    // End of frame
                    if (g_kiss.len > 0u && !AFSK_IsTxActive()) {
                        AFSK_TxFrame(g_kiss.frame, g_kiss.len);
                    }
                    g_kiss.state = KISS_IDLE;
                    g_kiss.len   = 0u;
                    return true;
                } else if (b == KISS_FESC) {
                    g_kiss.state = KISS_ESCAPE;
                } else {
                    if (g_kiss.len < KISS_MAX_FRAME)
                        g_kiss.frame[g_kiss.len++] = b;
                }
                break;

            case KISS_ESCAPE:
                if (b == KISS_TFEND)       b = KISS_FEND;
                else if (b == KISS_TFESC)  b = KISS_FESC;
                if (g_kiss.len < KISS_MAX_FRAME)
                    g_kiss.frame[g_kiss.len++] = b;
                g_kiss.state = KISS_IN_DATA;
                break;
        }
    }

    return false;
}

#endif // ENABLE_KISS_TNC
