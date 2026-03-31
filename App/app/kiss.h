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
//
// The AFSK engine is shared between KISS TNC (receives AX.25 frames over USB)
// and APRS TX beacon (builds AX.25 frames on-device).  Both compile-in kiss.c.
//
// Bell 202 AFSK:
//   - 1200 baud, 1200 Hz mark (bit 1), 2200 Hz space (bit 0)
//   - NRZI encoding: 0 = tone transition, 1 = no transition
//   - AX.25 bit stuffing: insert 0 after 5 consecutive 1s in data
//   - TIM16 timer interrupt at 1200 Hz drives tone switching via BK4819 REG_71
//
// KISS protocol (per KA9Q spec):
//   - FEND (0xC0) as frame delimiter
//   - FESC (0xDB) escape: TFEND (0xDC) → 0xC0, TFESC (0xDD) → 0xDB
//   - Command byte after FEND: low nibble 0 = data frame
//   - Frame data = raw AX.25 frame WITHOUT FCS; FCS is computed here

#ifndef APP_KISS_H
#define APP_KISS_H

#include <stdint.h>
#include <stdbool.h>

// ----------------------------------------------------------------
// AFSK TX engine  (compiled when ENABLE_KISS_TNC or ENABLE_APRS_TX)
// ----------------------------------------------------------------

// Maximum AX.25 frame length (excluding FCS, which is appended internally).
#define AFSK_MAX_FRAME  340u

// Transmit a raw AX.25 frame as Bell 202 AFSK.
// frame: address+control+pid+info bytes, WITHOUT FCS
// len:   number of bytes in frame
// FCS is computed and appended automatically.
// Returns false if another TX is already in progress or len is out of range.
bool AFSK_TxFrame(const uint8_t *frame, uint16_t len);

// Returns true while AFSK transmission is active.
bool AFSK_IsTxActive(void);

// Must be called from the main loop — handles TX-complete cleanup.
// (Releases PTT via FUNCTION_Select; cannot be done from ISR context.)
void AFSK_Poll(void);

// TIM16 update interrupt handler — defined in kiss.c, called by ISR vector.
void TIM16_IRQHandler(void);

// ----------------------------------------------------------------
// KISS TNC protocol decoder  (compiled when ENABLE_KISS_TNC)
// ----------------------------------------------------------------
#ifdef ENABLE_KISS_TNC

// Returns true if byte is FEND (0xC0), used for protocol auto-detection.
static inline bool KISS_IsFEND(uint8_t b) { return b == 0xC0u; }

// Returns true if the KISS state machine is in the middle of receiving a frame.
// Used by app.c to keep routing VCP bytes to KISS even after FEND is consumed.
bool KISS_IsInFrame(void);

// Process bytes from the VCP RX buffer.
// Decodes KISS frames; when a complete data frame arrives, calls AFSK_TxFrame.
// Returns true if a TX was started.
bool KISS_ProcessVCP(void);

#endif // ENABLE_KISS_TNC

#endif // APP_KISS_H
