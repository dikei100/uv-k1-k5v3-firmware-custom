# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Custom firmware for Quansheng UV-K1 and UV-K5 V3 handheld radios. Fork of F4HWN firmware (armel/uv-k1-k5v3-firmware-custom), based on the DualTachyon open-source firmware.

- **MCU:** PY32F071xB (ARM Cortex-M0+, 48MHz)
- **Memory:** 118KB usable Flash (0x08002800–0x08020000), 16KB RAM
- **Radio IC:** BK4819/BK4829 (same register interface, bk4829.c is the active driver)
- **Display:** ST7565 LCD via SPI
- **EEPROM:** PY25Q16 SPI flash (2MB)

## Build Commands

Docker-based build (primary method):
```bash
./compile-with-docker.sh Custom                    # Build Custom preset
./compile-with-docker.sh Bandscope                 # Build Bandscope preset
./compile-with-docker.sh All                       # Build all presets
./compile-with-docker.sh Custom -DENABLE_MOD_DIG=ON  # With custom CMake flag
```

Output: `build/<Preset>/f4hwn.<preset>.{elf,bin,hex}`

Available presets: Custom, Bandscope, Broadcast, Basic, RescueOps, Game, Fusion, All.

Toolchain: `arm-none-eabi-gcc` 13.3.rel1, CMake 3.22+, Ninja generator.

## Architecture

**Layer structure (bottom-up):**

1. **Drivers** (`App/driver/`) — HAL for PY32F071 peripherals and radio chips
   - `bk4829.c` — Active BK4819 radio IC driver (register read/write, RX/TX, filters, FSK)
   - `bk4819.c` — Exists but NOT linked; `bk4829.c` is compiled per `App/CMakeLists.txt`
   - `st7565.c` — LCD, `py25q16.c` — EEPROM, `keyboard.c` — keypad scanning

2. **Application** (`App/`) — Radio logic and state management
   - `radio.c` — Frequency management, modulation setup, TX/RX configuration
   - `settings.c` — EEPROM persistence for all radio settings
   - `functions.c` — State machine transitions (RX, TX, foreground, power save)
   - `audio.c` — Audio path control

3. **App modules** (`App/app/`) — Feature-specific logic
   - `app.c` — Main update loop with 10ms/500ms time slices
   - `menu.c` — 69+ menu items
   - `spectrum.c` — Spectrum analyzer
   - `action.c` — User input dispatch

4. **UI** (`App/ui/`) — Display rendering
   - `main.c` — VFO/channel display
   - `status.c` — Status bar (battery, signal, timers)

5. **Core** (`Core/`) — MCU startup, interrupt handlers, linker script

## Feature Flag System

Features are toggled via CMake variables in `CMakePresets.json` and registered in `App/CMakeLists.txt`:

```cmake
enable_feature(ENABLE_FEATURE_NAME          # Adds compile definition
    optional_source.c                        # Only compiled when enabled
)
```

To add a new feature flag:
1. Add `"ENABLE_MY_FEATURE": false` to `CMakePresets.json` default preset
2. Add `enable_feature(ENABLE_MY_FEATURE)` in `App/CMakeLists.txt`
3. Guard code with `#ifdef ENABLE_MY_FEATURE`

## Naming Conventions

- Functions: `COMPONENT_Action()` — e.g., `BK4819_SetAGC()`, `RADIO_SetModulation()`
- Globals: `gVariableName` — e.g., `gRxVfo`, `gEnableSpeaker`
- Statics: local to file, no prefix convention
- Enums: `UPPER_CASE` values, typedef'd

## Key Patterns

**Modulation modes** (`ModulationMode_t` in `radio.h`): FM, AM, USB, BYP, RAW, DIG. Adding a new mode: add to enum, add string to `gModulationStr[]`, handle in `RADIO_SetModulation()` with early-return pattern (see BYP/RAW/DIG blocks).

**BK4819 register access**: `BK4819_ReadRegister(reg)` / `BK4819_WriteRegister(reg, val)` for direct access. `BK4819_SetRegValue(regSpec, val)` for named register fields via `RegisterSpec` structs.

**TX flow**: `RADIO_PrepareTX()` → `FUNCTION_Select(FUNCTION_TRANSMIT)` → `RADIO_SetTxParameters()`. TX is gated by `ENABLE_TX_WHEN_AM` for non-FM modes.

**Settings persistence**: 4-bit modulation stored in upper nibble of EEPROM byte. Unknown values fall back to FM on load (`settings.c`).

## Digital Mode (ENABLE_MOD_DIG)

TNC passthrough mode for M17 4-FSK via external Mobilinkd TNC4. The BK4819 is configured as a flat baseband I/O interface — no on-device codec.

**Key registers (bk4829.c):**
- REG_2B: bits 10,9,8 (RX filter bypass), bits 2,1,0 (TX filter bypass)
- REG_7E: bit 15 (TX DC filter), bits [5:3] (sub-audio filters) — only clear [5:3], NOT [2:0]
- REG_7D: 0xE940 (flat MIC sensitivity during TX, saved/restored)
- REG_43: digital bandwidth values — 0x47A8 (wide), 0x7800 (narrow) via `BK4819_FILTER_BW_DIGITAL_WIDE/NARROW`
- REG_30: MIC ADC (bit 2) for DC bias stability — use read-modify-write, NOT raw write

**TX flow hazard:** `BK4819_PrepareTransmit()` calls `ExitBypass()` which clears REG_2B filter bypasses. `DigitalTxSetup()` must re-apply TX filter bits after PrepareTransmit. `DigitalTxCleanup()` must call `BK4819_EnterDigital()` to re-apply RX bypasses.

**Reference implementations:** [mobilinkd/uv-k5-firmware-custom](https://github.com/mobilinkd/uv-k5-firmware-custom) (digital-modulation branch), [dikei100/fagci-digital](https://github.com/dikei100/fagci-digital).

**Hardware mod required:** C77→1µF, R69→1µF, C31+20µF, C71→0Ω, plus MIC/AF/PTT taps.

## KISS TNC (ENABLE_KISS_TNC)

KISS TNC over USB CDC — allows packet radio software (direwolf, YAAC, APRSdroid) to send AX.25 frames for AFSK transmission.

**Protocol auto-detection:** First byte 0xC0 (FEND) → KISS decoder; A-Z → CAT; else binary UART.

**AFSK TX engine** (`App/app/kiss.c`): Uses TIM16 at 1200 Hz to clock Bell 202 AFSK tones via BK4819 tone generator (REG_70/REG_71). Mark = 1200 Hz, Space = 2200 Hz. NRZI encoding + AX.25 bit stuffing done in TIM16 ISR. FCS (CRC-CCITT) computed and appended to frame.

**TIM16 setup:** PSC=0, ARR=39999 → 48 MHz / 40000 = 1200 Hz. Enabled via `RCC->APBENR2 |= RCC_APBENR2_TIM16EN`. NVIC priority 1.

**Note:** BK4819_WriteRegister is called from TIM16_IRQHandler (ISR context). Safe during TX because main loop doesn't access BK4819 for RX operations while in FUNCTION_TRANSMIT state.

**Cleanup:** `AFSK_Poll()` called from main loop checks `g_afsk.done` flag set by ISR, then stops timer, disables tone generator, and calls `FUNCTION_Select(FUNCTION_FOREGROUND)`.

## APRS TX Beacon (ENABLE_APRS_TX)

On-device APRS position beacon without external TNC.

**AX.25 frame:** Destination `APK1UV-0`, digipeater `WIDE1-1`. Info field: `!DDMM.MMN/DDDMM.MMEsymbol[comment]`.

**Settings** stored at PY25Q16 physical address `0x00B000` (free sector, not in eeprom_compat.c mapping). Struct: callsign (6 chars), SSID, lat (8 chars, DDMM.MMN), lon (9 chars, DDDMM.MME), symbol table/code, comment (43 chars), beacon interval.

**Auto-beacon:** `APRS_Tick500ms()` called from `APP_TimeSlice500ms()`. Intervals: off / 1 / 5 / 10 / 30 min.

**Manual TX:** `ACTION_OPT_APRS_BEACON` action assignable to F1/F2/M keys via side-function menu ("APRS BEACON").

**Initialization:** `APRS_Init()` called from `main.c` after `SETTINGS_LoadCalibration()`.

## CAT Control (ENABLE_CAT_CONTROL)

Kenwood TS-480 emulation over USB CDC (`App/app/cat.c`). Commands: FA, FB, MD, TX, RX, SM, IF, ID, PS, AG, SQ. Auto-detected by A-Z first byte.

**VCP_ReadIndex:** Non-static global in `app/uart.c` (line ~187), shared via `extern` by both `cat.c` and `kiss.c` so both consume from the same VCP RX ring buffer position.

## Remotes

- `origin` — dikei100 fork (push target)
- `upstream` — armel/uv-k1-k5v3-firmware-custom (original)
