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

## Remotes

- `origin` — dikei100 fork (push target)
- `upstream` — armel/uv-k1-k5v3-firmware-custom (original)
