# AGENTS.md

Firmware for the PLCJS Ethernet 8AIV module (8 isolated voltage inputs, one
`ADS8688` behind a `CA-IS3741LB` isolator on SPI1, STM32F407VGT6, KSZ8863
switch, Modbus TCP). This file is the orientation map for agents; user-facing
documentation lives in `README.md` / `README_EN.md`.

Schematic: `DOC/SCH_8AIV.pdf`. The board is the simple sibling of 8UAI: same
ADS8688 wiring (SPI1 PA5/PA6/PB5, CS PA9, same input-to-channel table), but no
key chain, no shunts and no protection ISR. It is the bring-up platform for the
ADS8688 path.

Derived from the 8UAI firmware (commit `a29219b` there), itself derived from
8AIC fw 1.6: the network, Modbus server, PDP discovery, ARP probe, KSZ8863, LED,
button, `fw_header`, nanomodbus, `flash_ram`, `ads8688` driver and the CubeMX
tree came over unchanged apart from renames; the analog path (`adc_scan`, `ai`)
is new and much simpler.

## Build (CMake)

Toolchain: STM32 Arm Clang (`starm-clang`) from STM32CubeCLT, Ninja. Toolchain
file `cmake/starm-clang.cmake`, presets in `CMakePresets.json`.

```
cmake --preset Debug
cmake --build --preset Debug
```

Release: substitute `Release`. Output:
`build/<preset>/PLCJS_ETH_MODULE_8AIV_D4MG_STM32F407VGT6.{elf,hex,bin,map}`.

Host tests (conversion maths) need a native gcc, on this workstation Qt's MinGW
(`C:\Qt\Tools\mingw1310_64\bin`):

```
cmake -S tests -B build/host-tests -G Ninja -DCMAKE_C_COMPILER=gcc
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

"Verified" without hardware means: both presets build warning-free and the host
tests pass. The scan engine (`adc_scan.c`) is register-level code and has no
host test; it is only verified on the board (SPI timing, ISR load, range
read-back).

## Repository layout

| Path | Owner | Notes |
|---|---|---|
| `Application/` | hand-written | All real logic. |
| `Core/`, `Drivers/`, `Middlewares/`, `LWIP/`, `cmake/stm32cubemx/` | STM32CubeMX | Regenerated from the `.ioc`. |
| `startup_stm32f407xx.s`, `STM32F407XX_FLASH.ld` | hand-edited | See *Linker*. |
| `tests/` | hand-written | Host tests (`test_ai.c`). |
| `DOC/` | assets | Schematic. |

**CubeMX regeneration hazard.** Regeneration overwrites `Core/`, `LWIP/` etc.
Hand edits outside `USER CODE` guards: `LWIP/Target/ethernetif.c`,
`lwipopts.h`, the pin block of `Core/Inc/main.h` / `Core/Src/gpio.c` (analog
pins removed, they belong to `board/`), `SystemClock_Config` (PLLN 320 ->
SYSCLK 160 MHz, so SPI1 = PCLK2/8 = 10.0 MHz and TIM2 = 80 MHz),
`configTOTAL_HEAP_SIZE` 20480 (also in the `.ioc`). `board_early_pins()` sits in
`USER CODE BEGIN 1` of `main()`.

## Module map (`Application/`)

| Module | Responsibility |
|---|---|
| `app/` | Orchestrator: boot order, factory reset, network, housekeeping loop, AI and LED tasks. Start here. |
| `board/board_pins.*` | **The only file with MCU pins**: SPI1, CS PA9, channel LEDs, `BOARD_ADC_MAP`. Checked against the schematic. |
| `board/board_io.*` | `spi_init()` and `board_adc_frame()` (one polled 32-bit ADS8688 frame). |
| `ads8688/` | Command/register codes (SBAS582C), blocking start-up init + range read-back. |
| `adc_scan/` | **Scan engine.** TIM2 slot clock, one frame per slot in the TIM2 update ISR, round-robin over the 8 channels, mains-synchronous windows, range writes and rotating register read-back. Read `adc_scan.h` first. |
| `ai/ai_math.*` | Code -> pin voltage -> node -> terminal voltage, calibration, status, scaling (host tested). |
| `ai/ai.*` | Channel layer: ADC bring-up and retry, window -> value, EMA filter, ADC_FAIL, LEDs. |
| `ai/ai_types.h` | Modes, ranges, status bits, `ai_ch_cfg_t` (stored in Flash). |
| `wdg/` | `wdg_kick()` for the ~8 s IWDG. |
| `flash_ram/` | Sector erase/program from RAM inside a critical section, scan paused around it. |
| `settings/` | Flash-backed settings (sector 10), CRC32. |
| `modbus/modbus_app.c` | Register map. **Documented in the header comment of `modbus_app.h`.** |
| others | `discovery/`, `addr_probe/`, `net_id/`, `ksz8863/`, `led/`, `button/`, `temp/`, `fw_header/`, `third_party/nanomodbus/`: shared with the family. |

## Analog path in one page

- **Slot:** TIM2 update every 3125 ticks (80 MHz) = 39.0625 µs. The ISR
  (priority 1, above FreeRTOS, no RTOS calls) runs one `board_adc_frame()`:
  command `MAN_Ch` of the NEXT channel, data = conversion of the channel selected
  in the PREVIOUS frame (ADS8688 pipeline). Frame k therefore belongs to channel
  `k & 7`. The ISR costs about 4 µs of 39 µs.
- **Window:** 2560 slots (100.000 ms = 320 samples per input, whole periods of
  50 AND 60 Hz) or 512 slots (20.000 ms, 50 Hz only). Per window and input: sum,
  sum of squares (noise), min, max, rail hits. Published into one of two buffers
  with a sequence number (`adc_scan_take()`).
- **Boundary:** `RANGE` state (one pending range write + read-back per slot, then
  one rotating read-back of a range register that heals a mismatch and counts
  it), one idle slot, two `PRIME` slots (`MAN_Ch(0)`, data discarded). So 2564
  slots per 100 ms window; no window ever contains a slot of another state.
- **Range changes:** the thread calls `adc_scan_set_range()`; the ISR applies it
  at the next boundary and publishes the range in force per window, so `ai.c`
  settles a channel until a window with the expected range arrives.
- **Flash erase:** `flash_ram_write_sector()` calls `adc_scan_pause()` (TIM2 and
  its IRQ off) and `adc_scan_resume()` (window restarted). The scan code lives
  in Flash and is never executed during an erase.
- **ADC_FAIL:** bring-up failure (retried every second) or a range read-back
  mismatch (held for 50 windows).

## Invariants

### Single sources of truth
- **Module identity** - `Application/fw_header/fw_header.h`:
  `FW_PRODUCT_ID = 0x504C0805`, `FW_HW_REVISION = 0x0101`,
  `FW_VERSION_VALUE = 0x0100`.
- **Module ID** - `MODULE_ID_08AIV = 0x08A5` (IR125).
- **Register map** - header comment of `modbus_app.h` + `MB_*` constants;
  ModbusTool `build8AIV()` mirrors it.
- **Pins / wiring tables** - `board_pins.h` only.

### Version policy - bump the minor on every change
**Mandatory.** Every change to firmware behaviour ships with `FW_VERSION_VALUE`
incremented by one minor (`0x0100` -> `0x0101`). Major only with a
`FW_HW_REVISION` major change. Documentation-only commits need no bump.
Bump checklist: `fw_header.h`, the version rows in `README.md` and
`README_EN.md` (image table).

Changelog: every version bump also adds an entry (`DATE -- reason`, appended at
the end) to the shared `CHANGELOG.md` / `CHANGELOG.txt` in the bootloader repo
(`BOOTLOADER_PLCJS_ETH_MODULE_STM32F407VGT6`), one file for all repos.

### Persistence
- `settings_t` (contains `ai_ch_cfg_t x8`, `ai_model_t`, `ai_diag_t`) layout is
  frozen and has no padding; any change needs `SETTINGS_VERSION` bumped
  (currently 1, magic `0x08A54A57`). A mismatch reverts units to defaults with
  CONFIG_ERR; a blank sector gives defaults without CONFIG_ERR.
- Settings live in sector 10 @ `0x080C0000`. Sector 11 is unused here.
- Modbus writes are staged in `settings_t` and applied via `ai_set_config` /
  `ai_set_common`; `modbus_app_sanitize_staged()` runs before every save so a
  half-written float never reaches Flash.

### Threading
- AI task (`osPriorityHigh`, 2 ms) owns the channel layer; Modbus reads go
  through `ai_get()` / `ai_get_stats()` (critical section).
- The scan ISR touches only `adc_scan.c` state; thread <-> ISR traffic goes
  through the atomic range mask, `s_win_next` and the double-buffered window.
- LwIP calls in the tcpip thread; deferred Flash writes/resets in the
  housekeeping loop (`*_take_pending_*()`), as in the family.

## Gotchas
- **SWD flashing alone leaves the board in the bootloader** (metadata CRC check
  since bootloader 1.8); use the bootloader's
  `scripts/service_flash.ps1 -AppBin <app.bin>`.
- ADS8688 register read data is the high byte of the second received halfword
  (`ADS_REG_DATA`). A wrong guess here shows up as ADC_FAIL at boot and as
  IR133 counting up: check this first on a new board.
- `/RST/PD` is pulled up and `REFSEL` / `DAISY` are tied to GND on this board:
  reset is the RST command, the reference is the internal 4.096 V one.
- The 2 kΩ resistor from `AIN_nGND` to ground is not part of the model; its
  effect (and the real ADC input resistance) must be measured on the board and
  absorbed by the per-channel calibration or the service constants.
- Shared gotchas (SET_NET probe, IR129/IR130 save bookkeeping, 16-byte name,
  4-slot Modbus server with RST close, KSZ8863 warm-reset policy, HR118 magics)
  are identical to 8AIC / 8UAI; see those repos' AGENTS.md.

## Linker / memory contract with the bootloader
`FLASH` origin `0x08040000` (256 K), `RAM` length `0x1FFF0` (boot-request cell at
`0x2001FFF0`), `.fw_header` at offset `0x200`; `fw_header_t` byte-identical to the
bootloader's `app_validate.h`. `.RamFunc` is placed in `.data` (copied to RAM at
start-up) and holds only the Flash erase/program loop. OTA acceptance:
`product_id` exact + `hw_revision` major.

## Open points
- Hardware acceptance not run: SPI timing through the isolator, ISR load, real
  gain of the input network, accuracy and noise (IR380..395 give the code noise).
- ADS8688 bias V_B per range (2.25 / 2.00 / 2.50 / 2.50 V) is taken from the TZ
  of the 8UAI work; the datasheet gives no explicit value for 0..1.25·VREF.

## Multi-repo workspace

| Repo | Role |
|---|---|
| `PLCJS_ETH_MODULE_8AIV_D4MG_...` | This module - `0x504C0805` / IR125 `0x08A5`. |
| `PLCJS_ETH_MODULE_8UAI_D4MG_...` | 8 universal inputs, `0x504C0807` / `0x08A7`; the full ADS8688 sequencer with protection ISR. |
| `PLCJS_ETH_MODULE_8AIC_D4MG_...` | 8x 4-20 mA, `0x504C0804` / `0x08AC`. |
| `BOOTLOADER_PLCJS_ETH_MODULE_STM32F407VGT6` | Shared bootloader; `scripts/variants.csv` row `8aiv,0x504C0805,0x010101`; shared CHANGELOG. |
| `PLCJS_Module_ModbusTool` | Qt6 client; `build8AIV()` in `src/maps/ModuleMaps.cpp`. |

`Application/` subsystems are copy-pasted between variants, not shared: a fix
here is not a fix elsewhere. Lockstep contracts: PDP wire format, `fw_header_t`
/ flash map / boot-request cell, `product_id` (+ `variants.csv`), register map
(+ `ModuleMaps.cpp`).

## Maintaining this file
Update it in the same commit as the change it describes: invariants, module
map, build/linker contract, register-map changes, version bumps, cross-repo
contracts.
