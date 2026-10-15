# AD9106 GUI

Desktop GUI for the STM32 firmware in `../Software`, which exposes a text CLI over the
USB virtual COM port to configure the AD9106BCPZ. Every CLI command is available in the GUI.

## Start

**Windows:** double-click `start_gui.bat`. The first run creates a `.venv` folder and installs
PySide6 and pyserial (needs Python 3.9 or newer); later runs start straight away.

**Manually:**

```
pip install -r requirements.txt
python ad9106_gui.py              # choose the port in the toolbar
python ad9106_gui.py --port COM7  # connect on start
python ad9106_gui.py --sim        # built-in simulator, no hardware needed
```

The STM32 virtual COM port (VID 0483, PID 5740) is marked with ★ and selected automatically.

## What is where

| CLI command | GUI |
|---|---|
| `output <1..4>` | Generator tab, DAC 1–4 buttons |
| `wave <sine\|triangle\|saw\|dc>` | Waveform buttons |
| `freq`, `phase`, `gain`, `offset` | Sliders and spin boxes |
| `run <0\|1>` | Run/stop button top right (F9) |
| `status` | Read status (F5); also runs automatically on connect |
| `update` | Update (RAMUPDATE) |
| `reset` | Reset AD9106, optionally re-applies the settings afterwards |
| `help` | Firmware help, output appears in the terminal |
| `read <reg>` / `write <reg> <value>` | Registers tab: full register map, read all, bit editor, register scripts |
| anything | Terminal at the bottom (history with arrow keys) |

Other features:

- **Apply immediately** sends a change 250 ms after you stop editing. Turn it off to collect
  changes and send them with *Apply*. Unconfirmed parameters are marked with an orange dot.
- **What the firmware will write** shows the actual output frequency (DDS tuning word or
  sawtooth step), gain/offset/phase codes and the exact register writes of
  `AD9106_ApplyConfig`, with warnings for clipping, out-of-range or coarse frequencies.
- **Register scripts** write a list of `address value` lines (names such as `WAV4_3CONFIG`
  also work). *Export…* saves a read-back as a script you can load again.
- **Presets** (File menu) save and load the generator settings as JSON.
- The **simulator** reproduces the firmware's responses byte for byte, including its
  register writes, so the GUI can be tried without a board.

## Firmware notes found while building this

1. **Floats print as empty strings.** The build links with `--specs=nano.specs` without
   `-u _printf_float`, so `status`, `OK phase` and `OK gain` print nothing for phase and gain.
   The GUI works around it by using the value it sent. To fix, add `-u _printf_float` to
   `CMAKE_EXE_LINKER_FLAGS` in `cmake/gcc-arm-none-eabi.cmake`.
2. **Nothing is written to the AD9106 at boot.** `status` reports sine 1 kHz, but
   `AD9106_ApplyConfig` only runs on the first parameter command. Use *Write all* after
   power-up.
3. **Triangle and saw frequencies are coarse.** `SAW_STEP` only has 63 values and the firmware
   truncates instead of rounding: with a 156.25 MHz clock a triangle runs from 75.7 Hz to
   4.77 kHz, and 3 kHz becomes 4.77 kHz. The DDS sine has 9.31 Hz resolution, also truncated.
4. **Worth checking:** `CLI_Printf` passes a buffer on its stack to `CDC_Transmit_FS`, which
   transmits asynchronously. Once the function returns, that buffer can be overwritten before
   USB has sent it. A `static` buffer, reused only after the previous transfer has finished
   (`hcdc->TxState == 0`), fixes it.
