"""
AD9106 model shared by the GUI and the simulator.

Everything in here mirrors the firmware in Software/Core/Src/ad9106.c and
cli.c *exactly* (including its 32-bit integer arithmetic and clamping), so
the GUI can show which register values the STM32 will actually write and
which output frequency results from a requested value.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, asdict

DEFAULT_FCLK_HZ = 156_250_000          # g_dac_clock_hz in ad9106.c
WAVES = ("sine", "triangle", "saw", "dc")
CLI_COMMANDS = ("help", "status", "read", "write", "update", "run", "reset",
                "output", "wave", "freq", "phase", "gain", "offset")

# --------------------------------------------------------------------------
# Register map (AD9106 datasheet, SPI register map)
# --------------------------------------------------------------------------
REGISTERS: list[tuple[int, str, str]] = [
    (0x0000, "SPICONFIG", "SPI configuration, soft reset, power-down"),
    (0x0001, "POWERCONFIG", "Power / sleep control of DACs and clock receiver"),
    (0x0002, "CLOCKCONFIG", "Clock configuration and DAC clock phase"),
    (0x0003, "REFADJ", "Reference (BGDR) adjust"),
    (0x0004, "DAC4AGAIN", "DAC4 analog gain"),
    (0x0005, "DAC3AGAIN", "DAC3 analog gain"),
    (0x0006, "DAC2AGAIN", "DAC2 analog gain"),
    (0x0007, "DAC1AGAIN", "DAC1 analog gain"),
    (0x0008, "DACxRANGE", "DAC analog gain range"),
    (0x0009, "DAC4RSET", "DAC4 internal RSET"),
    (0x000A, "DAC3RSET", "DAC3 internal RSET"),
    (0x000B, "DAC2RSET", "DAC2 internal RSET"),
    (0x000C, "DAC1RSET", "DAC1 internal RSET"),
    (0x000D, "CALCONFIG", "Calibration configuration"),
    (0x000E, "COMPOFFSET", "Comparator offset calibration"),
    (0x001D, "RAMUPDATE", "Update shadow registers (write 1)"),
    (0x001E, "PAT_STATUS", "Pattern status: RUN, memory access"),
    (0x001F, "PAT_TYPE", "Pattern type (continuous / repeat)"),
    (0x0020, "PATTERN_DLY", "Delay before pattern start"),
    (0x0022, "DAC4DOF", "DAC4 digital offset"),
    (0x0023, "DAC3DOF", "DAC3 digital offset"),
    (0x0024, "DAC2DOF", "DAC2 digital offset"),
    (0x0025, "DAC1DOF", "DAC1 digital offset"),
    (0x0026, "WAV4_3CONFIG", "Waveform source select DAC4/DAC3"),
    (0x0027, "WAV2_1CONFIG", "Waveform source select DAC2/DAC1"),
    (0x0028, "PAT_TIMEBASE", "Pattern time base"),
    (0x0029, "PAT_PERIOD", "Pattern period"),
    (0x002A, "DAC4_3PATx", "Pattern repeat count DAC4/DAC3"),
    (0x002B, "DAC2_1PATx", "Pattern repeat count DAC2/DAC1"),
    (0x002C, "DOUT_START_DLY", "DOUT start delay"),
    (0x002D, "DOUT_CONFIG", "DOUT configuration"),
    (0x002E, "DAC4_CST", "DAC4 constant value"),
    (0x002F, "DAC3_CST", "DAC3 constant value"),
    (0x0030, "DAC2_CST", "DAC2 constant value"),
    (0x0031, "DAC1_CST", "DAC1 constant value"),
    (0x0032, "DAC4_DGAIN", "DAC4 digital gain"),
    (0x0033, "DAC3_DGAIN", "DAC3 digital gain"),
    (0x0034, "DAC2_DGAIN", "DAC2 digital gain"),
    (0x0035, "DAC1_DGAIN", "DAC1 digital gain"),
    (0x0036, "SAW4_3CONFIG", "Sawtooth type / step DAC4/DAC3"),
    (0x0037, "SAW2_1CONFIG", "Sawtooth type / step DAC2/DAC1"),
    (0x003E, "DDS_TW32", "DDS tuning word [23:8]"),
    (0x003F, "DDS_TW1", "DDS tuning word [7:0] (bits 15:8)"),
    (0x0040, "DDS4_PW", "DDS4 phase offset"),
    (0x0041, "DDS3_PW", "DDS3 phase offset"),
    (0x0042, "DDS2_PW", "DDS2 phase offset"),
    (0x0043, "DDS1_PW", "DDS1 phase offset"),
    (0x0044, "TRIG_TW_SEL", "Trigger delay / tuning word select"),
    (0x0045, "DDSx_CONFIG", "DDS configuration (MSB, cosine, ...)"),
    (0x0047, "TW_RAM_CONFIG", "Tuning word from RAM configuration"),
    (0x0050, "START_DLY4", "DAC4 start delay"),
    (0x0051, "START_ADDR4", "DAC4 SRAM start address"),
    (0x0052, "STOP_ADDR4", "DAC4 SRAM stop address"),
    (0x0053, "DDS_CYC4", "DAC4 DDS cycles"),
    (0x0054, "START_DLY3", "DAC3 start delay"),
    (0x0055, "START_ADDR3", "DAC3 SRAM start address"),
    (0x0056, "STOP_ADDR3", "DAC3 SRAM stop address"),
    (0x0057, "DDS_CYC3", "DAC3 DDS cycles"),
    (0x0058, "START_DLY2", "DAC2 start delay"),
    (0x0059, "START_ADDR2", "DAC2 SRAM start address"),
    (0x005A, "STOP_ADDR2", "DAC2 SRAM stop address"),
    (0x005B, "DDS_CYC2", "DAC2 DDS cycles"),
    (0x005C, "START_DLY1", "DAC1 start delay"),
    (0x005D, "START_ADDR1", "DAC1 SRAM start address"),
    (0x005E, "STOP_ADDR1", "DAC1 SRAM stop address"),
    (0x005F, "DDS_CYC1", "DAC1 DDS cycles"),
    (0x0060, "CFG_ERROR", "Configuration error flags"),
]
REG_BY_ADDR = {a: (n, d) for a, n, d in REGISTERS}
REG_BY_NAME = {n.upper(): a for a, n, d in REGISTERS}

R_RAMUPDATE, R_PAT_STATUS, R_PAT_TYPE, R_PATTERN_DLY = 0x1D, 0x1E, 0x1F, 0x20
R_PAT_TIMEBASE, R_PAT_PERIOD = 0x28, 0x29
R_WAV43, R_WAV21, R_SAW43, R_SAW21 = 0x26, 0x27, 0x36, 0x37
R_TW32, R_TW1 = 0x3E, 0x3F
DOF_REG = {1: 0x25, 2: 0x24, 3: 0x23, 4: 0x22}
CST_REG = {1: 0x31, 2: 0x30, 3: 0x2F, 4: 0x2E}
DGAIN_REG = {1: 0x35, 2: 0x34, 3: 0x33, 4: 0x32}
PW_REG = {1: 0x43, 2: 0x42, 3: 0x41, 4: 0x40}

PRESTORE_NAMES = {0: "constant", 1: "sawtooth", 2: "pseudo-random", 3: "DDS"}
WAVESEL_NAMES = {0: "RAM", 1: "prestored", 2: "prestored + start delay",
                 3: "prestored modulated by RAM"}
SAWTYPE_NAMES = {0: "ramp up", 1: "ramp down", 2: "triangle", 3: "no wave (zero)"}


def reg_label(addr: int) -> str:
    return REG_BY_ADDR.get(addr, (f"REG_{addr:04X}", ""))[0]


def parse_int(text: str) -> int:
    """Parse like C strtoul(..., 0): 0x.. hex, leading 0 octal, else decimal."""
    t = text.strip().lower().replace("_", "")
    if t.startswith(("0x", "-0x")):
        return int(t, 16)
    if t.startswith("0b"):
        return int(t, 2)
    if len(t) > 1 and t.startswith("0") and t.isdigit():
        return int(t, 8)
    return int(t, 10)


# --------------------------------------------------------------------------
# Firmware arithmetic (mirrors ad9106.c)
# --------------------------------------------------------------------------
def _u32(x: int) -> int:
    return x & 0xFFFFFFFF


def _s12(raw: int) -> int:
    raw &= 0xFFF
    return raw - 0x1000 if raw & 0x800 else raw


def dds_tuning(freq_hz: int, fclk: int = DEFAULT_FCLK_HZ):
    """AD9106_SetFrequencyHz_DDS → (tw24, TW32 reg, TW1 reg, f_actual)."""
    f = freq_hz if freq_hz else 1
    tw = _u32((f << 24) // fclk)
    tw_msb = (tw >> 8) & 0xFFFF
    tw_lsb = (tw & 0xFF) << 8
    tw24 = ((tw_msb << 8) | (tw_lsb >> 8)) & 0xFFFFFF
    return tw24, tw_msb, tw_lsb, tw24 * fclk / 2 ** 24


def saw_setup(freq_hz: int, wave: str, fclk: int = DEFAULT_FCLK_HZ):
    """AD9106_SetSawFrequency → (step, saw_type, f_actual)."""
    f = freq_hz if freq_hz else 1
    n, saw_type = (2, 2) if wave == "triangle" else (1, 0)
    denom = _u32(f * n * 16384)            # 32-bit multiply on the STM32
    step = fclk // denom if denom else 0   # Cortex-M UDIV by zero yields 0
    step = max(1, min(63, step))
    return step, saw_type, fclk / (step * n * 16384)


def saw_range(wave: str, fclk: int = DEFAULT_FCLK_HZ):
    n = 2 if wave == "triangle" else 1
    return fclk / (63 * n * 16384), fclk / (1 * n * 16384)


def gain_code(gain: float):
    g = min(1.999, max(-2.0, gain))
    code = int(g * 1024.0 + 0.5) if g >= 0 else int(g * 1024.0 - 0.5)
    code = max(-2048, min(2047, code))
    return code, (code & 0xFFF) << 4


def offset_code(offset: int):
    o = max(-2048, min(2047, int(offset)))
    return o, (o & 0xFFF) << 4


def phase_word(phase_deg: float):
    p = math.fmod(phase_deg, 360.0)
    if p < 0:
        p += 360.0
    w = int(p * 65536.0 / 360.0) & 0xFFFF
    return w, w * 360.0 / 65536.0


def _wav_rmw(out: int, cur: int, prestore: int, wavesel: int) -> int:
    v = cur
    if out in (2, 4):
        v &= ~((0x3 << 12) | (0x3 << 8))
        if out == 2:
            v &= ~((1 << 11) | (1 << 10))
        v |= ((prestore & 3) << 12) | ((wavesel & 3) << 8)
    else:
        v &= ~((0x3 << 4) | 0x3)
        if out == 1:
            v &= ~((1 << 3) | (1 << 2))
        v |= ((prestore & 3) << 4) | (wavesel & 3)
    return v & 0xFFFF


def _saw_rmw(out: int, cur: int, saw_type: int, step: int) -> int:
    v = cur
    if out in (2, 4):
        v &= ~((0x3F << 10) | (0x3 << 8))
        v |= ((step & 0x3F) << 10) | ((saw_type & 3) << 8)
    else:
        v &= ~((0x3F << 2) | 0x3)
        v |= ((step & 0x3F) << 2) | (saw_type & 3)
    return v & 0xFFFF


@dataclass
class GenConfig:
    output: int = 3
    wave: str = "sine"
    freq: int = 1000
    phase: float = 0.0
    gain: float = 1.0
    offset: int = 0

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, d):
        c = cls()
        for k in asdict(c):
            if k in d and d[k] is not None:
                setattr(c, k, type(getattr(c, k))(d[k]))
        return c


def apply_config_writes(cfg: GenConfig, read_reg, fclk: int = DEFAULT_FCLK_HZ):
    """Register writes done by AD9106_ApplyConfig(), in order.

    read_reg(addr) supplies current values for the read-modify-write
    registers. Returns list of (addr, value).
    """
    out = cfg.output
    w = [(R_PAT_TYPE, 0x0000), (R_PATTERN_DLY, 0x000E),
         (R_PAT_TIMEBASE, 0x0111), (R_PAT_PERIOD, 0xFFFF),
         (CST_REG[out], 0x0800 << 4),
         (DGAIN_REG[out], gain_code(cfg.gain)[1]),
         (DOF_REG[out], offset_code(cfg.offset)[1]),
         (PW_REG[out], phase_word(cfg.phase)[0])]
    wav_reg = R_WAV43 if out in (3, 4) else R_WAV21
    saw_reg = R_SAW43 if out in (3, 4) else R_SAW21
    if cfg.wave == "sine":
        w.append((wav_reg, _wav_rmw(out, read_reg(wav_reg), 3, 1)))
        _, msb, lsb, _ = dds_tuning(cfg.freq, fclk)
        w += [(R_TW32, msb), (R_TW1, lsb)]
    elif cfg.wave in ("triangle", "saw"):
        w.append((wav_reg, _wav_rmw(out, read_reg(wav_reg), 1, 1)))
        step, st, _ = saw_setup(cfg.freq, cfg.wave, fclk)
        w.append((saw_reg, _saw_rmw(out, read_reg(saw_reg), st, step)))
    else:
        w.append((wav_reg, _wav_rmw(out, read_reg(wav_reg), 0, 1)))
    w.append((R_RAMUPDATE, 0x0001))
    return w


def actual_frequency(cfg: GenConfig, fclk: int = DEFAULT_FCLK_HZ):
    if cfg.wave == "sine":
        return dds_tuning(cfg.freq, fclk)[3]
    if cfg.wave in ("triangle", "saw"):
        return saw_setup(cfg.freq, cfg.wave, fclk)[2]
    return 0.0


def decode_register(addr: int, val: int, fclk: int = DEFAULT_FCLK_HZ,
                    regs: dict | None = None) -> str:
    """Human readable decode of the fields the firmware uses."""
    regs = regs or {}
    if addr == R_PAT_STATUS:
        return (f"RUN={val & 1}  PATTERN={(val >> 1) & 1}  "
                f"MEM_ACCESS={(val >> 2) & 1}  BUF_READ={(val >> 3) & 1}")
    if addr in (R_WAV43, R_WAV21):
        hi, lo = (4, 3) if addr == R_WAV43 else (2, 1)
        return (f"DAC{hi}: prestore={PRESTORE_NAMES[(val >> 12) & 3]}, "
                f"source={WAVESEL_NAMES[(val >> 8) & 3]}\n"
                f"DAC{lo}: prestore={PRESTORE_NAMES[(val >> 4) & 3]}, "
                f"source={WAVESEL_NAMES[val & 3]}")
    if addr in (R_SAW43, R_SAW21):
        hi, lo = (4, 3) if addr == R_SAW43 else (2, 1)
        return (f"DAC{hi}: {SAWTYPE_NAMES[(val >> 8) & 3]}, step={(val >> 10) & 0x3F}\n"
                f"DAC{lo}: {SAWTYPE_NAMES[val & 3]}, step={(val >> 2) & 0x3F}")
    if addr in DGAIN_REG.values():
        c = _s12(val >> 4)
        return f"gain code {c:+d}  →  {c / 1024:+.4f}"
    if addr in DOF_REG.values():
        return f"offset code {_s12(val >> 4):+d} LSB"
    if addr in CST_REG.values():
        return f"constant 0x{(val >> 4) & 0xFFF:03X} ({(val >> 4) & 0xFFF})"
    if addr in PW_REG.values():
        return f"phase {val * 360 / 65536:.3f}°  (DDS only)"
    if addr in (R_TW32, R_TW1):
        msb = val if addr == R_TW32 else regs.get(R_TW32)
        lsb = val if addr == R_TW1 else regs.get(R_TW1)
        if msb is None or lsb is None:
            return "DDS tuning word (read both TW32 and TW1 for the frequency)"
        tw = ((msb << 8) | (lsb >> 8)) & 0xFFFFFF
        return f"tuning word 0x{tw:06X}  →  {tw * fclk / 2 ** 24:,.3f} Hz"
    if addr == R_RAMUPDATE:
        return f"RAMUPDATE={val & 1}"
    return ""


# --------------------------------------------------------------------------
# Simulated STM32 + AD9106 (mirrors cli.c behaviour byte for byte)
# --------------------------------------------------------------------------
class SimulatedDevice:
    """Behaves like the firmware on the other end of the virtual COM port.

    float_printf=False reproduces the current build (newlib-nano without
    -u _printf_float), where every %f prints as an empty string.
    """

    BANNER = "\r\nAD9106 CLI ready\r\nType 'help' for commands\r\n> "

    def __init__(self, float_printf: bool = False):
        self.float_printf = float_printf
        self.cfg = GenConfig()
        self.regs: dict[int, int] = {}
        self._line = bytearray()
        self._out = bytearray(self.BANNER.encode())
        self._reset_regs()

    # transport-like API -------------------------------------------------
    def write(self, data: bytes):
        for b in data:
            c = chr(b)
            if c in "\r\n":
                if self._line:
                    line = self._line.decode(errors="replace")
                    self._line.clear()
                    self._execute(line[:127])
                    self._out += b"> "
            elif b in (8, 127):
                if self._line:
                    self._line.pop()
            elif len(self._line) < 127:
                self._line.append(b)

    def read(self, timeout: float = 0.0) -> bytes:
        if not self._out and timeout:
            time.sleep(min(timeout, 0.01))
        data, self._out = bytes(self._out), bytearray()
        return data

    # firmware emulation ---------------------------------------------------
    def _reset_regs(self):
        self.regs = {a: 0 for a, _, _ in REGISTERS}

    def _f(self, v: float, prec: int) -> str:
        return f"{v:.{prec}f}" if self.float_printf else ""

    def _print(self, s: str):
        self._out += s.encode()

    def _printf(self, s: str):          # vsnprintf into 128-byte buffer
        self._out += s[:127].encode()

    def _apply(self):
        for a, v in apply_config_writes(self.cfg, lambda r: self.regs.get(r, 0)):
            self.regs[a] = v

    def _execute(self, line: str):
        argv = line.split()[:8]
        if not argv:
            self._print("\r\n")
            return
        cmd, argc = argv[0], len(argv)

        def usage(u):
            self._print(f"\r\nERR usage: {u}\r\n")

        if cmd == "help":
            self._print("\r\nCommands:\r\n  help\r\n  status\r\n  read <reg>\r\n"
                        "  write <reg> <value>\r\n  update\r\n  run <0|1>\r\n"
                        "  reset\r\n  output <1|2|3|4>\r\n"
                        "  wave <sine|triangle|saw|dc>\r\n  freq <hz>\r\n"
                        "  phase <deg>\r\n  gain <value>\r\n  offset <value>\r\n")
        elif cmd == "status":
            c = self.cfg
            self._printf("\r\nState:\r\n"
                         f"  output : {c.output}\r\n"
                         f"  wave   : {c.wave}\r\n"
                         f"  freq   : {c.freq} Hz\r\n"
                         f"  phase  : {self._f(c.phase, 2)} deg\r\n"
                         f"  gain   : {self._f(c.gain, 3)}\r\n"
                         f"  offset : {c.offset}\r\n")
        elif cmd == "read":
            if argc < 2:
                return usage("read <reg>")
            reg = _c_strtoul(argv[1]) & 0xFFFF
            self._printf(f"\r\nREG[0x{reg:04X}] = 0x{self.regs.get(reg, 0):04X}\r\n")
        elif cmd == "write":
            if argc < 3:
                return usage("write <reg> <value>")
            reg, val = _c_strtoul(argv[1]) & 0xFFFF, _c_strtoul(argv[2]) & 0xFFFF
            self.regs[reg] = val
            self._printf(f"\r\nOK write 0x{reg:04X} = 0x{val:04X}\r\n")
        elif cmd == "update":
            self.regs[R_RAMUPDATE] = 1
            self._print("\r\nOK update\r\n")
        elif cmd == "run":
            if argc < 2:
                return usage("run <0|1>")
            en = 1 if (_c_strtoul(argv[1]) & 0xFF) else 0
            self.regs[R_PAT_STATUS] = en
            self._printf(f"\r\nOK run {en}\r\n")
        elif cmd == "reset":
            self._reset_regs()
            self._print("\r\nOK reset\r\n")
        elif cmd == "output":
            if argc < 2:
                return usage("output <1|2|3|4>")
            out = _c_strtoul(argv[1]) & 0xFF
            if not 1 <= out <= 4:
                self._print("\r\nERR output must be 1..4\r\n")
                return
            self.cfg.output = out
            self._apply()
            self._printf(f"\r\nOK output {out}\r\n")
        elif cmd == "wave":
            if argc < 2:
                return usage("wave <sine|triangle|saw|dc>")
            if argv[1] not in WAVES:
                self._print("\r\nERR wave must be sine, triangle, saw or dc\r\n")
                return
            self.cfg.wave = argv[1]
            self._apply()
            self._printf(f"\r\nOK wave {argv[1]}\r\n")
        elif cmd == "freq":
            if argc < 2:
                return usage("freq <hz>")
            self.cfg.freq = _c_strtoul(argv[1])
            self._apply()
            self._printf(f"\r\nOK freq {self.cfg.freq} Hz\r\n")
        elif cmd in ("phase", "gain"):
            if argc < 2:
                return usage(f"{cmd} <{'deg' if cmd == 'phase' else 'value'}>")
            v = _c_atof(argv[1])
            setattr(self.cfg, cmd, v)
            self._apply()
            if cmd == "phase":
                self._printf(f"\r\nOK phase {self._f(v, 2)} deg\r\n")
            else:
                self._printf(f"\r\nOK gain {self._f(v, 3)}\r\n")
        elif cmd == "offset":
            if argc < 2:
                return usage("offset <value>")
            self.cfg.offset = _c_strtol(argv[1])
            self._apply()
            self._printf(f"\r\nOK offset {self.cfg.offset}\r\n")
        else:
            self._print("\r\nERR unknown command\r\n")
        self._print("\r\n")


def _c_strtoul(s: str) -> int:
    import re
    m = re.match(r"\s*([+-]?)(0[xX][0-9a-fA-F]+|0[0-7]*|[1-9][0-9]*)", s)
    if not m:
        return 0
    v = parse_int(m.group(2))
    return _u32(-v if m.group(1) == "-" else v)


def _c_strtol(s: str) -> int:
    v = _c_strtoul(s)
    return v - (1 << 32) if v & 0x80000000 else v


def _c_atof(s: str) -> float:
    import re
    m = re.match(r"\s*[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?", s)
    return float(m.group(0)) if m else 0.0
