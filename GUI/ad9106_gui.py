#!/usr/bin/env python3
"""
AD9106 Function Generator GUI
=============================

Desktop front end for the STM32L476 firmware in ../Software, which exposes a
text CLI over USB CDC (virtual COM port). Every CLI command is available
from the GUI:

    help, status, read, write, update, run, reset,
    output, wave, freq, phase, gain, offset

Run:   python ad9106_gui.py            (pick the port in the toolbar)
       python ad9106_gui.py --port COM7
       python ad9106_gui.py --sim      (built-in simulator, no hardware)

Requires: PySide6 (or PySide6-Essentials) and pyserial.
"""

from __future__ import annotations

import argparse
import csv
import datetime as _dt
import html
import json
import math
import queue
import re
import sys
import threading
import time
from dataclasses import dataclass, field

from PySide6.QtCore import QObject, QPointF, QRectF, QSettings, Qt, QTimer, Signal
from PySide6.QtGui import (QAction, QColor, QFont, QFontDatabase, QKeySequence,
                           QPainter, QPainterPath, QPalette, QPen)
from PySide6.QtWidgets import (
    QAbstractItemView, QApplication, QButtonGroup, QCheckBox, QComboBox,
    QCompleter, QDialog, QDialogButtonBox, QDockWidget, QDoubleSpinBox,
    QFileDialog, QFormLayout, QFrame, QGridLayout, QGroupBox, QHBoxLayout,
    QHeaderView, QLabel, QLineEdit, QMainWindow, QMessageBox, QPlainTextEdit,
    QPushButton, QSizePolicy, QSlider, QSpinBox, QSplitter, QTableWidget,
    QTableWidgetItem, QTabWidget, QTextEdit, QToolButton, QVBoxLayout, QWidget)

from ad9106_model import (
    CLI_COMMANDS, DEFAULT_FCLK_HZ, REGISTERS, REG_BY_ADDR, REG_BY_NAME, WAVES,
    GenConfig, SimulatedDevice, _c_atof, actual_frequency, apply_config_writes,
    dds_tuning, decode_register, gain_code, offset_code, parse_int, phase_word,
    reg_label, saw_range, saw_setup)

try:
    import serial
    import serial.tools.list_ports
except ImportError:          # GUI still runs with the simulator
    serial = None

APP_TITLE = "AD9106 Function Generator"
ORG, APP = "Insyte", "AD9106 GUI"
STM32_VID, STM32_VCP_PID = 0x0483, 0x5740       # usbd_desc.c: 1155 / 22336
SIM_PORT = "__sim__"
PARAM_FIELDS = ("output", "wave", "freq", "phase", "gain", "offset")


# ==========================================================================
# Transport + command link
# ==========================================================================
class SerialTransport:
    def __init__(self, port: str):
        if serial is None:
            raise RuntimeError("pyserial is not installed (pip install pyserial)")
        self.ser = serial.Serial(port, 115200, timeout=0.02, write_timeout=2)
        try:
            self.ser.dtr = True
            self.ser.rts = True
        except Exception:
            pass

    def write(self, data: bytes):
        self.ser.write(data)
        self.ser.flush()

    def read(self) -> bytes:
        return self.ser.read(max(1, self.ser.in_waiting))

    def read_nowait(self) -> bytes:
        n = self.ser.in_waiting
        return self.ser.read(n) if n else b""

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


class SimTransport:
    def __init__(self, float_printf=False):
        self.dev = SimulatedDevice(float_printf=float_printf)
        self._lock = threading.Lock()

    def write(self, data: bytes):
        with self._lock:
            self.dev.write(data)

    def read(self) -> bytes:
        time.sleep(0.003)
        with self._lock:
            return self.dev.read()

    def read_nowait(self) -> bytes:
        with self._lock:
            return self.dev.read()

    def close(self):
        pass


@dataclass
class CliResponse:
    cmd: str
    text: str
    ok: bool
    timed_out: bool
    origin: str
    meta: dict = field(default_factory=dict)


class CliLink(QObject):
    """Serialises commands: send one line, wait for the '> ' prompt, next.

    The firmware has a single line buffer, so a new command must not be sent
    before the previous one has been answered.
    """
    traffic = Signal(str, str)          # kind ('tx','rx','info','err'), text
    response = Signal(object)           # CliResponse
    lost = Signal(str)
    pending_changed = Signal(int)

    def __init__(self):
        super().__init__()
        self._t = None
        self._q: queue.Queue = queue.Queue()
        self._thread = None
        self._stop = threading.Event()
        self._pending = 0
        self._lock = threading.Lock()

    @property
    def is_open(self):
        return self._t is not None

    def open(self, transport):
        self._t = transport
        self._stop.clear()
        self._q = queue.Queue()
        self._pending = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self):
        if not self._t:
            return
        self._stop.set()
        self._q.put(None)
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(1.5)
        self._t.close()
        self._t = None
        self._set_pending(0)

    def send(self, cmd: str, origin="gui", meta=None, timeout=2.0):
        if not self._t:
            return
        with self._lock:
            self._pending += 1
            n = self._pending
        self.pending_changed.emit(n)
        self._q.put((cmd.strip(), origin, meta or {}, timeout))

    def _set_pending(self, n):
        with self._lock:
            self._pending = n
        self.pending_changed.emit(n)

    def _done_one(self):
        with self._lock:
            self._pending = max(0, self._pending - 1)
            n = self._pending
        self.pending_changed.emit(n)

    def _run(self):
        t = self._t
        try:
            while not self._stop.is_set():
                try:
                    job = self._q.get(timeout=0.05)
                except queue.Empty:
                    data = t.read()
                    if data:
                        self.traffic.emit("rx", data.decode(errors="replace"))
                    continue
                if job is None:
                    break
                cmd, origin, meta, timeout = job
                # discard/show anything unsolicited before sending
                stale = t.read_nowait()
                if stale.strip(b"\r\n> "):
                    self.traffic.emit("rx", stale.decode(errors="replace"))
                t.write((cmd + "\r\n").encode())
                self.traffic.emit("tx", cmd)
                buf = ""
                done = False
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline and not self._stop.is_set():
                    d = t.read()
                    if d:
                        buf += d.decode(errors="replace")
                        if buf.endswith("> "):
                            done = True
                            break
                if self._stop.is_set():
                    break
                if buf:
                    self.traffic.emit("rx", buf)
                text = _clean_response(buf)
                ok = done and not re.search(r"^ERR", text, re.M)
                if not done:
                    self.traffic.emit("err", f"timeout: no prompt after '{cmd}'")
                self._done_one()
                self.response.emit(CliResponse(cmd, text, ok, not done, origin, meta))
        except Exception as e:                       # port unplugged etc.
            if not self._stop.is_set():
                self.lost.emit(str(e))


def _clean_response(buf: str) -> str:
    t = buf.replace("\r\n", "\n").replace("\r", "\n")
    if t.endswith("> "):
        t = t[:-2]
    return t.strip("\n ")


# ==========================================================================
# Formatting helpers
# ==========================================================================
def fmt_hz(f: float) -> str:
    if f >= 1e6:
        return f"{f / 1e6:.6f}".rstrip("0").rstrip(".") + " MHz"
    if f >= 1e3:
        return f"{f / 1e3:.6f}".rstrip("0").rstrip(".") + " kHz"
    return f"{f:.4f}".rstrip("0").rstrip(".") + " Hz"


def fmt_time(s: float) -> str:
    for unit, k in (("s", 1), ("ms", 1e-3), ("µs", 1e-6), ("ns", 1e-9)):
        if s >= k:
            return f"{s / k:.3g} {unit}"
    return f"{s:.3g} s"


def is_dark(widget: QWidget) -> bool:
    return widget.palette().color(QPalette.Window).lightness() < 128


def colors(widget: QWidget) -> dict:
    dark = is_dark(widget)
    return {
        "accent": QColor("#4ea1ff" if dark else "#1565c0"),
        "warn": QColor("#ffb74d" if dark else "#b26a00"),
        "err": QColor("#ff6b6b" if dark else "#c62828"),
        "ok": QColor("#66bb6a" if dark else "#2e7d32"),
        "muted": widget.palette().color(QPalette.PlaceholderText),
    }


def mono_font(pt=None) -> QFont:
    f = QFontDatabase.systemFont(QFontDatabase.FixedFont)
    if pt:
        f.setPointSizeF(pt)
    return f


# ==========================================================================
# Waveform preview
# ==========================================================================
class WavePreview(QWidget):
    def __init__(self):
        super().__init__()
        self.cfg = GenConfig()
        self.f_actual = 1000.0
        self.running = None
        self.setMinimumHeight(190)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_state(self, cfg: GenConfig, f_actual: float, running):
        self.cfg, self.f_actual, self.running = cfg, f_actual, running
        self.update()

    def _sample(self, t: float) -> float:
        c = self.cfg
        g = gain_code(c.gain)[0] / 1024.0
        o = offset_code(c.offset)[0]
        if c.wave == "sine":
            ph = math.radians(phase_word(c.phase)[1])
            s = math.sin(2 * math.pi * t + ph)
        elif c.wave == "triangle":
            x = t % 1.0
            s = 4 * x - 1 if x < 0.5 else 3 - 4 * x
        elif c.wave == "saw":
            s = 2 * (t % 1.0) - 1
        else:
            s = 0.0
        return g * 2047 * s + o

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        pal = self.palette()
        col = colors(self)
        p.fillRect(self.rect(), pal.color(QPalette.Base))
        fm = p.fontMetrics()
        left, right, top, bottom = fm.horizontalAdvance("−2048") + 14, 12, 12, fm.height() + 10
        r = QRectF(left, top, self.width() - left - right, self.height() - top - bottom)
        if r.width() < 20 or r.height() < 20:
            return

        def y_of(v):
            return r.bottom() - (v + 2048) / 4095.0 * r.height()

        grid = QColor(pal.color(QPalette.Text))
        grid.setAlpha(40)
        for v in (-2048, -1024, 0, 1024, 2047):
            p.setPen(QPen(grid, 1, Qt.DashLine if v not in (0,) else Qt.SolidLine))
            p.drawLine(QPointF(r.left(), y_of(v)), QPointF(r.right(), y_of(v)))
            p.setPen(col["muted"])
            p.drawText(QRectF(0, y_of(v) - fm.height() / 2, left - 6, fm.height()),
                       Qt.AlignRight | Qt.AlignVCenter, f"{v:+d}".replace("-", "−") if v else "0")
        for i in range(5):
            x = r.left() + i * r.width() / 4
            p.setPen(QPen(grid, 1, Qt.DashLine))
            p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()))
        # x labels
        p.setPen(col["muted"])
        if self.cfg.wave != "dc" and self.f_actual > 0:
            T = 1.0 / self.f_actual
            for i, lab in enumerate(("0", fmt_time(T / 2), fmt_time(T), fmt_time(1.5 * T), fmt_time(2 * T))):
                x = r.left() + i * r.width() / 4
                if i == 4:
                    box, al = QRectF(x - 100, r.bottom() + 4, 100, fm.height()), Qt.AlignRight
                elif i == 0:
                    box, al = QRectF(x, r.bottom() + 4, 100, fm.height()), Qt.AlignLeft
                else:
                    box, al = QRectF(x - 50, r.bottom() + 4, 100, fm.height()), Qt.AlignHCenter
                p.drawText(box, al, lab)
        # waveform
        path = QPainterPath()
        clipped = False
        n = max(200, int(r.width()))
        for i in range(n + 1):
            t = 2.0 * i / n
            v = self._sample(t)
            if v > 2047 or v < -2048:
                clipped = True
            v = max(-2048, min(2047, v))
            pt = QPointF(r.left() + i / n * r.width(), y_of(v))
            path.moveTo(pt) if i == 0 else path.lineTo(pt)
        pen = QPen(col["err"] if clipped else col["accent"], 2)
        if self.running is False:
            pen.setStyle(Qt.DashLine)
        p.setPen(pen)
        p.drawPath(path)
        p.setPen(QPen(grid, 1))
        p.drawRect(r)
        notes = []
        if clipped:
            notes.append("clipping: gain/offset exceed DAC range")
        if self.running is False:
            notes.append("pattern generator stopped (run 0)")
        if notes:
            text = "\n".join(notes)
            br = p.boundingRect(r.adjusted(8, 6, -8, -6), Qt.AlignTop | Qt.AlignLeft, text)
            bg = QColor(pal.color(QPalette.Base))
            bg.setAlpha(225)
            p.fillRect(br.adjusted(-5, -3, 5, 3), bg)
            p.setPen(col["err"] if clipped else col["muted"])
            p.drawText(br, Qt.AlignTop | Qt.AlignLeft, text)
        p.end()


# ==========================================================================
# Register script dialog
# ==========================================================================
class ScriptDialog(QDialog):
    def __init__(self, parent, initial=""):
        super().__init__(parent)
        self.setWindowTitle("Register script")
        self.resize(560, 480)
        lay = QVBoxLayout(self)
        info = QLabel("One register write per line: <b>address value</b> (hex 0x.., decimal or "
                      "register name). Commas, '=' and '#' comments are allowed. "
                      "Example: <tt>WAV4_3CONFIG 0x3232</tt>")
        info.setWordWrap(True)
        lay.addWidget(info)
        self.edit = QPlainTextEdit(initial)
        self.edit.setFont(mono_font())
        lay.addWidget(self.edit, 1)
        row = QHBoxLayout()
        b_load = QPushButton("Load file…")
        b_load.clicked.connect(self._load)
        b_save = QPushButton("Save as…")
        b_save.clicked.connect(self._save)
        self.chk_update = QCheckBox("Send update (RAMUPDATE) afterwards")
        self.chk_update.setChecked(True)
        row.addWidget(b_load)
        row.addWidget(b_save)
        row.addStretch(1)
        row.addWidget(self.chk_update)
        lay.addLayout(row)
        bb = QDialogButtonBox(QDialogButtonBox.Cancel)
        self.b_run = bb.addButton("Write to device", QDialogButtonBox.AcceptRole)
        bb.accepted.connect(self._accept)
        bb.rejected.connect(self.reject)
        lay.addWidget(bb)
        self.writes: list[tuple[int, int]] = []

    @staticmethod
    def parse(text: str):
        writes, errors = [], []
        for n, raw in enumerate(text.splitlines(), 1):
            line = raw.split("#", 1)[0].split("//", 1)[0]
            line = line.replace(",", " ").replace("=", " ").replace(";", " ").strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 2:
                errors.append(f"line {n}: expected 'address value'")
                continue
            try:
                a = REG_BY_NAME[parts[0].upper()] if parts[0].upper() in REG_BY_NAME else parse_int(parts[0])
                v = parse_int(parts[1])
                if not (0 <= a <= 0xFFFF and 0 <= v <= 0xFFFF):
                    raise ValueError
                writes.append((a, v))
            except (ValueError, KeyError):
                errors.append(f"line {n}: cannot parse '{raw.strip()}'")
        return writes, errors

    def _load(self):
        fn, _ = QFileDialog.getOpenFileName(self, "Load register script", "",
                                            "Scripts (*.txt *.csv);;All files (*)")
        if fn:
            with open(fn, encoding="utf-8") as f:
                self.edit.setPlainText(f.read())

    def _save(self):
        fn, _ = QFileDialog.getSaveFileName(self, "Save register script", "registers.txt",
                                            "Text (*.txt);;CSV (*.csv)")
        if fn:
            with open(fn, "w", encoding="utf-8") as f:
                f.write(self.edit.toPlainText())

    def _accept(self):
        writes, errors = self.parse(self.edit.toPlainText())
        if errors:
            QMessageBox.warning(self, "Script errors", "\n".join(errors[:20]))
            return
        if not writes:
            QMessageBox.information(self, "Register script", "Nothing to write.")
            return
        self.writes = writes
        self.accept()


# ==========================================================================
# Main window
# ==========================================================================
class MainWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.setWindowTitle(APP_TITLE)
        self.settings = QSettings(ORG, APP)
        self.link = CliLink()
        self.link.traffic.connect(self._on_traffic)
        self.link.response.connect(self._on_response)
        self.link.lost.connect(self._on_link_lost)
        self.link.pending_changed.connect(self._on_pending)

        self.confirmed: dict = {f: None for f in PARAM_FIELDS} | {"run": None}
        self.target: dict = dict(self.confirmed)
        self.reg_values: dict[int, int] = {}
        self.reg_changed: set[int] = set()
        self._guard = False
        self._float_note_shown = False
        self._history: list[str] = []
        self._hist_pos = 0
        self._port_name = ""

        self._apply_timer = QTimer(self, singleShot=True, interval=250)
        self._apply_timer.timeout.connect(lambda: self.apply_settings(False))

        self._build_ui()
        self._build_menu()
        self._restore_settings()
        self.refresh_ports()
        self._set_connected(False)
        self._update_derived()

        if args.sim:
            self._select_port(SIM_PORT)
            QTimer.singleShot(100, self.connect_device)
        elif args.port:
            self._select_port(args.port, add=True)
            QTimer.singleShot(100, self.connect_device)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        # --- connection bar ---
        bar = QWidget()
        hb = QHBoxLayout(bar)
        hb.setContentsMargins(10, 8, 10, 4)
        hb.addWidget(QLabel("Port"))
        self.cmb_port = QComboBox()
        self.cmb_port.setMinimumWidth(300)
        hb.addWidget(self.cmb_port)
        self.btn_refresh = QToolButton(text="⟳")
        self.btn_refresh.setToolTip("Rescan serial ports")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        hb.addWidget(self.btn_refresh)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setMinimumWidth(110)
        self.btn_connect.clicked.connect(self.toggle_connection)
        hb.addWidget(self.btn_connect)
        self.lbl_conn = QLabel()
        hb.addWidget(self.lbl_conn)
        hb.addStretch(1)
        self.btn_run = QPushButton("Output: unknown")
        self.btn_run.setCheckable(True)
        self.btn_run.setMinimumWidth(170)
        self.btn_run.setToolTip("run 1 / run 0 – starts or stops the AD9106 pattern generator "
                                "(PAT_STATUS.RUN)")
        self.btn_run.clicked.connect(self._on_run_clicked)
        hb.addWidget(self.btn_run)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_generator_tab(), "Generator")
        self.tabs.addTab(self._build_registers_tab(), "Registers")

        central = QWidget()
        v = QVBoxLayout(central)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        v.addWidget(bar)
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        v.addWidget(line)
        v.addWidget(self.tabs, 1)
        self.setCentralWidget(central)

        self._build_terminal_dock()

        self.lbl_busy = QLabel()
        self.statusBar().addPermanentWidget(self.lbl_busy)

    def _build_generator_tab(self):
        w = QWidget()
        root = QHBoxLayout(w)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(12)

        # ---------------- left column: settings ----------------
        left = QVBoxLayout()
        left.setSpacing(10)

        g_out = QGroupBox("Output channel")
        lo = QHBoxLayout(g_out)
        self.grp_out = QButtonGroup(self)
        for i in range(1, 5):
            b = QPushButton(f"DAC {i}")
            b.setCheckable(True)
            b.setToolTip(f"output {i} – settings below are applied to IOUT{i}")
            self.grp_out.addButton(b, i)
            lo.addWidget(b)
        self.grp_out.idClicked.connect(lambda _: self._param_edited("output"))
        left.addWidget(g_out)

        g_wave = QGroupBox("Waveform")
        lw = QHBoxLayout(g_wave)
        self.grp_wave = QButtonGroup(self)
        for i, (name, label) in enumerate((("sine", "Sine"), ("triangle", "Triangle"),
                                           ("saw", "Sawtooth"), ("dc", "DC"))):
            b = QPushButton(label)
            b.setCheckable(True)
            b.setToolTip({"sine": "DDS sine (frequency from 24-bit tuning word)",
                          "triangle": "Prestored sawtooth generator, triangle mode",
                          "saw": "Prestored sawtooth generator, ramp up",
                          "dc": "Constant value (DACx_CST) + digital offset"}[name])
            self.grp_wave.addButton(b, i)
            lw.addWidget(b)
        self.grp_wave.idClicked.connect(lambda _: self._param_edited("wave"))
        left.addWidget(g_wave)

        g_par = QGroupBox("Parameters")
        grid = QGridLayout(g_par)
        grid.setColumnStretch(1, 1)
        grid.setHorizontalSpacing(10)
        self.param_labels = {}

        def add_row(row, key, text, slider, spin, info):
            lab = QLabel(text)
            self.param_labels[key] = lab
            grid.addWidget(lab, row, 0)
            grid.addWidget(slider, row, 1)
            grid.addWidget(spin, row, 2)
            grid.addWidget(info, row + 1, 1, 1, 2)
            info.setWordWrap(True)
            info.setStyleSheet("font-size: 90%;")

        # frequency (log slider)
        self.sp_freq = QSpinBox()
        self.sp_freq.setRange(1, 100_000_000)
        self.sp_freq.setSuffix(" Hz")
        self.sp_freq.setGroupSeparatorShown(True)
        self.sp_freq.setMinimumWidth(150)
        self.sp_freq.setToolTip("freq <hz> – integer Hz")
        self.sl_freq = QSlider(Qt.Horizontal)
        self.sl_freq.setRange(0, 8000)
        self.lbl_freq_info = QLabel()
        add_row(0, "freq", "Frequency", self.sl_freq, self.sp_freq, self.lbl_freq_info)
        self._link_slider(self.sp_freq, self.sl_freq,
                          lambda f: round(math.log10(max(1, f)) * 1000),
                          lambda s: int(float(f"{10 ** (s / 1000):.3g}")))

        self.sp_phase = QDoubleSpinBox()
        self.sp_phase.setRange(0.0, 359.99)
        self.sp_phase.setDecimals(2)
        self.sp_phase.setSingleStep(1.0)
        self.sp_phase.setWrapping(True)
        self.sp_phase.setSuffix(" °")
        self.sp_phase.setToolTip("phase <deg> – DDS phase offset (sine only)")
        self.sl_phase = QSlider(Qt.Horizontal)
        self.sl_phase.setRange(0, 35999)
        self.lbl_phase_info = QLabel()
        add_row(2, "phase", "Phase", self.sl_phase, self.sp_phase, self.lbl_phase_info)
        self._link_slider(self.sp_phase, self.sl_phase, lambda v: round(v * 100), lambda s: s / 100)

        self.sp_gain = QDoubleSpinBox()
        self.sp_gain.setRange(-2.0, 1.999)
        self.sp_gain.setDecimals(3)
        self.sp_gain.setSingleStep(0.01)
        self.sp_gain.setToolTip("gain <value> – digital gain, −2.0 … +1.999 (12-bit, 1/1024 steps)")
        self.sl_gain = QSlider(Qt.Horizontal)
        self.sl_gain.setRange(-2000, 1999)
        self.lbl_gain_info = QLabel()
        add_row(4, "gain", "Gain", self.sl_gain, self.sp_gain, self.lbl_gain_info)
        self._link_slider(self.sp_gain, self.sl_gain, lambda v: round(v * 1000), lambda s: s / 1000)

        self.sp_offset = QSpinBox()
        self.sp_offset.setRange(-2048, 2047)
        self.sp_offset.setSuffix(" LSB")
        self.sp_offset.setToolTip("offset <value> – digital offset, signed 12-bit code")
        self.sl_offset = QSlider(Qt.Horizontal)
        self.sl_offset.setRange(-2048, 2047)
        self.lbl_offset_info = QLabel()
        add_row(6, "offset", "Offset", self.sl_offset, self.sp_offset, self.lbl_offset_info)
        self._link_slider(self.sp_offset, self.sl_offset, lambda v: int(v), lambda s: s)

        for key, sp in (("freq", self.sp_freq), ("phase", self.sp_phase),
                        ("gain", self.sp_gain), ("offset", self.sp_offset)):
            sp.setKeyboardTracking(False)
            sp.valueChanged.connect(lambda _=None, k=key: self._param_edited(k))
        left.addWidget(g_par)

        # apply row
        ar = QHBoxLayout()
        self.chk_auto = QCheckBox("Apply changes immediately")
        self.chk_auto.setChecked(True)
        self.chk_auto.toggled.connect(lambda on: self.apply_settings(False) if on else None)
        ar.addWidget(self.chk_auto)
        ar.addStretch(1)
        self.btn_revert = QPushButton("Revert")
        self.btn_revert.setToolTip("Discard unapplied changes (back to the device state)")
        self.btn_revert.clicked.connect(self.revert_widgets)
        self.btn_apply = QPushButton("Apply")
        self.btn_apply.setDefault(True)
        self.btn_apply.clicked.connect(lambda: self.apply_settings(False))
        self.btn_apply_all = QPushButton("Write all")
        self.btn_apply_all.setToolTip("Send every parameter, even unchanged ones "
                                      "(each command re-runs AD9106_ApplyConfig)")
        self.btn_apply_all.clicked.connect(lambda: self.apply_settings(True))
        for b in (self.btn_revert, self.btn_apply, self.btn_apply_all):
            ar.addWidget(b)
        left.addLayout(ar)

        g_dev = QGroupBox("Device")
        ld = QGridLayout(g_dev)
        self.btn_status = QPushButton("Read status")
        self.btn_status.setToolTip("status – reads the firmware's current settings into the GUI")
        self.btn_status.clicked.connect(lambda: self.send("status", "sync"))
        self.btn_update = QPushButton("Update (RAMUPDATE)")
        self.btn_update.setToolTip("update – copies shadow registers to active registers")
        self.btn_update.clicked.connect(lambda: self.send("update", "user"))
        self.btn_reset = QPushButton("Reset AD9106")
        self.btn_reset.setToolTip("reset – pulses the AD9106 RESET pin; all chip registers return "
                                  "to their defaults")
        self.btn_reset.clicked.connect(self.reset_device)
        self.chk_reapply = QCheckBox("Re-apply settings after reset")
        self.chk_reapply.setChecked(True)
        self.btn_help = QPushButton("Firmware help")
        self.btn_help.setToolTip("help – shows the firmware's command list in the terminal")
        self.btn_help.clicked.connect(lambda: self.send("help", "user"))
        ld.addWidget(self.btn_status, 0, 0)
        ld.addWidget(self.btn_update, 0, 1)
        ld.addWidget(self.btn_help, 0, 2)
        ld.addWidget(self.btn_reset, 1, 0)
        ld.addWidget(self.chk_reapply, 1, 1, 1, 2)
        left.addWidget(g_dev)
        left.addStretch(1)

        lw_ = QWidget()
        lw_.setLayout(left)
        lw_.setMinimumWidth(500)
        lw_.setMaximumWidth(620)

        # ---------------- right column: preview + computed ----------------
        right = QVBoxLayout()
        right.setSpacing(10)
        g_prev = QGroupBox("Preview (digital DAC code, two periods)")
        lp = QVBoxLayout(g_prev)
        self.preview = WavePreview()
        lp.addWidget(self.preview)
        right.addWidget(g_prev, 3)

        g_calc = QGroupBox("What the firmware will write")
        lc = QVBoxLayout(g_calc)
        fr = QHBoxLayout()
        fr.addWidget(QLabel("DAC clock (g_dac_clock_hz)"))
        self.sp_fclk = QDoubleSpinBox()
        self.sp_fclk.setRange(1.0, 200.0)
        self.sp_fclk.setDecimals(6)
        self.sp_fclk.setSuffix(" MHz")
        self.sp_fclk.setValue(DEFAULT_FCLK_HZ / 1e6)
        self.sp_fclk.setToolTip("Only used for the calculations in this GUI. Must match "
                                "g_dac_clock_hz in ad9106.c (156.25 MHz).")
        self.sp_fclk.valueChanged.connect(lambda _: self._update_derived())
        fr.addWidget(self.sp_fclk)
        fr.addStretch(1)
        lc.addLayout(fr)
        self.lbl_summary = QLabel()
        self.lbl_summary.setTextFormat(Qt.RichText)
        self.lbl_summary.setWordWrap(True)
        self.lbl_summary.setTextInteractionFlags(Qt.TextSelectableByMouse)
        lc.addWidget(self.lbl_summary)
        self.txt_writes = QPlainTextEdit()
        self.txt_writes.setReadOnly(True)
        self.txt_writes.setFont(mono_font())
        self.txt_writes.setMinimumHeight(120)
        lc.addWidget(self.txt_writes, 1)
        right.addWidget(g_calc, 4)

        rw = QWidget()
        rw.setLayout(right)
        root.addWidget(lw_)
        root.addWidget(rw, 1)
        return w

    def _link_slider(self, spin, slider, to_s, from_s):
        def s2p(v):
            spin.setValue(from_s(v))

        def p2s(v):
            slider.blockSignals(True)
            slider.setValue(to_s(v))
            slider.blockSignals(False)
        slider.valueChanged.connect(s2p)
        spin.valueChanged.connect(p2s)
        p2s(spin.value())

    def _build_registers_tab(self):
        w = QWidget()
        root = QHBoxLayout(w)
        root.setContentsMargins(10, 10, 10, 10)
        split = QSplitter(Qt.Horizontal)
        root.addWidget(split)

        # table
        tw = QWidget()
        tl = QVBoxLayout(tw)
        tl.setContentsMargins(0, 0, 0, 0)
        top = QHBoxLayout()
        self.btn_read_all = QPushButton("Read all")
        self.btn_read_all.setToolTip("read <reg> for every register in the map")
        self.btn_read_all.clicked.connect(self.read_all_registers)
        self.btn_read_sel = QPushButton("Read selected")
        self.btn_read_sel.clicked.connect(self.read_selected_registers)
        self.btn_script = QPushButton("Register script…")
        self.btn_script.setToolTip("Write a list of register values (e.g. one of the example "
                                   "configurations from main.c)")
        self.btn_script.clicked.connect(self.open_script)
        self.btn_export = QPushButton("Export…")
        self.btn_export.setToolTip("Save the last read values as a register script / CSV")
        self.btn_export.clicked.connect(self.export_registers)
        self.ed_filter = QLineEdit()
        self.ed_filter.setPlaceholderText("Filter…")
        self.ed_filter.setClearButtonEnabled(True)
        self.ed_filter.textChanged.connect(self._filter_regs)
        for b in (self.btn_read_all, self.btn_read_sel, self.btn_script, self.btn_export):
            top.addWidget(b)
        top.addStretch(1)
        top.addWidget(self.ed_filter)
        tl.addLayout(top)

        self.tbl = QTableWidget(len(REGISTERS), 4)
        self.tbl.setHorizontalHeaderLabels(["Address", "Name", "Value", "Description"])
        self.tbl.verticalHeader().setVisible(False)
        self.tbl.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tbl.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tbl.setAlternatingRowColors(True)
        hh = self.tbl.horizontalHeader()
        hh.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        hh.setStretchLastSection(True)
        mf = mono_font()
        for row, (a, n, d) in enumerate(REGISTERS):
            it = QTableWidgetItem(f"0x{a:04X}")
            it.setFont(mf)
            it.setData(Qt.UserRole, a)
            self.tbl.setItem(row, 0, it)
            self.tbl.setItem(row, 1, QTableWidgetItem(n))
            iv = QTableWidgetItem("—")
            iv.setFont(mf)
            iv.setTextAlignment(Qt.AlignCenter)
            self.tbl.setItem(row, 2, iv)
            self.tbl.setItem(row, 3, QTableWidgetItem(d))
        self.tbl.itemSelectionChanged.connect(self._on_reg_selected)
        self.tbl.itemDoubleClicked.connect(lambda _: self.reg_read_current())
        tl.addWidget(self.tbl, 1)
        split.addWidget(tw)

        # editor
        ew = QWidget()
        el = QVBoxLayout(ew)
        el.setContentsMargins(8, 0, 0, 0)
        g = QGroupBox("Register editor")
        gl = QVBoxLayout(g)
        form = QFormLayout()
        self.ed_addr = QLineEdit("0x0000")
        self.ed_addr.setFont(mf)
        self.ed_addr.setToolTip("Address (hex, decimal or register name)")
        self.ed_addr.editingFinished.connect(self._on_addr_edited)
        self.lbl_reg_name = QLabel()
        ar = QHBoxLayout()
        ar.addWidget(self.ed_addr)
        ar.addWidget(self.lbl_reg_name, 1)
        form.addRow("Address", ar)
        self.ed_val = QLineEdit("0x0000")
        self.ed_val.setFont(mf)
        self.ed_val.textEdited.connect(self._on_val_edited)
        self.lbl_val_dec = QLabel()
        vr = QHBoxLayout()
        vr.addWidget(self.ed_val)
        vr.addWidget(self.lbl_val_dec, 1)
        form.addRow("Value", vr)
        gl.addLayout(form)

        bits = QGridLayout()
        bits.setSpacing(3)
        self.bit_buttons = []
        for i in range(16):
            bit = 15 - i
            lab = QLabel(str(bit))
            lab.setAlignment(Qt.AlignCenter)
            lab.setStyleSheet("font-size: 85%;")
            b = QToolButton(text="0")
            b.setCheckable(True)
            b.setFixedSize(26, 26)
            b.setFont(mf)
            b.toggled.connect(lambda on, bb=b: bb.setText("1" if on else "0"))
            b.clicked.connect(self._on_bit_clicked)
            col = i + (1 if i >= 8 else 0)
            bits.addWidget(lab, 0, col)
            bits.addWidget(b, 1, col)
            self.bit_buttons.append((bit, b))
        bits.setColumnMinimumWidth(8, 10)
        gl.addLayout(bits)

        self.lbl_decode = QLabel()
        self.lbl_decode.setWordWrap(True)
        self.lbl_decode.setFont(mf)
        self.lbl_decode.setMinimumHeight(48)
        self.lbl_decode.setTextInteractionFlags(Qt.TextSelectableByMouse)
        gl.addWidget(self.lbl_decode)

        br = QHBoxLayout()
        self.btn_reg_read = QPushButton("Read")
        self.btn_reg_read.clicked.connect(self.reg_read_current)
        self.btn_reg_write = QPushButton("Write")
        self.btn_reg_write.clicked.connect(self.reg_write_current)
        self.chk_reg_update = QCheckBox("then update")
        self.chk_reg_update.setToolTip("Send 'update' (RAMUPDATE) after the write so the value "
                                       "becomes active")
        self.chk_reg_update.setChecked(True)
        br.addWidget(self.btn_reg_read)
        br.addWidget(self.btn_reg_write)
        br.addWidget(self.chk_reg_update)
        br.addStretch(1)
        gl.addLayout(br)
        note = QLabel("Most AD9106 registers are double-buffered: a write lands in the shadow "
                      "register and only takes effect after <i>update</i>. Note that the "
                      "Generator tab rewrites the registers it uses on every change.")
        note.setWordWrap(True)
        note.setStyleSheet("font-size: 90%;")
        gl.addWidget(note)
        el.addWidget(g)
        el.addStretch(1)
        split.addWidget(ew)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)
        self._set_editor_value(0)
        self._on_addr_edited()
        return w

    def _build_terminal_dock(self):
        dock = QDockWidget("Terminal", self)
        dock.setObjectName("terminal")
        dock.setAllowedAreas(Qt.BottomDockWidgetArea | Qt.RightDockWidgetArea)
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(8, 4, 8, 8)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setFont(mono_font())
        self.log.document().setMaximumBlockCount(5000)
        self.log.setLineWrapMode(QTextEdit.NoWrap)
        v.addWidget(self.log, 1)
        row = QHBoxLayout()
        self.ed_cmd = QLineEdit()
        self.ed_cmd.setFont(mono_font())
        self.ed_cmd.setPlaceholderText("Type a CLI command, e.g.  freq 2500   read 0x26   help   "
                                       "(↑/↓ history)")
        comp = QCompleter(list(CLI_COMMANDS) + [f"wave {w_}" for w_ in WAVES], self)
        comp.setCaseSensitivity(Qt.CaseInsensitive)
        self.ed_cmd.setCompleter(comp)
        self.ed_cmd.returnPressed.connect(self._send_raw)
        self.ed_cmd.installEventFilter(self)
        b_send = QPushButton("Send")
        b_send.clicked.connect(self._send_raw)
        self.chk_ts = QCheckBox("Timestamps")
        b_clear = QPushButton("Clear")
        b_clear.clicked.connect(self.log.clear)
        b_save = QPushButton("Save log…")
        b_save.clicked.connect(self._save_log)
        row.addWidget(self.ed_cmd, 1)
        row.addWidget(b_send)
        row.addWidget(self.chk_ts)
        row.addWidget(b_clear)
        row.addWidget(b_save)
        v.addLayout(row)
        dock.setWidget(w)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock)
        self.dock_term = dock

    def _build_menu(self):
        mb = self.menuBar()
        m = mb.addMenu("&File")
        a = QAction("&Load preset…", self, shortcut=QKeySequence.Open)
        a.triggered.connect(self.load_preset)
        m.addAction(a)
        a = QAction("&Save preset…", self, shortcut=QKeySequence.Save)
        a.triggered.connect(self.save_preset)
        m.addAction(a)
        m.addSeparator()
        a = QAction("E&xit", self, shortcut=QKeySequence.Quit)
        a.triggered.connect(self.close)
        m.addAction(a)

        m = mb.addMenu("&Device")
        self.act_connect = QAction("&Connect / disconnect", self, shortcut="Ctrl+K")
        self.act_connect.triggered.connect(self.toggle_connection)
        m.addAction(self.act_connect)
        a = QAction("Read &status", self, shortcut="F5")
        a.triggered.connect(lambda: self.send("status", "sync"))
        m.addAction(a)
        a = QAction("&Run / stop output", self, shortcut="F9")
        a.triggered.connect(lambda: (self.btn_run.toggle(), self._on_run_clicked()))
        m.addAction(a)
        a = QAction("&Update (RAMUPDATE)", self)
        a.triggered.connect(lambda: self.send("update", "user"))
        m.addAction(a)
        a = QAction("Reset &AD9106", self)
        a.triggered.connect(self.reset_device)
        m.addAction(a)
        m.addSeparator()
        a = QAction("Read &all registers", self, shortcut="Ctrl+R")
        a.triggered.connect(self.read_all_registers)
        m.addAction(a)
        a = QAction("Register &script…", self)
        a.triggered.connect(self.open_script)
        m.addAction(a)

        m = mb.addMenu("&View")
        m.addAction(self.dock_term.toggleViewAction())

        m = mb.addMenu("&Help")
        a = QAction("&CLI reference", self, shortcut=QKeySequence.HelpContents)
        a.triggered.connect(self.show_cli_reference)
        m.addAction(a)
        a = QAction("&About", self)
        a.triggered.connect(lambda: QMessageBox.about(
            self, "About", f"<b>{APP_TITLE}</b><br>GUI for the STM32 USB CLI that configures "
                           "the AD9106BCPZ.<br><br>Insyte"))
        m.addAction(a)

    # ------------------------------------------------------------ settings
    def _restore_settings(self):
        s = self.settings
        g = s.value("geometry")
        if g is not None:
            self.restoreGeometry(g)
        else:
            self.resize(1280, 860)
        st = s.value("windowState")
        if st is not None:
            self.restoreState(st)
        self.chk_auto.setChecked(s.value("autoApply", True, type=bool))
        self.chk_reapply.setChecked(s.value("reapply", True, type=bool))
        self.chk_ts.setChecked(s.value("timestamps", False, type=bool))
        self.sp_fclk.setValue(s.value("fclkMHz", DEFAULT_FCLK_HZ / 1e6, type=float))
        try:
            cfg = GenConfig.from_dict(json.loads(s.value("lastConfig", "{}")))
        except Exception:
            cfg = GenConfig()
        self._set_widgets(cfg)

    def closeEvent(self, ev):
        s = self.settings
        s.setValue("geometry", self.saveGeometry())
        s.setValue("windowState", self.saveState())
        s.setValue("autoApply", self.chk_auto.isChecked())
        s.setValue("reapply", self.chk_reapply.isChecked())
        s.setValue("timestamps", self.chk_ts.isChecked())
        s.setValue("fclkMHz", self.sp_fclk.value())
        s.setValue("lastConfig", json.dumps(self._widget_cfg().to_dict()))
        if self._port_name and self._port_name != SIM_PORT:
            s.setValue("port", self._port_name)
        self.link.close()
        super().closeEvent(ev)

    # ---------------------------------------------------------- connection
    def refresh_ports(self):
        current = self.cmb_port.currentData()
        self.cmb_port.clear()
        best = None
        if serial is not None:
            for p in sorted(serial.tools.list_ports.comports(), key=lambda p: p.device):
                is_stm = p.vid == STM32_VID and p.pid == STM32_VCP_PID
                label = f"{p.device}  –  {p.description}"
                if is_stm:
                    label += "   ★ STM32 VCP"
                    best = best or p.device
                self.cmb_port.addItem(label, p.device)
        self.cmb_port.addItem("Simulator (no hardware)", SIM_PORT)
        want = current or self.settings.value("port", "")
        if not (want and self._select_port(want)) and best:
            self._select_port(best)

    def _select_port(self, port, add=False):
        i = self.cmb_port.findData(port)
        if i < 0 and add:
            self.cmb_port.insertItem(0, port, port)
            i = 0
        if i >= 0:
            self.cmb_port.setCurrentIndex(i)
            return True
        return False

    def toggle_connection(self):
        if self.link.is_open:
            self.disconnect_device()
        else:
            self.connect_device()

    def connect_device(self):
        port = self.cmb_port.currentData()
        if not port:
            return
        try:
            transport = SimTransport() if port == SIM_PORT else SerialTransport(port)
        except Exception as e:
            QMessageBox.critical(self, "Connection failed", f"Could not open {port}:\n{e}")
            return
        self._port_name = port
        self.link.open(transport)
        self._set_connected(True)
        self._log("info", f"connected to {'simulator' if port == SIM_PORT else port}")
        self.send("status", "sync")
        self.send("read 0x001E", "sync")

    def disconnect_device(self):
        self.link.close()
        self._set_connected(False)
        self._log("info", "disconnected")

    def _on_link_lost(self, reason):
        self.link.close()
        self._set_connected(False)
        self._log("err", f"connection lost: {reason}")
        self._status(f"Connection lost: {reason}", "err")

    def _set_connected(self, on: bool):
        c = colors(self)
        self.btn_connect.setText("Disconnect" if on else "Connect")
        self.cmb_port.setEnabled(not on)
        self.btn_refresh.setEnabled(not on)
        name = "simulator" if self._port_name == SIM_PORT else self._port_name
        self.lbl_conn.setText(f"<span style='color:{(c['ok'] if on else c['muted']).name()}'>●</span> "
                              + (f"Connected ({name})" if on else "Not connected"))
        for wdg in (self.btn_run, self.btn_apply, self.btn_apply_all, self.btn_status,
                    self.btn_update, self.btn_reset, self.btn_help, self.btn_read_all,
                    self.btn_read_sel, self.btn_script, self.btn_reg_read, self.btn_reg_write,
                    self.ed_cmd):
            wdg.setEnabled(on)
        if not on:
            self.confirmed = {k: None for k in self.confirmed}
            self.target = dict(self.confirmed)
            self.lbl_busy.setText("")
        self._update_run_button()
        self._refresh_dirty()
        self._update_derived()

    def send(self, cmd, origin="user", meta=None, timeout=2.0):
        if not self.link.is_open:
            self._status("Not connected", "err")
            return
        self.link.send(cmd, origin, meta, timeout)

    def _on_pending(self, n):
        self.lbl_busy.setText(f"⏳ {n} command{'s' if n != 1 else ''} queued" if n else "")

    # ------------------------------------------------------------- widgets
    def _widget_cfg(self) -> GenConfig:
        return GenConfig(output=max(1, self.grp_out.checkedId()),
                         wave=WAVES[max(0, self.grp_wave.checkedId())],
                         freq=self.sp_freq.value(), phase=round(self.sp_phase.value(), 2),
                         gain=round(self.sp_gain.value(), 3), offset=self.sp_offset.value())

    def _set_widgets(self, cfg: GenConfig, fields=PARAM_FIELDS):
        self._guard = True
        try:
            if "output" in fields and cfg.output in (1, 2, 3, 4):
                self.grp_out.button(cfg.output).setChecked(True)
            if "wave" in fields and cfg.wave in WAVES:
                self.grp_wave.button(WAVES.index(cfg.wave)).setChecked(True)
            if "freq" in fields:
                self.sp_freq.setValue(int(min(cfg.freq, self.sp_freq.maximum())))
            if "phase" in fields:
                self.sp_phase.setValue(math.fmod(cfg.phase, 360.0) % 360.0)
            if "gain" in fields:
                self.sp_gain.setValue(cfg.gain)
            if "offset" in fields:
                self.sp_offset.setValue(int(cfg.offset))
        finally:
            self._guard = False
        self._refresh_dirty()
        self._update_derived()

    def _set_widget_field(self, f, value):
        cfg = self._widget_cfg()
        setattr(cfg, f, value)
        self._set_widgets(cfg, (f,))

    def _param_edited(self, f):
        if self._guard:
            return
        self._refresh_dirty()
        self._update_derived()
        if self.chk_auto.isChecked() and self.link.is_open:
            self._apply_timer.start()

    @staticmethod
    def _same(f, a, b):
        if a is None or b is None:
            return False
        if f == "phase":
            return round(a, 2) == round(b, 2)
        if f == "gain":
            return round(a, 3) == round(b, 3)
        return a == b

    def _cmd_for(self, f, v):
        if f == "phase":
            return f"phase {float(v):.2f}"
        if f == "gain":
            return f"gain {float(v):.3f}"
        if f in ("freq", "offset", "output"):
            return f"{f} {int(v)}"
        return f"wave {v}"

    def apply_settings(self, force=False):
        self._apply_timer.stop()
        if not self.link.is_open:
            return
        cfg = self._widget_cfg()
        for f in PARAM_FIELDS:
            v = getattr(cfg, f)
            if force or not self._same(f, v, self.target.get(f)):
                self.send(self._cmd_for(f, v), "gui", {"field": f, "value": v})
                self.target[f] = v

    def revert_widgets(self):
        vals = {f: self.confirmed[f] for f in PARAM_FIELDS if self.confirmed[f] is not None}
        if not vals:
            return
        cfg = self._widget_cfg()
        for f, v in vals.items():
            setattr(cfg, f, v)
        self.target.update(vals)
        self._set_widgets(cfg, tuple(vals))

    def _refresh_dirty(self):
        c = colors(self)
        cfg = self._widget_cfg()
        connected = self.link.is_open
        dirty = [f for f in PARAM_FIELDS
                 if connected and not self._same(f, getattr(cfg, f), self.confirmed.get(f))]
        for f, lab in self.param_labels.items():
            if f in dirty:
                lab.setText(f"<span style='color:{c['warn'].name()}'>● </span>{f.capitalize()}")
                lab.setToolTip("Not yet confirmed by the device"
                               + (" (value unknown – firmware cannot print floats)"
                                  if self.confirmed.get(f) is None else ""))
            else:
                lab.setText(f.capitalize())
                lab.setToolTip("")
        n = len(dirty)
        self.btn_apply.setText(f"Apply ({n})" if n else "Apply")
        self.btn_apply.setEnabled(connected and n > 0)
        self.btn_revert.setEnabled(connected and n > 0)

    # ------------------------------------------------------------- derived
    def _fclk(self) -> int:
        return int(round(self.sp_fclk.value() * 1e6))

    def _update_derived(self):
        cfg = self._widget_cfg()
        fclk = self._fclk()
        c = colors(self)
        warn = c["warn"].name()
        muted = c["muted"].name()
        f_act = actual_frequency(cfg, fclk)
        lines, warnings = [], []

        # frequency info under the slider
        if cfg.wave == "sine":
            tw, msb, lsb, _ = dds_tuning(cfg.freq, fclk)
            res = fclk / 2 ** 24
            err = f_act - cfg.freq
            self.lbl_freq_info.setText(
                f"<span style='color:{muted}'>actual {fmt_hz(f_act)}  ({err:+.3f} Hz) · "
                f"DDS resolution {res:.3f} Hz</span>")
            lines.append(f"<b>Mode</b>: DDS sine · tuning word <tt>0x{tw:06X}</tt> "
                         f"(TW32 <tt>0x{msb:04X}</tt>, TW1 <tt>0x{lsb:04X}</tt>)")
            if tw == 0:
                warnings.append(f"Below the DDS resolution ({res:.2f} Hz): tuning word is 0, "
                                "the output will not move.")
            if cfg.freq > fclk / 2:
                warnings.append("Above Nyquist (f<sub>clk</sub>/2): the output will alias.")
            elif cfg.freq > fclk * 0.4:
                warnings.append("Close to Nyquist: expect strong images unless filtered.")
        elif cfg.wave in ("triangle", "saw"):
            step, st, _ = saw_setup(cfg.freq, cfg.wave, fclk)
            lo, hi = saw_range(cfg.wave, fclk)
            self.lbl_freq_info.setText(
                f"<span style='color:{muted}'>actual {fmt_hz(f_act)} · SAW_STEP {step}/63 · "
                f"range {lo:.1f} Hz … {hi / 1e3:.3f} kHz</span>")
            lines.append(f"<b>Mode</b>: prestored sawtooth "
                         f"({'triangle' if cfg.wave == 'triangle' else 'ramp up'}), "
                         f"SAW_STEP = {step}")
            if not lo * 0.999 <= cfg.freq <= hi * 1.001:
                warnings.append(f"{cfg.wave.capitalize()} frequency is limited to "
                                f"{fmt_hz(lo)} … {fmt_hz(hi)} with this clock; "
                                f"the firmware clamps to {fmt_hz(f_act)}.")
            elif abs(f_act - cfg.freq) / cfg.freq > 0.02:
                warnings.append(f"Coarse frequency steps: {fmt_hz(cfg.freq)} becomes "
                                f"{fmt_hz(f_act)} ({(f_act / cfg.freq - 1) * 100:+.1f} %).")
        else:
            self.lbl_freq_info.setText(f"<span style='color:{muted}'>not used for DC</span>")
            lines.append("<b>Mode</b>: constant (DACx_CST = 0x800) + digital offset")

        pw, p_act = phase_word(cfg.phase)
        self.lbl_phase_info.setText(
            f"<span style='color:{muted}'>word <tt>0x{pw:04X}</tt> → {p_act:.3f}°"
            + ("" if cfg.wave == "sine" else " · only affects the DDS sine") + "</span>")
        gc, greg = gain_code(cfg.gain)
        self.lbl_gain_info.setText(
            f"<span style='color:{muted}'>code {gc:+d} (reg <tt>0x{greg:04X}</tt>) → "
            f"{gc / 1024:+.4f}</span>")
        oc, oreg = offset_code(cfg.offset)
        self.lbl_offset_info.setText(
            f"<span style='color:{muted}'>reg <tt>0x{oreg:04X}</tt> · {oc / 2048 * 100:+.1f} % "
            f"of full scale</span>")

        if cfg.wave != "dc":
            lines.append(f"<b>Output frequency</b>: {fmt_hz(f_act)} "
                         f"(period {fmt_time(1 / f_act) if f_act else '—'})")
        lines.append("<b>Note</b>: the DDS tuning word and pattern registers are shared by all "
                     "four DACs; output, gain, offset and phase registers are per DAC.")
        if warnings:
            lines += [f"<span style='color:{warn}'>⚠ {w}</span>" for w in warnings]
        self.lbl_summary.setText("<br>".join(lines))

        writes = apply_config_writes(cfg, lambda r: self.reg_values.get(r, 0), fclk)
        txt = ["Register writes per parameter command (AD9106_ApplyConfig):"]
        for a, v in writes:
            txt.append(f"  0x{a:04X}  {reg_label(a):<13} ← 0x{v:04X}")
        if not any(r in self.reg_values for r in (0x26, 0x27, 0x36, 0x37)):
            txt.append("  (WAV/SAW config are read-modify-write; read them for exact values)")
        self.txt_writes.setPlainText("\n".join(txt))
        self.preview.set_state(cfg, f_act, self.confirmed.get("run"))

    # ----------------------------------------------------------- run/reset
    def _on_run_clicked(self):
        on = self.btn_run.isChecked()
        self.send(f"run {1 if on else 0}", "user")
        self._update_run_button()

    def _update_run_button(self):
        c = colors(self)
        st = self.confirmed.get("run")
        self.btn_run.blockSignals(True)
        self.btn_run.setChecked(bool(st))
        self.btn_run.blockSignals(False)
        if st is None:
            self.btn_run.setText("Output: unknown")
            self.btn_run.setStyleSheet("")
        elif st:
            self.btn_run.setText("■  Running – click to stop")
            self.btn_run.setStyleSheet(f"QPushButton {{ background:{c['ok'].name()}; color:white;"
                                       f" font-weight:600; padding:5px 10px; }}")
        else:
            self.btn_run.setText("▶  Stopped – click to run")
            self.btn_run.setStyleSheet("QPushButton { font-weight:600; padding:5px 10px; }")
        self.preview.running = st
        self.preview.update()

    def reset_device(self):
        self.send("reset", "user", {"reapply": self.chk_reapply.isChecked()})

    # --------------------------------------------------------- registers
    def read_all_registers(self):
        for a, _, _ in REGISTERS:
            self.send(f"read 0x{a:04X}", "regs")

    def read_selected_registers(self):
        rows = sorted({i.row() for i in self.tbl.selectedItems()})
        for r in rows:
            self.send(f"read 0x{self.tbl.item(r, 0).data(Qt.UserRole):04X}", "regs")

    def _filter_regs(self, text):
        t = text.strip().lower()
        for r in range(self.tbl.rowCount()):
            hay = " ".join(self.tbl.item(r, c).text().lower() for c in (0, 1, 3))
            self.tbl.setRowHidden(r, bool(t) and t not in hay)

    def _set_reg_value(self, a, v):
        old = self.reg_values.get(a)
        self.reg_values[a] = v
        c = colors(self)
        for r in range(self.tbl.rowCount()):
            if self.tbl.item(r, 0).data(Qt.UserRole) == a:
                it = self.tbl.item(r, 2)
                it.setText(f"0x{v:04X}")
                it.setToolTip(decode_register(a, v, self._fclk(), self.reg_values))
                if old is not None and old != v:
                    it.setForeground(c["warn"])
                    f = it.font()
                    f.setBold(True)
                    it.setFont(f)
                else:
                    it.setForeground(self.palette().color(QPalette.Text))
                    f = it.font()
                    f.setBold(False)
                    it.setFont(f)
                break
        if self._editor_addr() == a and not self.ed_val.hasFocus():
            self._set_editor_value(v)

    def _clear_reg_values(self):
        self.reg_values.clear()
        for r in range(self.tbl.rowCount()):
            self.tbl.item(r, 2).setText("—")
            self.tbl.item(r, 2).setToolTip("")

    def _editor_addr(self):
        t = self.ed_addr.text().strip()
        try:
            return REG_BY_NAME[t.upper()] if t.upper() in REG_BY_NAME else parse_int(t) & 0xFFFF
        except ValueError:
            return None

    def _editor_val(self):
        try:
            return parse_int(self.ed_val.text()) & 0xFFFF
        except ValueError:
            return None

    def _on_reg_selected(self):
        items = self.tbl.selectedItems()
        if not items:
            return
        a = self.tbl.item(items[0].row(), 0).data(Qt.UserRole)
        self.ed_addr.setText(f"0x{a:04X}")
        self._on_addr_edited()

    def _on_addr_edited(self):
        a = self._editor_addr()
        if a is None:
            self.lbl_reg_name.setText("<span style='color:red'>invalid address</span>")
            return
        self.ed_addr.setText(f"0x{a:04X}")
        n, d = REG_BY_ADDR.get(a, ("", "not in register map"))
        self.lbl_reg_name.setText(f"<b>{n}</b> {d}" if n else d)
        self._set_editor_value(self.reg_values.get(a, self._editor_val() or 0))

    def _set_editor_value(self, v):
        self.ed_val.setText(f"0x{v:04X}")
        self._sync_bits(v)

    def _on_val_edited(self, _):
        v = self._editor_val()
        if v is not None:
            self._sync_bits(v)

    def _sync_bits(self, v):
        for bit, b in self.bit_buttons:
            b.blockSignals(True)
            b.setChecked(bool(v >> bit & 1))
            b.setText("1" if v >> bit & 1 else "0")
            b.blockSignals(False)
        self.lbl_val_dec.setText(f"= {v}   0b{v:016b}")
        a = self._editor_addr()
        dec = decode_register(a, v, self._fclk(), self.reg_values) if a is not None else ""
        self.lbl_decode.setText(dec)

    def _on_bit_clicked(self):
        v = 0
        for bit, b in self.bit_buttons:
            if b.isChecked():
                v |= 1 << bit
        self.ed_val.setText(f"0x{v:04X}")
        self._sync_bits(v)

    def reg_read_current(self):
        a = self._editor_addr()
        if a is not None:
            self.send(f"read 0x{a:04X}", "regs")

    def reg_write_current(self):
        a, v = self._editor_addr(), self._editor_val()
        if a is None or v is None:
            self._status("Invalid address or value", "err")
            return
        self.send(f"write 0x{a:04X} 0x{v:04X}", "regs")
        if self.chk_reg_update.isChecked():
            self.send("update", "regs")
        self.send(f"read 0x{a:04X}", "regs")

    def open_script(self):
        initial = ("# Example: write register values, then update\n"
                   "# PAT_STATUS 0x0001\n")
        dlg = ScriptDialog(self, initial)
        if dlg.exec() == QDialog.Accepted:
            for a, v in dlg.writes:
                self.send(f"write 0x{a:04X} 0x{v:04X}", "regs")
            if dlg.chk_update.isChecked():
                self.send("update", "regs")
            self._log("info", f"register script: {len(dlg.writes)} writes queued")

    def export_registers(self):
        if not self.reg_values:
            QMessageBox.information(self, "Export", "Read the registers first.")
            return
        fn, _ = QFileDialog.getSaveFileName(
            self, "Export registers",
            f"ad9106_registers_{_dt.datetime.now():%Y%m%d_%H%M%S}.txt",
            "Register script (*.txt);;CSV (*.csv)")
        if not fn:
            return
        items = sorted(self.reg_values.items())
        if fn.lower().endswith(".csv"):
            with open(fn, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["address", "name", "value"])
                for a, v in items:
                    w.writerow([f"0x{a:04X}", reg_label(a), f"0x{v:04X}"])
        else:
            with open(fn, "w", encoding="utf-8") as f:
                f.write(f"# AD9106 register snapshot {_dt.datetime.now():%Y-%m-%d %H:%M:%S}\n")
                f.write("# Load with Registers → Register script… to write it back.\n")
                for a, v in items:
                    if a in (0x1D, 0x1E, 0x60):      # don't replay update/run/error flags
                        f.write(f"# 0x{a:04X} 0x{v:04X}   # {reg_label(a)}\n")
                    else:
                        f.write(f"0x{a:04X} 0x{v:04X}   # {reg_label(a)}\n")
        self._status(f"Saved {fn}")

    # --------------------------------------------------------- responses
    def _on_response(self, r: CliResponse):
        parts = r.cmd.split()
        name = parts[0] if parts else ""
        args = parts[1:]
        if r.timed_out:
            self._status(f"No response to '{r.cmd}'", "err")
        elif not r.ok:
            self._status(r.text.splitlines()[-1] if r.text else "error", "err")

        good = r.ok and not r.timed_out
        if name in PARAM_FIELDS:
            self._handle_param_response(name, args, r, good)
        elif name == "status" and good:
            self._handle_status(r)
        elif name == "read" and good:
            m = re.search(r"REG\[0x([0-9A-Fa-f]+)\]\s*=\s*0x([0-9A-Fa-f]+)", r.text)
            if m:
                a, v = int(m.group(1), 16), int(m.group(2), 16)
                self._set_reg_value(a, v)
                if a == 0x1E:
                    self.confirmed["run"] = bool(v & 1)
                    self._update_run_button()
                if a in (0x26, 0x27, 0x36, 0x37, 0x3E, 0x3F):
                    self._update_derived()
        elif name == "write" and good:
            m = re.search(r"OK write 0x([0-9A-Fa-f]+)\s*=\s*0x([0-9A-Fa-f]+)", r.text)
            if m:
                a, v = int(m.group(1), 16), int(m.group(2), 16)
                if a == 0x1E:
                    self.confirmed["run"] = bool(v & 1)
                    self._update_run_button()
        elif name == "run":
            if good:
                m = re.search(r"OK run (\d)", r.text)
                self.confirmed["run"] = bool(int(m.group(1))) if m else None
                self._status("Output running" if self.confirmed["run"] else "Output stopped")
            self._update_run_button()
        elif name == "reset" and good:
            self.confirmed["run"] = False
            self._clear_reg_values()
            self._update_run_button()
            self._status("AD9106 reset – chip registers are back at their defaults")
            if r.meta.get("reapply") and self.confirmed.get("output"):
                # every parameter command re-runs AD9106_ApplyConfig with the
                # firmware's stored settings
                self.send(f"output {self.confirmed['output']}", "sync")
                self._log("info", "re-applying settings after reset "
                                  "(press Run to restart the output)")
        elif name in ("update",) and good:
            self._status("Update sent (shadow → active registers)")
        self._refresh_dirty()
        self._update_derived()

    def _handle_param_response(self, name, args, r, good):
        f = name
        if good:
            v = None
            pat = {"output": r"OK output (\d+)", "wave": r"OK wave (\w+)",
                   "freq": r"OK freq (\d+)", "offset": r"OK offset (-?\d+)",
                   "phase": r"OK phase (-?[\d.]+)", "gain": r"OK gain (-?[\d.]+)"}[f]
            m = re.search(pat, r.text)
            if m:
                g = m.group(1)
                v = g if f == "wave" else (float(g) if f in ("phase", "gain") else int(g))
            elif f in ("phase", "gain") and args:
                v = _c_atof(args[0])      # firmware printed an empty %f
                self._note_floats()
            elif r.origin == "gui":
                v = r.meta.get("value")
            if f == "phase" and v is not None:
                v = math.fmod(v, 360.0) % 360.0
            self.confirmed[f] = v
            if r.origin != "gui":
                self.target[f] = v
                if v is not None:
                    self._set_widget_field(f, v)
        else:
            self.target[f] = self.confirmed[f]
            if r.origin == "gui" and self.confirmed[f] is not None:
                self._set_widget_field(f, self.confirmed[f])

    def _handle_status(self, r):
        rx = {"output": r"output\s*:\s*(\d+)", "wave": r"wave\s*:\s*(\w+)",
              "freq": r"freq\s*:\s*(\d+)", "phase": r"phase\s*:\s*(-?[\d.]+)\s*deg",
              "gain": r"gain\s*:\s*(-?[\d.]+)", "offset": r"offset\s*:\s*(-?\d+)"}
        got = {}
        for f, pat in rx.items():
            m = re.search(pat, r.text)
            if m:
                g = m.group(1)
                got[f] = g if f == "wave" else (float(g) if f in ("phase", "gain") else int(g))
        if re.search(r"phase\s*:\s*deg", r.text) or re.search(r"gain\s*:\s*\n", r.text + "\n"):
            self._note_floats()
        self.confirmed.update(got)
        self.target.update(got)
        if got:
            cfg = self._widget_cfg()
            for f, v in got.items():
                setattr(cfg, f, v)
            self._set_widgets(cfg, tuple(got))
        missing = [f for f in PARAM_FIELDS if f not in got]
        if missing:
            self._status(f"Status read; not reported by firmware: {', '.join(missing)}", "warn")
        else:
            self._status("Status read")

    def _note_floats(self):
        if self._float_note_shown:
            return
        self._float_note_shown = True
        self._log("info", "note: this firmware build prints floats as empty strings "
                          "(newlib-nano printf). Phase and gain cannot be read back with 'status'; "
                          "the GUI uses the values it sent. Add '-u _printf_float' to the linker "
                          "flags to fix.")

    # ----------------------------------------------------------- terminal
    def _on_traffic(self, kind, text):
        self._log(kind, text)

    def _log(self, kind, text):
        c = colors(self)
        ts = (f"<span style='color:{c['muted'].name()}'>{_dt.datetime.now():%H:%M:%S.%f}"[:-3]
              + " </span>") if self.chk_ts.isChecked() else ""
        if kind == "rx":
            t = text.replace("\r\n", "\n").replace("\r", "\n")
            t = re.sub(r"(^|\n)> $", "", t).rstrip("\n> ").lstrip("\n")
            if not t.strip():
                return
            body = html.escape(t).replace("\n", "<br>")
            style = "white-space:pre;"
            if re.search(r"^ERR", t, re.M):
                style += f"color:{c['err'].name()};"
            self.log.append(f"{ts}<span style='{style}'>{body}</span>")
        elif kind == "tx":
            self.log.append(f"{ts}<span style='color:{c['accent'].name()}'><b>&gt; "
                            f"{html.escape(text)}</b></span>")
        elif kind == "err":
            self.log.append(f"{ts}<span style='color:{c['err'].name()}'>✖ {html.escape(text)}</span>")
        else:
            self.log.append(f"{ts}<span style='color:{c['muted'].name()}'><i>— "
                            f"{html.escape(text)}</i></span>")
        sb = self.log.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _send_raw(self):
        cmd = self.ed_cmd.text().strip()
        if not cmd:
            return
        self._history.append(cmd)
        self._hist_pos = len(self._history)
        self.ed_cmd.clear()
        if len(cmd) > 127:
            self._status("Command longer than the firmware's 127-character line buffer", "err")
            return
        self.send(cmd, "raw")

    def eventFilter(self, obj, ev):
        if obj is self.ed_cmd and ev.type() == ev.Type.KeyPress and self._history:
            if ev.key() == Qt.Key_Up:
                self._hist_pos = max(0, self._hist_pos - 1)
                self.ed_cmd.setText(self._history[self._hist_pos])
                return True
            if ev.key() == Qt.Key_Down:
                self._hist_pos = min(len(self._history), self._hist_pos + 1)
                self.ed_cmd.setText(self._history[self._hist_pos]
                                    if self._hist_pos < len(self._history) else "")
                return True
        return super().eventFilter(obj, ev)

    def _save_log(self):
        fn, _ = QFileDialog.getSaveFileName(self, "Save log",
                                            f"ad9106_log_{_dt.datetime.now():%Y%m%d_%H%M%S}.txt",
                                            "Text (*.txt)")
        if fn:
            with open(fn, "w", encoding="utf-8") as f:
                f.write(self.log.toPlainText())

    def _status(self, text, level="info"):
        c = colors(self)
        col = {"err": c["err"], "warn": c["warn"]}.get(level)
        self.statusBar().setStyleSheet(f"QStatusBar {{ color: {col.name()}; }}" if col else "")
        self.statusBar().showMessage(text, 8000)

    # ------------------------------------------------------------ presets
    def save_preset(self):
        fn, _ = QFileDialog.getSaveFileName(self, "Save preset", "ad9106_preset.json",
                                            "Preset (*.json)")
        if not fn:
            return
        data = self._widget_cfg().to_dict()
        data["run"] = bool(self.confirmed.get("run"))
        data["dac_clock_hz"] = self._fclk()
        with open(fn, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        self._status(f"Preset saved: {fn}")

    def load_preset(self):
        fn, _ = QFileDialog.getOpenFileName(self, "Load preset", "", "Preset (*.json)")
        if not fn:
            return
        try:
            with open(fn, encoding="utf-8") as f:
                data = json.load(f)
            cfg = GenConfig.from_dict(data)
        except Exception as e:
            QMessageBox.warning(self, "Load preset", f"Could not read preset:\n{e}")
            return
        self._set_widgets(cfg)
        if self.link.is_open:
            self.apply_settings(False)
            if "run" in data:
                self.send(f"run {1 if data['run'] else 0}", "user")
        self._status(f"Preset loaded: {fn}")

    def show_cli_reference(self):
        rows = [
            ("help", "Firmware command list", "Firmware help button"),
            ("status", "Current settings (output, wave, freq, phase, gain, offset)", "Read status / F5"),
            ("read &lt;reg&gt;", "Read a 16-bit register", "Registers tab → Read"),
            ("write &lt;reg&gt; &lt;value&gt;", "Write a 16-bit register", "Registers tab → Write / script"),
            ("update", "RAMUPDATE: shadow → active registers", "Update button"),
            ("run &lt;0|1&gt;", "Stop / start the pattern generator", "Run button / F9"),
            ("reset", "Hardware reset of the AD9106", "Reset AD9106"),
            ("output &lt;1..4&gt;", "Select the DAC the settings apply to", "DAC 1–4 buttons"),
            ("wave &lt;sine|triangle|saw|dc&gt;", "Waveform source", "Waveform buttons"),
            ("freq &lt;hz&gt;", "Frequency (integer Hz)", "Frequency"),
            ("phase &lt;deg&gt;", "DDS phase offset", "Phase"),
            ("gain &lt;value&gt;", "Digital gain −2.0 … 1.999", "Gain"),
            ("offset &lt;value&gt;", "Digital offset −2048 … 2047", "Offset"),
        ]
        body = "".join(f"<tr><td><tt>{a}</tt></td><td>{b}</td><td>{c}</td></tr>" for a, b, c in rows)
        QMessageBox.information(
            self, "CLI reference",
            "<table cellspacing=6><tr><th align=left>Command</th><th align=left>Function</th>"
            f"<th align=left>In the GUI</th></tr>{body}</table>"
            "<p>Every parameter command (output … offset) re-runs AD9106_ApplyConfig, which "
            "rewrites all registers for the selected DAC and sends an update.</p>")


def apply_app_style(app: QApplication):
    app.setStyle("Fusion")
    accent = "#4ea1ff" if app.palette().color(QPalette.Window).lightness() < 128 else "#1565c0"
    app.setStyleSheet(
        "QPushButton:checked, QToolButton:checked {"
        " background: %s; color: white; font-weight: 600;"
        " border: 1px solid %s; border-radius: 3px; }"
        "QPushButton:checkable { padding: 4px; min-height: 18px; }" % (accent, accent))


def main():
    ap = argparse.ArgumentParser(description=APP_TITLE)
    ap.add_argument("--port", help="serial port to connect to on start (e.g. COM7)")
    ap.add_argument("--sim", action="store_true", help="start connected to the simulator")
    args = ap.parse_args()
    app = QApplication(sys.argv)
    app.setApplicationName(APP)
    app.setOrganizationName(ORG)
    apply_app_style(app)
    win = MainWindow(args)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
