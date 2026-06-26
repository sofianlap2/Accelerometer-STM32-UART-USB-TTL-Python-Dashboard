"""
OpCode Labs – Serial LED Dashboard  (v2 — Accelero Edition)
════════════════════════════════════════════════════════════
Frame format  PC→STM32 / STM32→PC:
  [SOF=0x41('A')][CMD=1B][CONFIG=2B LE][DATA=4B LE]  = 8 bytes

PC → STM32 commands:
  CMD 0x31  SET_LED1   CONFIG=freq×10  DATA=state
  CMD 0xD2  SET_LED2   CONFIG=0        DATA=state
  CMD 0xD3  SET_LED3   CONFIG=0        DATA=state
  CMD 0xD5  SET_THRESH CONFIG=X_thresh DATA[15:0]=Y_thresh

STM32 → PC commands:
  CMD 0xA1  ACCEL_DATA  CONFIG=0  DATA[31:16]=Xval(int16)  DATA[15:0]=Yval(int16)
"""

import sys, struct, datetime, ctypes
import serial, serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QTextEdit, QGroupBox,
    QFrame, QDoubleSpinBox, QLineEdit, QSpinBox, QSplitter,
    QStatusBar
)
from PyQt5.QtCore  import Qt, QThread, pyqtSignal, QRegExp, QRectF, QPointF, QTimer
from PyQt5.QtGui   import (QColor, QTextCharFormat, QBrush, QPainter, QPen,
                            QRadialGradient, QLinearGradient, QRegExpValidator,
                            QFont, QPainterPath)

# ── Protocol constants ────────────────────────────────────────────────────────
SOF            = 0x41          # 'A'  — matches STM32 SOF_PATTERN
FRAME_LEN      = 8

CMD_SET_LED1   = 0x31
CMD_SET_LED2   = 0xD2
CMD_SET_LED3   = 0xD3
CMD_SET_THRESH = 0xD5
CMD_ACCEL      = 0xA1          # STM32 → PC

ACCEL_RANGE    = 2000          # ±2000 mg  (LIS3DSH on Discovery)


def build_frame(cmd: int, config: int, data: int) -> bytes:
    return struct.pack('<BBHI', SOF, cmd, config & 0xFFFF, data & 0xFFFFFFFF)


def s16(val: int) -> int:
    """Reinterpret lower 16 bits as signed."""
    v = val & 0xFFFF
    return ctypes.c_int16(v).value


# ── Serial RX thread ─────────────────────────────────────────────────────────
class SerialReader(QThread):
    raw_received   = pyqtSignal(bytes)
    frame_received = pyqtSignal(int, int, int)   # cmd, config, data
    error_occurred = pyqtSignal(str)

    def __init__(self, port: serial.Serial):
        super().__init__()
        self._port    = port
        self._running = True
        self._buf     = bytearray()

    def run(self):
        while self._running:
            try:
                if self._port.is_open and self._port.in_waiting:
                    chunk = self._port.read(self._port.in_waiting)
                    self.raw_received.emit(bytes(chunk))
                    self._buf.extend(chunk)
                    self._parse()
                self.msleep(10)
            except serial.SerialException as e:
                self.error_occurred.emit(str(e))
                break

    # Known valid CMD bytes — used to reject false SOF hits inside payload
    VALID_CMDS = frozenset([CMD_SET_LED1, CMD_SET_LED2, CMD_SET_LED3,
                            CMD_SET_THRESH, CMD_ACCEL])

    def _parse(self):
        """Scan buffer for complete 8-byte frames.

        Guard against false SOF resync: SOF=0x41 can appear inside CONFIG/DATA
        bytes. Validate CMD byte before accepting a frame start position.
        """
        buf = self._buf
        i = 0
        while i < len(buf):
            # Find SOF candidate
            if buf[i] != SOF:
                i += 1
                continue
            # Need at least CMD byte too
            if i + 1 >= len(buf):
                break
            # False SOF inside payload — CMD must be a known command
            if buf[i + 1] not in self.VALID_CMDS:
                i += 1
                continue
            # Wait until full 8-byte frame is buffered
            if len(buf) - i < FRAME_LEN:
                break
            _, cmd, config, data = struct.unpack_from('<BBHI', buf, i)
            self.frame_received.emit(cmd, config, data)
            i += FRAME_LEN
        self._buf = buf[i:]

    def stop(self):
        self._running = False
        self.wait()


# ── Board visualiser widget ───────────────────────────────────────────────────
class BoardWidget(QWidget):
    """Draws a top-down PCB that tilts/translates with accelero data."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 300)
        self._x  = 0.0    # normalised −1..+1
        self._y  = 0.0
        self._xt = 0.0    # threshold −1..+1
        self._yt = 0.0
        self._alert = False

    def update_accel(self, x_mg: int, y_mg: int):
        self._x = max(-1.0, min(1.0,  x_mg / ACCEL_RANGE))
        self._y = max(-1.0, min(1.0, -y_mg / ACCEL_RANGE))   # invert Y for screen
        self._alert = (abs(x_mg) > abs(self._xt * ACCEL_RANGE) or
                       abs(y_mg) > abs(self._yt * ACCEL_RANGE))
        self.update()

    def set_thresholds(self, x_mg: int, y_mg: int):
        self._xt = x_mg / ACCEL_RANGE
        self._yt = y_mg / ACCEL_RANGE
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        W, H  = self.width(), self.height()
        cx, cy = W / 2, H / 2
        radius = min(W, H) * 0.42

        # ── Arena circle ──────────────────────────────────────────────────
        arena_color = QColor("#1A1A2E")
        border_color = QColor("#EF5350") if self._alert else QColor("#3A3A5C")
        p.setPen(QPen(border_color, 2))
        p.setBrush(QBrush(arena_color))
        p.drawEllipse(QPointF(cx, cy), radius, radius)

        # ── Cross-hair ────────────────────────────────────────────────────
        p.setPen(QPen(QColor("#2A2A4A"), 1))
        p.drawLine(int(cx - radius), int(cy), int(cx + radius), int(cy))
        p.drawLine(int(cx), int(cy - radius), int(cx), int(cy + radius))

        # ── Threshold ring ────────────────────────────────────────────────
        xt_r = abs(self._xt) * radius
        yt_r = abs(self._yt) * radius
        t_r  = max(xt_r, yt_r, 10)
        p.setPen(QPen(QColor("#F9A825"), 1, Qt.DashLine))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(cx, cy), t_r, t_r)

        # ── PCB board representation ──────────────────────────────────────
        bw, bh = 80, 60
        bx = cx + self._x * (radius - bw * 0.7)
        by = cy + self._y * (radius - bh * 0.7)

        angle = (self._x * 15)   # slight rotation for "tilt" feel

        p.save()
        p.translate(bx, by)
        p.rotate(angle)

        # PCB body
        pcb_color = QColor("#1B5E20") if not self._alert else QColor("#5D1010")
        p.setBrush(QBrush(pcb_color))
        p.setPen(QPen(QColor("#33691E") if not self._alert else QColor("#B71C1C"), 1.5))
        p.drawRoundedRect(-bw//2, -bh//2, bw, bh, 5, 5)

        # MCU chip
        p.setBrush(QBrush(QColor("#263238")))
        p.setPen(QPen(QColor("#607D8B"), 1))
        p.drawRect(-14, -14, 28, 28)

        # MCU label
        p.setPen(QColor("#90A4AE"))
        p.setFont(QFont("Consolas", 5))
        p.drawText(QRectF(-12, -6, 24, 12), Qt.AlignCenter, "STM32")

        # LEDs (top-right corner)
        led_colors = ["#F9A825", "#42A5F5", "#EF5350", "#66BB6A"]
        for i, lc in enumerate(led_colors):
            p.setBrush(QBrush(QColor(lc)))
            p.setPen(Qt.NoPen)
            p.drawEllipse(22 + (i % 2) * 8 - 4, -20 + (i // 2) * 8 - 4, 6, 6)

        # Connector
        p.setBrush(QBrush(QColor("#37474F")))
        p.setPen(QPen(QColor("#546E7A"), 1))
        p.drawRect(-bw//2, -8, 10, 16)

        p.restore()

        # ── X/Y readout ───────────────────────────────────────────────────
        p.setPen(QColor("#8888AA"))
        p.setFont(QFont("Consolas", 9))
        x_mg = int(self._x  * ACCEL_RANGE)
        y_mg = int(-self._y * ACCEL_RANGE)
        p.drawText(6, H - 20, f"X={x_mg:+5d} mg   Y={y_mg:+5d} mg")
        if self._alert:
            p.setPen(QColor("#EF5350"))
            p.setFont(QFont("Consolas", 9, QFont.Bold))
            p.drawText(W - 90, H - 20, "⚠ THRESHOLD")


# ── Theme ─────────────────────────────────────────────────────────────────────
DARK_BG    = "#1E1E2E"
PANEL_BG   = "#252535"
BORDER     = "#3A3A5C"
ACCENT     = "#7C6AF7"
ACCENT2    = "#5BC8AF"
LED1_COLOR = "#F9A825"
LED2_COLOR = "#42A5F5"
LED3_COLOR = "#EF5350"
ACCEL_CLR  = "#A5D6A7"
TEXT_MAIN  = "#E0E0F0"
TEXT_DIM   = "#8888AA"
SUCCESS    = "#66BB6A"
DANGER     = "#EF5350"

STYLE = f"""
QMainWindow, QWidget {{
    background-color: {DARK_BG};
    color: {TEXT_MAIN};
    font-family: 'Consolas', 'Courier New', monospace;
}}
QGroupBox {{
    border: 1px solid {BORDER};
    border-radius: 8px;
    margin-top: 10px;
    padding: 8px;
    font-size: 11px;
    color: {TEXT_DIM};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 0 6px;
    color: {ACCENT};
    font-weight: bold;
}}
QPushButton {{
    border-radius: 6px;
    padding: 8px 16px;
    font-size: 12px;
    font-weight: bold;
    border: none;
}}
QPushButton#btn_led1  {{ background-color:{LED1_COLOR}; color:#1a1a1a; }}
QPushButton#btn_led1:hover  {{ background-color:#FFD54F; }}
QPushButton#btn_led2  {{ background-color:{LED2_COLOR}; color:#1a1a1a; }}
QPushButton#btn_led2:hover  {{ background-color:#90CAF9; }}
QPushButton#btn_led3  {{ background-color:{LED3_COLOR}; color:#fff; }}
QPushButton#btn_led3:hover  {{ background-color:#EF9A9A; }}
QPushButton#btn_thresh {{ background-color:{ACCEL_CLR}; color:#1a1a1a; }}
QPushButton#btn_thresh:hover {{ background-color:#C8E6C9; }}
QPushButton#btn_accel_start {{ background-color:{ACCENT}; color:#fff; }}
QPushButton#btn_accel_start:hover {{ background-color:#9E8DF9; }}
QPushButton#btn_accel_stop  {{ background-color:{BORDER}; color:{TEXT_MAIN}; }}
QPushButton#btn_connect     {{ background-color:{SUCCESS}; color:#1a1a1a; min-width:100px; }}
QPushButton#btn_connect:hover {{ background-color:#A5D6A7; }}
QPushButton#btn_disconnect  {{ background-color:{DANGER};  color:#fff;    min-width:100px; }}
QPushButton#btn_disconnect:hover {{ background-color:#EF9A9A; }}
QPushButton#btn_clear   {{ background-color:{BORDER}; color:{TEXT_MAIN}; }}
QPushButton#btn_refresh {{ background-color:{PANEL_BG}; color:{ACCENT2}; border:1px solid {ACCENT2}; padding:4px 10px; font-size:11px; }}
QPushButton:disabled    {{ background-color:#333350; color:#555570; }}
QComboBox, QDoubleSpinBox, QSpinBox {{
    background-color:{PANEL_BG}; color:{TEXT_MAIN};
    border:1px solid {BORDER}; border-radius:4px; padding:4px 8px; font-size:12px;
}}
QComboBox::drop-down {{ border:none; }}
QComboBox QAbstractItemView {{ background-color:{PANEL_BG}; color:{TEXT_MAIN}; selection-background-color:{ACCENT}; }}
QTextEdit {{
    background-color:#12121E; color:{ACCENT2};
    border:1px solid {BORDER}; border-radius:6px;
    font-family:'Consolas','Courier New',monospace; font-size:11px; padding:6px;
}}
QStatusBar {{ background-color:{PANEL_BG}; color:{TEXT_DIM}; font-size:11px; }}
QSplitter::handle {{ background:{BORDER}; }}
"""


# ── Main window ───────────────────────────────────────────────────────────────
class Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpCode Labs — Serial Dashboard  [Accelero Edition]")
        self.setMinimumSize(1000, 680)

        self._serial : serial.Serial | None = None
        self._reader : SerialReader  | None = None
        self._frame_tx = 0
        self._frame_rx = 0
        self._accel_streaming = False

        self._build_ui()
        self.setStyleSheet(STYLE)
        self._refresh_ports()
        self._set_connected(False)

    # ── UI ────────────────────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(8)
        root.setContentsMargins(12, 12, 12, 8)

        # Connection bar
        root.addWidget(self._make_conn_bar())

        # Main splitter: left controls | right visualiser
        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setSpacing(8)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.addWidget(self._make_led_box())
        left_layout.addWidget(self._make_accel_box())
        left_layout.addStretch()

        splitter.addWidget(left)
        splitter.addWidget(self._make_viz_box())
        splitter.setSizes([420, 400])
        root.addWidget(splitter, 1)

        # Log
        root.addWidget(self._make_log_box())
        self.statusBar().showMessage("Ready — select a COM port and connect.")

    # ── Connection bar ────────────────────────────────────────────────────

    def _make_conn_bar(self):
        box = QGroupBox("Serial Connection")
        lay = QHBoxLayout(box)
        lay.setSpacing(8)

        self.cmb_port = QComboBox(); self.cmb_port.setMinimumWidth(140)
        self.cmb_baud = QComboBox()
        for b in ["9600","19200","38400","57600","115200","230400","460800","921600"]:
            self.cmb_baud.addItem(b)
        self.cmb_baud.setCurrentText("115200"); self.cmb_baud.setMinimumWidth(90)

        btn_ref = QPushButton("⟳ Scan"); btn_ref.setObjectName("btn_refresh")
        btn_ref.clicked.connect(self._refresh_ports)

        self.btn_connect    = QPushButton("Connect");    self.btn_connect.setObjectName("btn_connect")
        self.btn_disconnect = QPushButton("Disconnect"); self.btn_disconnect.setObjectName("btn_disconnect")
        self.btn_connect.clicked.connect(self._connect)
        self.btn_disconnect.clicked.connect(self._disconnect)

        self.lbl_dot  = QLabel("●")
        self.lbl_dot.setStyleSheet("font-size:16px;")
        self.lbl_conn = QLabel("Disconnected")
        self.lbl_conn.setStyleSheet(f"color:{TEXT_DIM}; font-size:12px;")

        for w in [QLabel("Port:"), self.cmb_port, QLabel("Baud:"),
                  self.cmb_baud, btn_ref, self.btn_connect, self.btn_disconnect]:
            lay.addWidget(w)
        lay.addSpacing(12)
        lay.addWidget(self.lbl_dot); lay.addWidget(self.lbl_conn)
        lay.addStretch()
        return box

    # ── LED controls ──────────────────────────────────────────────────────

    def _make_led_box(self):
        box = QGroupBox("LED Commands")
        lay = QHBoxLayout(box); lay.setSpacing(10)
        lay.addWidget(self._led_panel("LED 1", LED1_COLOR, "btn_led1", 0, has_freq=True))
        lay.addWidget(self._led_panel("LED 2", LED2_COLOR, "btn_led2", 1))
        lay.addWidget(self._led_panel("LED 3", LED3_COLOR, "btn_led3", 2))
        return box

    def _led_panel(self, title, color, obj, idx, has_freq=False):
        fr = QFrame()
        fr.setStyleSheet(f"QFrame{{background:{PANEL_BG};border:1px solid {BORDER};border-radius:8px;padding:6px;}}")
        lay = QVBoxLayout(fr); lay.setSpacing(5)

        lbl = QLabel(title); lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet(f"color:{color};font-size:13px;font-weight:bold;border:none;")
        lay.addWidget(lbl)

        if has_freq:
            row = QHBoxLayout()
            row.addWidget(self._dim("Freq Hz:"))
            self.spin_freq = QDoubleSpinBox()
            self.spin_freq.setRange(0.1, 100.0); self.spin_freq.setValue(1.0)
            self.spin_freq.setDecimals(1); self.spin_freq.setFixedWidth(75)
            row.addWidget(self.spin_freq); row.addStretch()
            lay.addLayout(row)

        row2 = QHBoxLayout()
        row2.addWidget(self._dim("Data:"))
        he = QLineEdit("00000001"); he.setMaxLength(8); he.setFixedWidth(90)
        he.setValidator(QRegExpValidator(QRegExp("[0-9A-Fa-f]{1,8}")))
        he.setStyleSheet(f"background:{PANEL_BG};color:{TEXT_MAIN};border:1px solid {BORDER};"
                         f"border-radius:4px;padding:3px 6px;font-size:11px;")
        he.setPlaceholderText("hex")
        setattr(self, f"hex_data{idx+1}", he)
        row2.addWidget(he); row2.addStretch()
        lay.addLayout(row2)

        btn = QPushButton(f"▶ SET LED{idx+1}"); btn.setObjectName(obj)
        btn.clicked.connect(lambda _, i=idx: self._send_led(i))
        setattr(self, f"btn_led{idx+1}", btn)
        lay.addWidget(btn)
        return fr

    # ── Accelero box ──────────────────────────────────────────────────────

    def _make_accel_box(self):
        box = QGroupBox("Accelerometer Configuration")
        lay = QVBoxLayout(box); lay.setSpacing(8)

        # Threshold row
        thr_row = QHBoxLayout()
        thr_row.addWidget(self._dim("X thresh (mg):"))
        self.spin_xthr = QSpinBox(); self.spin_xthr.setRange(0, 2000); self.spin_xthr.setValue(500)
        self.spin_xthr.setFixedWidth(75)
        thr_row.addWidget(self.spin_xthr)
        thr_row.addSpacing(12)
        thr_row.addWidget(self._dim("Y thresh (mg):"))
        self.spin_ythr = QSpinBox(); self.spin_ythr.setRange(0, 2000); self.spin_ythr.setValue(500)
        self.spin_ythr.setFixedWidth(75)
        thr_row.addWidget(self.spin_ythr)
        thr_row.addStretch()
        lay.addLayout(thr_row)

        btn_thr = QPushButton("📡  Send Thresholds"); btn_thr.setObjectName("btn_thresh")
        btn_thr.clicked.connect(self._send_thresholds)
        self.btn_thresh = btn_thr
        lay.addWidget(btn_thr)

        # Live values display
        val_row = QHBoxLayout()
        self.lbl_xval = QLabel("X:    0 mg")
        self.lbl_yval = QLabel("Y:    0 mg")
        for l in [self.lbl_xval, self.lbl_yval]:
            l.setStyleSheet(f"color:{ACCEL_CLR};font-size:12px;font-family:Consolas;")
        val_row.addWidget(self.lbl_xval); val_row.addSpacing(20)
        val_row.addWidget(self.lbl_yval); val_row.addStretch()
        lay.addLayout(val_row)

        return box

    # ── Visualiser box ────────────────────────────────────────────────────

    def _make_viz_box(self):
        box = QGroupBox("Board Motion Simulator")
        lay = QVBoxLayout(box)
        self.board_widget = BoardWidget()
        lay.addWidget(self.board_widget)

        info = QLabel("Board position = live accelero  │  Dashed ring = threshold")
        info.setStyleSheet(f"color:{TEXT_DIM};font-size:10px;")
        info.setAlignment(Qt.AlignCenter)
        lay.addWidget(info)
        return box

    # ── Log box ───────────────────────────────────────────────────────────

    def _make_log_box(self):
        box = QGroupBox("Frame Log"); lay = QVBoxLayout(box)
        hdr = QHBoxLayout()
        self.lbl_tx = QLabel("TX: 0"); self.lbl_tx.setStyleSheet(f"color:{TEXT_DIM};font-size:11px;")
        self.lbl_rx = QLabel("RX: 0"); self.lbl_rx.setStyleSheet(f"color:{ACCENT2};font-size:11px;")
        btn_cl = QPushButton("Clear"); btn_cl.setObjectName("btn_clear")
        btn_cl.setFixedWidth(65); btn_cl.clicked.connect(self._clear_log)
        hdr.addWidget(self.lbl_tx); hdr.addSpacing(16)
        hdr.addWidget(self.lbl_rx); hdr.addStretch(); hdr.addWidget(btn_cl)
        lay.addLayout(hdr)
        self.log = QTextEdit(); self.log.setReadOnly(True); self.log.setMaximumHeight(160)
        lay.addWidget(self.log)
        return box

    # ── Helpers ───────────────────────────────────────────────────────────

    def _dim(self, txt):
        l = QLabel(txt); l.setStyleSheet(f"color:{TEXT_DIM};font-size:11px;")
        return l

    def _refresh_ports(self):
        self.cmb_port.clear()
        ports = serial.tools.list_ports.comports()
        for p in sorted(ports):
            self.cmb_port.addItem(f"{p.device}  —  {p.description}", p.device)
        if not ports:
            self.cmb_port.addItem("No ports found")

    def _set_connected(self, state: bool):
        self.btn_connect.setEnabled(not state)
        self.btn_disconnect.setEnabled(state)
        self.cmb_port.setEnabled(not state)
        self.cmb_baud.setEnabled(not state)
        for i in range(1, 4):
            getattr(self, f"btn_led{i}").setEnabled(state)
        self.btn_thresh.setEnabled(state)
        dot_color = SUCCESS if state else DANGER
        self.lbl_dot.setStyleSheet(f"color:{dot_color};font-size:16px;")
        if state:
            port = self.cmb_port.currentData() or self.cmb_port.currentText().split()[0]
            self.lbl_conn.setText(f"Connected  [{port}]")
            self.lbl_conn.setStyleSheet(f"color:{SUCCESS};font-size:12px;")
        else:
            self.lbl_conn.setText("Disconnected")
            self.lbl_conn.setStyleSheet(f"color:{TEXT_DIM};font-size:12px;")

    # ── Connect / Disconnect ──────────────────────────────────────────────

    def _connect(self):
        port = self.cmb_port.currentData() or self.cmb_port.currentText().split()[0]
        baud = int(self.cmb_baud.currentText())
        try:
            self._serial = serial.Serial(port, baud, timeout=0.1)
            self._reader = SerialReader(self._serial)
            self._reader.raw_received.connect(self._on_raw_rx)
            self._reader.frame_received.connect(self._on_frame_rx)
            self._reader.error_occurred.connect(self._on_serial_error)
            self._reader.start()
            self._set_connected(True)
            self.statusBar().showMessage(f"Connected  {port} @ {baud}")
            self._log_info(f"[CONNECT] {port} @ {baud} baud")
        except serial.SerialException as e:
            self._log_error(f"[ERROR] {e}")

    def _disconnect(self):
        if self._reader:  self._reader.stop();  self._reader = None
        if self._serial and self._serial.is_open: self._serial.close()
        self._serial = None
        self._set_connected(False)
        self.statusBar().showMessage("Disconnected.")
        self._log_info("[DISCONNECT]")

    # ── TX ────────────────────────────────────────────────────────────────

    def _send_frame(self, cmd, config, data):
        if not self._serial or not self._serial.is_open:
            self._log_error("[TX] Not connected"); return False
        frame = build_frame(cmd, config, data)
        try:
            self._serial.write(frame)
            self._frame_tx += 1
            self.lbl_tx.setText(f"TX: {self._frame_tx}")
            return frame
        except serial.SerialException as e:
            self._log_error(f"[TX ERROR] {e}"); return False

    def _send_led(self, idx: int):
        he = getattr(self, f"hex_data{idx+1}")
        try:
            data_val = int(he.text() or "0", 16) & 0xFFFFFFFF
        except ValueError:
            self._log_error("[TX] Bad hex"); return

        if idx == 0:
            config = int(self.spin_freq.value() * 10) & 0xFFFF
            cmd = CMD_SET_LED1
        elif idx == 1:
            config, cmd = 0, CMD_SET_LED2
        else:
            config, cmd = 0, CMD_SET_LED3

        frame = self._send_frame(cmd, config, data_val)
        if frame:
            names = {CMD_SET_LED1:"SET_LED1", CMD_SET_LED2:"SET_LED2", CMD_SET_LED3:"SET_LED3"}
            hex_s = " ".join(f"{b:02X}" for b in frame)
            colors= [LED1_COLOR, LED2_COLOR, LED3_COLOR]
            self._log(f"[TX] {names[cmd]}  cfg={config:#06x}  data={data_val:#010x}   {hex_s}",
                      colors[idx])

    def _send_thresholds(self):
        xt = self.spin_xthr.value()
        yt = self.spin_ythr.value()
        # CONFIG = X threshold (uint16 mg), DATA lower 16 bits = Y threshold
        frame = self._send_frame(CMD_SET_THRESH, xt & 0xFFFF, yt & 0xFFFF)
        if frame:
            hex_s = " ".join(f"{b:02X}" for b in frame)
            self._log(f"[TX] SET_THRESH  Xthr={xt} mg  Ythr={yt} mg   {hex_s}", ACCEL_CLR)
            self.board_widget.set_thresholds(xt, yt)

    # ── RX ────────────────────────────────────────────────────────────────

    def _on_raw_rx(self, raw: bytes):
        # Only log non-accelero traffic as raw hex (avoids flooding at 100ms)
        pass   # handled by _on_frame_rx

    def _on_frame_rx(self, cmd: int, config: int, data: int):
        self._frame_rx += 1
        self.lbl_rx.setText(f"RX: {self._frame_rx}")

        if cmd == CMD_ACCEL:
            # DATA: upper 16 bits = Xval, lower 16 bits = Yval  (packed by STM32)
            x_raw = s16((data >> 16) & 0xFFFF)
            y_raw = s16(data & 0xFFFF)
            self.board_widget.update_accel(x_raw, y_raw)
            self.lbl_xval.setText(f"X: {x_raw:+5d} mg")
            self.lbl_yval.setText(f"Y: {y_raw:+5d} mg")
            # Log only 1 in 5 to avoid flooding
            if self._frame_rx % 5 == 0:
                self._log(f"[RX] ACCEL  X={x_raw:+5d} mg  Y={y_raw:+5d} mg", ACCEL_CLR)
        else:
            frame_bytes = build_frame(cmd, config, data)
            hex_s = " ".join(f"{b:02X}" for b in frame_bytes)
            self._log(f"[RX] CMD={cmd:#04x}  cfg={config:#06x}  data={data:#010x}   {hex_s}",
                      ACCENT2)

    def _on_serial_error(self, msg: str):
        self._log_error(f"[ERROR] {msg}"); self._disconnect()

    # ── Log ───────────────────────────────────────────────────────────────

    def _log(self, msg, color=None):
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        cursor = self.log.textCursor()
        cursor.movePosition(cursor.End)
        fmt = QTextCharFormat()
        fmt.setForeground(QBrush(QColor(color or TEXT_MAIN)))
        cursor.setCharFormat(fmt)
        cursor.insertText(f"[{ts}]  {msg}\n")
        self.log.setTextCursor(cursor)
        self.log.ensureCursorVisible()

    def _log_info(self, msg):  self._log(msg, TEXT_DIM)
    def _log_error(self, msg): self._log(msg, DANGER)

    def _clear_log(self):
        self.log.clear()
        self._frame_tx = self._frame_rx = 0
        self.lbl_tx.setText("TX: 0"); self.lbl_rx.setText("RX: 0")

    def closeEvent(self, event):
        self._disconnect(); event.accept()


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = Dashboard()
    win.show()
    sys.exit(app.exec_())