import collections
import os
import subprocess
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from scipy.signal import iirnotch, sosfiltfilt, tf2sos
from adsShm import ShmReader

_BINARY = os.path.join(os.path.dirname(os.path.abspath(__file__)), "adc_oscilloscope")

class Oscilloscope(QtWidgets.QMainWindow):

    UPDATE_INTERVAL_MS = 250
    DISPLAY_SAMPLES    = 500

    def __init__(self):
        super().__init__()

        self.shm_reader = None
        self._buf  = collections.deque(maxlen=self.DISPLAY_SAMPLES)
        self._proc = None
        self._abs_raw_min  = None
        self._abs_raw_max  = None
        self._gain_offsets = {}   # {gain_reg: offset_volts}
        self._notch_sos   = None
        self._notch_fs    = None

        self.setWindowTitle("ADS1263 Oscilloscope")
        self.setGeometry(100, 100, 900, 600)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── Left panel ───────────────────────────────────────
        left_panel = QtWidgets.QFrame()
        left_panel.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        left_panel.setFixedWidth(130)
        left_layout = QtWidgets.QVBoxLayout(left_panel)
        left_layout.setAlignment(QtCore.Qt.AlignmentFlag.AlignTop)
        root.addWidget(left_panel)

        left_layout.addWidget(QtWidgets.QLabel("<b>Sensor</b>"))
        left_layout.addSpacing(8)
        left_layout.addWidget(QtWidgets.QLabel("Sens. Exc. V:"))
        self.exc_voltage_spin = QtWidgets.QDoubleSpinBox()
        self.exc_voltage_spin.setRange(1.0, 10.0)
        self.exc_voltage_spin.setDecimals(3)
        self.exc_voltage_spin.setSingleStep(0.001)
        self.exc_voltage_spin.setSuffix(" V")
        self.exc_voltage_spin.setValue(5.0)
        left_layout.addWidget(self.exc_voltage_spin)

        left_layout.addSpacing(10)
        self._sens_1mmhg_label  = QtWidgets.QLabel()
        self._sens_01mmhg_label = QtWidgets.QLabel()
        self._sens_001mmhg_label = QtWidgets.QLabel()
        for lbl in (self._sens_1mmhg_label, self._sens_01mmhg_label, self._sens_001mmhg_label):
            lbl.setWordWrap(True)
            left_layout.addWidget(lbl)

        self.exc_voltage_spin.valueChanged.connect(self._on_exc_voltage_changed)
        self._on_exc_voltage_changed(self.exc_voltage_spin.value())

        left_layout.addSpacing(8)
        left_layout.addWidget(QtWidgets.QLabel("Vref:"))
        self.vref_spin = QtWidgets.QSpinBox()
        self.vref_spin.setRange(0, 5000)
        self.vref_spin.setSingleStep(1)
        self.vref_spin.setSuffix(" mV")
        self.vref_spin.setValue(2500)
        left_layout.addWidget(self.vref_spin)

        left_layout.addSpacing(8)
        left_layout.addWidget(QtWidgets.QLabel("<b>Filter</b>"))
        self.notch_50hz_chk = QtWidgets.QCheckBox("50 Hz")
        left_layout.addWidget(self.notch_50hz_chk)

        # ── Main area ────────────────────────────────────────
        main_widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(main_widget)
        root.addWidget(main_widget, stretch=1)

        # ------------- Control bar — row 1
        ctrl_row1 = QtWidgets.QHBoxLayout()
        clear_btn = QtWidgets.QPushButton("Clear")
        clear_btn.clicked.connect(self._clear)
        ctrl_row1.addWidget(clear_btn)

        self.freeze_btn = QtWidgets.QPushButton("Freeze")
        self.freeze_btn.setCheckable(True)
        self.freeze_btn.toggled.connect(self._on_freeze)
        ctrl_row1.addWidget(self.freeze_btn)

        ctrl_row1.addSpacing(12)
        self.cursors_chk = QtWidgets.QCheckBox("Cursors")
        self.cursors_chk.toggled.connect(self._on_cursors_toggled)
        ctrl_row1.addWidget(self.cursors_chk)
        self.cursor_label = QtWidgets.QLabel("Δt — | f —")
        self.cursor_label.setFixedWidth(170)
        ctrl_row1.addWidget(self.cursor_label)

        ctrl_row1.addStretch()

        ctrl_row1.addWidget(QtWidgets.QLabel("Window:"))
        self.samples_dial = QtWidgets.QDial()
        self.samples_dial.setRange(10, 4000)
        self.samples_dial.setValue(self.DISPLAY_SAMPLES)
        self.samples_dial.setNotchesVisible(True)
        self.samples_dial.setFixedSize(48, 48)
        self.samples_dial.valueChanged.connect(self._on_samples_changed)
        ctrl_row1.addWidget(self.samples_dial)
        self.samples_val_label = QtWidgets.QLabel(f"{self.DISPLAY_SAMPLES} smp")
        self.samples_val_label.setFixedWidth(60)
        ctrl_row1.addWidget(self.samples_val_label)

        # ------------- Control bar — row 2
        ctrl_row2 = QtWidgets.QHBoxLayout()

        self.avg_line_chk = QtWidgets.QCheckBox("Avg:")
        self.avg_line_chk.toggled.connect(self._on_avg_line_toggled)
        ctrl_row2.addWidget(self.avg_line_chk)
        self.avg_label = QtWidgets.QLabel("—")
        self.avg_label.setFixedWidth(100)
        ctrl_row2.addWidget(self.avg_label)

        ctrl_row2.addSpacing(6)
        self.vrms_chk = QtWidgets.QCheckBox("Vrms noise")
        ctrl_row2.addWidget(self.vrms_chk)
        self.vrms_label = QtWidgets.QLabel("—")
        self.vrms_label.setFixedWidth(90)
        ctrl_row2.addWidget(self.vrms_label)

        self.vpp_chk = QtWidgets.QCheckBox("Vpp noise")
        ctrl_row2.addWidget(self.vpp_chk)
        self.vpp_label = QtWidgets.QLabel("—")
        self.vpp_label.setFixedWidth(90)
        ctrl_row2.addWidget(self.vpp_label)

        self.lsb_chk = QtWidgets.QCheckBox("Min,Max RAW")
        ctrl_row2.addWidget(self.lsb_chk)
        self.lsb_label = QtWidgets.QLabel("—")
        self.lsb_label.setFixedWidth(280)
        self.lsb_label.setWordWrap(True)
        ctrl_row2.addWidget(self.lsb_label)
        abs_clear_btn = QtWidgets.QPushButton("Abs Clear")
        abs_clear_btn.clicked.connect(self._clear_abs)
        ctrl_row2.addWidget(abs_clear_btn)

        ctrl_row2.addSpacing(12)
        self.zero_btn = QtWidgets.QPushButton("Zero")
        self.zero_btn.setToolTip("Capture mean of current window as offset for the active gain")
        self.zero_btn.clicked.connect(self._on_zero)
        ctrl_row2.addWidget(self.zero_btn)
        zero_clr_btn = QtWidgets.QPushButton("Clr Zero")
        zero_clr_btn.setToolTip("Clear offset for the active gain")
        zero_clr_btn.clicked.connect(self._on_zero_clear)
        ctrl_row2.addWidget(zero_clr_btn)
        self.zero_label = QtWidgets.QLabel("")
        self.zero_label.setFixedWidth(90)
        ctrl_row2.addWidget(self.zero_label)

        ctrl_row2.addStretch()

        self.hist_chk = QtWidgets.QCheckBox("Histogram")
        self.hist_chk.toggled.connect(self._on_hist_toggled)
        ctrl_row2.addWidget(self.hist_chk)

        self.fft_chk = QtWidgets.QCheckBox("FFT")
        self.fft_chk.toggled.connect(self._on_fft_toggled)
        ctrl_row2.addWidget(self.fft_chk)

        layout.addLayout(ctrl_row1)
        layout.addLayout(ctrl_row2)

        # ------------- Plot area
        pg.setConfigOptions(antialias=True)
        self.plot_widget = pg.GraphicsLayoutWidget()

        # ------------- CH0 Plot
        self.pair0_plot = self.plot_widget.addPlot(title="pair0", row=0, col=0)
        self.pair0_plot.setLabel('left', "Voltage", units='V')
        self.pair0_plot.setLabel('bottom', "Time", units='s')
        self.pair0_plot.showGrid(x=True, y=True)
        self.pair0_curve = self.pair0_plot.plot(pen=pg.mkPen('y', width=1))

        _cur_pen = pg.mkPen(color='w', style=QtCore.Qt.PenStyle.DashLine, width=1)
        self._cursor_a = pg.InfiniteLine(angle=90, movable=True, pen=_cur_pen,
                                         label='A', labelOpts={'position': 0.95, 'color': 'w'})
        self._cursor_b = pg.InfiniteLine(angle=90, movable=True, pen=_cur_pen,
                                         label='B', labelOpts={'position': 0.90, 'color': 'w'})
        self._cursor_a.setVisible(False)
        self._cursor_b.setVisible(False)
        self.pair0_plot.addItem(self._cursor_a)
        self.pair0_plot.addItem(self._cursor_b)
        self._cursor_a.sigPositionChanged.connect(self._update_cursor_label)
        self._cursor_b.sigPositionChanged.connect(self._update_cursor_label)

        self._avg_line = pg.InfiniteLine(
            angle=0,
            pen=pg.mkPen(color='g', style=QtCore.Qt.PenStyle.DotLine, width=1),
        )
        self._avg_line.setVisible(False)
        self.pair0_plot.addItem(self._avg_line)

        _yellow_dot  = pg.mkPen(color='y', style=QtCore.Qt.PenStyle.DotLine, width=1)
        _yellow_opts = {'position': 0.95, 'color': 'y', 'fill': (0, 0, 0, 120)}
        self._avg_upper_line = pg.InfiniteLine(angle=0, pen=_yellow_dot,
                                               label='1 mmHg', labelOpts=_yellow_opts)
        self._avg_lower_line = pg.InfiniteLine(angle=0, pen=_yellow_dot,
                                               label='1 mmHg', labelOpts=_yellow_opts)
        self._avg_upper_line.setVisible(False)
        self._avg_lower_line.setVisible(False)
        self.pair0_plot.addItem(self._avg_upper_line)
        self.pair0_plot.addItem(self._avg_lower_line)

        _orange_pen  = pg.mkPen(color=(255, 140, 0), style=QtCore.Qt.PenStyle.DotLine, width=1)
        _orange_opts = {'position': 0.05, 'color': (255, 140, 0), 'fill': (0, 0, 0, 120)}
        self._avg_orange_upper = pg.InfiniteLine(angle=0, pen=_orange_pen,
                                                 label='0.1 mmHg', labelOpts=_orange_opts)
        self._avg_orange_lower = pg.InfiniteLine(angle=0, pen=_orange_pen,
                                                 label='0.1 mmHg', labelOpts=_orange_opts)
        self._avg_orange_upper.setVisible(False)
        self._avg_orange_lower.setVisible(False)
        self.pair0_plot.addItem(self._avg_orange_upper)
        self.pair0_plot.addItem(self._avg_orange_lower)

        _red_pen  = pg.mkPen(color=(220, 0, 0), style=QtCore.Qt.PenStyle.DotLine, width=1)
        _red_opts = {'position': 0.15, 'color': (220, 0, 0), 'fill': (0, 0, 0, 120)}
        self._avg_red_upper = pg.InfiniteLine(angle=0, pen=_red_pen,
                                              label='0.01 mmHg', labelOpts=_red_opts)
        self._avg_red_lower = pg.InfiniteLine(angle=0, pen=_red_pen,
                                              label='0.01 mmHg', labelOpts=_red_opts)
        self._avg_red_upper.setVisible(False)
        self._avg_red_lower.setVisible(False)
        self.pair0_plot.addItem(self._avg_red_upper)
        self.pair0_plot.addItem(self._avg_red_lower)

        # ------------- Bottom row: histogram + FFT side by side
        self._bottom_splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        self._bottom_splitter.setVisible(False)

        self.hist_widget = pg.PlotWidget()
        self.hist_widget.setLabel('left', "Count")
        self.hist_widget.setLabel('bottom', "Voltage", units='V')
        self.hist_widget.showGrid(x=True, y=True)
        self._hist_bar = pg.BarGraphItem(x=[], height=[], width=1.0, brush=pg.mkBrush('y'))
        self.hist_widget.addItem(self._hist_bar)
        self._bottom_splitter.addWidget(self.hist_widget)

        self.fft_widget = pg.PlotWidget()
        self.fft_widget.setLabel('left', "Magnitude", units='dBµV')
        self.fft_widget.setLabel('bottom', "Frequency", units='Hz')
        self.fft_widget.showGrid(x=True, y=True)
        self._fft_curve = self.fft_widget.plot(pen=pg.mkPen('c', width=1))
        self._bottom_splitter.addWidget(self.fft_widget)

        # Splitter lets the user drag the divider between waveform and bottom row
        self._plot_splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        self._plot_splitter.addWidget(self.plot_widget)
        self._plot_splitter.addWidget(self._bottom_splitter)
        self._plot_splitter.setCollapsible(0, False)
        self._plot_splitter.setCollapsible(1, True)
        layout.addWidget(self._plot_splitter)

        # ------------- Status bar
        self.status_label = QtWidgets.QLabel("Sample count: —")
        layout.addWidget(self.status_label)

        # ── Right sidebar ─────────────────────────────────────
        sidebar = QtWidgets.QFrame()
        sidebar.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        sidebar.setFixedWidth(220)
        sidebar_layout = QtWidgets.QVBoxLayout(sidebar)
        sidebar_layout.setAlignment(QtCore.Qt.AlignmentFlag.AlignTop)
        root.addWidget(sidebar)

        sidebar_layout.addWidget(QtWidgets.QLabel("<b>ADS1263</b>"))
        sidebar_layout.addSpacing(8)

        self.startstop_btn = QtWidgets.QPushButton("Start Worker")
        self.startstop_btn.setStyleSheet("background-color: #2d6a2d; color: white;")
        self.startstop_btn.clicked.connect(self._toggle_ads)
        sidebar_layout.addWidget(self.startstop_btn)

        self._conv_running = True
        self.conv_btn = QtWidgets.QPushButton("Stop Conv")
        self.conv_btn.setStyleSheet("background-color: #6a4a2d; color: white;")
        self.conv_btn.setEnabled(False)
        self.conv_btn.clicked.connect(self._toggle_conv)
        sidebar_layout.addWidget(self.conv_btn)

        sidebar_layout.addSpacing(12)
        self.proc_status_label = QtWidgets.QLabel("Stopped")
        self.proc_status_label.setWordWrap(True)
        sidebar_layout.addWidget(self.proc_status_label)

        sidebar_layout.addSpacing(12)
        self.dr_group = QtWidgets.QGroupBox("ADC Config")
        self.dr_group.setEnabled(False)
        dr_layout = QtWidgets.QVBoxLayout(self.dr_group)
        dr_layout.setContentsMargins(4, 4, 4, 4)

        dr_layout.addWidget(QtWidgets.QLabel("Sample rate:"))
        self.dr_combo = QtWidgets.QComboBox()
        for label, code in [
            ("2.5 SPS",   0x00), ("5 SPS",     0x01), ("10 SPS",   0x02),
            ("16.6 SPS",  0x03), ("20 SPS",    0x04), ("50 SPS",   0x05),
            ("60 SPS",    0x06), ("100 SPS",   0x07), ("400 SPS",  0x08),
            ("1200 SPS",  0x09), ("2400 SPS",  0x0A), ("4800 SPS", 0x0B),
            ("7200 SPS",  0x0C), ("14400 SPS", 0x0D), ("19200 SPS",0x0E),
            ("38400 SPS", 0x0F),
        ]:
            self.dr_combo.addItem(label, code)
        self.dr_combo.activated.connect(self._on_dr_changed)
        dr_layout.addWidget(self.dr_combo)

        dr_layout.addSpacing(4)
        dr_layout.addWidget(QtWidgets.QLabel("Gain:"))
        self.gain_combo = QtWidgets.QComboBox()
        for label, reg in [
            ("×1",  0x00), ("×2",  0x10), ("×4",  0x20),
            ("×8",  0x30), ("×16", 0x40), ("×32", 0x50),
        ]:
            self.gain_combo.addItem(label, reg)
        self.gain_combo.setCurrentIndex(5)   # ×32 matches ADC_GAIN default
        self.gain_combo.activated.connect(self._on_gain_changed)
        dr_layout.addWidget(self.gain_combo)

        dr_layout.addSpacing(4)
        dr_layout.addWidget(QtWidgets.QLabel("Filter:"))
        self.filter_combo = QtWidgets.QComboBox()
        for label, idx in [
            ("SINC1", 0), ("SINC2", 1), ("SINC3", 2), ("SINC4", 3), ("FIR", 4),
        ]:
            self.filter_combo.addItem(label, idx)
        self.filter_combo.setCurrentIndex(3)   # SINC4 matches ADC_FILTER default
        self.filter_combo.activated.connect(self._on_filter_changed)
        dr_layout.addWidget(self.filter_combo)

        dr_layout.addSpacing(4)
        dr_layout.addWidget(QtWidgets.QLabel("Reference:"))
        # REFMUX bits[5:3]=RMUXP, bits[2:0]=RMUXN
        _refp_pins = [
            ("Internal", 0), ("AIN0", 1), ("AIN2", 2), ("AIN4", 3), ("AVDD", 4),
        ]
        _refn_pins = [
            ("Internal", 0), ("AIN1", 1), ("AIN3", 2), ("AIN5", 3), ("AVSS", 4),
        ]
        self.refp_combo = QtWidgets.QComboBox()
        self.refn_combo = QtWidgets.QComboBox()
        for label, val in _refp_pins:
            self.refp_combo.addItem(label, val)
        for label, val in _refn_pins:
            self.refn_combo.addItem(label, val)
        self.refp_combo.setCurrentIndex(2)   # AIN2 — matches REFMUX_AIN2_AIN3 default
        self.refn_combo.setCurrentIndex(2)   # AIN3
        self.refp_combo.activated.connect(self._on_refmux_changed)
        self.refn_combo.activated.connect(self._on_refmux_changed)

        ref_pos_row = QtWidgets.QHBoxLayout()
        ref_pos_row.addWidget(QtWidgets.QLabel("+"))
        ref_pos_row.addWidget(self.refp_combo)
        dr_layout.addLayout(ref_pos_row)

        ref_neg_row = QtWidgets.QHBoxLayout()
        ref_neg_row.addWidget(QtWidgets.QLabel("−"))
        ref_neg_row.addWidget(self.refn_combo)
        dr_layout.addLayout(ref_neg_row)

        dr_layout.addSpacing(4)
        dr_layout.addWidget(QtWidgets.QLabel("Input MUX:"))
        _mux_pins = [
            ("AIN0", 0x0), ("AIN1", 0x1), ("AIN2", 0x2), ("AIN3", 0x3),
            ("AIN4", 0x4), ("AIN5", 0x5), ("AIN6", 0x6), ("AIN7", 0x7),
            ("AIN8", 0x8), ("AIN9", 0x9), ("AINCOM", 0xA),
        ]
        self.muxp_combo = QtWidgets.QComboBox()
        self.muxn_combo = QtWidgets.QComboBox()
        for label, val in _mux_pins:
            self.muxp_combo.addItem(label, val)
            self.muxn_combo.addItem(label, val)
        self.muxp_combo.setCurrentIndex(0)   # AIN0 — matches MUX_CH0_CH1 default
        self.muxn_combo.setCurrentIndex(1)   # AIN1
        self.muxp_combo.activated.connect(self._on_inpmux_changed)
        self.muxn_combo.activated.connect(self._on_inpmux_changed)

        mux_pos_row = QtWidgets.QHBoxLayout()
        mux_pos_row.addWidget(QtWidgets.QLabel("+"))
        mux_pos_row.addWidget(self.muxp_combo)
        dr_layout.addLayout(mux_pos_row)

        mux_neg_row = QtWidgets.QHBoxLayout()
        mux_neg_row.addWidget(QtWidgets.QLabel("−"))
        mux_neg_row.addWidget(self.muxn_combo)
        dr_layout.addLayout(mux_neg_row)

        sidebar_layout.addWidget(self.dr_group)

        # ── ADC Register group ────────────────────────────────
        self.reg_group = QtWidgets.QGroupBox("ADC Register")
        reg_layout = QtWidgets.QVBoxLayout(self.reg_group)
        reg_layout.setContentsMargins(4, 4, 4, 4)
        reg_layout.setSpacing(2)

        # (name, register address, SHM read-back attribute)
        _regs = [
            ("MODE0",  0x03, "rb_mode0"),
            ("MODE1",  0x04, "rb_mode1"),
            ("MODE2",  0x05, "rb_mode2"),
            ("REFMUX", 0x0F, "rb_refmux"),
            ("INPMUX", 0x06, "rb_inpmux"),
        ]
        self._reg_rows = []   # list of (addr, rb_attr, val_label, spinbox)
        grid = QtWidgets.QGridLayout()
        grid.setColumnStretch(1, 1)
        for row_idx, (name, addr, rb_attr) in enumerate(_regs):
            name_lbl = QtWidgets.QLabel(name)
            name_lbl.setFixedWidth(52)
            val_lbl = QtWidgets.QLabel("0x--")
            val_lbl.setFixedWidth(36)
            spin = QtWidgets.QSpinBox()
            spin.setRange(0, 255)
            spin.setDisplayIntegerBase(16)
            spin.setPrefix("0x")
            spin.setFixedWidth(60)
            write_btn = QtWidgets.QPushButton("W")
            write_btn.setFixedWidth(24)
            write_btn.clicked.connect(lambda _checked, a=addr, s=spin: self._on_raw_wreg(a, s.value()))
            grid.addWidget(name_lbl,   row_idx, 0)
            grid.addWidget(val_lbl,    row_idx, 1)
            grid.addWidget(spin,       row_idx, 2)
            grid.addWidget(write_btn,  row_idx, 3)
            self._reg_rows.append((addr, rb_attr, val_lbl, spin))
        reg_layout.addLayout(grid)

        refresh_btn = QtWidgets.QPushButton("Refresh")
        refresh_btn.clicked.connect(self._refresh_regs)
        reg_layout.addWidget(refresh_btn)

        sidebar_layout.addWidget(self.reg_group)

        # ── Timers ────────────────────────────────────────────
        self.frozen = False

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update_plot)
        self.timer.start(self.UPDATE_INTERVAL_MS)

        self._proc_poll_timer = QtCore.QTimer()
        self._proc_poll_timer.timeout.connect(self._poll_proc)
        self._proc_poll_timer.start(500)

        self._resize_timer = QtCore.QTimer()
        self._resize_timer.setSingleShot(True)
        self._resize_timer.timeout.connect(self._apply_window_resize)

    # ── Shared memory connection ─────────────────────────────

    def _try_connect_shm(self) -> bool:
        if self.shm_reader is not None:
            return True
        try:
            self.shm_reader = ShmReader()
            self._sync_dr_combo()
            self._refresh_regs()
            return True
        except Exception:
            return False

    def _disconnect_shm(self):
        self.shm_reader = None

    # ── ADS1263 process control ───────────────────────────────

    def _toggle_ads(self):
        if self._proc and self._proc.poll() is None:
            self._stop_ads()
        else:
            self._start_ads()

    def _start_ads(self):
        try:
            self._proc = subprocess.Popen(
                [_BINARY],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            self._set_proc_running(True)
        except Exception as e:
            self.proc_status_label.setText(f"Error:\n{e}")

    def _stop_ads(self):
        if self._proc:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._proc.kill()
            self._proc = None
        self._disconnect_shm()
        self._set_proc_running(False)

    def _poll_proc(self):
        if self._proc and self._proc.poll() is not None:
            code = self._proc.returncode
            self._proc = None
            self._disconnect_shm()
            self._set_proc_running(False)
            self.proc_status_label.setText(f"Exited ({code})")
        elif self._proc is None:
            self._try_connect_shm()

    def _set_proc_running(self, running: bool):
        self.startstop_btn.setText("Stop Worker" if running else "Start Worker")
        self.startstop_btn.setStyleSheet(
            "background-color: #6a2d2d; color: white;" if running
            else "background-color: #2d6a2d; color: white;"
        )
        self.proc_status_label.setText("Running" if running else "Stopped")
        self.dr_group.setEnabled(running)
        self.conv_btn.setEnabled(running)
        if not running:
            self._conv_running = True
            self.conv_btn.setText("Stop Conv")
            self.conv_btn.setStyleSheet("background-color: #6a4a2d; color: white;")

    def _on_dr_changed(self, idx):
        if self.shm_reader is None:
            return
        self.shm_reader.shm.cmd_dr = self.dr_combo.itemData(idx)

    def _on_gain_changed(self, idx):
        if self.shm_reader is None:
            return
        self.shm_reader.shm.cmd_gain = self.gain_combo.itemData(idx)

    def _on_refmux_changed(self, _idx=None):
        if self.shm_reader is None:
            return
        refp = self.refp_combo.currentData()
        refn = self.refn_combo.currentData()
        self.shm_reader.shm.cmd_refmux = (refp << 3) | refn

    def _on_inpmux_changed(self, _idx=None):
        if self.shm_reader is None:
            return
        muxp = self.muxp_combo.currentData()
        muxn = self.muxn_combo.currentData()
        self.shm_reader.shm.cmd_inpmux = (muxp << 4) | muxn

    def _on_filter_changed(self, idx):
        if self.shm_reader is None:
            return
        self.shm_reader.shm.cmd_filter = self.filter_combo.itemData(idx)

    def _toggle_conv(self):
        if self.shm_reader is None:
            return
        self._conv_running = not self._conv_running
        self.shm_reader.shm.cmd_conv = 1 if self._conv_running else 0
        self.conv_btn.setText("Stop Conv" if self._conv_running else "Start Conv")
        self.conv_btn.setStyleSheet(
            "background-color: #6a4a2d; color: white;" if self._conv_running
            else "background-color: #2d6a2d; color: white;"
        )

    def _on_raw_wreg(self, addr: int, val: int):
        if self.shm_reader is None:
            return
        self.shm_reader.shm.cmd_raw_wreg = (addr << 8) | (val & 0xFF)

    def _refresh_regs(self):
        if self.shm_reader is None:
            return
        shm = self.shm_reader.shm
        for _addr, rb_attr, val_lbl, spin in self._reg_rows:
            v = getattr(shm, rb_attr) & 0xFF
            val_lbl.setText(f"0x{v:02X}")
            spin.setValue(v)

    def _sync_dr_combo(self):
        if self.shm_reader is None:
            return
        sps = self.shm_reader.shm.sample_rate
        dr_sps = [3,5,10,17,20,50,60,100,400,1200,2400,4800,7200,14400,19200,38400]
        if sps in dr_sps:
            self.dr_combo.setCurrentIndex(dr_sps.index(sps))

    # ── Plot update ───────────────────────────────────────────

    def _update_plot(self):
        if not self._try_connect_shm():
            self.status_label.setText("Waiting for ADS1263 backend…")
            return
        try:
            if not self.frozen:
                self._buf.extend(self.shm_reader.drain_pair0())

                sample_rate = self.shm_reader.shm.sample_rate or 100
                window_s = self._buf.maxlen / sample_rate
                self.pair0_plot.setXRange(0, window_s, padding=0)
                if self._buf:
                    data = np.array(self._buf, dtype=np.float32)
                    gain_reg = self.gain_combo.currentData()
                    gain_off = self._gain_offsets.get(gain_reg, 0.0)
                    if gain_off != 0.0:
                        data = data - np.float32(gain_off)
                    if self.notch_50hz_chk.isChecked() and sample_rate > 100:
                        if self._notch_fs != sample_rate:
                            b, a = iirnotch(50.0, 30.0, float(sample_rate))
                            self._notch_sos = tf2sos(b, a)
                            self._notch_fs  = sample_rate
                        if self._notch_sos is not None and len(data) > 9:
                            try:
                                data = sosfiltfilt(
                                    self._notch_sos,
                                    data.astype(np.float64)
                                ).astype(np.float32)
                            except Exception:
                                pass
                    t = (np.arange(len(data)) + (self._buf.maxlen - len(data))) / sample_rate
                    self.pair0_curve.setData(t, data)

                    avg = float(np.mean(data))
                    if abs(avg) < 1e-3:
                        self.avg_label.setText(f"{avg*1e6:.2f} µV")
                    elif abs(avg) < 1.0:
                        self.avg_label.setText(f"{avg*1e3:.4f} mV")
                    else:
                        self.avg_label.setText(f"{avg:.6f} V")
                    if self.avg_line_chk.isChecked():
                        self._avg_line.setValue(avg)
                        offset = self.exc_voltage_spin.value() * 5e-6
                        self._avg_upper_line.setValue(avg + offset)
                        self._avg_lower_line.setValue(avg - offset)
                        offset_01 = offset * 0.1
                        self._avg_orange_upper.setValue(avg + offset_01)
                        self._avg_orange_lower.setValue(avg - offset_01)
                        offset_001 = offset * 0.01
                        self._avg_red_upper.setValue(avg + offset_001)
                        self._avg_red_lower.setValue(avg - offset_001)

                    if self.vrms_chk.isChecked():
                        vrms = float(np.std(data))
                        self.vrms_label.setText(f"{vrms*1e6:.2f} µV" if vrms < 1e-3
                                                else f"{vrms*1e3:.4f} mV")
                    else:
                        self.vrms_label.setText("—")

                    if self.vpp_chk.isChecked():
                        vpp = float(data.max() - data.min())
                        self.vpp_label.setText(f"{vpp*1e6:.2f} µV" if vpp < 1e-3
                                               else f"{vpp*1e3:.4f} mV")
                    else:
                        self.vpp_label.setText("—")

                    if self.lsb_chk.isChecked():
                        gain_reg = self.gain_combo.currentData()
                        gain_factor = 1 << (gain_reg >> 4)
                        refmux_byte = (self.refp_combo.currentData() << 3) | self.refn_combo.currentData()
                        vref = {0x00: 2.5, 0x24: 5.0}.get(
                            refmux_byte, self.exc_voltage_spin.value())
                        raw = np.round(
                            data.astype(np.float64) * gain_factor * (1 << 31) / vref
                        ).astype(np.int64)
                        w_min, w_max = int(raw.min()), int(raw.max())
                        if self._abs_raw_min is None:
                            self._abs_raw_min, self._abs_raw_max = w_min, w_max
                        else:
                            self._abs_raw_min = min(self._abs_raw_min, w_min)
                            self._abs_raw_max = max(self._abs_raw_max, w_max)
                        abs_delta = self._abs_raw_max - self._abs_raw_min
                        delta_uv = abs_delta * self.vref_spin.value() * 1000 / (gain_factor * (1 << 31))
                        self.lsb_label.setText(
                            f"win  min {w_min}  max {w_max}\n"
                            f"abs  min {self._abs_raw_min}  max {self._abs_raw_max}"
                            f"  Δ {abs_delta} ({delta_uv:.3f} µV)"
                        )
                    else:
                        self.lsb_label.setText("—")

                    if self.hist_chk.isChecked():
                        counts, edges = np.histogram(data, bins=100)
                        centers = (edges[:-1] + edges[1:]) / 2
                        width = float(edges[1] - edges[0])
                        self._hist_bar.setOpts(x=centers, height=counts, width=width)

                    if self.fft_chk.isChecked() and len(data) > 1:
                        n = len(data)
                        win = np.hanning(n)
                        mag = np.abs(np.fft.rfft(data * win)) * 2.0 / win.sum()
                        freqs = np.fft.rfftfreq(n, d=1.0 / sample_rate)
                        db = 20.0 * np.log10(np.maximum(mag * 1e6, 1e-12))
                        self._fft_curve.setData(freqs[1:], db[1:])

            shm  = self.shm_reader.shm
            head = shm.pair0_head
            tail = shm.pair0_tail
            used = (head - tail) % self.shm_reader.SHM_SAMPLES
            free_pct = 100.0 * (self.shm_reader.SHM_SAMPLES - used) / self.shm_reader.SHM_SAMPLES
            lost, _ = self.shm_reader.read_lost()
            frozen_tag = "  [FROZEN]" if self.frozen else ""
            self.status_label.setText(
                f"Samples: {self.shm_reader.read_sample_count()}"
                f"  Lost: {lost}"
                f"  Overruns: {self.shm_reader.read_overruns()}"
                f"  |  used: {used}  head: {head}  tail: {tail}  free: {free_pct:.1f}%"
                f"{frozen_tag}"
            )

        except Exception as e:
            self.status_label.setText(f"Error: {e}")

    def _on_exc_voltage_changed(self, v):
        s1   = v * 5.0          # µV  — 1 mmHg
        s01  = v * 0.5          # µV  — 0.1 mmHg
        s001 = v * 0.05         # µV  — 0.01 mmHg
        self._sens_1mmhg_label.setText(f"1 mmHg:\n{s1:.3f} µV")
        self._sens_01mmhg_label.setText(f"0.1 mmHg:\n{s01:.4f} µV")
        self._sens_001mmhg_label.setText(f"0.01 mmHg:\n{s001:.5f} µV")

    def _on_cursors_toggled(self, checked):
        self._cursor_a.setVisible(checked)
        self._cursor_b.setVisible(checked)
        if checked:
            r = self.pair0_plot.viewRange()[0]
            span = r[1] - r[0]
            self._cursor_a.setValue(r[0] + span / 3)
            self._cursor_b.setValue(r[0] + span * 2 / 3)
            self._update_cursor_label()
        else:
            self.cursor_label.setText("Δt — | f —")

    def _update_cursor_label(self):
        dt = abs(self._cursor_b.value() - self._cursor_a.value())
        if dt < 1e-9:
            self.cursor_label.setText("Δt — | f —")
            return
        freq = 1.0 / dt
        if dt < 1e-3:
            dt_str = f"{dt*1e6:.2f} µs"
        elif dt < 1.0:
            dt_str = f"{dt*1e3:.3f} ms"
        else:
            dt_str = f"{dt:.4f} s"
        if freq >= 1000:
            f_str = f"{freq/1000:.3f} kHz"
        else:
            f_str = f"{freq:.3f} Hz"
        self.cursor_label.setText(f"Δt {dt_str} | f {f_str}")

    def _on_avg_line_toggled(self, checked):
        self._avg_line.setVisible(checked)
        self._avg_upper_line.setVisible(checked)
        self._avg_lower_line.setVisible(checked)
        self._avg_orange_upper.setVisible(checked)
        self._avg_orange_lower.setVisible(checked)
        self._avg_red_upper.setVisible(checked)
        self._avg_red_lower.setVisible(checked)

    def _on_hist_toggled(self, checked):
        self.hist_widget.setVisible(checked)
        if not checked:
            self._hist_bar.setOpts(x=[], height=[], width=1.0)
        self._update_bottom_visible()

    def _on_fft_toggled(self, checked):
        self.fft_widget.setVisible(checked)
        if not checked:
            self._fft_curve.setData([], [])
        self._update_bottom_visible()

    def _update_bottom_visible(self):
        visible = self.hist_chk.isChecked() or self.fft_chk.isChecked()
        self._bottom_splitter.setVisible(visible)
        if visible:
            total = self._plot_splitter.height()
            self._plot_splitter.setSizes([total * 2 // 3, total // 3])

    def _on_samples_changed(self, value):
        self.samples_val_label.setText(f"{value} smp")
        self._resize_timer.start(150)

    def _apply_window_resize(self):
        value = self.samples_dial.value()
        self._buf = collections.deque(self._buf, maxlen=value)

    def _on_freeze(self, checked):
        self.frozen = checked
        self.freeze_btn.setText("Unfreeze" if checked else "Freeze")

    def _clear_abs(self):
        self._abs_raw_min = None
        self._abs_raw_max = None
        if not self.lsb_chk.isChecked():
            self.lsb_label.setText("—")

    def _on_zero(self):
        if not self._buf:
            return
        gain_reg = self.gain_combo.currentData()
        off = float(np.mean(np.array(self._buf, dtype=np.float64)))
        self._gain_offsets[gain_reg] = off
        if abs(off) < 1e-3:
            self.zero_label.setText(f"{off*1e6:.2f} µV")
        elif abs(off) < 1.0:
            self.zero_label.setText(f"{off*1e3:.4f} mV")
        else:
            self.zero_label.setText(f"{off:.6f} V")

    def _on_zero_clear(self):
        gain_reg = self.gain_combo.currentData()
        self._gain_offsets.pop(gain_reg, None)
        self.zero_label.setText("")

    def _clear(self):
        self._buf.clear()
        self._abs_raw_min = None
        self._abs_raw_max = None
        self.lsb_label.setText("—")
        self.pair0_curve.setData([], [])
        shm = self.shm_reader.shm
        shm.pair0_tail = shm.pair0_head
        shm.pair0_lost = 0
        shm.overruns = 0

    def closeEvent(self, event):
        self.timer.stop()
        self._proc_poll_timer.stop()
        self._stop_ads()
        super().closeEvent(event)
