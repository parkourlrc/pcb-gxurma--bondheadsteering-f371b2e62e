"""Python 3.9+ / PyQt5 GUI for NTC_BLE_Controller.ino (serial connection).
Install: python -m pip install PyQt5 PyQtChart pyserial
Run:     python NTC_Controller_GUI.py

Commands: o1 = force mode, s = one shot, o0 = stop, t = tare,
          p{index},{seconds},{degrees} = profile point, r{raw force} = threshold.
Profile edits remain LOCAL until 'Profil übertragen' is clicked.
Select the force unit produced by the firmware calibration (default mN, as in
its original GUI). Both force display and transmitted threshold use this unit.
Only active-sequence samples are plotted. The preceding temperature trace is
retained when the next sequence begins; no other history is accumulated.
"""
import math
import re
import sys
import time
from collections import deque

import serial
import serial.tools.list_ports
from PyQt5.QtCore import Qt, QTimer, QPointF, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QDialog, QListWidget, QDialogButtonBox,
    QGraphicsEllipseItem, QGraphicsItem, QPlainTextEdit, QDoubleSpinBox,
    QGridLayout, QSlider, QComboBox, QSplitter, QMessageBox, QScrollArea,
)
from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis

DEFAULT_PROFILE = [(0.0, 40.0), (30.0, 120.0), (60.0, 130.0),
                   (90.0, 200.0), (110.0, 210.0), (150.0, 50.0)]
MAX_TIME = 250.0
MAX_TEMP = 250.0
MIN_TIME_GAP = 0.1


def parse_telemetry(line):
    """Firmware may prepend debug fields; its final SIX fields are stable."""
    fields = line.split(',')
    if len(fields) < 6:
        return None
    try:
        active, enabled, target, milliseconds, temperature, raw_force = (
            float(field.strip()) for field in fields[-6:])
    except ValueError:
        return None
    values = (active, enabled, target, milliseconds, temperature, raw_force)
    if not all(math.isfinite(value) for value in values):
        return None
    if active not in (0, 1) or enabled not in (0, 1) or milliseconds < 0:
        return None
    if active and not enabled:
        return None
    return bool(active), bool(enabled), target, milliseconds / 1000.0, temperature, raw_force


class DraggablePoint(QGraphicsEllipseItem):
    def __init__(self, owner, index):
        # QChart.mapToPosition()/mapToValue() use CHART-local coordinates.
        # Parenting the handle to the chart gives it exactly this coordinate space.
        super().__init__(-6, -6, 12, 12, owner.chart)
        self.owner = owner
        self.index = index
        self.setBrush(QColor('#168457'))
        self.setPen(QPen(QColor('white'), 1.5))
        self.setFlags(QGraphicsItem.ItemIsMovable |
                      QGraphicsItem.ItemSendsGeometryChanges)
        self.setCursor(Qt.SizeAllCursor)
        self.setZValue(20)

    def itemChange(self, change, value):
        if (change == QGraphicsItem.ItemPositionChange and
                hasattr(self, 'owner') and not self.owner.syncing):
            return self.owner.move_handle(self.index, value)
        return super().itemChange(change, value)


class ChartWidget(QWidget):
    profileChanged = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.point_data = list(DEFAULT_PROFILE)
        self.syncing = False
        self.drag_points = []
        self.chart = QChart()
        self.chart.setAnimationOptions(QChart.NoAnimation)
        self.chart.setTitle('Temperaturprofil und Messwerte')
        self.series_points = QLineSeries()
        self.series_previous = QLineSeries()
        self.series_temp = QLineSeries()
        self.series_force = QLineSeries()
        specs = [
            (self.series_points, 'Sollprofil', QColor('#276ac4'), 2.0, Qt.DashLine),
            (self.series_previous, 'Temp. vorher', QColor(214, 47, 47, 65), 2.0, Qt.SolidLine),
            (self.series_temp, 'Temp. aktuell', QColor('#d62f2f'), 2.5, Qt.SolidLine),
            (self.series_force, 'Kraft', QColor('#777777'), 1.5, Qt.SolidLine),
        ]
        for series, name, color, width, style in specs:
            series.setName(name)
            series.setPen(QPen(color, width, style))
            self.chart.addSeries(series)
        self.axisX = QValueAxis()
        self.axisX.setRange(0, MAX_TIME)
        self.axisX.setTickCount(6)
        self.axisX.setLabelFormat('%.0f')
        self.axisX.setTitleText('Zeit (s)')
        self.axisYTemp = QValueAxis()
        self.axisYTemp.setRange(0, MAX_TEMP)
        self.axisYTemp.setTickCount(6)
        self.axisYTemp.setLabelFormat('%.0f')
        self.axisYTemp.setTitleText('Temperatur (°C)')
        self.axisYForce = QValueAxis()
        self.axisYForce.setRange(0, 30)
        self.axisYForce.setTitleText('Kraft (N)')
        self.chart.addAxis(self.axisX, Qt.AlignBottom)
        self.chart.addAxis(self.axisYTemp, Qt.AlignLeft)
        self.chart.addAxis(self.axisYForce, Qt.AlignRight)
        for series in (self.series_points, self.series_previous, self.series_temp):
            series.attachAxis(self.axisX)
            series.attachAxis(self.axisYTemp)
        self.series_force.attachAxis(self.axisX)
        self.series_force.attachAxis(self.axisYForce)
        self.chart.legend().setAlignment(Qt.AlignBottom)
        legend_font = self.chart.legend().font()
        legend_font.setPointSize(9)
        self.chart.legend().setFont(legend_font)
        self.view = QChartView(self.chart)
        self.view.setRenderHint(QPainter.Antialiasing)
        self.view.setMinimumSize(400, 300)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.view)
        self.series_points.replace([QPointF(x, y) for x, y in self.point_data])
        self.drag_points = [DraggablePoint(self, i) for i in range(6)]
        self.chart.plotAreaChanged.connect(self.sync_handles)
        self.sync_handles()

    def sync_handles(self, *_):
        if self.syncing:
            return
        self.syncing = True
        try:
            for item, (x, y) in zip(self.drag_points, self.point_data):
                item.setPos(self.chart.mapToPosition(QPointF(x, y), self.series_points))
                item.setToolTip(f'{x:.1f} s / {y:.1f} °C')
        finally:
            self.syncing = False

    def constrained_point(self, index, x, y):
        low = 0 if index == 0 else self.point_data[index - 1][0] + MIN_TIME_GAP
        high = MAX_TIME if index == 5 else self.point_data[index + 1][0] - MIN_TIME_GAP
        return (round(max(low, min(high, round(x, 1))), 1),
                round(max(0, min(MAX_TEMP, y)), 1))

    def move_handle(self, index, chart_position):
        value = self.chart.mapToValue(chart_position, self.series_points)
        self.point_data[index] = self.constrained_point(index, value.x(), value.y())
        self.series_points.replace([QPointF(x, y) for x, y in self.point_data])
        self.profileChanged.emit()
        x, y = self.point_data[index]
        self.drag_points[index].setToolTip(f'{x:.1f} s / {y:.1f} °C')
        return self.chart.mapToPosition(QPointF(x, y), self.series_points)

    def set_point(self, index, x, y):
        self.point_data[index] = self.constrained_point(index, x, y)
        self.series_points.replace([QPointF(a, b) for a, b in self.point_data])
        self.sync_handles()
        self.profileChanged.emit()

    def reset_profile(self):
        self.point_data = list(DEFAULT_PROFILE)
        self.series_points.replace([QPointF(x, y) for x, y in self.point_data])
        self.sync_handles()
        self.profileChanged.emit()

    def begin_sequence(self):
        # Replace, do not append: at most one previous temperature trace.
        self.series_previous.replace(self.series_temp.pointsVector())
        self.series_temp.clear()
        self.series_force.clear()
        self.axisYForce.setRange(0, 30)

    def add_sample(self, seconds, temperature, force):
        if not 0 <= seconds <= MAX_TIME:
            return
        self.series_temp.append(seconds, temperature)
        self.series_force.append(seconds, force)
        if force < self.axisYForce.min():
            self.axisYForce.setMin(math.floor(force * 1.1))
        if force > self.axisYForce.max():
            self.axisYForce.setMax(math.ceil(force * 1.1))


class DebugDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Serielle Kommunikation')
        self.resize(750, 400)
        self.textbox = QPlainTextEdit()
        self.textbox.setReadOnly(True)
        self.textbox.document().setMaximumBlockCount(3000)
        clear = QPushButton('Leeren')
        clear.clicked.connect(self.textbox.clear)
        layout = QVBoxLayout(self)
        layout.addWidget(self.textbox)
        layout.addWidget(clear)

    def append_text(self, text):
        self.textbox.appendPlainText(text)


class COMDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Seriellen Port auswählen')
        self.resize(440, 260)
        layout = QVBoxLayout(self)
        self.port_list = QListWidget()
        for port in serial.tools.list_ports.comports():
            self.port_list.addItem(f'{port.device} — {port.description}')
            self.port_list.item(self.port_list.count() - 1).setData(Qt.UserRole, port.device)
        if self.port_list.count():
            self.port_list.setCurrentRow(0)
        layout.addWidget(self.port_list)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def selected_port(self):
        item = self.port_list.currentItem()
        return item.data(Qt.UserRole) if item else None


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('NTC Controller — Temperatursequenz')
        self.resize(1200, 760)
        self.setMinimumSize(850, 580)
        self.serial_port = None
        self.ready = False
        self.rx_buffer = bytearray()
        self.tx_queue = deque()
        self.pending_profile = None
        self.pending_since = 0
        self.was_active = False
        self.last_elapsed = None
        self.active = False
        self.profile_batch = False
        self.timer = QTimer(self)
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.read_serial_data)
        self.tx_timer = QTimer(self)
        self.tx_timer.setInterval(250)
        self.tx_timer.timeout.connect(self.flush_tx)
        self.tx_timer.start()
        self.threshold_timer = QTimer(self)
        self.threshold_timer.setSingleShot(True)
        self.threshold_timer.setInterval(400)
        self.threshold_timer.timeout.connect(self.send_threshold)
        self.chart_widget = ChartWidget()
        self.debug_window = DebugDialog(self)
        self.connection_label = QLabel('Nicht verbunden')
        self.state_label = QLabel('Zustand: —')
        self.measurement_label = QLabel('Temperatur: —   Kraft: —   Zeit: —')
        self.profile_label = QLabel('Profil lokal: zum Anwenden „Profil übertragen“ klicken.')
        self.profile_label.setWordWrap(True)
        com = QPushButton('COM verbinden')
        com.clicked.connect(self.select_com_port)
        disconnect = QPushButton('Trennen')
        disconnect.clicked.connect(self.disconnect_port)
        debug = QPushButton('Debug')
        debug.clicked.connect(self.debug_window.show)
        self.start_btn = QPushButton('Kraftmodus aktivieren')
        self.start_btn.clicked.connect(lambda: self.send_command('o1\n'))
        self.start_btn.setToolTip('Wartet auf eine neue Kraft-Unterschreitung und anschließende Überschreitung.')
        self.immediate_btn = QPushButton('Sequenz sofort starten')
        self.immediate_btn.clicked.connect(lambda: self.send_command('s\n'))
        self.stop_btn = QPushButton('STOPP')
        self.stop_btn.setStyleSheet('font-weight: bold; color: #b32020;')
        self.stop_btn.clicked.connect(self.stop_controller)
        self.tare_btn = QPushButton('Tara')
        self.tare_btn.clicked.connect(lambda: self.send_command('t\n'))
        self.send_btn = QPushButton('Profil übertragen')
        self.send_btn.clicked.connect(self.send_points)
        reset = QPushButton('Standardprofil laden')
        reset.clicked.connect(self.chart_widget.reset_profile)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 200)
        self.slider_label = QLabel('Kraft-Schwelle: 0.0 N')
        self.slider.valueChanged.connect(self.slider_changed)
        self.force_unit = QComboBox()
        self.force_unit.addItems(['mN', 'N'])
        self.force_unit.setToolTip('Einheit des Kraft-Rohwerts der Firmware, abhängig von forceCalib.')
        self.force_unit.currentIndexChanged.connect(self.force_unit_changed)
        self.time_inputs, self.temp_inputs = [], []
        editor = QWidget()
        editor_layout = QVBoxLayout(editor)
        editor_layout.addWidget(QLabel('Sequenz bearbeiten'))
        grid = QGridLayout()
        for column, text in enumerate(['Punkt', 'Zeit (s)', 'Temp. (°C)']):
            grid.addWidget(QLabel(text), 0, column)
        for index, (seconds, temperature) in enumerate(DEFAULT_PROFILE):
            grid.addWidget(QLabel(str(index + 1)), index + 1, 0)
            time_input = QDoubleSpinBox()
            time_input.setMinimumHeight(28)
            time_input.setDecimals(1)
            time_input.setSingleStep(0.1)
            time_input.setRange(0, MAX_TIME)
            time_input.setValue(seconds)
            time_input.setKeyboardTracking(False)
            temp_input = QDoubleSpinBox()
            temp_input.setMinimumHeight(28)
            temp_input.setDecimals(1)
            temp_input.setRange(0, MAX_TEMP)
            temp_input.setValue(temperature)
            temp_input.setKeyboardTracking(False)
            self.time_inputs.append(time_input)
            self.temp_inputs.append(temp_input)
            grid.addWidget(time_input, index + 1, 1)
            grid.addWidget(temp_input, index + 1, 2)
            time_input.valueChanged.connect(lambda value, i=index: self.edit_point(i))
            temp_input.valueChanged.connect(lambda value, i=index: self.edit_point(i))
        editor_layout.addLayout(grid)
        hint = QLabel('Grüne Punkte ziehen oder Werte eingeben.\nZeitpunkte bleiben aufsteigend sortiert.')
        hint.setWordWrap(True)
        editor_layout.addWidget(hint)
        editor_layout.addWidget(reset)
        editor_layout.addWidget(self.send_btn)
        editor_layout.addWidget(self.profile_label)
        editor_layout.addStretch()
        editor.setMinimumWidth(270)
        self.splitter = QSplitter(Qt.Horizontal)
        self.splitter.addWidget(self.chart_widget)
        editor_scroll = QScrollArea()
        editor_scroll.setWidgetResizable(True)
        editor_scroll.setFrameShape(QScrollArea.NoFrame)
        editor_scroll.setMinimumWidth(290)
        editor_scroll.setWidget(editor)
        self.splitter.addWidget(editor_scroll)
        self.splitter.setStretchFactor(0, 1)
        self.splitter.setStretchFactor(1, 0)
        self.splitter.setChildrenCollapsible(False)
        self.splitter.setSizes([860, 300])
        top = QHBoxLayout()
        for widget in (com, disconnect, self.connection_label):
            top.addWidget(widget)
        top.addStretch()
        top.addWidget(debug)
        controls = QHBoxLayout()
        for widget in (self.start_btn, self.immediate_btn, self.stop_btn, self.tare_btn):
            controls.addWidget(widget)
        threshold = QHBoxLayout()
        threshold.addWidget(self.slider_label)
        threshold.addWidget(self.slider, 1)
        threshold.addWidget(QLabel('Firmware-Krafteinheit:'))
        threshold.addWidget(self.force_unit)
        layout = QVBoxLayout()
        layout.addLayout(top)
        layout.addWidget(self.splitter, 1)
        layout.addWidget(self.measurement_label)
        layout.addWidget(self.state_label)
        layout.addLayout(controls)
        layout.addLayout(threshold)
        central = QWidget()
        central.setLayout(layout)
        self.setCentralWidget(central)
        self.chart_widget.profileChanged.connect(self.profile_edited)
        self.sync_table()
        self.update_controls()

    def sync_table(self):
        points = self.chart_widget.point_data
        for i, (seconds, temperature) in enumerate(points):
            ti, te = self.time_inputs[i], self.temp_inputs[i]
            ti.blockSignals(True)
            te.blockSignals(True)
            ti.setRange(0 if i == 0 else round(points[i-1][0] + MIN_TIME_GAP, 1),
                        MAX_TIME if i == 5 else round(points[i+1][0] - MIN_TIME_GAP, 1))
            ti.setValue(seconds)
            te.setValue(temperature)
            ti.blockSignals(False)
            te.blockSignals(False)

    def edit_point(self, index):
        self.chart_widget.set_point(index, self.time_inputs[index].value(),
                                    self.temp_inputs[index].value())

    def profile_edited(self):
        self.sync_table()
        self.profile_label.setText('Profil geändert — noch nicht übertragen.')

    def unit_scale(self):
        return 1000.0 if self.force_unit.currentText() == 'mN' else 1.0

    def force_unit_changed(self):
        # Existing force points would otherwise carry mixed units.
        self.chart_widget.series_force.clear()
        self.chart_widget.axisYForce.setRange(0, 30)
        self.statusBar().showMessage('Krafteinheit geändert. Schwelle bei Bedarf erneut am Regler einstellen.', 8000)

    def slider_changed(self, value):
        self.slider_label.setText(f'Kraft-Schwelle: {value / 10.0:.1f} N')
        self.threshold_timer.start()

    def send_threshold(self):
        if self.ready:
            raw = self.slider.value() / 10.0 * self.unit_scale()
            self.tx_queue = deque(cmd for cmd in self.tx_queue if not cmd.startswith('r'))
            self.send_command(f'r{raw:.3f}\n')

    def update_controls(self):
        idle = self.ready and not self.active and not self.profile_batch
        for button in (self.start_btn, self.immediate_btn, self.send_btn):
            button.setEnabled(idle)
        self.stop_btn.setEnabled(self.ready)
        self.tare_btn.setEnabled(self.ready and not self.active)

    def send_command(self, command):
        if not self.ready:
            self.statusBar().showMessage('Controller ist noch nicht verbunden/bereit.', 5000)
            return
        self.tx_queue.append(command)

    def write_command(self, command):
        try:
            if not self.serial_port or not self.serial_port.is_open:
                raise serial.SerialException('Port ist geschlossen')
            payload = command.encode('ascii')
            if self.serial_port.write(payload) != len(payload):
                raise serial.SerialException('Befehl nicht vollständig übertragen')
            self.debug_window.append_text(f'TX: {command.rstrip()}')
            return True
        except (serial.SerialException, OSError) as error:
            self.disconnect_port()
            self.statusBar().showMessage(f'Serieller Fehler: {error}')
            return False

    def flush_tx(self):
        if not self.ready:
            return
        if self.pending_profile is not None:
            if time.monotonic() - self.pending_since > 5:
                self.tx_queue.clear()
                self.pending_profile = None
                self.profile_batch = False
                self.profile_label.setText('Übertragung unvollständig: Profilbestätigung fehlt. Erneut übertragen.')
                self.update_controls()
            return
        if self.tx_queue:
            command = self.tx_queue.popleft()
            if self.write_command(command) and command.startswith('p'):
                self.pending_profile = int(command[1:].split(',')[0])
                self.pending_since = time.monotonic()
        elif self.profile_batch:
            self.profile_batch = False
            if self.chart_widget.point_data == self.sent_profile:
                self.profile_label.setText('Alle 6 Profilpunkte vom Controller bestätigt.')
            else:
                self.profile_label.setText('Profil übertragen; lokale Änderungen noch nicht übertragen.')
            self.update_controls()

    def stop_controller(self):
        self.tx_queue.clear()
        self.pending_profile = None
        if self.profile_batch:
            self.profile_label.setText('Profilübertragung abgebrochen — erneut übertragen.')
        self.profile_batch = False
        self.threshold_timer.stop()
        if self.ready:
            self.write_command('o0\n')
        self.update_controls()

    def send_points(self):
        if not self.ready or self.active or self.profile_batch:
            return
        self.profile_batch = True
        self.sent_profile = list(self.chart_widget.point_data)
        for i, (seconds, temperature) in enumerate(self.sent_profile):
            self.tx_queue.append(f'p{i},{seconds:.1f},{temperature:.1f}\n')
        self.profile_label.setText('Übertragung läuft …')
        self.update_controls()

    def select_com_port(self):
        dialog = COMDialog(self)
        if dialog.exec_() != QDialog.Accepted or not dialog.selected_port():
            return
        self.disconnect_port()
        try:
            self.serial_port = serial.Serial(dialog.selected_port(), 115200,
                                             timeout=0, write_timeout=0.2)
        except (serial.SerialException, OSError) as error:
            QMessageBox.critical(self, 'Verbindung fehlgeschlagen', str(error))
            return
        self.connection_label.setText(f'{dialog.selected_port()} — warte auf Controller …')
        self.profile_label.setText('Profil lokal — bei Bedarf an den Controller übertragen.')
        self.timer.start()

    def disconnect_port(self):
        self.timer.stop()
        self.threshold_timer.stop()
        self.tx_queue.clear()
        self.pending_profile = None
        self.profile_batch = False
        if self.serial_port:
            try:
                self.serial_port.close()
            except (serial.SerialException, OSError):
                pass
        self.serial_port = None
        self.ready = self.active = self.was_active = False
        self.last_elapsed = None
        self.rx_buffer.clear()
        self.connection_label.setText('Nicht verbunden')
        self.state_label.setText('Zustand: unbekannt (Verbindung getrennt)')
        self.update_controls()

    def mark_ready(self):
        if not self.ready:
            self.ready = True
            self.connection_label.setText(f'{self.serial_port.port} — bereit')
            self.update_controls()

    def read_serial_data(self):
        if not self.serial_port:
            return
        try:
            count = self.serial_port.in_waiting
            if count:
                self.rx_buffer.extend(self.serial_port.read(min(count, 65536)))
            # Partial lines stay buffered; the GUI never waits for a newline.
            for _ in range(500):
                end = self.rx_buffer.find(b'\n')
                if end < 0:
                    break
                line = self.rx_buffer[:end].decode('utf-8', errors='replace').strip()
                del self.rx_buffer[:end + 1]
                self.process_line(line)
            if len(self.rx_buffer) > 262144:
                self.rx_buffer.clear()
                self.debug_window.append_text('RX-Puffer verworfen: keine gültigen Zeilen.')
        except (serial.SerialException, OSError) as error:
            self.disconnect_port()
            self.statusBar().showMessage(f'Serieller Fehler: {error}')

    def process_line(self, line):
        self.debug_window.append_text(f'RX: {line}')
        if line == 'BLE device is ready!':
            self.mark_ready()
        match = re.search(r'Profile Written:\s*(\d+)\s*:', line)
        if match and self.pending_profile == int(match.group(1)):
            self.pending_profile = None
        sample = parse_telemetry(line)
        if sample is None:
            return
        self.mark_ready()
        active, enabled, target, seconds, temperature, raw_force = sample
        force = raw_force / self.unit_scale()
        new_sequence = active and (not self.was_active or
                       (self.last_elapsed is not None and seconds < self.last_elapsed))
        if new_sequence:
            self.chart_widget.begin_sequence()
        if active:
            self.chart_widget.add_sample(seconds, temperature, force)
        elif self.was_active and self.last_elapsed is not None and seconds >= self.last_elapsed:
            # The firmware emits one final inactive sample with the end time.
            self.chart_widget.add_sample(seconds, temperature, force)
        self.was_active = self.active = active
        self.last_elapsed = seconds if active else None
        self.measurement_label.setText(
            f'Temperatur: {temperature:.1f} °C   Soll: {target:.1f} °C   '
            f'Kraft: {force:.3f} N   Sequenzzeit: {seconds:.1f} s')
        state = ('Sequenz läuft' if active else
                 'Kraftmodus — wartet auf neue Kraftflanke' if enabled else
                 'Gestoppt — wartet auf Startbefehl')
        self.state_label.setText(f'Zustand: {state}')
        self.update_controls()

    def closeEvent(self, event):
        if self.ready:
            self.stop_controller()
        self.disconnect_port()
        self.debug_window.close()
        event.accept()


if __name__ == '__main__':
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
