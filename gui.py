#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import csv
import struct
import numpy as np
from datetime import datetime

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import serial
from serial.tools import list_ports

# ===================================================
# НАСТРОЙКИ (меняйте здесь под свою прошивку)
# ===================================================
BAUDRATE = 230400               # скорость COM-порта
TIMEOUT = 0.05                  # таймаут чтения (сек)
MAX_HISTORY_BLOCKS = 200        # сколько блоков хранить на спектрограмме
BLOCK_SIZE = 1024                # ДОЛЖНО СОВПАДАТЬ с BLOCK_SIZE в прошивке
SAMPLE_RATE_HZ = 10000            # частота дискретизации (совпадает с SAMPLE_RATE_HZ в прошивке)
# ===================================================

# Формат пакета
PACKET_SYNC = b"\xA5\x5A"
PACKET_END_SYNC = b"\xAB\xBA"
HEADER_FMT = "<I I H h h H"           # block_id, dropped, mean_raw, min_centered, max_centered, sample_count
HEADER_SIZE = struct.calcsize(HEADER_FMT)   # 16 байт
PACKET_SIZE_WITHOUT_SAMPLES = 2 + HEADER_SIZE + 2   # 20 байт
FULL_PACKET_SIZE = PACKET_SIZE_WITHOUT_SAMPLES + BLOCK_SIZE * 2   # 20 + 1024 = 1044 байт

class Packet:
    __slots__ = ('block_id', 'dropped_triggers', 'mean_raw', 'min_centered', 'max_centered', 'sample_count', 'samples')
    def __init__(self, block_id, dropped, mean_raw, min_c, max_c, sample_cnt, samples):
        self.block_id = block_id
        self.dropped_triggers = dropped
        self.mean_raw = mean_raw
        self.min_centered = min_c
        self.max_centered = max_c
        self.sample_count = sample_cnt
        self.samples = samples

class SerialReader(QtCore.QThread):
    packet_received = QtCore.pyqtSignal(object)

    def __init__(self, port, baudrate, timeout):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.running = True
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            buf = bytearray()
            while self.running:
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    buf.extend(data)
                    while True:
                        sync_pos = buf.find(PACKET_SYNC)
                        if sync_pos == -1:
                            if len(buf) > 1:
                                buf = buf[-1:]
                            break
                        if sync_pos > 0:
                            buf = buf[sync_pos:]

                        if len(buf) < PACKET_SIZE_WITHOUT_SAMPLES:
                            break

                        header = struct.unpack_from(HEADER_FMT, buf, 2)
                        block_id, dropped, mean_raw, min_c, max_c, sample_cnt = header

                        if sample_cnt != BLOCK_SIZE:
                            buf = buf[1:]
                            continue

                        needed = PACKET_SIZE_WITHOUT_SAMPLES + sample_cnt * 2
                        if len(buf) < needed:
                            break

                        if buf[needed-2:needed] != PACKET_END_SYNC:
                            buf = buf[1:]
                            continue

                        samples_start = 2 + HEADER_SIZE
                        samples_fmt = f"<{sample_cnt}h"
                        samples = list(struct.unpack_from(samples_fmt, buf, samples_start))

                        packet = Packet(block_id, dropped, mean_raw, min_c, max_c, sample_cnt, samples)
                        self.packet_received.emit(packet)

                        buf = buf[needed:]
        except Exception as e:
            print(f"Serial error: {e}")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.quit()
        self.wait()

class PortDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Выбор COM-порта")
        self.setModal(True)
        layout = QtWidgets.QVBoxLayout(self)
        self.combo = QtWidgets.QComboBox()
        ports = list(list_ports.comports())
        if not ports:
            self.combo.addItem("Нет доступных портов")
        else:
            for p in ports:
                self.combo.addItem(f"{p.device} - {p.description}")
        layout.addWidget(self.combo)
        btn_box = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        btn_box.accepted.connect(self.accept)
        btn_box.rejected.connect(self.reject)
        layout.addWidget(btn_box)

    def get_port(self):
        return self.combo.currentText().split()[0] if self.combo.currentText() else None

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, port):
        super().__init__()
        self.port = port
        self.setWindowTitle(f"VLF Signal Analyzer - {port} @ {BAUDRATE} baud")
        self.setGeometry(100, 100, 1300, 900)

        self.block_ids = []
        self.mean_values = []
        self.min_values = []
        self.max_values = []
        self.dropped_history = []
        self.spectrogram_data = []   # список спектров мощности (по блокам)
        self.freq_axis = None

        self.logging_active = False
        self.csv_file = None
        self.csv_writer = None

        self.init_ui()

        self.reader = SerialReader(self.port, BAUDRATE, TIMEOUT)
        self.reader.packet_received.connect(self.update_display)
        self.reader.start()

    def init_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)

        # Левая панель
        left_widget = QtWidgets.QWidget()
        left_layout = QtWidgets.QFormLayout(left_widget)

        self.lbl_block_id = QtWidgets.QLabel("—")
        self.lbl_dropped = QtWidgets.QLabel("—")
        self.lbl_mean_raw = QtWidgets.QLabel("—")
        self.lbl_mean_volts = QtWidgets.QLabel("—")
        self.lbl_min_centered = QtWidgets.QLabel("—")
        self.lbl_max_centered = QtWidgets.QLabel("—")
        self.lbl_sample_count = QtWidgets.QLabel("—")

        left_layout.addRow("Block ID:", self.lbl_block_id)
        left_layout.addRow("Dropped triggers:", self.lbl_dropped)
        left_layout.addRow("Mean raw (ADC):", self.lbl_mean_raw)
        left_layout.addRow("Mean (V):", self.lbl_mean_volts)
        left_layout.addRow("Min centered:", self.lbl_min_centered)
        left_layout.addRow("Max centered:", self.lbl_max_centered)
        left_layout.addRow("Sample count:", self.lbl_sample_count)

        self.btn_log = QtWidgets.QPushButton("Start logging")
        self.btn_log.clicked.connect(self.toggle_logging)
        left_layout.addRow(self.btn_log)

        # Правая область
        right_widget = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_widget)

        # График амплитуды
        self.amplitude_plot = pg.PlotWidget(title="Signal Amplitude (last block)")
        self.amplitude_plot.setLabel('bottom', 'Sample index')
        self.amplitude_plot.setLabel('left', 'Amplitude (centered)')
        self.amplitude_curve = self.amplitude_plot.plot(pen='y')
        right_layout.addWidget(self.amplitude_plot)

        # Спектрограмма
        self.spectrogram_widget = pg.PlotWidget(title="Spectrogram (Waterfall)")
        self.spectrogram_widget.setLabel('bottom', 'Frequency (Hz)')
        self.spectrogram_widget.setLabel('left', 'Block ID (time)')
        self.spectrogram_image = pg.ImageItem()
        self.spectrogram_widget.addItem(self.spectrogram_image)
        self.spectrogram_widget.setAspectLocked(False)

        # Цветовая шкала (исправленный вызов)
        self.color_bar = pg.ColorBarItem(values=(0, 1), colorMap='inferno')
        # Получаем PlotItem из виджета, чтобы вставить в него ColorBar
        plot_item = self.spectrogram_widget.getPlotItem()
        self.color_bar.setImageItem(self.spectrogram_image, insert_in=plot_item)
        # альтернативно: можно добавить цветовую шкалу как отдельный элемент, но выше работает

        right_layout.addWidget(self.spectrogram_widget)

        main_layout.addWidget(left_widget, 1)
        main_layout.addWidget(right_widget, 4)

        self.setStyleSheet("""
            QMainWindow { background-color: #2b2b2b; }
            QLabel { color: #ddd; }
            QPushButton { background-color: #4a6ea9; color: white; border-radius: 5px; padding: 5px; }
            QPushButton:hover { background-color: #6b8cba; }
        """)
        pg.setConfigOptions(background='k', foreground='w')

    def toggle_logging(self):
        if not self.logging_active:
            filename = f"vlf_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["timestamp", "block_id", "dropped_triggers", "mean_raw",
                                      "mean_volts", "min_centered", "max_centered", "sample_count"])
            self.btn_log.setText("Stop logging")
            self.logging_active = True
        else:
            if self.csv_file:
                self.csv_file.close()
            self.btn_log.setText("Start logging")
            self.logging_active = False

    def adc_to_volts(self, raw):
        return raw * 1.2 / 4095.0

    def compute_spectrum(self, samples, fs):
        n = len(samples)
        window = np.hanning(n)
        fft_vals = np.fft.rfft(np.array(samples, dtype=float) * window)
        power = np.abs(fft_vals) ** 2
        power_db = 10 * np.log10(power + 1e-12)
        freq = np.fft.rfftfreq(n, d=1/fs)
        return freq, power_db

    def update_display(self, packet):
        # --- текстовые параметры ---
        self.lbl_block_id.setText(str(packet.block_id))
        self.lbl_dropped.setText(str(packet.dropped_triggers))
        self.lbl_mean_raw.setText(str(packet.mean_raw))
        mean_v = self.adc_to_volts(packet.mean_raw)
        self.lbl_mean_volts.setText(f"{mean_v:.3f} V")
        self.lbl_min_centered.setText(str(packet.min_centered))
        self.lbl_max_centered.setText(str(packet.max_centered))
        self.lbl_sample_count.setText(str(packet.sample_count))

        # --- логирование ---
        if self.logging_active:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            self.csv_writer.writerow([timestamp, packet.block_id, packet.dropped_triggers,
                                      packet.mean_raw, mean_v, packet.min_centered,
                                      packet.max_centered, packet.sample_count])
            self.csv_file.flush()

        # --- график амплитуды ---
        samples = packet.samples
        if len(samples) > 0:
            self.amplitude_curve.setData(samples[:len(samples)])

        # --- спектрограмма ---
        freq, power_db = self.compute_spectrum(samples, SAMPLE_RATE_HZ)
        self.spectrogram_data.append(power_db)
        if len(self.spectrogram_data) > MAX_HISTORY_BLOCKS:
            self.spectrogram_data.pop(0)

        img = np.array(self.spectrogram_data, dtype=np.float32).T
        self.spectrogram_image.setImage(img, autoLevels=False)
        # if img.size > 0:
        #     self.spectrogram_image.setLevels([np.min(img), np.max(img)])

        x0, x1 = freq[0], freq[-1]
        y0 = 0
        y1 = len(self.spectrogram_data) # Высота картинки растет вместе с данными
        self.spectrogram_image.setRect(QtCore.QRectF(x0, y0, x1 - x0, y1))

        if self.freq_axis is None:
            self.freq_axis = freq
            self.spectrogram_widget.setXRange(x0, x1)
            self.spectrogram_widget.setYRange(0, MAX_HISTORY_BLOCKS)

        # --- статистика ---
        self.block_ids.append(packet.block_id)
        self.mean_values.append(packet.mean_raw)
        self.min_values.append(packet.min_centered)
        self.max_values.append(packet.max_centered)
        self.dropped_history.append(packet.dropped_triggers)

        if len(self.block_ids) > MAX_HISTORY_BLOCKS:
            self.block_ids.pop(0)
            self.mean_values.pop(0)
            self.min_values.pop(0)
            self.max_values.pop(0)
            self.dropped_history.pop(0)

    def closeEvent(self, event):
        self.reader.stop()
        if self.logging_active and self.csv_file:
            self.csv_file.close()
        event.accept()

def main():
    app = QtWidgets.QApplication(sys.argv)
    dialog = PortDialog()
    if dialog.exec_() != QtWidgets.QDialog.Accepted:
        sys.exit(0)
    port = dialog.get_port()
    if not port:
        sys.exit(0)

    window = MainWindow(port)
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
