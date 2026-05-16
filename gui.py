import sys
import csv
import struct
import time
import threading
from datetime import datetime

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import serial
from serial.tools import list_ports

# ----------------------------------------------------------------------
# Конфигурация последовательного порта (можно будет выбрать при запуске)
# ----------------------------------------------------------------------
BAUDRATE = 921600
TIMEOUT = 0.05

# Формат пакета (без массива samples)
PACKET_SYNC = b"\xA5\x5A"
PACKET_END_SYNC = b"\xAB\xBA"
HEADER_FMT = "<I I H h h H"  # block_id, dropped_triggers, mean_raw, min_centered, max_centered, sample_count
HEADER_SIZE = struct.calcsize(HEADER_FMT)   # 16 байт
PACKET_SIZE = 2 + HEADER_SIZE + 2           # 20 байт

# ----------------------------------------------------------------------
# Класс для хранения данных одного пакета
# ----------------------------------------------------------------------
class Packet:
    __slots__ = ('block_id', 'dropped_triggers', 'mean_raw', 'min_centered', 'max_centered', 'sample_count')
    def __init__(self, block_id, dropped, mean_raw, min_c, max_c, sample_cnt):
        self.block_id = block_id
        self.dropped_triggers = dropped
        self.mean_raw = mean_raw
        self.min_centered = min_c
        self.max_centered = max_c
        self.sample_count = sample_cnt

# ----------------------------------------------------------------------
# Поток для чтения из UART
# ----------------------------------------------------------------------
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
                        # Поиск sync-слова
                        sync_pos = buf.find(PACKET_SYNC)
                        if sync_pos == -1:
                            if len(buf) > 1:
                                buf = buf[-1:]   # оставляем только последний байт
                            break
                        if sync_pos > 0:
                            buf = buf[sync_pos:]   # отрезаем мусор до sync

                        if len(buf) < PACKET_SIZE:
                            break

                        # Распаковка заголовка
                        header = struct.unpack_from(HEADER_FMT, buf, 2)
                        block_id, dropped, mean_raw, min_c, max_c, sample_cnt = header

                        # Проверка end_sync
                        if buf[PACKET_SIZE-2:PACKET_SIZE] != PACKET_END_SYNC:
                            # Сдвигаемся на 1 байт и ищем заново
                            buf = buf[1:]
                            continue

                        # Успешно приняли пакет
                        packet = Packet(block_id, dropped, mean_raw, min_c, max_c, sample_cnt)
                        self.packet_received.emit(packet)

                        # Удаляем обработанный пакет из буфера
                        buf = buf[PACKET_SIZE:]

        except Exception as e:
            print(f"Ошибка в потоке последовательного порта: {e}")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.quit()
        self.wait()

# ----------------------------------------------------------------------
# Диалог выбора порта при запуске
# ----------------------------------------------------------------------
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

# ----------------------------------------------------------------------
# Главное окно
# ----------------------------------------------------------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, port):
        super().__init__()
        self.port = port
        self.setWindowTitle(f"VLF Signal Analyzer - {port}")
        self.setGeometry(100, 100, 1200, 800)

        # Данные для графиков (по мере поступления пакетов)
        self.block_ids = []        # номера блоков
        self.mean_values = []      # mean_raw
        self.min_values = []       # min_centered
        self.max_values = []       # max_centered
        self.max_history = 200     # сколько последних блоков показывать

        # Флаг логирования
        self.logging_active = False
        self.csv_file = None
        self.csv_writer = None

        self.init_ui()

        # Запуск потока чтения
        self.reader = SerialReader(self.port, BAUDRATE, TIMEOUT)
        self.reader.packet_received.connect(self.update_display)
        self.reader.start()

    def init_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)

        # Левая панель с параметрами
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

        # Правая область с графиком
        right_widget = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_widget)

        # График значений mean, min, max
        self.plot_widget = pg.PlotWidget(title="Mean / Min / Max vs Block ID")
        self.plot_widget.setLabel('bottom', 'Block ID')
        self.plot_widget.setLabel('left', 'Value (ADC counts)')
        self.plot_widget.addLegend()

        self.curve_mean = self.plot_widget.plot(pen=pg.mkPen('b', width=2), name='Mean raw')
        self.curve_min = self.plot_widget.plot(pen=pg.mkPen('g', width=1), name='Min centered')
        self.curve_max = self.plot_widget.plot(pen=pg.mkPen('r', width=1), name='Max centered')

        # Заливка области между min и max (опционально)
        self.fill_item = pg.FillBetweenItem(self.curve_min, self.curve_max, brush=(100,100,200,50))
        self.plot_widget.addItem(self.fill_item)

        right_layout.addWidget(self.plot_widget)

        # Второй график — "сине-оранжевые штучки": пока заглушка, потом заменим на спектрограмму
        self.hist_widget = pg.PlotWidget(title="Dropped triggers over time")
        self.hist_widget.setLabel('bottom', 'Block ID')
        self.hist_widget.setLabel('left', 'Dropped count')
        self.dropped_curve = self.hist_widget.plot(pen='y', symbol='o', symbolSize=4)
        right_layout.addWidget(self.hist_widget)

        main_layout.addWidget(left_widget, 1)
        main_layout.addWidget(right_widget, 4)

        # Стилизация
        self.setStyleSheet("""
            QMainWindow { background-color: #2b2b2b; }
            QLabel { color: #ddd; }
            QPushButton { background-color: #4a6ea9; color: white; border-radius: 5px; padding: 5px; }
            QPushButton:hover { background-color: #6b8cba; }
        """)
        pg.setConfigOptions(background='w', foreground='k')

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
        # Предполагаем опорное напряжение 1.2 В, разрешение 12 бит (0-4095)
        return raw * 1.2 / 4095.0

    def update_display(self, packet):
        # 1. Обновляем текстовые поля
        self.lbl_block_id.setText(str(packet.block_id))
        self.lbl_dropped.setText(str(packet.dropped_triggers))
        self.lbl_mean_raw.setText(str(packet.mean_raw))
        mean_v = self.adc_to_volts(packet.mean_raw)
        self.lbl_mean_volts.setText(f"{mean_v:.3f} V")
        self.lbl_min_centered.setText(str(packet.min_centered))
        self.lbl_max_centered.setText(str(packet.max_centered))
        self.lbl_sample_count.setText(str(packet.sample_count))

        # 2. Логирование в CSV если нужно
        if self.logging_active:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            self.csv_writer.writerow([timestamp, packet.block_id, packet.dropped_triggers,
                                      packet.mean_raw, mean_v, packet.min_centered,
                                      packet.max_centered, packet.sample_count])
            self.csv_file.flush()

        # 3. Добавляем данные в списки для графиков
        self.block_ids.append(packet.block_id)
        self.mean_values.append(packet.mean_raw)
        self.min_values.append(packet.min_centered)
        self.max_values.append(packet.max_centered)

        # Ограничиваем длину истории
        if len(self.block_ids) > self.max_history:
            self.block_ids.pop(0)
            self.mean_values.pop(0)
            self.min_values.pop(0)
            self.max_values.pop(0)

        # Обновляем графики
        self.curve_mean.setData(self.block_ids, self.mean_values)
        self.curve_min.setData(self.block_ids, self.min_values)
        self.curve_max.setData(self.block_ids, self.max_values)

        # График dropped triggers (тоже в зависимости от block_id)
        # Чтобы не хранить отдельно, используем те же block_ids и массив dropped
        if not hasattr(self, 'dropped_history'):
            self.dropped_history = []
        self.dropped_history.append(packet.dropped_triggers)
        if len(self.dropped_history) > self.max_history:
            self.dropped_history.pop(0)
        self.dropped_curve.setData(self.block_ids, self.dropped_history)

    def closeEvent(self, event):
        self.reader.stop()
        if self.logging_active and self.csv_file:
            self.csv_file.close()
        event.accept()

# ----------------------------------------------------------------------
# Точка входа
# ----------------------------------------------------------------------
def main():
    app = QtWidgets.QApplication(sys.argv)

    # Диалог выбора порта
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
