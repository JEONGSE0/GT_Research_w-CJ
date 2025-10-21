import numpy as np
import asyncio
import csv
import os
from datetime import datetime
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QGridLayout, QMenuBar,
    QHBoxLayout, QLabel, QPushButton, QApplication, QAction
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg
from bluetooth_dialog import BluetoothDialog
import qasync
from scipy.signal import butter, lfilter, iirnotch

EMG_CHAR_UUID = "abcdef12-3456-789a-bcde-f123456789ab"


# ----------------------------
# 필터 함수 정의
# ----------------------------
def butter_bandpass(lowcut, highcut, fs, order=4):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype="band")
    return b, a

def bandpass_filter(x, fs=900, lowcut=20, highcut=400, order=4):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype="band")
    return lfilter(b, a, x)

def notch_filter(x, fs=900, freq=60, Q=30):
    b, a = iirnotch(freq, Q, fs)
    return lfilter(b, a, x)


# ----------------------------
# EMG 데이터 파싱
# ----------------------------
def parse_emg_bundle(data: bytes, vref=4.5, gain=24):
    lsb = 2 * vref / (gain * (2**24 - 1))
    samples = []
    for i in range(0, len(data), 24):
        chunk = data[i:i+24]
        ch = []
        for j in range(8):
            raw = chunk[j*3:j*3+3]
            val = int.from_bytes(raw, byteorder="big", signed=False)
            if val & 0x800000:
                val -= 1 << 24
            voltage_uV = val * lsb * 1e6
            ch.append(voltage_uV)
        samples.append(ch)
    return samples


# ----------------------------
# 메인 클래스
# ----------------------------
class EMGVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EMG Visualizer with Filters & Labels")
        self.setGeometry(100, 100, 1200, 800)

        self.num_channels = 8
        self.sample_rate = 900
        self.display_seconds = 2
        self.display_size = self.sample_rate * self.display_seconds
        self.buffer = np.zeros((self.num_channels, self.display_size))

        self.sample_queue = []
        self.current_y_range = [200.0 for _ in range(self.num_channels)]
        self.sample_counter = 0

        self.ble_client = None
        self.csv_file = None
        self.csv_writer = None
        self.label_state = 0

        self.filter_actions = {}  # 메뉴 필터 액션들 저장

        self._init_ui()
        self._init_timer()

    # ----------------------------
    # UI 초기화
    # ----------------------------
    def _init_ui(self):
        self.setStyleSheet("background-color: white;")
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        layout = QVBoxLayout(self.central_widget)

        menubar = QMenuBar()
        layout.setMenuBar(menubar)
        menubar.addMenu("File")
        connect_menu = menubar.addMenu("Connect")
        bluetooth_action = connect_menu.addAction("Bluetooth")
        bluetooth_action.triggered.connect(self.connect_bluetooth)
        menubar.addMenu("Data")
        menubar.addMenu("Run")
        menubar.addMenu("Help")

        # ---------------- Filter 메뉴 ----------------
        filter_menu = menubar.addMenu("Filter")

        self.filter_actions["Bandpass"] = QAction("Bandpass (20-100Hz)", self, checkable=True)
        filter_menu.addAction(self.filter_actions["Bandpass"])

        self.filter_actions["Notch"] = QAction("Notch (60Hz)", self, checkable=True)
        filter_menu.addAction(self.filter_actions["Notch"])

        self.filter_actions["RMS"] = QAction("RMS Envelope", self, checkable=True)
        filter_menu.addAction(self.filter_actions["RMS"])

        # ---------------- Status Bar ----------------
        self.freq_label = QLabel("📡 Sampling: -- Hz")
        self.freq_label.setStyleSheet("color: #0044AA; font-weight: bold;")
        self.freq_label.setAlignment(Qt.AlignLeft)

        freq_layout = QHBoxLayout()
        freq_layout.addWidget(self.freq_label)
        freq_layout.addStretch()
        layout.addLayout(freq_layout)

        self.status_label = QLabel("🔌 Not Connected")
        self.status_label.setStyleSheet("color: gray; font-weight: bold;")
        self.status_label.setAlignment(Qt.AlignRight)

        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.setStyleSheet("font-weight: bold; padding: 4px;")
        self.disconnect_button.clicked.connect(self.disconnect_bluetooth)
        self.disconnect_button.hide()

        status_layout = QHBoxLayout()
        status_layout.addStretch()
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(self.disconnect_button)
        layout.addLayout(status_layout)

        grid = QGridLayout()
        grid.setHorizontalSpacing(20)
        grid.setVerticalSpacing(20)
        layout.addLayout(grid)

        self.plots = []
        self.curves = []

        for i in range(self.num_channels):
            title = QLabel(f"CH {i+1} (µV)")
            title.setAlignment(Qt.AlignCenter)
            title.setFont(QFont("Segoe UI", 10, QFont.Bold))

            plot = pg.PlotWidget()
            plot.setBackground("white")
            plot.showGrid(x=True, y=True)
            curve = plot.plot(pen=pg.mkPen("#0044AA", width=3))

            vbox = QVBoxLayout()
            vbox.addWidget(title)
            vbox.addWidget(plot)

            widget = QWidget()
            widget.setLayout(vbox)
            grid.addWidget(widget, i // 4, i % 4)

            self.plots.append(plot)
            self.curves.append(curve)

        # ---------------- Label 버튼 (아래 가운데) ----------------
        self.label_button = QPushButton("Label OFF")
        self.label_button.setCheckable(True)
        self.label_button.clicked.connect(self.toggle_label)
        self.label_button.setFixedWidth(100)
        self.label_button.setStyleSheet("font-weight: bold; padding: 4px;")
        self.label_button.hide()

        label_layout = QHBoxLayout()
        label_layout.addStretch()
        label_layout.addWidget(self.label_button)
        label_layout.addStretch()
        layout.addLayout(label_layout)

        # ---------------- Filter 상태 표시 (오른쪽 아래) ----------------
        self.filter_status_label = QLabel("Filters: None")
        self.filter_status_label.setStyleSheet("color: #AA4400; font-weight: bold;")
        self.filter_status_label.setAlignment(Qt.AlignRight)

        filter_status_layout = QHBoxLayout()
        filter_status_layout.addStretch()
        filter_status_layout.addWidget(self.filter_status_label)
        layout.addLayout(filter_status_layout)

    # ----------------------------
    # 타이머
    # ----------------------------
    def _init_timer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(20)

        self.freq_timer = QTimer()
        self.freq_timer.timeout.connect(self.update_sampling_rate)
        self.freq_timer.start(1000)

    # ----------------------------
    # Plot 업데이트
    # ----------------------------
    def update_plot(self):
        while self.sample_queue:
            sample = self.sample_queue.pop(0)
            self.insert_new_sample(sample)

    def insert_new_sample(self, sample):
        self.buffer = np.roll(self.buffer, -1, axis=1)
        self.buffer[:, -1] = sample
        t = np.linspace(-self.display_seconds, 0, self.display_size)

        self.sample_counter += 1

        for i in range(self.num_channels):
            self.curves[i].setData(t, self.buffer[i])

            buf = self.buffer[i][-200:]
            center = np.mean(buf)
            peak = np.max(np.abs(buf - center))
            target_range = max(peak * 1.2, 100)

            self.current_y_range[i] = (
                0.8 * self.current_y_range[i] + 0.2 * target_range
            )
            yr = self.current_y_range[i]
            self.plots[i].setYRange(center - yr, center + yr)

    def update_sampling_rate(self):
        self.freq_label.setText(f"📡 Sampling: {self.sample_counter} Hz")
        self.sample_counter = 0

    # ----------------------------
    # CSV 저장
    # ----------------------------
    def start_csv_logging(self):
        os.makedirs("data", exist_ok=True)
        filename = datetime.now().strftime("emg_data_%Y%m%d_%H%M%S.csv")
        filepath = os.path.join("data", filename)
        self.csv_file = open(filepath, mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([f"CH{i+1}" for i in range(self.num_channels)] + ["Label"])

    # ----------------------------
    # Bluetooth 연결
    # ----------------------------
    def connect_bluetooth(self):
        dialog = BluetoothDialog()
        result = dialog.exec_()

        if result == dialog.Accepted and dialog.connected_device_name:
            self.status_label.setText(f"✅ Connected: {dialog.connected_device_name}")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            self.disconnect_button.show()

            self.ble_client = dialog.get_client()
            self.start_csv_logging()

            self.label_state = 0
            self.label_button.setChecked(False)
            self.label_button.setText("Label OFF")
            self.label_button.show()

            asyncio.create_task(self.start_emg_notify())
        else:
            self.status_label.setText("❌ Failed to connect")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
            self.disconnect_button.hide()

    def toggle_label(self):
        if self.label_button.isChecked():
            self.label_state = 1
            self.label_button.setText("Label ON")
            self.label_button.setStyleSheet("background-color: yellow; font-weight: bold; padding: 4px;")
        else:
            self.label_state = 0
            self.label_button.setText("Label OFF")
            self.label_button.setStyleSheet("font-weight: bold; padding: 4px;")

    # ----------------------------
    # 필터 상태 추적
    # ----------------------------
    def get_active_filters(self):
        active = [name for name, action in self.filter_actions.items() if action.isChecked()]
        if active:
            self.filter_status_label.setText("Filters: " + ", ".join(active))
        else:
            self.filter_status_label.setText("Filters: None")
        return active

    # ----------------------------
    # 데이터 수신 & 처리
    # ----------------------------
    async def start_emg_notify(self):
        async def handler(_, data):
            samples = parse_emg_bundle(data)

            # 항상 raw 데이터 저장
            if self.csv_writer:
                for sample in samples:
                    self.csv_writer.writerow(sample + [self.label_state])

            # 그래프용 필터 적용
            processed_samples = []
            active_filters = self.get_active_filters()

            for s in samples:
                arr = np.array(s)
                if "Bandpass" in active_filters:
                    arr = bandpass_filter(arr, fs=self.sample_rate, lowcut=20, highcut=100)
                if "Notch" in active_filters:
                    arr = notch_filter(arr, fs=self.sample_rate)
                processed_samples.append(arr.tolist())

            self.sample_queue.extend(processed_samples)

        await self.ble_client.start_notify(EMG_CHAR_UUID, handler)

    # ----------------------------
    # Bluetooth 해제
    # ----------------------------
    def disconnect_bluetooth(self):
        self.status_label.setText("🔌 Not Connected")
        self.status_label.setStyleSheet("color: gray; font-weight: bold;")
        self.disconnect_button.hide()
        self.label_button.hide()

        if self.ble_client:
            asyncio.create_task(self.ble_client.disconnect())
            self.ble_client = None

        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None


# ----------------------------
# 실행부
# ----------------------------
if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(loop)

    window = EMGVisualizer()
    window.show()

    with loop:
        loop.run_forever()
