import asyncio
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QPushButton, QListWidget, QHBoxLayout,
    QLabel, QLineEdit, QMessageBox, QSpacerItem, QSizePolicy
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from bleak import BleakScanner, BleakClient
from qasync import asyncSlot


class ScanWorker(QThread):
    device_found = pyqtSignal(str, str)
    scan_stopped = pyqtSignal()

    def __init__(self, filter_keyword=None):
        super().__init__()
        self._running = True
        self.filter_keyword = filter_keyword

    def run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.scan())

    async def scan(self):
        while self._running:
            devices = await BleakScanner.discover()
            for d in devices:
                name = d.name or ""
                if name == "Unknown" or name == "":
                    continue
                if self.filter_keyword and self.filter_keyword.lower() not in name.lower():
                    continue
                self.device_found.emit(name, d.address)
            await asyncio.sleep(2.0)
        self.scan_stopped.emit()

    def stop(self):
        self._running = False


class BluetoothDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Bluetooth Scan")
        self.setMinimumWidth(400)

        self.connected_device_name = None
        self.selected_address = None
        self.client = None

        self.device_list = QListWidget()
        self.filter_input = QLineEdit()
        self.filter_input.setPlaceholderText("Search device name...")

        self.scan_button = QPushButton("Scan")
        self.stop_button = QPushButton("Stop")
        self.connect_button = QPushButton("Connect")
        self.status_label = QLabel("")

        self.stop_button.setEnabled(False)
        self.connect_button.setEnabled(False)

        layout = QVBoxLayout()
        layout.addWidget(self.filter_input)
        layout.addWidget(self.device_list)

        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.scan_button)
        btn_layout.addWidget(self.stop_button)
        layout.addLayout(btn_layout)

        layout.addWidget(self.connect_button)
        layout.addWidget(self.status_label)
        layout.addItem(QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Expanding))
        self.setLayout(layout)

        self.scan_button.clicked.connect(self.start_scan)
        self.stop_button.clicked.connect(self.stop_scan)
        self.connect_button.clicked.connect(self.connect_device)
        self.device_list.itemSelectionChanged.connect(self.handle_selection)

        self.worker = None

    def start_scan(self):
        self.device_list.clear()
        self.scan_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.connect_button.setEnabled(False)
        keyword = self.filter_input.text()
        self.worker = ScanWorker(filter_keyword=keyword)
        self.worker.device_found.connect(self.add_device)
        self.worker.scan_stopped.connect(self.scan_finished)
        self.worker.start()

    def stop_scan(self):
        if self.worker:
            self.worker.stop()
        self.stop_button.setEnabled(False)
        self.scan_button.setEnabled(True)

    def scan_finished(self):
        self.scan_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def add_device(self, name, address):
        label = f"{name} ({address})"
        for i in range(self.device_list.count()):
            if self.device_list.item(i).text() == label:
                return
        self.device_list.addItem(label)

    def handle_selection(self):
        selected = self.device_list.selectedItems()
        self.connect_button.setEnabled(bool(selected))

    @asyncSlot()
    async def connect_device(self):
        selected = self.device_list.selectedItems()
        if not selected:
            return

        label = selected[0].text()
        name, address = label.split(" (")
        address = address.rstrip(")")

        self.status_label.setText("🔄 Connecting...")
        self.status_label.repaint()
        self.scan_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self.connect_button.setEnabled(False)

        result = await self.try_connect(address)

        if result:
            self.status_label.setText("✅ Connecting Success!")
            self.connected_device_name = name
            self.accept()
        else:
            self.status_label.setText("❌ Connection Failed")
            self.scan_button.setEnabled(True)
            self.connect_button.setEnabled(True)

    async def try_connect(self, address):
        try:
            print(f"Trying to connect to {address}...")
            self.client = BleakClient(address)
            await self.client.connect()
            print(f"Connected? {self.client.is_connected}")
            return self.client.is_connected
        except Exception as e:
            print(f"Connection error: {e}")
            return False

    def get_client(self):
        return self.client

    def closeEvent(self, event):
        if self.worker:
            self.worker.stop()
        event.accept()
