#!/usr/bin/env python3

import glob
import os
import os.path
import sys

import imageio

from cepton_util.capture import *
from cepton_util.gui import *


class Capture:
    def __init__(self):
        self.window = QMainWindow()
        self.window.closeEvent = self.on_close

        self.update_callbacks = []  # Called every 100Hz
        self._update_timer = QTimer()
        self._update_timer.timeout.connect(self.update)
        self._update_timer.start(100)

        self.monitor_callbacks = []
        self._monitor_timer = QTimer()
        self._monitor_timer.timeout.connect(self.monitor)
        self._monitor_timer.start(10)

        self.settings_dir = os.getcwd()
        self.name = ""
        try:
            self.network_interface = find_network_interface()
        except:
            self.network_interface = None
        self.enable_network = self.network_interface is not None
        self.camera_devices = {}

        self.is_monitors_open = False
        self.camera_readers = {}

        self.is_started = False
        self.capture = None
        self.dummy_capture = None
        self.pcap_capture = None
        self.camera_captures = []

        self.window.setWindowTitle("Cepton Capture")

        widget = QWidget()
        self.window.setCentralWidget(widget)
        self.create_toolbox(widget)

        self.window.show()

    def __del__(self):
        self.stop()
        self.close_monitors()

    def on_close(self, *args):
        self._update_timer.stop()
        self._monitor_timer.stop()

    def update(self):
        for f in self.update_callbacks:
            f()

    def monitor(self):
        for f in self.monitor_callbacks:
            f()

    def open_monitors(self):
        if self.is_monitors_open:
            return
        self.is_monitors_open = True
        self.camera_readers = {
            device: imageio.get_reader(
                "<{}>".format(os.path.basename(device)))
            for device in get_all_camera_devices()
        }

    def close_monitors(self):
        if not self.is_monitors_open:
            return
        self.is_monitors_open = False
        for device, reader in self.camera_readers.items():
            reader.close()
        self.camera_readers = {}

    def create_toolbox(self, widget):
        layout = QVBoxLayout()
        widget.setLayout(layout)
        layout.setAlignment(Qt.AlignTop)

        self.create_toolbox_setup(layout)
        self.create_toolbox_monitor(layout)
        self.create_toolbox_info(layout)

        # Start
        start_stop = QPushButton("Start")
        layout.addWidget(start_stop)

        def on_start_stop():
            if self.is_started:
                self.stop()
            else:
                self.start()
        start_stop.clicked.connect(on_start_stop)

        def update():
            start_stop.setText("Stop" if self.is_started else "Start")
        self.update_callbacks.append(update)

    def create_toolbox_setup(self, parent_layout):
        widget = QGroupBox("Setup")
        parent_layout.addWidget(widget)
        layout = QFormLayout()
        widget.setLayout(layout)

        # Name
        name = QLineEdit()
        layout.addRow("Name", name)

        def on_name():
            self.name = name.text()
        name.editingFinished.connect(on_name)

        # Settings directory
        settings_dir = ButtonLineEdit(self.window.style().standardIcon(
            QStyle.SP_FileDialogStart))
        layout.addRow("Settings Directory", settings_dir)
        settings_dir.setText(self.settings_dir)

        def on_settings_dir():
            self.settings_dir = settings_dir.text()
        settings_dir.editingFinished.connect(on_settings_dir)

        def on_settings_dir():
            self.settings_dir = QFileDialog.getExistingDirectory(
                caption="Settings directory")
            if self.settings_dir is None:
                settings_dir.setText("")
            else:
                settings_dir.setText(self.settings_dir)
        settings_dir.button.clicked.connect(on_settings_dir)

        header = create_toolbox_header("LiDAR")
        layout.addRow(header)

        # Enable network
        enable_network = QCheckBox("Enable")
        layout.addRow(enable_network)

        def on_enable_network():
            self.enable_network = enable_network.isChecked()
        enable_network.stateChanged.connect(on_enable_network)

        # Network interface
        network_interface = VariableLabel()
        layout.addRow("Interface", network_interface)

        header = create_toolbox_header("Camera")
        layout.addRow(header)

        # Camera devices
        camera_devices = QListWidget()
        layout.addRow(create_expanding_label("Devices"), camera_devices)
        camera_devices.setFixedHeight(100)
        camera_devices.setSelectionMode(QAbstractItemView.MultiSelection)

        def on_camera_devices():
            self.camera_devices = \
                set([x.text() for x in camera_devices.selectedItems()])
        camera_devices.selectionModel().selectionChanged.connect(on_camera_devices)

        def update():
            enable_network.setChecked(self.enable_network)
            if self.network_interface is None:
                enable_network.setEnabled(False)
                network_interface.setText("")
            else:
                enable_network.setEnabled(True)
                network_interface.setText(self.network_interface)

            all_camera_devices = get_all_camera_devices()
            if set([camera_devices.item(i).text() for i in range(camera_devices.count())]) \
                    != set(all_camera_devices):
                camera_devices.clear()
                for device in all_camera_devices:
                    item = QListWidgetItem(device)
                    item.setSelected = device in self.camera_devices
                    camera_devices.addItem(item)

        self.update_callbacks.append(update)

    def create_toolbox_monitor(self, parent_layout):
        widget = QGroupBox("Monitor")
        parent_layout.addWidget(widget)
        layout = QFormLayout()
        widget.setLayout(layout)

        enable = QCheckBox("Enable")
        layout.addRow(enable)

        def on_enable():
            if enable.isChecked():
                self.open_monitors()
            else:
                self.close_monitors()
        enable.stateChanged.connect(on_enable)

        def update():
            enable.setEnabled(not self.is_started)
            enable.setChecked(self.is_monitors_open)
        self.update_callbacks.append(update)

        for device in get_all_camera_devices():
            canvas = QLabel()
            canvas.setFixedHeight(100)
            layout.addRow(create_expanding_label(device), canvas)

            def monitor(device=device, canvas=canvas):
                if device not in self.camera_readers:
                    return
                reader = self.camera_readers[device]
                image = reader.get_next_data()
                image = QImage(
                    image.tobytes(), image.shape[1], image.shape[0],
                    QImage.Format_RGB888)
                height = canvas.frameGeometry().height()
                image = image.scaledToHeight(height)
                pixmap = QPixmap(image)
                canvas.setPixmap(pixmap)
            self.monitor_callbacks.append(monitor)

    def create_toolbox_info(self, parent_layout):
        widget = QGroupBox("Info")
        parent_layout.addWidget(widget)
        layout = QFormLayout()
        widget.setLayout(layout)

        # Path
        path = VariableLabel()
        layout.addRow("Path", path)

        # Length
        length = QLabel()
        layout.addRow("Length", length)

        def update():
            if self.capture is None:
                return
            path.setText(self.capture.path)
            length.setText("{:3d}s".format(
                int(round(self.dummy_capture.length))))
        self.update_callbacks.append(update)

    def start(self):
        if self.is_started:
            return

        self.is_started = True
        self.close_monitors()
        self.capture = OutputDataDirectory(name=self.name)
        if self.settings_dir is not None:
            self.capture.copy_settings(self.settings_dir)
        self.dummy_capture = CaptureBase()
        if self.network_interface is not None:
            self.pcap_capture = PCAPCapture(
                self.capture.pcap_path, interface=self.network_interface)
        self.camera_captures = [
            CameraCapture(camera_device, self.capture.camera_path(i))
            for i, camera_device in enumerate(self.camera_devices)
        ]

    def stop(self):
        if not self.is_started:
            return

        self.is_started = False
        if self.capture is not None:
            self.capture = None
        self.dummy_capture = None
        if self.pcap_capture is not None:
            self.pcap_capture.close()
            self.pcap_capture = None
        for camera_capture in self.camera_captures:
            camera_capture.close()
        self.camera_captures = []


def main():
    app = QApplication(sys.argv)

    capture = Capture()
    code = app.exec_()
    sys.exit(code)


if __name__ == "__main__":
    main()
