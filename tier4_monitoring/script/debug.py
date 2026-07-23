#!/usr/bin/env python3

# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys

from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtWidgets import QMainWindow
import rclpy
import rclpy.executors
import rclpy.node
from tier4_external_api_msgs.msg import MonitoringHeartbeat
from tier4_external_api_msgs.msg import MonitoringMode
from tier4_external_api_msgs.msg import MonitoringStatus
from tier4_external_api_msgs.srv import ChangeMonitoringMode


class RclpyWorker(QtCore.QObject):
    def __init__(self):
        super().__init__()
        self.exec = rclpy.executors.SingleThreadedExecutor()
        self.node = rclpy.create_node("monitoring_api_debug_tool")

    def spin(self):
        self.exec.add_node(self.node)
        self.exec.spin()

    def quit(self):  # noqa: A003
        self.exec.shutdown()


class RclpyThread:
    def __init__(self):
        self.worker = RclpyWorker()
        self.thread = QtCore.QThread()

    def exec(self):  # noqa: A003
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.spin)
        self.thread.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)
        self.thread.start()

    def quit(self):  # noqa: A003
        self.worker.quit()
        self.thread.quit()
        self.thread.wait()


class RclpyManager:
    def __init__(self, argv):
        self.argv = argv

    def init(self):
        rclpy.init()
        self.thread = RclpyThread()

    def exec(self):  # noqa: A003
        self.thread.exec()

    def quit(self):  # noqa: A003
        self.thread.quit()
        rclpy.shutdown()

    def node(self):
        return self.thread.worker.node


class HeartbeatButton:
    def __init__(self, node: rclpy.node.Node, topic: str):
        self.node = node
        self.pub = node.create_publisher(MonitoringHeartbeat, topic, 1)
        self.timer = node.create_timer(0.5, self.on_timer)
        self.button = QtWidgets.QPushButton("Heartbeat")
        self.button.setCheckable(True)

    def on_timer(self):
        if self.button.isChecked():
            msg = MonitoringHeartbeat()
            msg.stamp = self.node.get_clock().now().to_msg()
            self.pub.publish(msg)


class ChangeButtons:
    def __init__(self, node: rclpy.node.Node, service: str):
        self.node = node
        self.cli = node.create_client(ChangeMonitoringMode, service)
        self.button1 = QtWidgets.QPushButton("Unavailable")
        self.button2 = QtWidgets.QPushButton("Available")
        self.button3 = QtWidgets.QPushButton("Operating")
        self.button1.clicked.connect(lambda: self.request(MonitoringMode.UNAVAILABLE))
        self.button2.clicked.connect(lambda: self.request(MonitoringMode.AVAILABLE))
        self.button3.clicked.connect(lambda: self.request(MonitoringMode.OPERATING))

    def request(self, mode):
        req = ChangeMonitoringMode.Request()
        req.mode = mode
        self.cli.call_async(req)


class StatusDisplay:
    def __init__(self, node: rclpy.node.Node, topic: str):
        self.node = node
        self.sub = node.create_subscription(MonitoringStatus, topic, self.on_status, 1)
        self.label1 = QtWidgets.QLabel()
        self.label2 = QtWidgets.QLabel()

    def on_status(self, msg: MonitoringStatus):
        self.label1.setText(self.mode_text.get(msg.mode, "Unknown"))
        self.label2.setText("Operating" if msg.operating else "")

    mode_text = {
        MonitoringMode.TIMEOUT: "Timeout",
        MonitoringMode.UNAVAILABLE: "Unavailable",
        MonitoringMode.AVAILABLE: "Available",
        MonitoringMode.OPERATING: "Operating",
    }


class Operator:
    def __init__(self, node: rclpy.node.Node, ns: str):
        self.heartbeat = HeartbeatButton(node, ns + "/heartbeat")
        self.change = ChangeButtons(node, ns + "/change")
        self.status = StatusDisplay(node, ns + "/status")

    def set_layout(self, layout, row, label):
        layout.addWidget(QtWidgets.QLabel(label), row, 0)
        layout.addWidget(self.heartbeat.button, row, 1)
        layout.addWidget(self.change.button1, row, 2)
        layout.addWidget(self.change.button2, row, 3)
        layout.addWidget(self.change.button3, row, 4)
        layout.addWidget(self.status.label1, row, 5)
        layout.addWidget(self.status.label2, row, 6)


class MainWidget(QtWidgets.QWidget):
    def __init__(self, node):
        super().__init__()
        self.supervisor_driver = Operator(node, "/supervisor/driver")
        self.supervisor_mot = Operator(node, "/supervisor/mot")
        self.supervisor_fms = Operator(node, "/supervisor/fms")
        self.advisor_mot = Operator(node, "/advisor/mot")
        self.advisor_fms = Operator(node, "/advisor/fms")

        layout = QtWidgets.QGridLayout()
        self.setLayout(layout)
        self.supervisor_driver.set_layout(layout, 1, "Supervisor Driver")
        self.supervisor_mot.set_layout(layout, 2, "Supervisor MOT")
        self.supervisor_fms.set_layout(layout, 3, "Supervisor FMS")
        self.advisor_mot.set_layout(layout, 4, "Advisor MOT")
        self.advisor_fms.set_layout(layout, 5, "Advisor FMS")

        layout.addWidget(QtWidgets.QLabel("Operator"), 0, 0)
        layout.addWidget(QtWidgets.QLabel("Current Mode"), 0, 5)
        layout.addWidget(QtWidgets.QLabel("Operating Flag"), 0, 6)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ros = RclpyManager(sys.argv)
    ros.init()

    window = QMainWindow()
    window.setCentralWidget(MainWidget(ros.node()))
    window.show()

    ros.exec()
    ret = app.exec_()
    ros.quit()
    sys.exit(ret)
