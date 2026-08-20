#!/usr/bin/env python3

# Copyright 2026 The Autoware Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import pathlib

from autoware_internal_debug_msgs.msg import StringStamped
import rclpy
import rclpy.node
import rclpy.qos
from tier4_external_api_msgs.msg import FailureNotificationArray


class ErrorCodeMessage(rclpy.node.Node):
    def __init__(self):
        super().__init__("error_code_message")
        qos_status = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.sub = self.create_subscription(
            FailureNotificationArray,
            "/api/external/get/failure_notification",
            self.callback,
            qos_status,
        )
        self.pub = self.create_publisher(StringStamped, "/error_code_message", qos_status)
        self.info = self.load_info()

    def callback(self, msg):
        text = StringStamped()
        for failure in msg.notifications:
            info = self.info.get(failure.code, None)
            situation = info["situation"]["ja"] if info else "NO INFO"
            solution = info["solution"]["ja"] if info else "NO INFO"
            text.data += f"{failure.code}: {situation} {solution}\n"
        text.stamp = self.get_clock().now().to_msg()
        self.pub.publish(text)

    def load_info(self):
        path = pathlib.Path(self.declare_parameter("path", "").value)
        if not path.exists():
            raise FileNotFoundError(f"File not found: {path}")
        return json.loads(path.read_text())


if __name__ == "__main__":
    try:
        rclpy.init()
        rclpy.spin(ErrorCodeMessage())
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
