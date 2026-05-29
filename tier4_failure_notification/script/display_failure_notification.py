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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from tier4_external_api_msgs.msg import FailureNotificationArray
import yaml


class Message:
    def __init__(self, data):
        self.priority = data["priority"]
        self.text = data["text"]


class DisplayFailureNotification(Node):
    def __init__(self):
        super().__init__("display_failure_notification")
        qos_profile = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub = self.create_subscription(
            FailureNotificationArray,
            "/api/external/get/failure_notification",
            self.callback,
            qos_profile,
        )
        self.messages = self.load_message_file()

    def callback(self, msg):
        messages = [self.messages[item.code] for item in msg.notifications]
        messages.sort(key=lambda message: message.priority)
        print("==========")
        for message in messages:
            print(message.text["ja"])

    def load_message_file(self):
        path = self.declare_parameter("message_file", "").value
        with open(path) as fp:
            data = yaml.safe_load(fp)
        return {code: Message(definition) for code, definition in data.items()}


if __name__ == "__main__":
    try:
        rclpy.init()
        rclpy.spin(DisplayFailureNotification())
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
