# Copyright 2021 Tier IV, Inc.
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

# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import base64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage



class ImageCallback():
    def __init__(self, node, topic, publisher):
        self._node = node
        self._topic = topic
        self._publisher = publisher

    def get_callback(self):
        return self.callback

    def callback(self, message):
        try:
            base64_image = String()
            data = base64.b64encode(bytes(message.data)).decode()
            base64_image.data = data
            self._publisher.publish(base64_image)
        except:
            self._node.get_logger().warn('Cannot subscribe: "%s"' % self._topic)


class ImageRePublisher(Node):

    def __init__(self):
        super().__init__('image_converter')

        self._publisher_list = {}
        self._subscribe_callback_list = {}

        self.declare_parameter("image_topic_name_list", [''])
        self._image_topic_name_list = (
            self.get_parameter("image_topic_name_list").value
        )

        self.get_logger().info('I heard: "%s"' % type(self._image_topic_name_list))

        for topic in self._image_topic_name_list:
            self.get_logger().info('I heard: "%s"' % topic)
            self._publisher_list[topic] = self.create_publisher(String, '/api/external/get/base64/image' + topic, 10)

            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            self._subscribe_callback_list[topic] = ImageCallback(self, topic, self._publisher_list[topic])

            self.create_subscription(
                CompressedImage, topic, self._subscribe_callback_list[topic].get_callback() , qos_profile=qos_profile
            )

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ImageRePublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
