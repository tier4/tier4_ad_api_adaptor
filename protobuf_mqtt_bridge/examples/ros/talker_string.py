import rclpy
import rclpy.node
import std_msgs.msg


class TalkStringNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("talk_string_node")
        self.pub = self.create_publisher(std_msgs.msg.String, "/string", 10)
        self.count = 0
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = std_msgs.msg.String()
        msg.data = f"Hello {self.count}"
        self.pub.publish(msg)
        self.count += 1
        print("publish:", msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = TalkStringNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
