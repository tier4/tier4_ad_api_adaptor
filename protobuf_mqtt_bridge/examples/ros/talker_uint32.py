import rclpy
import rclpy.node
import std_msgs.msg


class TalkUInt32Node(rclpy.node.Node):

    def __init__(self):
        super().__init__("talk_uint32_node")
        self.pub = self.create_publisher(std_msgs.msg.UInt32, "/uint32", 10)
        self.count = 0
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = std_msgs.msg.UInt32()
        msg.data = self.count * 10000
        self.pub.publish(msg)
        self.count += 1
        print("publish:", msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = TalkUInt32Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
