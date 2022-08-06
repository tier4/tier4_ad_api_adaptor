import paho.mqtt.client
import rclpy
import rclpy.node
import std_msgs.msg
import geometry_msgs.msg
import autoware_protobuf.geometry_msgs.msg.pose_stamped_pb2
import autoware_protobuf.std_msgs.msg.string_pb2
import autoware_protobuf.std_msgs.msg.uint32_pb2

class PubBridge(rclpy.node.Node):

    def __init__(self, mqtt):
        super().__init__("protobuf_mqtt_server")
        self.mqtt = mqtt
        self.sub_string = self.create_subscription(std_msgs.msg.String, "/string", self.on_string, 1)
        self.sub_uint32 = self.create_subscription(std_msgs.msg.UInt32, "/uint32", self.on_uint32, 1)
        self.pub_pose = self.create_publisher(geometry_msgs.msg.PoseStamped, "/pose_stamped", qos_profile=1)

    def on_string(self, msg):
        proto = autoware_protobuf.std_msgs.msg.string_pb2.String()
        proto.data = msg.data
        self.mqtt.publish("/string", proto.SerializeToString())

    def on_uint32(self, msg):
        proto = autoware_protobuf.std_msgs.msg.uint32_pb2.UInt32()
        proto.data = msg.data
        self.mqtt.publish("/uint32", proto.SerializeToString())

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

def on_message(client, userdata, msg):
    if msg.topic == "/pose_stamped":
        proto = autoware_protobuf.geometry_msgs.msg.pose_stamped_pb2.PoseStamped()
        proto.ParseFromString(msg.payload)
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.stamp.sec = proto.header.stamp.sec
        pose.header.stamp.nanosec = proto.header.stamp.nanosec
        pose.header.frame_id = proto.header.frame_id
        pose.pose.position.x = proto.pose.position.x
        pose.pose.position.y = proto.pose.position.y
        pose.pose.position.z = proto.pose.position.z
        pose.pose.orientation.x = proto.pose.orientation.x
        pose.pose.orientation.y = proto.pose.orientation.y
        pose.pose.orientation.z = proto.pose.orientation.z
        pose.pose.orientation.w = proto.pose.orientation.w
        userdata.pub_pose.publish(pose)

def create_mqtt_client():
    client = paho.mqtt.client.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883, 60)
    return client

def ros2_main(node):
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("sigint")

if __name__ == "__main__":
    rclpy.init()
    mqtt = create_mqtt_client()
    node = PubBridge(mqtt)
    mqtt.user_data_set(node)
    mqtt.subscribe("/pose_stamped")
    mqtt.loop_start()
    ros2_main(node)
    mqtt.loop_stop()
    rclpy.shutdown()
