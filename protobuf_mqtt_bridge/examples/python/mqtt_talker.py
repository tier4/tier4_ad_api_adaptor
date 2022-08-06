import paho.mqtt.client
import math
import time
import autoware_protobuf.geometry_msgs.msg.pose_stamped_pb2

def create_mqtt_client():
    client = paho.mqtt.client.Client()
    client.on_connect = on_connect
    client.connect("localhost", 1883, 60)
    return client

def on_connect(client, userdata, flags, result):
    print(f"Connected: {result}")

def create_message(count):
    stamp = math.modf(time.time())
    msg = autoware_protobuf.geometry_msgs.msg.pose_stamped_pb2.PoseStamped()
    msg.header.stamp.sec = int(stamp[1])
    msg.header.stamp.nanosec = int(stamp[0] * 1e9)
    msg.header.frame_id = "map"
    msg.pose.position.x = math.cos(math.radians(count))
    msg.pose.position.y = math.sin(math.radians(count))
    msg.pose.orientation.z = math.sin(math.radians(count/2.0))
    msg.pose.orientation.w = math.cos(math.radians(count/2.0))
    return msg

def publish_loop(client):
    try:
        count = 0
        while True:
            message = create_message(count)
            print(message)
            client.publish("/pose_stamped", message.SerializeToString())
            count = (count + 10) % 360
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    client = create_mqtt_client()
    client.loop_start()
    publish_loop(client)
    client.loop_stop()
