import paho.mqtt.client
import autoware_protobuf.std_msgs.msg.string_pb2
import autoware_protobuf.std_msgs.msg.uint32_pb2

def create_mqtt_client():
    client = paho.mqtt.client.Client()
    client.on_connect = on_connect
    client.message_callback_add("/string",on_string)
    client.message_callback_add("/uint32",on_uint32)
    client.on_message = on_message
    client.connect("localhost", 1883, 60)
    return client

def on_connect(client, userdata, flags, result):
    print(f"Connected: {result}")

def on_message(client, userdata, msg):
    print("unknown topic:", msg.topic)

def on_string(client, userdata, msg):
    message = autoware_protobuf.std_msgs.msg.string_pb2.String()
    message.ParseFromString(msg.payload)
    print(message.data)

def on_uint32(client, userdata, msg):
    message = autoware_protobuf.std_msgs.msg.uint32_pb2.UInt32()
    message.ParseFromString(msg.payload)
    print(message.data)

if __name__ == "__main__":
    client = create_mqtt_client()
    client.subscribe("/string")
    client.subscribe("/uint32")
    client.loop_forever()
