const client = mqtt.connect("ws://localhost:9001");
const autoware_protobuf = {}

protobuf.load("autoware_protobuf.json", function (err, root) {
    if (err) throw err;

    autoware_protobuf.geometry_msgs = {msg: {}}
    autoware_protobuf.std_msgs = {msg: {}}
    autoware_protobuf.geometry_msgs.msg.PoseStamped = root.lookupType("geometry_msgs.msg.PoseStamped");
    autoware_protobuf.std_msgs.msg.UInt32 = root.lookupType("std_msgs.msg.UInt32");
    autoware_protobuf.std_msgs.msg.String = root.lookupType("std_msgs.msg.String");

    client.on("connect", function() {
        console.log("connected")
        client.subscribe('/uint32')
        client.subscribe('/string')
        setInterval(loop, 100);
    })
})

client.on('message', function (topic, payload) {
    if (topic == '/uint32') {
        console.log(autoware_protobuf.std_msgs.msg.UInt32.decode(payload).data)
    }
    if (topic == '/string') {
        console.log(autoware_protobuf.std_msgs.msg.String.decode(payload).data)
    }
})

let count = 0
const loop = function()
{
    const PoseStamped = autoware_protobuf.geometry_msgs.msg.PoseStamped;
    data = {
        "header": {"stamp": {"sec": 1, "nanosec": 2}, "frame_id": "map"},
        "pose": {
            "position": {
                "x": Math.cos(count * Math.PI / 180),
                "y": Math.sin(count * Math.PI / 180),
            },
            "orientation": {
                "z": Math.sin((count + 180) / 2.0 * Math.PI / 180),
                "w": Math.cos((count + 180) / 2.0 * Math.PI / 180),
            },
        }
    }
    const err = PoseStamped.verify(data);
    if (err) throw Error(err);

    const payload = PoseStamped.encode(data).finish();
    client.publish("/pose_stamped", payload);
    count = (count + 10) % 360
}
