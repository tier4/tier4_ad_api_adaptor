const autoware_protobuf = require('./autoware_protobuf');
const mqtt = require('mqtt')
const client  = mqtt.connect('mqtt://localhost')

client.on('connect', function () {
    console.log('connected');
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

setInterval(loop, 100);
