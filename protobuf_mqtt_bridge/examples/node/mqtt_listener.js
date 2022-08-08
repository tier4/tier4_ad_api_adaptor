const uint32 = require('./autoware_protobuf/uint32');
const mqtt = require('mqtt')
const client  = mqtt.connect('mqtt://localhost')

protobuf.load("proto/autoware_protobuf/std_msgs/msg/uint32.proto", function(err, root) {
    if (err) { throw err; }
})

client.on('connect', function () {
    client.subscribe('/uint32')
})

client.on('message', function (topic, payload) {
    console.log(payload)
    client.end()
})
