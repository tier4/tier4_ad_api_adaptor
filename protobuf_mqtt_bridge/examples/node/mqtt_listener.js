const autoware_protobuf = require('./autoware_protobuf');
const mqtt = require('mqtt')
const client  = mqtt.connect('mqtt://localhost')

client.on('connect', function () {
    console.log('connected');
    client.subscribe('/uint32')
    client.subscribe('/string')
})

client.on('message', function (topic, payload) {
    console.log(topic, payload)
    if (topic == '/uint32') {
        console.log(autoware_protobuf.std_msgs.msg.UInt32.decode(payload).data)
    }
    if (topic == '/string') {
        console.log(autoware_protobuf.std_msgs.msg.String.decode(payload).data)
    }
})
