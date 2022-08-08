const uint32 = require('./autoware_protobuf/uint32.pbjs');
// const uint32 = require('../../resource/proto/autoware_protobuf/std_msgs/msg/uint32.proto');
const mqtt = require('mqtt')
const client  = mqtt.connect('mqtt://localhost')
const protobuf = require("protobufjs");

protobuf.load(uint32, function(err, root) {
    console.log('load');
    if (err) { throw err; }
})

client.on('connect', function () {
    console.log('connect');
    client.subscribe('/uint32')
    console.log('subscribe');
})

client.on('message', function (topic, payload) {
    console.log(topic, payload)
    client.end()
})
