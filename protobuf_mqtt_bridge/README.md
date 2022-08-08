# protobuf_mqtt_server


```bash
sudo apt-add-repository ppa:mosquitto-dev/mosquitto-ppa
sudo apt-get install mosquitto
sudo apt-get install mosquitto-clients
```

```bash
pip install paho-mqtt
pip install grpcio
pip install grpcio-tools
```

```
npm install mqtt
npm install protobufjs  (?)
npm install -g goog
```

nodejsをテストする場合

```
cd protobuf_mqtt_bridge/
npm i
node examples/node/mqtt_listener.js
```