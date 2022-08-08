# protobuf_mqtt_server

## MQTT のブローカー

### 準備

```bash
sudo apt-add-repository ppa:mosquitto-dev/mosquitto-ppa
sudo apt-get install mosquitto
sudo apt-get install mosquitto-clients  # 確認用なので入れなくても良い
```

### 起動

```bash
mosquitto
mosquitto -c mosquitto.conf  # ブラウザを使う場合
```

## ROS-MQTT 変換

### 準備

```bash
pip install paho-mqtt
pip install grpcio-tools  # メッセージファイルを作る場合？
pip install grpcio
pip install protobuf      # こっちだけで良いかも
```

### 起動

※ /pose_stamped は rviz2 でみる。

```bash
source python-path.bash
python3 protobuf_mqtt_bridge/server.py  # 端末１
python3 examples/ros/talker_uint32.py   # 端末２
python3 examples/ros/talker_string.py   # 端末３
```

## python で使う場合

### 準備

```bash
pip install paho-mqtt
pip install protobuf
```

### 起動

```bash
source python-path.bash
python3 examples/python/mqtt_listener.py  # 端末１
python3 examples/python/mqtt_talker.py    # 端末２
```

## nodejs で使う場合

### 準備

```bash
cd protobuf_mqtt_bridge
npm i
```

### 起動

```bash
node examples/node/mqtt_listener.js  # 端末１
node examples/node/mqtt_talker.js    # 端末２
```

## ブラウザで使う場合

### 準備

```bash
python3 -m http.server 8085 -d examples/browser
```

### 起動

ブラウザから `http://localhost:8085/` にアクセスする。

## License

paho-mqtt: Eclipse Public License 2.0
protobufjs-cli: BSD 3-Clause License
