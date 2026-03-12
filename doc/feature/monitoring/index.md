# Monitoring API

Lv2の監視状態についてはAD APIでサポートされており、[Manual Control API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/features/manual-control/)のHeartbeatで表現できる。
運転操作への切り替えについては[Operation Mode API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/features/operation_mode/)により管理されている。

ただし、介入操作が緊急停止のみなのか運転操作まで可能なのかは区別できず、また、Remote/Localのどちらのオペレーターが運転手としての責任を負っているかも区別できない。

| 状態                         | 定義                                |
| ---------------------------- | ----------------------------------- |
| リモート介入不可（未監視）   | ManualControl/RemoteHeartbeat=False |
| リモート介入可能（緊急停止） | ManualControl/RemoteHeartbeat=True  |
| リモート介入可能（遠隔運転） | ManualControl/RemoteHeartbeat=True  |
| リモート介入中               | OperationMode=Remote                |
| ローカル介入不可（未監視）   | ManualControl/LocalHeartbeat=False  |
| ローカル介入可能（緊急停止） | ManualControl/LocalHeartbeat=True   |
| ローカル介入可能（遠隔運転） | ManualControl/LocalHeartbeat=True   |
| リモート介入中               | OperationMode=Remote                |
