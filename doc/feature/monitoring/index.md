# Monitoring API

## 全体図(3層)

![3-layer](./3-layer.drawio.svg)

## 全体図(2層)

![2-layer](./2-layer.drawio.svg)

## Lv2監視状態

Lv2の監視状態についてはAD APIでサポートされており、[Manual Control API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/features/manual-control/)のHeartbeatで表現できる。
運転操作への切り替えについては[Operation Mode API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/features/operation_mode/)により管理されている。

ただし、介入操作が緊急停止のみなのか運転操作まで可能なのかは区別しない。また、Remote/Localのどちらのオペレーターが運転手としての責任を負っているかも区別できない。

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

## Lv4監視状態

Lv4の監視状態についてはAD APIではサポートされていないためPilot.Autoでの拡張となる。

| 状態               | 定義                         |
| ------------------ | ---------------------------- |
| 未監視（接続途絶） | Heartbeat=False              |
| 監視中断           | Haertbeat=True, Status=False |
| 監視状態           | Heartbeat=True, Status=True  |

## 監視状態要求

車両の監視状態要求は以下の値を取り、それぞれの条件が満たされた場合に自律走行モードへの遷移が許可される。
また、自律走行中に条件が無効化された場合、走行が一時的に中断される。

| モード     | 条件                                                             |
| ---------- | ---------------------------------------------------------------- |
| Lv4        | 何れかのLv4-HeartbeatまたはLv2-Heartbeatが有効な場合に許可される |
| Lv2 remote | Lv2のRemoteHeartbeatが有効な場合に許可される                     |
| Lv2 local  | Lv2のLocalHeartbeatが有効な場合に許可される                      |

## 監視中断要因

ハートビートの途絶や監視中断した場合、その原因を取得できる。また、復帰する条件として追加操作を要求するか設定できる。
