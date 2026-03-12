# Monitoring API

## 車両側の要件

- 車両は複数の監視元を識別することができる（Web.Auto, MOT, etc.）
- 車両は監視元から以下の監視状態を受け取る。
  - 監視中断
  - Lv4監視 (車両からの要求に反応して受動的にサポートを実施できる)
  - Lv2監視 (運転手と同様の情報が提供されており能動的に介入できる)
- 車両は監視元ごとに以下の監視状態を管理する。これは後述する監視中断理由の保持の影響を受ける。
  - 監視中断
  - Lv4監視
  - Lv2監視
- 車両は監視元ごとに監視中断理由を管理し、明示的に解除されるまで保持し続けることができる。
  - オペレーターによる中断
  - ハートビートの途絶検知
- 車両は自律走行に必要な監視状態を把握できる
  - Lv2
  - Lv4
- 車両はLv2の監視元を持たないか、唯一のLv2の監視元を持つ
- 車両は以下の条件を満たした場合、監視状態が成立したとみなす
  - Lv4が要求されている場合、何れかの監視元がLv4監視状態である
  - Lv2が要求されている場合、選択中の監視元がLv2監視状態である
- 車両は要求Lvに対応した監視状態が成立していない場合、以下の動作を取る
  - 自律走行の開始要求を拒否する
  - 自律走行中は緩やかに停止する

## 全体図(3層)

![3-layer](./3-layer.drawio.svg)

## 全体図(2層)

![2-layer](./2-layer.drawio.svg)

## 監視状態

Pilot.Autoでの監視状態は以下の単位で管理される

| 状態 |
| 介入待機状態
| 監視状態

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
