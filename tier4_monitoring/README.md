# tier4_monitoring

## 概要

本パッケージはオペレーターによるモニタリングを必要とする自動走行に向けた状態管理と変更操作を行います。
自動走行レベルを Lv. 2 と Lv. 4 とで切り替えたいようなプロジェクトで活用される想定です。
[機能の設計についてはこちらを参照してください。](./doc/feature/index.md)

## 前提条件

レベル4で自動走行する場合、運行する Lanelet2 地図が level4_operation_end_lanelet の情報を含んでいること。

- [地図フォーマットについてはこちらを参考にしてください。](https://docs.pilot.auto/reference-design/common/map-requirements/vector-map-requirements/category_others/#vm-07-09-%E8%87%AA%E5%8B%95%E9%81%8B%E8%BB%A2%E3%83%AC%E3%83%99%E3%83%AB-2%E3%83%AC%E3%83%99%E3%83%AB-4-%E3%81%AE%E5%88%87%E3%82%8A%E6%9B%BF%E3%81%88)
- 上記タグが埋め込まれる Lanelet は想定されるルートの最初のセグメントか、それに隣接する road_shoulder であること。
- 上記タグの値は想定されるルートの最後のセグメントか、それに隣接する road_shoulder であること。
- 上記タグの値に複数の候補がある場合は、カンマ区切りで指定し、スペースなどの他の起動は含めないこと。

以下のインターフェースを置き換えて使用すること。置き換え前のインターフェースは直接使用しないでください。

| 置き換え前                               | 置き換え後                                  |
| ---------------------------------------- | ------------------------------------------- |
| /api/operation_mode/state                | /api/external/get/monitoring/driving/status |
| /api/operation_mode/change_to_stop       | /api/external/set/monitoring/driving/enable |
| /api/operation_mode/change_to_autonomous | /api/external/set/monitoring/driving/enable |

## パラメーター

| 名前    | 型    | 説明                                                                                |
| ------- | ----- | ----------------------------------------------------------------------------------- |
| timeout | float | オペレーターのハートビートの途絶判定時間 [s]。経過すると状態が TIMEOUT になります。 |

## オペレーター

本ノードは以下のオペレーターを作成します。名前はインターフェースのパスに使用され、表の順序はオペレーター優先度の高い順を示します。

| グループ   | 名前 | 説明                     |
| ---------- | ---- | ------------------------ |
| supervisor | mot  | 車内の監視オペレーター。 |
| supervisor | fms  | 遠隔の監視オペレーター。 |
| advisor    | mot  | 車内の助言オペレーター。 |
| advisor    | fms  | 遠隔の助言オペレーター。 |

### 提供インターフェース

以下のインターフェースが上記の各オペレーターに対して作成されます。`<group>` と `<operator>` は上記の表のグループと名前です。

| インターフェース                                                                                           | 型                                                 |
| ---------------------------------------------------------------------------------------------------------- | -------------------------------------------------- |
| [/api/external/get/monitoring/&lt;group&gt;/&lt;operator&gt;/status](./doc/api/supervisor-status.md)       | tier4_external_api_msgs/msg/MonitoringStatus       |
| [/api/external/set/monitoring/&lt;group&gt;/&lt;operator&gt;/heartbeat](./doc/api/supervisor-heartbeat.md) | tier4_external_api_msgs/msg/MonitoringHeartbeat    |
| [/api/external/set/monitoring/&lt;group&gt;/&lt;operator&gt;/change](./doc/api/supervisor-change.md)       | tier4_external_api_msgs/srv/ChangeMonitoringStatus |

以下のインターフェースは車両の走行状態の管理のために作成されます。

| インターフェース                                                           | 型                                        |
| -------------------------------------------------------------------------- | ----------------------------------------- |
| [/api/external/get/monitoring/driving/status](./doc/api/driving-status.md) | tier4_external_api_msgs/msg/DrivingStatus |
| [/api/external/set/monitoring/driving/enable](./doc/api/driving-enable.md) | tier4_external_api_msgs/srv/EnableDriving |

### 依存インターフェース

| インターフェース                                    | 型                                                            |
| --------------------------------------------------- | ------------------------------------------------------------- |
| /map/vector_map                                     | autoware_map_msgs/msg/LaneletMapBin                           |
| /api/routing/route                                  | autoware_adapi_v1_msgs/msg/Route                              |
| /api/operation_mode/state                           | autoware_adapi_v1_msgs/msg/OperationModeState                 |
| /api/operation_mode/change_to_stop                  | autoware_adapi_v1_msgs/srv/ChangeOperationMode                |
| /api/operation_mode/change_to_autonomous            | autoware_adapi_v1_msgs/srv/ChangeOperationMode                |
| /planning/scenario_planning/max_velocity_candidates | autoware_internal_planning_msgs/msg/VelocityLimit             |
| /planning/scenario_planning/clear_velocity_limit    | autoware_internal_planning_msgs/msg/VelocityLimitClearCommand |

## 機能概略

- ルート取得時、地図情報から現在のルートでレベル4自動走行が行えるか判別します。レベル2自動走行は常に可能です。
- オペレーターのモニタリング状態から、レベル4/レベル2自動走行が行えるか判別します。
- ルートとオペレーターの両方で該当レベルの自動走行が可能で、Autowareが自動走行可能であればそれを通知します。
- ルートとオペレーターのいずれかで自動走行ができなくなった場合、該当レベルで自動走行中であれば停止を要求します。

## デバッグツール

本パッケージは、全オペレーターのハートビートと要求の送信および現在の状態の表示を行う GUI ツールを提供します。実際のオペレーターのシステムを用意せずに開発を行う用途を想定しています。

```bash
ros2 run tier4_monitoring debug.py
```
