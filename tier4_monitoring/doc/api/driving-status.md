# /api/external/get/monitoring/driving/status

## Classification

- Behavior: Topic
- DataType: tier4_external_api_msgs/msg/DrivingStatus

## Description

車両の走行状態の通知を行います。通知される状態は以下の表に示す通りです。<br>
モニタリング状態による自動走行可否判定の影響については [Monitoring API](../feature/index.md) ページを参照してください。

| 状態   | 説明                   |
| ------ | ---------------------- |
| STOP   | 停止状態               |
| LEVEL2 | 自律走行状態 (Level 2) |
| LEVEL4 | 自律走行状態 (Level 4) |

## Message Definition

```txt
builtin_interfaces/Time stamp
uint8 mode
bool is_level2_available
bool is_level4_available
bool is_level2_route
bool is_level4_route
```
