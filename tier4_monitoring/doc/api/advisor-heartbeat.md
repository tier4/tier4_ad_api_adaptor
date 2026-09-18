# /api/external/set/monitoring/advisor/&lt;operator&gt;/heartbeat

## Classification

- Behavior: Topic
- DataType: tier4_external_api_msgs/msg/MonitoringHeartbeat

## Description

助言オペレーターとの通信状態を確認します。途絶判定時間は車両側のパラメーターに依存します。<br>
途絶が検知された場合、モニタリング状態がTIMEOUTに変更されます。<br>
モニタリング状態による自動走行可否判定の影響については [Monitoring API](../feature/index.md) ページを参照してください。

## Message Definition

```txt
builtin_interfaces/Time stamp
```
