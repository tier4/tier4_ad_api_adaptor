# /api/external/set/monitoring/supervisor/&lt;operator&gt;/change

## Classification

- Behavior: Service
- DataType: tier4_external_api_msgs/srv/ChangeMonitoringStatus

## Description

監視オペレーターのモニタリング状態の変更を行います。指定できる状態は以下の表に示す通りです。<br>
モニタリング状態による自動走行可否判定の影響については [Monitoring API](../feature/index.md) ページを参照してください。

| 状態        | 説明                           |
| ----------- | ------------------------------ |
| UNAVAILABLE | 監視を行うことができない状態。 |
| AVAILABLE   | 要求があれば監視を行える状態。 |
| OPERATING   | 監視を行っている状態。         |

## Errors

| メッセージ     | 説明                                                 |
| -------------- | ---------------------------------------------------- |
| unknown status | 上記の表に示した状態以外を指定した場合に発生します。 |

## Service Definition

```txt
uint8 status
---
tier4_external_api_msgs/ResponseStatus status
```
