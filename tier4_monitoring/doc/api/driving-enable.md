# /api/external/set/monitoring/driving/enable

## Classification

- Behavior: Service
- DataType: tier4_external_api_msgs/srv/EnableDriving

## Description

車両の走行状態の変更を行います。指定できる状態は以下の表に示す通りです。<br>
モニタリング状態による自動走行可否判定の影響については [Monitoring API](../feature/index.md) ページを参照してください。

| 状態   | 説明                   |
| ------ | ---------------------- |
| STOP   | 停止状態               |
| LEVEL2 | 自律走行状態 (Level 2) |
| LEVEL4 | 自律走行状態 (Level 4) |

## Errors

| メッセージ                       | 説明                                                                           |
| -------------------------------- | ------------------------------------------------------------------------------ |
| unknown mode                     | 上記の表に示した状態以外を指定しています。                                     |
| autonomous mode is not available | 自律走行の準備ができていません。                                               |
| level2 is not available          | 自律走行の準備はできているが、必要な監視オペレーターの状態を満たしていません。 |
| level4 is not available          | 自律走行の準備はできているが、必要な助言オペレーターの状態を満たしていません。 |

## Service Definition

```txt
uint8 mode
---
tier4_external_api_msgs/ResponseStatus status
```
