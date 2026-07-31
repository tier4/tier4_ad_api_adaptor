# /api/external/get/monitoring/advisor/&lt;operator&gt;/status

## Classification

- Behavior: Topic
- DataType: tier4_external_api_msgs/msg/MonitoringStatus

## Description

助言オペレーターのモニタリング状態の通知を行います。通知される状態は以下の表に示す通りです。<br>
モニタリング状態はハートビート途絶によるTIMEOUTへの遷移を除き自動的に行われることはありません。<br>
TIMEOUTから復帰するには [状態変更API](./advisor-change.md) を使用して明示的に元のステータスに戻す必要があります。<br>
また、`responsible` フィールドはオペレーターが助言責任者に選ばれているかどうかを示します。<br>
モニタリング状態による自動走行可否判定の影響については [Monitoring API](../feature/index.md) ページを参照してください。

| 状態        | 説明                               |
| ----------- | ---------------------------------- |
| TIMEOUT     | ハートビートの途絶を検知した状態。 |
| UNAVAILABLE | 助言を行うことができない状態。     |
| AVAILABLE   | 要求があれば助言を行える状態。     |
| OPERATING   | 助言を行っている状態。             |

## Message Definition

```txt
builtin_interfaces/Time stamp
uint8 status
bool responsible
```
