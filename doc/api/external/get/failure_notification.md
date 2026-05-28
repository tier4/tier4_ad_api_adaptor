# /api/external/get/failure_notification/audience

## Classification

- Behavior: Topic
- DataType: tier4_external_api_msgs/msg/FailureNotificationArray

## Description

特定のユーザーに向けて用意された異常通知を受け取る。トピック名の最後の`audience`の部分は対象とするユーザーに応じて例えば以下のように変化する。

異常通知は以下の構造を持ったデータの配列として出力される。配列はデータの`priority`が降順となるよう整列されており、出力は通知内容が変化した場合にのみ行われる。
通知内容は異常の発生、異常の解消、もしくはAutowareの内部状態により変化し、継続中の異常を含めた通知内容全体が送信される。異常が全て解消した場合は空配列となる。

| 名称 | 型     | 説明                                      |
| ---- | ------ | ----------------------------------------- |
| code | string | 異常通知API向けに定義されたエラーコード。 |

## Examples

```yaml
stamp:
  sec: 1778738510
  nanosec: 991026040
notifications:
  - code: LOC-001
  - code: PLN-001
```

## Requirement

- 通知内容が変化した場合のみ出力を行うこと。QoSはTransient Localとする。
