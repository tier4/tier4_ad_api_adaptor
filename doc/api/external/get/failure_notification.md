# /api/external/get/failure_notification

## Classification

- Behavior: Topic
- DataType: tier4_external_api_msgs/msg/FailureNotificationArray

## Description

車両の監視者に向けて現在の異常状態を通知する。通知内容の詳細や対処方法については別途管理されるマニュアルやメッセージファイルにより定義されることを前提としている。
通知内容は異常の発生、異常の解消、もしくは車両システムの内部状態により変化し、継続中の異常を含めた通知内容全体が送信される。異常が全て解消した場合は空配列となる。

| 名称 | 型     | 説明                                     |
| ---- | ------ | ---------------------------------------- |
| code | string | 車両監視者向けに定義されたエラーコード。 |

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
