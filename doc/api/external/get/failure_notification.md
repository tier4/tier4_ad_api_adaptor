# /api/external/get/failure_notification/audience

## Classification

- Behavior: Topic
- DataType: tier4_external_api_msgs/msg/FailureNotificationArray

## Description

特定のユーザーに向けて用意された異常通知を受け取る。トピック名の最後の`audience`の部分は対象とするユーザーに応じて例えば以下のように変化する。

- /api/external/get/failure_notification/operator
- /api/external/get/failure_notification/developer

異常通知は以下の構造を持ったデータの配列として出力される。配列はデータの`priority`が降順となるよう整列されている。出力は通知内容が変化した場合にのみ行われる。

| 名称               | 型       | 説明                                                       |
| ------------------ | -------- | ---------------------------------------------------------- |
| diag_path          | string   | 元になった異常の診断グラフでの識別名。                     |
| diag_level         | uint8    | 元になった異常の診断レベル。                               |
| error_code         | string   | 異常通知API向けに定義されたエラーコード。                  |
| notification_level | uint8    | 異常通知API向けに定義された通知レベル。                    |
| priority           | uint32   | 異常通知API向けに定義された表示優先度。                    |
| language_codes     | string[] | 下記`situations`と`solutions`に対応した言語コード。        |
| situations         | string[] | 異常の状況説明。上記`language_codes`と同順で言語別に並ぶ。 |
| solutions          | string[] | 異常の解消方法。上記`language_codes`と同順で言語別に並ぶ。 |

## Examples

```yaml
stamp:
  sec: 1778738510
  nanosec: 991026040
notifications:
  - diag_path: /autoware/localization/state
    error_code: LOC-001
    diag_level: 2
    notification_level: 1
    priority: 50
    language_codes:
      - ja
      - en
    situations:
      - 初期位置推定が完了していません
      - Initial pose estimation is not complete
    solutions:
      - 自己位置推定の完了を待ってください
      - Please wait for localization to complete
  - diag_path: /autoware/planning/routing/state
    error_code: PLN-001
    diag_level: 2
    notification_level: 2
    priority: 0
    language_codes:
      - ja
      - en
    situations:
      - ルートが引かれていません
      - Route is not set
    solutions:
      - ルートを設定してください
      - Please set a route
```

## Requirement

- 通知内容が変化した場合のみ出力を行うこと。QoSはTransient Localとする。
- 少なくとも一つ以上の言語でメッセージが用意されていること。
- 事前にnotification_levelの使用方法についてユーザーと合意しておくこと。
- 優先度についてはpriorityの数値の大きいものが優先度が高いと定義する。
- 言語コードについてはISO 639-1を用いるものとする。
