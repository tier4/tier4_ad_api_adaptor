# /api/external/get/hazard_status

## Classification

- Behavior: Topic
- DataType: tier4_external_api_msgs/msg/HazardStatusStamped

## Description

`/awapi/autoware/get/status` に含まれていた `hazard_status` に相当するデータを取得する。

## Requirement

以下の変換テーブルに従い、自動モード全体でのエラーレベル(列)と各診断のエラーレベル(行)を使って分類を行う。

|       | OK  | WARN | ERROR | STALE |
| ----- | --- | ---- | ----- | ----- |
| OK    | NF  | NF   | NF    | NF    |
| WARN  | SF  | LF   | LF    | LF    |
| ERROR | SF  | LF   | SPF   | SPF   |
| STALE | SF  | LF   | SPF   | SPF   |

また、全体のフラグについては以下のように算出する。

| Flag                | Description                                                                                |
| ------------------- | ------------------------------------------------------------------------------------------ |
| `emergency`         | 分類に `SPF` または `LF` が含まれている。                                                  |
| `emergency_holding` | 診断の `latch_level` を使用して同様に分類を行った場合に `SPF` または `LF` が含まれている。 |
