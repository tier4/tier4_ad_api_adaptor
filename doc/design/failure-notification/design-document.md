# 故障通知 API 設計たたき台

## 1. 背景・目的

### 1.1 現状

現在の ADK（Pilot.Auto X2 v4.3）では、故障診断情報を以下の 2 つの AD API として外部に公開している。

- `/api/system/diagnostics/struct` (DiagGraphStruct) — 診断グラフの静的構造（ノード・リーフ・リンク）
- `/api/system/diagnostics/status` (DiagGraphStatus) — 各ユニットの現在のレベル・メッセージ

MOT（車内運行者モニター）での故障表示は、上記の診断 API から得られる diag パスをキーとして、別途 MOT 側に配置された `mrm_messages.json` を参照し、situation（状況説明）・solution（対処方法）を画面に表示する方式をとっている。

![現行アーキテクチャ](figures/current_architecture.png)

### 1.2 課題

遠隔監視システム（FMS/Drive）など、自動運転システムとは独立したデプロイサイクルを持つ外部システムが増加する中で、以下の課題が顕在化している。

- **バージョン不整合**: 遠隔システムは ADK と異なるタイミングでアップデートされる。故障と文書を紐付ける JSON を外部システム側に持っていると、ADK のアップデートに伴い diag パスの変更・追加・削除が発生した際に、文書の不整合が起きる
- **実装の分散**: 各消費システム（MOT、遠隔、将来のシステム）がそれぞれ JSON のルックアップロジックを実装する必要がある
- **拡張の困難さ**: 多言語対応、対象システムに応じた文書差異、状態に応じた文言切り替えなどの要件を外部システム側で個別に実装するのは困難

### 1.3 目的

ADK 側から、故障時に表示すべき文書（situation / solution）を含めた通知情報を API として配信する仕組みを設計する。

## 2. 要件整理

### 2.1 機能要件

| &nbsp;#&nbsp; | 要件                             | 詳細                                                                                                                                                                                                                              |
| :-----------: | -------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
|      F-1      | audience 別文書対応              | 対象システム（audience）に応じて異なる situation / solution を配信する                                                                                                                                                            |
|      F-2      | audience 拡張性                  | audience は任意個に追加可能な設計とする。初期 audience: `mot`（車内運行者向け）/ `remote`（遠隔監視者向け）/ `developer`（開発者向け）                                                                                            |
|      F-3      | 多言語対応                       | 最低限、日本語（ja）と英語（en）を初期サポートする。言語の追加は設定ファイルの拡張で対応可能とする                                                                                                                                |
|      F-4      | 状態条件による文言切り替え       | 車両の状態に応じて、同じ diag パスでも異なる situation / solution を出力する。主に `/autoware/state`（AutowareState）を参照する。汎用的な条件指定（`state_topic` / `field` / `values`）により将来的に他のトピックも条件に追加可能 |
|      F-5      | 優先度（priority）               | 対応アクションの重要度に基づく `priority`（uint32）を設定し、数値が大きいほど高優先度とする。API はこの値でソート済みの通知リストを配信する。例: 「再起動してください」側の数値を「復帰操作をしてください」より大きく設定する     |
|      F-6      | 通知レベル（notification_level） | 通知の表示方式を ADK 側で `uint8` の数値として指定する（値と UI の対応は消費側との合意で定める）。消費側は数値に応じて全画面通知・スナックバー等にマッピングする                                                                  |

## 3. アプローチ比較

以下の 3 案を比較検討し、案 2 を推奨案として詳細設計を行う。

### 案 1: JSON 配信型（現行拡張）

ADK 側で `mrm_messages.json` 相当のマッピングテーブル全体を API トピックとして配信する。消費側は diag status とマッピングテーブルを突合して表示を行う。

**Pros:**

- 現行 MOT の仕組みに最も近く、移行コストが小さい
- フィルタリング・表示方法を消費側に委ねることができる

**Cons:**

- 消費側にルックアップ + 状態条件判定 + audience フィルタ等の解釈ロジックの実装が必要。JSON の所在（ADK から配信）は解決するが、解釈ロジックの実装が消費側ごとに必要であり、そのロジックのバージョン不整合リスクは残る
- audience 別・状態条件による文言切り替えロジックが各消費側に分散し、実装負荷が高い

### 案 2: 通知リスト型 API（推奨）

ADK 内部で diag 状態 + 車両状態を突合し、解決済みの通知リスト（priority 付き、audience 別、言語別の situation / solution）を API として配信する。消費側はリストをそのまま表示するだけでよい。

**Pros:**

- 受け取ったリストをそのまま表示するだけでよく、消費側の実装が最もシンプル
- 状態条件の判定・audience 別メッセージ解決・優先度ソートがすべて ADK 側で完結し、バージョン不整合を根本的に解決
- 多言語・対象システム拡張も ADK 側の設定変更で完結
- 表示されるべき情報だけが配信されるため帯域効率がよい

**Cons:**

- 新規 API + 新規ノードの開発が必要
- ADK 側にメッセージ解決の設定ファイル管理が増える

### 案 3: 拡張診断グラフ型

既存の `DiagGraphStatus` メッセージを拡張し、各 diag ノードに situation / solution 等の文書情報を埋め込む。

**Pros:**

- 既存 API の自然な拡張で後方互換性を保ちやすい
- diag の tree 構造と文書が一体化するためコンテキストが保持される

**Cons:**

- DiagGraphStatus は OK のものも含む全ノードのステータスを送信しているため、全ノード x 全 audience x 全言語でメッセージサイズが大幅に増加する
- ADAPI 仕様への影響が大きく、Pilot.Auto X2 スコープを超える可能性がある
- audience 別の内容差異や priority 付きソートを消費側が行う必要がある

### 比較まとめ

| 評価軸                  | 案 1: JSON 配信型 | 案 2: 通知リスト型 | 案 3: 拡張診断グラフ型 |
| ----------------------- | :---------------: | :----------------: | :--------------------: |
| バージョン不整合の解決  |         △         |         ◎          |           △            |
| 消費側の実装負荷        |         ×         |         ◎          |           ×            |
| 多言語・audience 拡張性 |         △         |         ◎          |           △            |
| 既存 API への影響       |         ○         |         ◎          |           ×            |
| 開発コスト              |         ◎         |         ○          |           ○            |
| 帯域効率                |         ×         |         ◎          |           ×            |

**結論: 案 2（通知リスト型 API）を推奨案として採用する。**

## 4. 推奨案（案 2）の詳細設計

### 4.1 アーキテクチャ

新規ノード `failure_notification_node` を `tier4_ad_api_adaptor` 配下に追加する。

![提案アーキテクチャ](figures/proposed_architecture.png)

**Subscribe するトピック:**

| トピック                         | メッセージ型      | 用途                                   |
| -------------------------------- | ----------------- | -------------------------------------- |
| `/api/system/diagnostics/struct` | `DiagGraphStruct` | diag パスの取得（静的構造）            |
| `/api/system/diagnostics/status` | `DiagGraphStatus` | 各 diag ユニットの現在の状態           |
| `/autoware/state`                | `AutowareState`   | 車両の統合状態（状態条件の評価に使用） |

設定ファイルの `state_topic` / `field` / `values` による汎用条件指定を採用しているため、将来的に `/autoware/state` 以外のトピックも条件に追加可能。

**Publish するトピック:**

| トピック                                  | メッセージ型               | QoS         | 用途                   |
| ----------------------------------------- | -------------------------- | ----------- | ---------------------- |
| `/system/failure_notifications/mot`       | `FailureNotificationArray` | best_effort | MOT 向け通知リスト     |
| `/system/failure_notifications/remote`    | `FailureNotificationArray` | best_effort | 遠隔監視向け通知リスト |
| `/system/failure_notifications/developer` | `FailureNotificationArray` | best_effort | 開発者向け通知リスト   |

ノードパラメータで、実際に publish する audience を選択する（初期値は上記 3 本すべて、など運用で定める）。

### 4.2 メッセージ型

**1 通知（`FailureNotification` 1 要素）**は、diag 上の 1 件の非 OK に対応する表示単位とする。多言語は **同一要素内の平行配列**で持ち、**`FailureNotificationArray.notifications` の要素数＝その audience 向けに表示すべき通知の件数**と読む（消費側は配列を順にマッピングしやすい）。

**`priority` の意味と「どちらが自然か」:** 優先度の数値設計には次の 2 スタイルがある。

- **スタイル A（小さいほど強い）:** OS の nice 値や「priority 1 が最優先」のように、0 に近いほど重要。
- **スタイル B（大きいほど強い）:** 重大度スコアのように、**数値が大きいほど緊急度が高い**と解釈する方式。

いずれも業界であり得るため、**API ではスタイル B を採用する**（**数値が大きいほど高優先度**）。議事録の「優先順位は高い数字」と整合する。設定 YAML の `priority` も同じ解釈で記述する。混乱を避けるため、メッセージ定義と本節にコメントで必ず明記する。

**ソート順:** `FailureNotificationArray.notifications` は、各要素の `priority` の **降順**（大きい値が先頭）に並べる。先頭 index 0 が「最も先にユーザーに見せるべき通知」に相当する。`priority` が同一の要素同士の順序は実装定義とし、安定ソートを推奨する。

```
# FailureNotification.msg
string diag_path            # 対応する diag のパス（例: "/localization/001-topic_status/initialpose"）
string error_code           # エラーコード（例: "LOC-00-E00-001E"）
uint8 diag_level            # 元の diag level（0=OK, 1=WARN, 2=ERROR, 3=STALE）
uint8 notification_level    # 通知レベル（数値。意味は消費側との合意で定める。MOT の 4 系統表示などは拡張で別途定義してもよい）
uint32 priority             # 表示優先度。数値が大きいほど高優先（notifications[] では降順で並ぶ）
string[] language_codes     # 言語コード（例: "ja", "en"）。index i が situations[i] / solutions[i] に対応
string[] situations         # 状況説明（状態条件適用済み）。language_codes と同じ長さ必須
string[] solutions          # 対処方法（状態条件適用済み）。language_codes と同じ長さ必須
```

```
# FailureNotificationArray.msg
builtin_interfaces/Time stamp
FailureNotification[] notifications  # priority 降順にソート済み（[4.2](#42-メッセージ型)）
```

### 4.3 設定ファイル形式

現行の `mrm_messages.json` を拡張した YAML 形式。audience 別・言語別・状態条件付きのメッセージ定義を行う。

```yaml
notifications:
  "/localization/001-topic_status/initialpose":
    error_code: "LOC-00-E00-001"
    priority: 100
    notification_level: 2 # ERROR
    conditions:
      # AutowareState = WAITING_FOR_ROUTE のとき: ルート未設定用のメッセージ
      - state_topic: "/autoware/state"
        field: "state"
        values: [2]
        messages:
          mot:
            ja:
              situation: "ルートが引かれていません"
              solution: "ルートを設定してください"
            en:
              situation: "Route is not set"
              solution: "Please set a route"
          remote:
            ja:
              situation: "ルート未設定です"
              solution: "運行者にルート設定を指示してください"
            en:
              situation: "Route is not set"
              solution: "Please instruct the operator to set a route"
          developer:
            ja:
              situation: "RouteState=UNSET: ルート未設定"
              solution: "/api/routing/set_route でルートを設定"
            en:
              situation: "RouteState=UNSET: route not set"
              solution: "Set route via /api/routing/set_route"
      # AutowareState = PLANNING〜ARRIVED_GOAL のとき: 通常の故障メッセージ
      - state_topic: "/autoware/state"
        field: "state"
        values: [3, 4, 5, 6]
        messages:
          mot:
            ja:
              situation: "初期位置推定が完了していません"
              solution: "自己位置推定の完了を待ってください"
            en:
              situation: "Initial pose estimation is not complete"
              solution: "Please wait for localization to complete"
          remote:
            ja:
              situation: "初期位置推定が完了していません"
              solution: "自己位置推定完了をお待ちください"
            en:
              situation: "Initial pose estimation is not complete"
              solution: "Please wait for localization to complete"
          developer:
            ja:
              situation: "LocalizationState!=INITIALIZED: 初期位置推定未完了"
              solution: "initialpose を設定するか NDT の収束を待つ"
            en:
              situation: "LocalizationState!=INITIALIZED: initial pose not set"
              solution: "Set initialpose or wait for NDT convergence"
    # どの条件にもマッチしない場合のフォールバック
    default_messages:
      mot:
        ja:
          situation: "初期位置推定が完了していません"
          solution: "自己位置推定の完了を待ってください"
        en:
          situation: "Initial pose estimation is not complete"
          solution: "Please wait for localization to complete"
```

**設定ファイルの解釈ルール:**

1. diag パスに対応するエントリの `conditions` を上から順に評価する
2. `state_topic` で指定されたトピックの `field` フィールドの値が `values` のいずれかに一致すれば、その condition のメッセージを採用する
3. どの condition にもマッチしない場合は `default_messages` を使用する
4. `default_messages` にも該当 audience / 言語がない場合は通知を出さない（非表示）

### 4.4 通知解決フロー

![通知解決フロー](figures/notification_flow.png)

`failure_notification_node` は以下のフローで通知リストを生成する:

1. `DiagGraphStatus` の更新を受信するたびに処理を実行
2. 各 diag リーフの `level` が OK 以外（WARN / ERROR / STALE）のものを抽出。ただし、diag が OK に復帰したがラッチ状態（MRM 介入等により復帰操作が必要な状態）が解除されていない場合は、故障メッセージではなく復帰操作の通知（例: 「復帰ボタンを押してください」）を配信する
3. diag パスをキーとして設定ファイルのエントリを検索
4. 現在の `/autoware/state` の値と `conditions` を照合し、適用するメッセージを決定
5. 設定された audience と言語の全組み合わせについてメッセージを解決
6. 設定ファイルの `notification_level` を各通知に付与
7. priority でソートし、`FailureNotificationArray` として publish

### 4.5 配信方式

| 方式                                     | Pros                                                                                                  | Cons                                                                     |
| ---------------------------------------- | ----------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------ |
| **audience ごとに 1 トピック**（本設計） | 消費側は自 audience のトピックだけ subscribe すればよい。言語はメッセージ内の配列でまとめて受け取れる | 1 メッセージのサイズが audience 内の全言語分を含む                       |
| **単一トピックに audience フィールド**   | トピック本数が最少                                                                                    | 全消費者が同一ストリームを受け、不要な audience をフィルタする必要がある |

**採用:** **audience ごとに 1 トピック**（[4.1](#41-アーキテクチャ) の表）。言語はトピックで分けない。

トピック名の構造:

```
/system/failure_notifications/<audience>
```

具体例:

| トピック                                  | 用途                                               |
| ----------------------------------------- | -------------------------------------------------- |
| `/system/failure_notifications/mot`       | MOT 向け（`language_codes` 等に ja / en を載せる） |
| `/system/failure_notifications/remote`    | 遠隔監視向け                                       |
| `/system/failure_notifications/developer` | 開発者向け                                         |

`FailureNotificationArray` に audience フィールドは持たない（トピック名で区別する）。ノードパラメータで publish する audience と、YAML に含める言語集合を指定する。

### 4.6 既存システムとの関係

- 既存の `diagnostics/struct` / `diagnostics/status` API はそのまま維持（変更なし）

## 5. 現行 mrm_messages.json からの移行

### 5.1 現行フォーマットとの対応

| 現行 mrm_messages.json           | 新設定ファイル                                          |
| -------------------------------- | ------------------------------------------------------- |
| キー（diag パス）                | `notifications` の第一階層キー                          |
| `ERROR` / `default` ブロック     | `diag_level` に応じた条件として設定可能（将来拡張）     |
| `situation`                      | `messages.<audience>.<lang>.situation`                  |
| `solution`                       | `messages.<audience>.<lang>.solution`                   |
| `error_code`                     | `error_code`                                            |
| `initialization_state_condition` | `conditions` の `state_topic: "/autoware/state"` で表現 |
| `routing_state_condition`        | `conditions` の `state_topic: "/autoware/state"` で表現 |

## 変更履歴

| 日付       | 概要                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 2026-05-11 | 通知リスト型 API の詳細を更新。配信は audience ごと 1 トピック（言語はトピックで分離しない）、`FailureNotification` は `language_codes` / `situations` / `solutions` の平行配列（B'）、`priority` は大きいほど高優先・配列は降順ソート、F-5 / F-6 および 4.1〜4.2・4.5 の整合、`proposed_architecture` 図の更新。根拠議事録: [2026-04-01 MRM message.json API meeting](https://tier4.atlassian.net/wiki/spaces/AIP/pages/5080352231/2026-04-01+MRM_message.json+API+meeting)（Atlassian Confluence。アクセスにはログインが必要な場合あり） |
