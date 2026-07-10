# tier4_failure_notification

このパッケージは autoware_diagnostic_graph_aggregator が出力する診断グラフを監視し、
各診断ユニットのエラーレベルがOKではない場合に関連付けられたエラーコードを出力します。

## エラーコード定義

以下のようなYAMLファイルを用意し、Launch fileの `error_file` 引数にファイルのパスを指定します。
該当する診断ユニットのエラーレベルがOKではない場合、`failures` に指定したリストを順に確認し、
最初に `condition` で指定した条件の成立したエラーコードを採用します。
[条件の記法についてはこちらを確認してください。](./doc/conditions.md)

```yaml
notifications:
  /autoware/control:
    failures:
      - condition: Always
        code: CTL-001

  /autoware/localization:
    failures:
      - condition: LocalizationState(Initialized)
        code: LOC-001

  /autoware/planning:
    failures:
      - condition: RouteState(Set)
        code: PLN-001
      - condition: RouteState(Unset)
        code: PLN-002
      - condition: Always
        code: PLN-003
```
