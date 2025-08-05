# /api/external/get/system_monitor

## Classification

- Behavior: Topic
- DataType: tier4_external_api_msgs/msg/SystemMonitor

## Description

System Monitorの計測結果を取得する。取得できる値は以下の通り。

- CPU温度、サーマルスロットリング状態
- メモリ使用率
- GPUの状態（GPU使用率、GPU温度、GPUサーマルスロットリング状態）
- ネットワークの状態（通信帯域、エラー）
- ディスクの状態（使用率、IOアクセス）

## Requirement

System Monitorの実装でサポートされている計測結果を提供すること。
