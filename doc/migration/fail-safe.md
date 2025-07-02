# Migrate to Fail-safe API

従来の Autoware では異常を検知すると直ちに停止する挙動をしていて、異常が検知された状態を emergency 、また、その時の動作を emergency_stop と呼んでいた。
現在はMRMという概念が導入されており、これは異常発生時に車両を安全な状態に持っていくための挙動全般を指している。
これにより異常の発生箇所に応じて緩やかに停止するなど状況に応じて適切な動作を選択できるようになったが、今後は目的によっては複数の挙動を区別する必要が出てくる。
例として `/api/external/set/emergency` はMRM requestに置き換わるが、要求時の挙動はAutowareに委ねられているので注意すること。
従来と同様の減速度を要求する場合、システムの設定に反映するか、APIを特定のMRMを指定できるよう拡張する必要があるため個別に対応が必要になる。
詳細は [Fail-safe API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/fail-safe/) を参照のこと。
