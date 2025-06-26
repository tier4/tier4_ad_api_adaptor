# Migrate to Diagnostics API

従来の仕組みではHazardStatusでSPFやLFに分類すると停止挙動に紐付いてしまうため、通知専用の診断を送信するために `/api/external/get/diagnostics` が用意されていたが、
最新の診断では挙動と関連させない通知を出せるようになり、HazardStatus と合わせて新しい診断 API に移行することになった。
診断データの構造についてはシステムに依存するため、通知関連の診断をどのように出しているかはインテグレーションの担当者に確認する必要がある。
詳細は [Diagnostics API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/diagnostics/) を参照のこと。
