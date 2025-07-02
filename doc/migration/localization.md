# Migrate to Localization API

自己位置初期化のAPIにはリクエストで指定した姿勢を用いるものとGNSSの姿勢を用いるものに別れていたが、AD APIでは一つのAPIに統合されている。
リクエストのposeが要素数0か1の配列になっており、要素数が0であればGNSSの姿勢を、1であれば指定した姿勢を用いて実行される。
上記を除けば使用方法に大きな変更はない。
詳細は[Localization API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/localization/)を参照のこと。
