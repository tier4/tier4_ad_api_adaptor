# Migrate to Routing API

基本的な使用方法は同じだが、従来の API と同様に計算済みルートを Lanelet2 形式で指定する set_route に加え、経由地点の座標を指定する set_route_points が追加された。
また、地図から計算可能であるため continued_lane_ids は削除され、データが重複するので preferred に指定したレーンは alternatives には含めないように変更されている。
詳細は [Routing API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/routing/) を参照のこと。
