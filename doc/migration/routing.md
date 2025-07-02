# Migrate to Routing API

基本的な使用方法は同じだが、従来のAPIと同様に計算済みルートをLanelet2形式で指定するset_routeに加え、経由地点の座標を指定するset_route_pointsが追加された。
また、地図から計算可能であるためcontinued_lane_idsは削除され、データが重複するのでpreferredに指定したレーンはalternativesには含めないように変更されている。
詳細は[Routing API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/routing/)を参照のこと。
