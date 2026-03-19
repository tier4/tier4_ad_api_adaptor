# Migrate to Control API

従来は現在選択されている遠隔操作・ジョイスティック操作について取得する機能であったが、AD APIでは自律制御を含め、Autowareが車両に送信している指示値を取得する機能として統合された。
新旧データの対応関係を以下に示す。
詳細は[Control API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/features/control/)を参照のこと。

<table>
    <tr>
        <th>旧トピック名</th>
        <th>旧フィールド名</th>
        <th>新トピック名</th>
        <th>新フィールド名</th>
    </tr>
    <tr>
        <td rowspan="2">/api/external/get/command/selected/vehicle</td>
        <td>velocity</td>
        <td>/api/control/command/velocity</td>
        <td>velocity</td>
    </tr>
    <tr>
        <td>acceleration</td>
        <td>/api/control/command/acceleration</td>
        <td>acceleration</td>
    </tr>
    <tr>
        <td rowspan="4">/api/external/get/command/selected/control</td>
        <td>steering_angle</td>
        <td rowspan="2">/api/control/command/steering</td>
        <td>steering_tire_angle</td>
    </tr>
    <tr>
        <td>steering_angle_velocity</td>
        <td>steering_tire_velocity</td>
    </tr>
    <tr>
        <td>throttle</td>
        <td rowspan="2">/api/control/command/pedals</td>
        <td>throttle</td>
    </tr>
    <tr>
        <td>brake</td>
        <td>brake</td>
    </tr>
</table>
