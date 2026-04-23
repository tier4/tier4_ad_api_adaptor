# /api/external/set/lateral_offset

## Classification

- Behavior: Service
- DataType: tier4_external_api_msgs/srv/SetLateralOffset

## Description

指定したシフト量を走行経路に適用し、走行経路を左右にずらす。直接シフト量をメートル単位で指示する方法と、抽象的な指示（左/右/リセット）を与える方法がある。

## Requirement

現在の車両の状態を考慮した適切なシフト幅を設定すること。
