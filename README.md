# TIER IV Autoware API Extension

This repository provides TIER IV Autoware API that specialized for use cases in TIER IV.
Some pages in this documentation are only in Japanese.

## Schedule

- v0.5.0: Remove APIs with EOL 2025/09
- v1.0.0: Remove APIs with EOL 2025/12

## Note

TIER IV Autoware API is sometimes called High-level API or TIER IV External API.

## TIER IV Autoware API

These APIs are specialized for use cases in TIER IV. There are currently no plans to move to AD API, but this may be done if there are many requests from the community.
If the version is empty, it means version v0.4.0 or earlier.

| Version | Type    | Name                                                                                                           | Message/Service                                                                                                                                                                                 |
| ------- | ------- | -------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| -       | service | [/api/external/get/version](./doc/api/external/get/version.md)                                                 | [autoware_external_api_msgs/srv/GetVersion](https://github.com/tier4/tier4_ad_api_adaptor/blob/tier4/universe/autoware_external_api_msgs/srv/GetVersion.srv)                                    |
| -       | service | [/api/external/get/metadata/packages](./doc/api/external/get/metadata/packages.md)                             | [tier4_external_api_msgs/srv/GetMetadataPackages](https://github.com/tier4/tier4_autoware_msgs/tree/tier4/universe/tier4_external_api_msgs/srv/GetMetadataPackages.srv)                         |
| v0.4.1  | topic   | [/api/external/get/planning_factors](./doc/api/external/get/planning_factors.md)                               | [tier4_external_api_msgs/msg/PlanningFactorArray](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_external_api_msgs/msg/PlanningFactorArray.msg)                         |
| v0.4.1  | topic   | [/api/external/get/route_distance](./doc/api/external/get/route_distance.md)                                   | [tier4_external_api_msgs/msg/RouteDistance](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_external_api_msgs/msg/RouteDistance.msg)                                     |
| v0.4.1  | topic   | [/api/external/get/nearest_traffic_light_group](./doc/api/external/get/nearest_traffic_light_group.md)         | [tier4_external_api_msgs/msg/TrafficLightGroup](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_external_api_msgs/msg/TrafficLightGroup.msg)                             |
| v0.4.1  | topic   | [/api/external/set/virtual_traffic_light/states](./doc/api/external/set/virtual_traffic_light/states.md)       | [tier4_v2x_msgs/msg/VirtualTrafficLightStateArray](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_v2x_msgs/msg/VirtualTrafficLightStateArray.msg)                       |
| v0.4.1  | topic   | [/api/external/get/virtual_traffic_light/commands](./doc/api/external/get/virtual_traffic_light/commands.md)   | [tier4_v2x_msgs/msg/InfrastructureCommandArray](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_v2x_msgs/msg/InfrastructureCommandArray.msg)                             |
| -       | service | [/api/external/set/velocity_limit](./doc/api/external/set/velocity_limit.md)                                   | [tier4_external_api_msgs/srv/SetVelocityLimit](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_external_api_msgs/srv/SetVelocityLimit.srv)                               |
| -       | topic   | [/api/external/get/localization_scores](./doc/api/external/get/localization_scores.md)                         | [tier4_external_api_msgs/msg/LocalizationScoreArray](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_external_api_msgs/msg/LocalizationScoreArray.msg)                   |
| -       | topic   | [/api/external/get/map/info/hash](./doc/api/external/get/map/info/hash.md)                                     | [tier4_external_api_msgs/msg/MapHash](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_external_api_msgs/msg/MapHash.msg)                                                 |
| -       | service | [/api/external/get/map/lanelet/xml](./doc/api/external/get/map/lanelet/xml.md)                                 | [tier4_external_api_msgs/srv/GetTextFile](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_external_api_msgs/srv/GetTextFile.srv)                                         |
| -       | service | [/api/external/set/rosbag_logging_mode](./doc/api/external/set/rosbag_logging_mode.md)                         | [tier4_external_api_msgs/srv/SetRosbagLoggingMode](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_external_api_msgs/srv/SetRosbagLoggingMode.srv)                       |
| -       | topic   | [/api/external/get/rosbag_logging_mode](./doc/api/external/get/rosbag_logging_mode.md)                         | [tier4_external_api_msgs/msg/RosbagLoggingMode](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_external_api_msgs/msg/RosbagLoggingMode.msg)                             |
| -       | topic   | [/api/external/get/calibration_status](./doc/api/external/get/calibration_status.md)                           | [tier4_external_api_msgs/msg/CalibrationStatusArray](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_external_api_msgs/msg/CalibrationStatusArray.msg)                   |
| -       | service | [/api/external/get/accel_brake_map_calibrator/data](./doc/api/external/get/accel_brake_map_calibrator/data.md) | [tier4_external_api_msgs/srv/GetAccelBrakeMapCalibrationData](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_external_api_msgs/srv/GetAccelBrakeMapCalibrationData.srv) |
| -       | topic   | [/api/external/get/system_monitor](./doc/api/external/get/system_monitor.md)                                   | [tier4_external_api_msgs/msg/SystemMonitor](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_external_api_msgs/msg/SystemMonitor.msg)                                     |
| -       | service | [/api/exteranl/set/lateral_offset](./doc/api/external/set/lateral_offset.md)                                   | [tier4_external_api_msgs/msg/SetLateralOffset](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_external_api_msgs/srv/SetLateralOffset/srv)                               |

## Deprecated API

These are old implementations used internally by TIER IV.
Please use [AD API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/) instead.

| EOL     | Type    | Name                                                                                                                               | Migration Guide                                                                                                                                                |
| ------- | ------- | ---------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2025/09 | service | [/api/external/set/service](./doc/api/external/set/service.md)                                                                     | This API is not used.                                                                                                                                          |
| 2025/09 | topic   | [/api/external/get/service](./doc/api/external/get/service.md)                                                                     | This API is not used.                                                                                                                                          |
| 2025/12 | service | [/api/external/set/initialize_pose](./doc/api/external/set/initialize_pose.md)                                                     | [Migrate to Localization API](./doc/migration/localization.md)                                                                                                 |
| 2025/12 | service | [/api/external/set/initialize_pose_auto](./doc/api/external/set/initialize_pose_auto.md)                                           | [Migrate to Localization API](./doc/migration/localization.md)                                                                                                 |
| 2025/12 | service | [/api/external/set/route](./doc/api/external/set/route.md)                                                                         | [Migrate to Routing API](./doc/migration/routing.md)                                                                                                           |
| 2025/12 | topic   | [/api/external/get/route](./doc/api/external/get/route.md)                                                                         | [Migrate to Routing API](./doc/migration/routing.md)                                                                                                           |
| 2025/12 | service | [/api/external/set/clear_route](./doc/api/external/set/clear_route.md)                                                             | [Migrate to Routing API](./doc/migration/routing.md)                                                                                                           |
| 2025/12 | service | [/api/external/set/engage](./doc/api/external/set/engage.md)                                                                       | [Migrate to Operation Mode API](./doc/migration/operation-mode.md)                                                                                             |
| 2025/12 | topic   | [/api/external/get/engage](./doc/api/external/get/engage.md)                                                                       | [Migrate to Operation Mode API](./doc/migration/operation-mode.md)                                                                                             |
| 2025/12 | service | [/api/external/set/operator](./doc/api/external/set/operator.md)                                                                   | [Migrate to Operation Mode API](./doc/migration/operation-mode.md)                                                                                             |
| 2025/12 | topic   | [/api/external/get/operator](./doc/api/external/get/operator.md)                                                                   | [Migrate to Operation Mode API](./doc/migration/operation-mode.md)                                                                                             |
| 2025/12 | service | [/api/external/set/observer](./doc/api/external/set/observer.md)                                                                   | [Migrate to Operation Mode API](./doc/migration/operation-mode.md)                                                                                             |
| 2025/12 | topic   | [/api/external/get/observer](./doc/api/external/get/observer.md)                                                                   | [Migrate to Operation Mode API](./doc/migration/operation-mode.md)                                                                                             |
| 2025/12 | service | [/api/external/set/pause_driving](./doc/api/external/set/pause_driving.md)                                                         | [Migrate to Operation Mode API](./doc/migration/operation-mode.md)                                                                                             |
| 2025/12 | service | [/api/autoware/set/start_request](./doc/api/autoware/set/start_request.md)                                                         | [Migrate to Motion API](./doc/migration/motion.md)                                                                                                             |
| 2025/12 | service | [/api/external/set/emergency](./doc/api/external/set/emergency.md)                                                                 | [Migrate to Fail-safe API](./doc/migration/fail-safe.md)                                                                                                       |
| 2025/12 | topic   | [/api/external/get/emergency](./doc/api/external/get/emergency.md)                                                                 | [Migrate to Fail-safe API](./doc/migration/fail-safe.md)                                                                                                       |
| 2025/12 | topic   | [/api/external/get/diagnostics](./doc/api/external/get/diagnostics.md)                                                             | [Migrate to Diagnostics API](./doc/migration/diagnostics.md)                                                                                                   |
| 2025/12 | topic   | [/api/external/get/vehicle/status](./doc/api/external/get/vehicle/status.md)                                                       | [Migrate to Vehicle Status API](./doc/migration/vehicle-status.md)                                                                                             |
| 2025/12 | service | [/api/external/set/door](./doc/api/external/set/door.md)                                                                           | [Migrate to Vehicle Doors API](./doc/migration/vehicle-doors.md)                                                                                               |
| 2025/12 | topic   | [/api/external/get/door](./doc/api/external/get/door.md)                                                                           | [Migrate to Vehicle Doors API](./doc/migration/vehicle-doors.md)                                                                                               |
| 2025/12 | topic   | [/api/external/set/command/local/control](./doc/api/external/set/command/local/control.md)                                         | [Migrate to Manual Control API](./doc/migration/manual-control.md)                                                                                             |
| 2025/12 | topic   | [/api/external/set/command/local/shift](./doc/api/external/set/command/local/shift.md)                                             | [Migrate to Manual Control API](./doc/migration/manual-control.md)                                                                                             |
| 2025/12 | topic   | [/api/external/set/command/local/turn_signal](./doc/api/external/set/command/local/turn_signal.md)                                 | [Migrate to Manual Control API](./doc/migration/manual-control.md)                                                                                             |
| 2025/12 | topic   | [/api/external/set/command/local/heartbeat](./doc/api/external/set/command/local/heartbeat.md)                                     | [Migrate to Manual Control API](./doc/migration/manual-control.md)                                                                                             |
| 2025/12 | topic   | [/api/external/set/command/remote/control](./doc/api/external/set/command/remote/control.md)                                       | [Migrate to Manual Control API](./doc/migration/manual-control.md)                                                                                             |
| 2025/12 | topic   | [/api/external/set/command/remote/shift](./doc/api/external/set/command/remote/shift.md)                                           | [Migrate to Manual Control API](./doc/migration/manual-control.md)                                                                                             |
| 2025/12 | topic   | [/api/external/set/command/remote/turn_signal](./doc/api/external/set/command/remote/turn_signal.md)                               | [Migrate to Manual Control API](./doc/migration/manual-control.md)                                                                                             |
| 2025/12 | topic   | [/api/external/set/command/remote/heartbeat](./doc/api/external/set/command/remote/heartbeat.md)                                   | [Migrate to Manual Control API](./doc/migration/manual-control.md)                                                                                             |
| 2025/12 | topic   | [/api/external/get/command/selected/control](./doc/api/external/get/command/selected/control.md)                                   | [Migrate to Control API](./doc/migration/control.md)                                                                                                           |
| 2025/12 | topic   | [/api/external/get/command/selected/vehicle](./doc/api/external/get/command/selected/vehicle.md)                                   | [Migrate to Control API](./doc/migration/control.md)                                                                                                           |
| 2025/12 | topic   | [/api/iv_msgs/planning/scenario_planning/trajectory](./doc/api/iv_msgs/planning/scenario_planning/trajectory.md)                   | [Migrate to Evaluation Interface](./doc/migration/evaluation.md)                                                                                               |
| 2025/12 | topic   | [/api/iv_msgs/perception/object_recognition/tracking/objects](./doc/api/iv_msgs/perception/object_recognition/tracking/objects.md) | [Migrate to Evaluation Interface](./doc/migration/evaluation.md)                                                                                               |
| 2025/12 | topic   | [/api/iv_msgs/autoware/state](./doc/api/iv_msgs/autoware/state.md)                                                                 | [Make Autoware State from AD API](https://autowarefoundation.github.io/autoware_universe/main/system/autoware_default_adapi_universe/document/autoware-state/) |

## Deprecated API (AWAPI)

| EOL     | Name (Field)                                         |
| ------- | ---------------------------------------------------- |
| 2025/12 | /awapi/autoware/get/status (autoware_state)          |
| 2025/12 | /awapi/autoware/get/status (control_mode)            |
| 2025/12 | /awapi/autoware/get/status (gate_mode)               |
| 2025/12 | /awapi/autoware/get/status (emergency_stopped)       |
| 2025/12 | /awapi/autoware/get/status (current_max_velocity)    |
| 2025/12 | /awapi/autoware/get/status (hazard_status)           |
| 2025/12 | /awapi/autoware/get/status (stop_reason)             |
| 2025/12 | /awapi/autoware/get/status (diagnostics)             |
| 2025/12 | /awapi/autoware/get/status (error_diagnostics)       |
| 2025/12 | /awapi/autoware/get/status (autonomous_overridden)   |
| 2025/12 | /awapi/autoware/get/status (arrived_goal)            |
| 2025/12 | /awapi/vehicle/get/status (pose)                     |
| 2025/12 | /awapi/vehicle/get/status (eulerangle)               |
| 2025/12 | /awapi/vehicle/get/status (geo_point)                |
| 2025/12 | /awapi/vehicle/get/status (velocity)                 |
| 2025/12 | /awapi/vehicle/get/status (acceleration)             |
| 2025/12 | /awapi/vehicle/get/status (steering)                 |
| 2025/12 | /awapi/vehicle/get/status (steering_velocity)        |
| 2025/12 | /awapi/vehicle/get/status (angular_velocity)         |
| 2025/12 | /awapi/vehicle/get/status (gear)                     |
| 2025/12 | /awapi/vehicle/get/status (energy_level)             |
| 2025/12 | /awapi/vehicle/get/status (turn_signal)              |
| 2025/12 | /awapi/vehicle/get/status (target_velocity)          |
| 2025/12 | /awapi/vehicle/get/status (target_acceleration)      |
| 2025/12 | /awapi/vehicle/get/status (target_steering)          |
| 2025/12 | /awapi/vehicle/get/status (target_steering_velocity) |
| 2025/12 | /awapi/tmp/infrastructure_commands                   |
| 2025/09 | /awapi/autoware/put/engage                           |
| 2025/09 | /awapi/autoware/put/route                            |
| 2025/09 | /awapi/autoware/put/goal                             |
| 2025/09 | /awapi/autoware/get/route                            |
| 2025/09 | /awapi/autoware/put/pose_initialization_request      |
| 2025/09 | /awapi/vehicle/put/stop                              |
| 2025/09 | /awapi/vehicle/put/velocity                          |
| 2025/09 | /awapi//tmp/virtual_traffic_light_states             |
| 2025/09 | /awapi/prediction/get/objects                        |
| 2025/09 | /awapi/autoware/get/stop_speed_exceeded              |
| 2025/09 | /awapi/autoware/put/expand_stop_range                |
| 2025/09 | /awapi/traffic_light/get/traffic_signals             |
| 2025/09 | /awapi/traffic_light/get/nearest_traffic_signal      |
| 2025/09 | /awapi/path_change/get/ready_module                  |
| 2025/09 | /awapi/path_change/get/force_available               |
| 2025/09 | /awapi/path_change/get/running_modules               |
| 2025/09 | /awapi/path_change/put/approval                      |
| 2025/09 | /awapi/path_change/put/force                         |
| 2025/09 | /awapi/lane_change/put/approval                      |
| 2025/09 | /awapi/lane_change/put/force                         |
| 2025/09 | /awapi/object_avoidance/put/approval                 |
| 2025/09 | /awapi/object_avoidance/put/force                    |
| 2025/09 | /awapi/traffic_light/put/traffic_signals             |
| 2025/09 | /awapi/autoware/put/crosswalk_states                 |
| 2025/09 | /awapi/autoware/put/intersection_states              |
| 2025/09 | /awapi/lane_change/get/status                        |
| 2025/09 | /awapi/object_avoidance/get/status                   |

## Removed API

These APIs are removed.
Please use [AD API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/) instead.

| Type  | Name                                                                                         | AD API                                                                                                                                            |
| ----- | -------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| topic | [/api/iv_msgs/vehicle/status/control_mode](./doc/api/iv_msgs/vehicle/status/control_mode.md) | [Operation mode API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/operation_mode/) |
