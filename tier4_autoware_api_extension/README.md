# tier4_autoware_api_extension

This package provides API that specialized for use cases in TIER IV.
[See here for the TIER IV API specification.](https://tier4.github.io/autoware-documentation/tier4-main/design/autoware-interfaces/prototyping/)

## Planning factor

This module merges and publishes internal planning factor message for the planning factor API.

### Parameters

| Name                    | Type        | Description                                                                                                                                                                                                                         |
| ----------------------- | ----------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| rate                    | float       | Publication rate of the external topic.                                                                                                                                                                                             |
| timeout                 | float       | Internal topic timeout. The factor is removed from the merged factors when it times out.                                                                                                                                            |
| topics                  | string list | List of internal topic names.                                                                                                                                                                                                       |
| behavior_name_remapping | string      | Remapping of behavior names. Due to ROS 2 parameter limitations, specify this as a yaml-like map string. Any name not in the remapping is replaced with "unknown". If an empty string is specified, no remapping will be performed. |
