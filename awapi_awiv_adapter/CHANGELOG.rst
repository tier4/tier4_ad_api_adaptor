^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package awapi_awiv_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.46.0 (2025-06-20)
-------------------

0.43.0 (2025-05-12)
-------------------
* feat: use control command gate (`#148 <https://github.com/tier4/tier4_ad_api_adaptor/issues/148>`_)
  feat: add use control command gate (`#140 <https://github.com/tier4/tier4_ad_api_adaptor/issues/140>`_)
* Contributors: Tetsuhiro Kawaguchi

0.42.0 (2025-04-03)
-------------------
* feat(awapi_awiv_adapter): use glog (`#137 <https://github.com/tier4/tier4_ad_api_adaptor/issues/137>`_)
* feat!: replace tier4_planning_msgs/VelocityLimit to autoware_internal_planning_msgs/VelocityLimit (`#136 <https://github.com/tier4/tier4_ad_api_adaptor/issues/136>`_)
* Contributors: Ryohsuke Mitsudome, Takagi, Isamu

0.41.0 (2025-02-10)
-------------------
* feat(awapi): add velocity factor converter (`#129 <https://github.com/tier4/tier4_ad_api_adaptor/issues/129>`_)
  * feat(awapi): add velocity factor converter
  * fix: use velocity factor converter
  * fix: remove unused subscriber
  ---------
* feat: set timestamp to max_velocity topic (`#125 <https://github.com/tier4/tier4_ad_api_adaptor/issues/125>`_)
  * set timestamp to max_velocity topic
  * fix constructor order
  ---------
* feat: remove autoware_auto messages (`#124 <https://github.com/tier4/tier4_ad_api_adaptor/issues/124>`_)
* fix: fix smoother namespace (`#118 <https://github.com/tier4/tier4_ad_api_adaptor/issues/118>`_)
* feat!: change from autoware_auto_msgs to autoware_msgs (`#115 <https://github.com/tier4/tier4_ad_api_adaptor/issues/115>`_)
  * fix:autoware_planning_msgs
  * feat(tier4_ad_api_adaptor): replace autoware_control_msg with autoware_control_msg
  * feat(tier4_ad_api_adaptor): replace autoware_auto_system_msg with autoware_system_msg
  * feat(tier4_ad_api_adaptor): replace autoware_auto_msgs with autoware_msgs
  * feat: replace DetectedObjects and TrackedObjects messages to autoware_perception_msgs
  * style(pre-commit): autofix
  ---------
  Co-authored-by: jack.song <jack.song@autocore.ai>
  Co-authored-by: NorahXiong <norah.xiong@autocore.ai>
  Co-authored-by: liu cui <cynthia.liu@autocore.ai>
  Co-authored-by: Ryohsuke Mitsudome <ryohsuke.mitsudome@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* build: use <buildtool_depend> for autoware_cmake (`#103 <https://github.com/tier4/tier4_ad_api_adaptor/issues/103>`_)
  * build: use <buildtool_depend> for autoware_cmake
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(stop_reason_aggregator): fix threshold for closest index (`#99 <https://github.com/tier4/tier4_ad_api_adaptor/issues/99>`_)
* fix: rename use_external_emergency_stop to check_external_emergency_heartbeat (`#84 <https://github.com/tier4/tier4_ad_api_adaptor/issues/84>`_)
* feat: apply mrm state to awapi (`#82 <https://github.com/tier4/tier4_ad_api_adaptor/issues/82>`_)
  * feat: apply mrm state
  * feat: rename emergency state to mrm state
* refactor!: remove tier4 control mode msg (`#60 <https://github.com/tier4/tier4_ad_api_adaptor/issues/60>`_)
  * remove t4-auto message converter for ControlMode
  * fix: keep the interface
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
* chore: use autoware_cmake (`#71 <https://github.com/tier4/tier4_ad_api_adaptor/issues/71>`_)
* feat(rtc_controller): add_finish_distance (`#64 <https://github.com/tier4/tier4_ad_api_adaptor/issues/64>`_)
* change energy level default value for rclpy bug on humble (`#48 <https://github.com/tier4/tier4_ad_api_adaptor/issues/48>`_)
  fix default value
* feat: add external api adaptor for calibration status (`#43 <https://github.com/tier4/tier4_ad_api_adaptor/issues/43>`_)
  * (editting) save current work
  * fix bug and run pre-commit
  * pull the latest tier4/universe
  * pull the latest tier4/univerese
* fix(awapi_awiv_adapter): change velocity limit publisher QoS(Volatile to TransientLocal) (`#46 <https://github.com/tier4/tier4_ad_api_adaptor/issues/46>`_)
* chore: replace topic tools (`#44 <https://github.com/tier4/tier4_ad_api_adaptor/issues/44>`_)
* Contributors: Autumn60, Daisuke Nishimatsu, Fumiya Watanabe, Hiroki OTA, Satoshi OTA, Shumpei Wakabayashi, Takagi, Isamu, TakumiKozaka-T4, Vincent Richard, Yukihiro Saito, yabuta

0.40.0 (2024-11-25)
-------------------
* feat(awapi): add velocity factor converter (`#129 <https://github.com/tier4/tier4_ad_api_adaptor/issues/129>`_)
  * feat(awapi): add velocity factor converter
  * fix: use velocity factor converter
  * fix: remove unused subscriber
  ---------
* feat: set timestamp to max_velocity topic (`#125 <https://github.com/tier4/tier4_ad_api_adaptor/issues/125>`_)
  * set timestamp to max_velocity topic
  * fix constructor order
  ---------
* feat: remove autoware_auto messages (`#124 <https://github.com/tier4/tier4_ad_api_adaptor/issues/124>`_)
* fix: fix smoother namespace (`#118 <https://github.com/tier4/tier4_ad_api_adaptor/issues/118>`_)
* feat!: change from autoware_auto_msgs to autoware_msgs (`#115 <https://github.com/tier4/tier4_ad_api_adaptor/issues/115>`_)
  * fix:autoware_planning_msgs
  * feat(tier4_ad_api_adaptor): replace autoware_control_msg with autoware_control_msg
  * feat(tier4_ad_api_adaptor): replace autoware_auto_system_msg with autoware_system_msg
  * feat(tier4_ad_api_adaptor): replace autoware_auto_msgs with autoware_msgs
  * feat: replace DetectedObjects and TrackedObjects messages to autoware_perception_msgs
  * style(pre-commit): autofix
  ---------
  Co-authored-by: jack.song <jack.song@autocore.ai>
  Co-authored-by: NorahXiong <norah.xiong@autocore.ai>
  Co-authored-by: liu cui <cynthia.liu@autocore.ai>
  Co-authored-by: Ryohsuke Mitsudome <ryohsuke.mitsudome@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* build: use <buildtool_depend> for autoware_cmake (`#103 <https://github.com/tier4/tier4_ad_api_adaptor/issues/103>`_)
  * build: use <buildtool_depend> for autoware_cmake
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(stop_reason_aggregator): fix threshold for closest index (`#99 <https://github.com/tier4/tier4_ad_api_adaptor/issues/99>`_)
* fix: rename use_external_emergency_stop to check_external_emergency_heartbeat (`#84 <https://github.com/tier4/tier4_ad_api_adaptor/issues/84>`_)
* feat: apply mrm state to awapi (`#82 <https://github.com/tier4/tier4_ad_api_adaptor/issues/82>`_)
  * feat: apply mrm state
  * feat: rename emergency state to mrm state
* refactor!: remove tier4 control mode msg (`#60 <https://github.com/tier4/tier4_ad_api_adaptor/issues/60>`_)
  * remove t4-auto message converter for ControlMode
  * fix: keep the interface
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
* chore: use autoware_cmake (`#71 <https://github.com/tier4/tier4_ad_api_adaptor/issues/71>`_)
* feat(rtc_controller): add_finish_distance (`#64 <https://github.com/tier4/tier4_ad_api_adaptor/issues/64>`_)
* change energy level default value for rclpy bug on humble (`#48 <https://github.com/tier4/tier4_ad_api_adaptor/issues/48>`_)
  fix default value
* feat: add external api adaptor for calibration status (`#43 <https://github.com/tier4/tier4_ad_api_adaptor/issues/43>`_)
  * (editting) save current work
  * fix bug and run pre-commit
  * pull the latest tier4/universe
  * pull the latest tier4/univerese
* fix(awapi_awiv_adapter): change velocity limit publisher QoS(Volatile to TransientLocal) (`#46 <https://github.com/tier4/tier4_ad_api_adaptor/issues/46>`_)
* chore: replace topic tools (`#44 <https://github.com/tier4/tier4_ad_api_adaptor/issues/44>`_)
* feat: add rtc controller (`#37 <https://github.com/tier4/tier4_ad_api_adaptor/issues/37>`_)
  * add rtc_controller with subscriber and publisher
  * add handling service
  * fix subscriber error
  * fix services
  * fix syntax
  * fix pre-commit error
  * remove stopline
  * fix naming!
  * fix pointer
* chore: brand guideline (`#26 <https://github.com/tier4/tier4_ad_api_adaptor/issues/26>`_)
  * chore: brand guideline
  * chore: brand guideline
* fix(awapi_awiv_adapter): modify build error in rolling (`#36 <https://github.com/tier4/tier4_ad_api_adaptor/issues/36>`_)
* refactor: remove pacmod related from awapi (`#23 <https://github.com/tier4/tier4_ad_api_adaptor/issues/23>`_)
  * remove pacmod related from awapi
  * remove unused stuff
* chore: sync files (`#14 <https://github.com/tier4/tier4_ad_api_adaptor/issues/14>`_)
  Co-authored-by: Takagi, Isamu <isamu.takagi@tier4.jp>
* Fix awapi autoware state for emergency (`#9 <https://github.com/tier4/tier4_ad_api_adaptor/issues/9>`_)
* Move api pkgs (`#7 <https://github.com/tier4/tier4_ad_api_adaptor/issues/7>`_)
  * Move awapi package
  * WIP
  * Cancel external api adaptor
  * Fix package name
  * Fix package name
  * Move external api msgs
* Contributors: Autumn60, Daisuke Nishimatsu, Fumiya Watanabe, Hiroki OTA, Kah Hooi Tan, Satoshi OTA, Shumpei Wakabayashi, Takagi, Isamu, TakumiKozaka-T4, Vincent Richard, Yukihiro Saito, tier4-autoware-public-bot[bot], yabuta
