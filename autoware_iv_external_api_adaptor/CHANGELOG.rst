^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_iv_external_api_adaptor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.46.0 (2025-06-20)
-------------------
* fix: api version v0.4.1 (`#162 <https://github.com/tier4/tier4_ad_api_adaptor/issues/162>`_ )
* feat: release v0.5.0 (`#160 <https://github.com/tier4/tier4_ad_api_adaptor/issues/160>`_)
* chore: update autoware.universe with autoware_universe (`#158 <https://github.com/tier4/tier4_ad_api_adaptor/issues/158>`_)
* Contributors: Takagi, Isamu

0.43.0 (2025-05-12)
-------------------
* feat(tier4_deprecated_api_adapter): create adapter for manual control (`#138 <https://github.com/tier4/tier4_ad_api_adaptor/issues/138>`_)
  * feat(tier4_deprecated_api_adapter): create manual control adapter
  * apply rename
  * add steering velocity
  * fix for spell check
  * add readme
  * add launch
  * fix topic name
  * fix indicator value
  * translate document
  * reflect review
  * test build depend
  ---------
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* Contributors: Takagi, Isamu

0.42.0 (2025-04-03)
-------------------

0.41.0 (2025-02-10)
-------------------
* feat: rename autoware_component_interface_specs to autoware_component_interface_specs_universe (`#130 <https://github.com/tier4/tier4_ad_api_adaptor/issues/130>`_)
* fix(external): fix old package name (`#128 <https://github.com/tier4/tier4_ad_api_adaptor/issues/128>`_)
  * fix(external): fix old package name
  * fix: typo
  ---------
* refactor(autoware_iv_external_api_adaptor): update component_interface_utils dependency (`#127 <https://github.com/tier4/tier4_ad_api_adaptor/issues/127>`_)
  * refactor(autoware_iv_external_api_adaptor): update component_interface_utils dependency
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(autoware_iv_external_api_adaptor): update component_interface_specs dependency (`#126 <https://github.com/tier4/tier4_ad_api_adaptor/issues/126>`_)
  * refactor(autoware_iv_external_api_adaptor): update component_interface_specs dependency
  * shorten namespaces
  * update includes
  ---------
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
* chore(rtc_controller): update module name (`#114 <https://github.com/tier4/tier4_ad_api_adaptor/issues/114>`_)
* feat: localization initialization method (`#112 <https://github.com/tier4/tier4_ad_api_adaptor/issues/112>`_)
* feat: add support auto mode status (`#111 <https://github.com/tier4/tier4_ad_api_adaptor/issues/111>`_)
  * feat: add support auto mode status
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(start_planner): rename pull out to start planner (`#108 <https://github.com/tier4/tier4_ad_api_adaptor/issues/108>`_)
* build: use <buildtool_depend> for autoware_cmake (`#103 <https://github.com/tier4/tier4_ad_api_adaptor/issues/103>`_)
  * build: use <buildtool_depend> for autoware_cmake
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(behavior_path_planner): rename pull_over to goal_planner (`#102 <https://github.com/tier4/tier4_ad_api_adaptor/issues/102>`_)
  refactor: renaem pull_over to goal_planenr
* feat(behavior_velocity_planner::intersection): add occlusion detection feature (`#100 <https://github.com/tier4/tier4_ad_api_adaptor/issues/100>`_)
  add intersection occlusion
* feat: add rosbag_logging_mode (`#98 <https://github.com/tier4/tier4_ad_api_adaptor/issues/98>`_)
* feat: change the external lane change name (`#97 <https://github.com/tier4/tier4_ad_api_adaptor/issues/97>`_)
* feat(rtc_controller): add new module avoidance_by_lc (`#89 <https://github.com/tier4/tier4_ad_api_adaptor/issues/89>`_)
  * feat(rtc_controller): add new module avoidance_by_lc
  * chore(internal): apply pre-commit
  ---------
* feat(rtc_controller): add external request lane change module (`#83 <https://github.com/tier4/tier4_ad_api_adaptor/issues/83>`_)
* feat: move launch files (`#90 <https://github.com/tier4/tier4_ad_api_adaptor/issues/90>`_)
* feat: external api v0.4.0 (`#79 <https://github.com/tier4/tier4_ad_api_adaptor/issues/79>`_)
* feat(rtc_controller): add auto mode api (`#86 <https://github.com/tier4/tier4_ad_api_adaptor/issues/86>`_)
  add auto mode api
* feat: apply mrm state to awapi (`#82 <https://github.com/tier4/tier4_ad_api_adaptor/issues/82>`_)
  * feat: apply mrm state
  * feat: rename emergency state to mrm state
* feat: apply adapi routing (`#75 <https://github.com/tier4/tier4_ad_api_adaptor/issues/75>`_)
  * feat: apply adapi routing
  * feat: add auto route clear
  * Update autoware_iv_external_api_adaptor/src/converter/routing.hpp
  Co-authored-by: Kah Hooi Tan <41041286+tkhmy@users.noreply.github.com>
  Co-authored-by: Kah Hooi Tan <41041286+tkhmy@users.noreply.github.com>
* docs: fix url (`#78 <https://github.com/tier4/tier4_ad_api_adaptor/issues/78>`_)
* refactor!: remove tier4 control mode msg (`#60 <https://github.com/tier4/tier4_ad_api_adaptor/issues/60>`_)
  * remove t4-auto message converter for ControlMode
  * fix: keep the interface
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
* feat: apply adapi initial pose (`#69 <https://github.com/tier4/tier4_ad_api_adaptor/issues/69>`_)
  * feat: apply adapi initial pose
  * feat: overwrite covariance for backward compatibility
* chore: use autoware_cmake (`#71 <https://github.com/tier4/tier4_ad_api_adaptor/issues/71>`_)
* fix(rtc_controller): fix RTC name space (`#61 <https://github.com/tier4/tier4_ad_api_adaptor/issues/61>`_)
* feat(rtc_controller): add_finish_distance (`#64 <https://github.com/tier4/tier4_ad_api_adaptor/issues/64>`_)
* fix: change initial pose timeout (`#65 <https://github.com/tier4/tier4_ad_api_adaptor/issues/65>`_)
* feat: change version to 0.2.1 (`#57 <https://github.com/tier4/tier4_ad_api_adaptor/issues/57>`_)
* feature: change error code for engage in driving (`#52 <https://github.com/tier4/tier4_ad_api_adaptor/issues/52>`_)
* fix: rtc distance (`#56 <https://github.com/tier4/tier4_ad_api_adaptor/issues/56>`_)
  * fix distance value
  * temp fix
  * clean
  * delete comment
  * add comment
  Co-authored-by: khtan <tkh.my.p@gmail.com>
* feat: add localization score adapter (`#54 <https://github.com/tier4/tier4_ad_api_adaptor/issues/54>`_)
  * add localization score relay node
  * change output topic name
  * Delete commented out codes.
  * fixed typo.
* fix: remove unneeded namespace (`#53 <https://github.com/tier4/tier4_ad_api_adaptor/issues/53>`_)
  remove unneeded namespace
* refactor: refactor rtc adaptor (`#49 <https://github.com/tier4/tier4_ad_api_adaptor/issues/49>`_)
  * fix client crash node error
  * refractor rtc controller
  * refactor service
  * fix syntax
* feat: add external api adaptor for calibration status (`#43 <https://github.com/tier4/tier4_ad_api_adaptor/issues/43>`_)
  * (editting) save current work
  * fix bug and run pre-commit
  * pull the latest tier4/universe
  * pull the latest tier4/univerese
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kah Hooi Tan, Kosuke Takeuchi, Mamoru Sobue, Ryohsuke Mitsudome, Satoshi OTA, Takagi, Isamu, Takayuki Murooka, Takeshi Miura, TakumiKozaka-T4, Vincent Richard, Yukihiro Saito, hidenaga

0.40.0 (2024-11-25)
-------------------
* fix(external): fix old package name (`#128 <https://github.com/tier4/tier4_ad_api_adaptor/issues/128>`_)
  * fix(external): fix old package name
  * fix: typo
  ---------
* refactor(autoware_iv_external_api_adaptor): update component_interface_utils dependency (`#127 <https://github.com/tier4/tier4_ad_api_adaptor/issues/127>`_)
  * refactor(autoware_iv_external_api_adaptor): update component_interface_utils dependency
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(autoware_iv_external_api_adaptor): update component_interface_specs dependency (`#126 <https://github.com/tier4/tier4_ad_api_adaptor/issues/126>`_)
  * refactor(autoware_iv_external_api_adaptor): update component_interface_specs dependency
  * shorten namespaces
  * update includes
  ---------
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
* chore(rtc_controller): update module name (`#114 <https://github.com/tier4/tier4_ad_api_adaptor/issues/114>`_)
* feat: localization initialization method (`#112 <https://github.com/tier4/tier4_ad_api_adaptor/issues/112>`_)
* feat: add support auto mode status (`#111 <https://github.com/tier4/tier4_ad_api_adaptor/issues/111>`_)
  * feat: add support auto mode status
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(start_planner): rename pull out to start planner (`#108 <https://github.com/tier4/tier4_ad_api_adaptor/issues/108>`_)
* build: use <buildtool_depend> for autoware_cmake (`#103 <https://github.com/tier4/tier4_ad_api_adaptor/issues/103>`_)
  * build: use <buildtool_depend> for autoware_cmake
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(behavior_path_planner): rename pull_over to goal_planner (`#102 <https://github.com/tier4/tier4_ad_api_adaptor/issues/102>`_)
  refactor: renaem pull_over to goal_planenr
* feat(behavior_velocity_planner::intersection): add occlusion detection feature (`#100 <https://github.com/tier4/tier4_ad_api_adaptor/issues/100>`_)
  add intersection occlusion
* feat: add rosbag_logging_mode (`#98 <https://github.com/tier4/tier4_ad_api_adaptor/issues/98>`_)
* feat: change the external lane change name (`#97 <https://github.com/tier4/tier4_ad_api_adaptor/issues/97>`_)
* feat(rtc_controller): add new module avoidance_by_lc (`#89 <https://github.com/tier4/tier4_ad_api_adaptor/issues/89>`_)
  * feat(rtc_controller): add new module avoidance_by_lc
  * chore(internal): apply pre-commit
  ---------
* feat(rtc_controller): add external request lane change module (`#83 <https://github.com/tier4/tier4_ad_api_adaptor/issues/83>`_)
* feat: move launch files (`#90 <https://github.com/tier4/tier4_ad_api_adaptor/issues/90>`_)
* feat: external api v0.4.0 (`#79 <https://github.com/tier4/tier4_ad_api_adaptor/issues/79>`_)
* feat(rtc_controller): add auto mode api (`#86 <https://github.com/tier4/tier4_ad_api_adaptor/issues/86>`_)
  add auto mode api
* feat: apply mrm state to awapi (`#82 <https://github.com/tier4/tier4_ad_api_adaptor/issues/82>`_)
  * feat: apply mrm state
  * feat: rename emergency state to mrm state
* feat: apply adapi routing (`#75 <https://github.com/tier4/tier4_ad_api_adaptor/issues/75>`_)
  * feat: apply adapi routing
  * feat: add auto route clear
  * Update autoware_iv_external_api_adaptor/src/converter/routing.hpp
  Co-authored-by: Kah Hooi Tan <41041286+tkhmy@users.noreply.github.com>
  Co-authored-by: Kah Hooi Tan <41041286+tkhmy@users.noreply.github.com>
* docs: fix url (`#78 <https://github.com/tier4/tier4_ad_api_adaptor/issues/78>`_)
* refactor!: remove tier4 control mode msg (`#60 <https://github.com/tier4/tier4_ad_api_adaptor/issues/60>`_)
  * remove t4-auto message converter for ControlMode
  * fix: keep the interface
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
* feat: apply adapi initial pose (`#69 <https://github.com/tier4/tier4_ad_api_adaptor/issues/69>`_)
  * feat: apply adapi initial pose
  * feat: overwrite covariance for backward compatibility
* chore: use autoware_cmake (`#71 <https://github.com/tier4/tier4_ad_api_adaptor/issues/71>`_)
* fix(rtc_controller): fix RTC name space (`#61 <https://github.com/tier4/tier4_ad_api_adaptor/issues/61>`_)
* feat(rtc_controller): add_finish_distance (`#64 <https://github.com/tier4/tier4_ad_api_adaptor/issues/64>`_)
* fix: change initial pose timeout (`#65 <https://github.com/tier4/tier4_ad_api_adaptor/issues/65>`_)
* feat: change version to 0.2.1 (`#57 <https://github.com/tier4/tier4_ad_api_adaptor/issues/57>`_)
* feature: change error code for engage in driving (`#52 <https://github.com/tier4/tier4_ad_api_adaptor/issues/52>`_)
* fix: rtc distance (`#56 <https://github.com/tier4/tier4_ad_api_adaptor/issues/56>`_)
  * fix distance value
  * temp fix
  * clean
  * delete comment
  * add comment
  Co-authored-by: khtan <tkh.my.p@gmail.com>
* feat: add localization score adapter (`#54 <https://github.com/tier4/tier4_ad_api_adaptor/issues/54>`_)
  * add localization score relay node
  * change output topic name
  * Delete commented out codes.
  * fixed typo.
* fix: remove unneeded namespace (`#53 <https://github.com/tier4/tier4_ad_api_adaptor/issues/53>`_)
  remove unneeded namespace
* refactor: refactor rtc adaptor (`#49 <https://github.com/tier4/tier4_ad_api_adaptor/issues/49>`_)
  * fix client crash node error
  * refractor rtc controller
  * refactor service
  * fix syntax
* feat: add external api adaptor for calibration status (`#43 <https://github.com/tier4/tier4_ad_api_adaptor/issues/43>`_)
  * (editting) save current work
  * fix bug and run pre-commit
  * pull the latest tier4/universe
  * pull the latest tier4/univerese
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
* fix: set timeout of service to 190 sec (`#32 <https://github.com/tier4/tier4_ad_api_adaptor/issues/32>`_)
  * fix: set timeout of service to 190 sec
  * follow pre-commit
  * remove unnecessary brackets
  * follow pre-commit
  * set timeout to 190sec considering systemd default timeout(start/stop) plus extra
  * set timeout to 190sec considering systemd default timeout(start/stop) plus extra
  * add description for timeout
  * add web site
* feat: add rosbag logging mode (`#28 <https://github.com/tier4/tier4_ad_api_adaptor/issues/28>`_)
  * add rosbag logging mode
  * feat: add rosbag logging mode adaptor
  * follow clang-format
* ci: pre-commit for include guard (`#24 <https://github.com/tier4/tier4_ad_api_adaptor/issues/24>`_)
* feat: add external adaptor for cpu usage topic (`#19 <https://github.com/tier4/tier4_ad_api_adaptor/issues/19>`_)
  * add external adaptor for cpu usage topic
  * run pre-commit
  * modified along the comment on PR
  * modify a comment
* chore: sync files (`#14 <https://github.com/tier4/tier4_ad_api_adaptor/issues/14>`_)
  Co-authored-by: Takagi, Isamu <isamu.takagi@tier4.jp>
* add get door status (`#16 <https://github.com/tier4/tier4_ad_api_adaptor/issues/16>`_)
  * add get door status
  * add timestamp
* Add checking operator mode to start request (`#11 <https://github.com/tier4/tier4_ad_api_adaptor/issues/11>`_)
* Add auto operator change option to engage (`#10 <https://github.com/tier4/tier4_ad_api_adaptor/issues/10>`_)
* Move api pkgs (`#7 <https://github.com/tier4/tier4_ad_api_adaptor/issues/7>`_)
  * Move awapi package
  * WIP
  * Cancel external api adaptor
  * Fix package name
  * Fix package name
  * Move external api msgs
* change packages name (`#6 <https://github.com/tier4/tier4_ad_api_adaptor/issues/6>`_)
  * autoware_iv_auto_msgs_converter -> tier4_auto_msgs_converter
  * autoware_external_api_msgs -> tier4_external_api_msgs
  * autoware_api_utils -> tier4_api_utils
  * autoware_vehicle_msgs -> tier4_vehicle_msgs
  * fix format
* Merge pull request `#4 <https://github.com/tier4/tier4_ad_api_adaptor/issues/4>`_ from tier4/fix/api-readme
* Fix api readme
* Change api version (`#3 <https://github.com/tier4/tier4_ad_api_adaptor/issues/3>`_)
* Fix vehicle status (`#1 <https://github.com/tier4/tier4_ad_api_adaptor/issues/1>`_)
* Add autoware api readme (`#33 <https://github.com/tier4/tier4_ad_api_adaptor/issues/33>`_)
  * Add readme
  * Add readme
  * Fix for pre-commit
* Port Autoware API to .auto  (`#32 <https://github.com/tier4/tier4_ad_api_adaptor/issues/32>`_)
  * Move autoware_api_utils
  * Use autoware_auto_system_msgs
  * Use autoware_auto_vehicle_msgs
* Add fail safe state API (`#20 <https://github.com/tier4/tier4_ad_api_adaptor/issues/20>`_)
* Add vehicle command API (`#11 <https://github.com/tier4/tier4_ad_api_adaptor/issues/11>`_)
* Add vehicle status API (`#8 <https://github.com/tier4/tier4_ad_api_adaptor/issues/8>`_)
  * Add vehicle status API
  * Fix logging
  * Fix lint
* Add lanelet XML API (`#26 <https://github.com/tier4/tier4_ad_api_adaptor/issues/26>`_)
* Add package version API (`#22 <https://github.com/tier4/tier4_ad_api_adaptor/issues/22>`_)
  * Add package version API
  * Fix message type
  * Fix ament index
  * Modify api name
* Add emergency status API (`#24 <https://github.com/tier4/tier4_ad_api_adaptor/issues/24>`_)
* Merge pull request `#5 <https://github.com/tier4/tier4_ad_api_adaptor/issues/5>`_ from tier4/feature/move-start-request-api
  Move start request API
* Move start request API
* Move autoware api launch files (`#4 <https://github.com/tier4/tier4_ad_api_adaptor/issues/4>`_)
* Add default api
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kah Hooi Tan, Kosuke Takeuchi, Mamoru Sobue, Satoshi OTA, Taichi Higashide, Takagi, Isamu, Takayuki Murooka, Takeshi Miura, TakumiKozaka-T4, Tomoya Kimura, Vincent Richard, Yukihiro Saito, hidenaga, kk-inoue-esol, tier4-autoware-public-bot[bot]
