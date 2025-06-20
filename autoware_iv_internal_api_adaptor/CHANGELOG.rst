^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_iv_internal_api_adaptor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.46.0 (2025-06-20)
-------------------
* feat!: replace tier4_system_msgs with autoware_system_msgs (`#159 <https://github.com/tier4/tier4_ad_api_adaptor/issues/159>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome

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
* feat!: replace tier4_planning_msgs/VelocityLimit to autoware_internal_planning_msgs/VelocityLimit (`#136 <https://github.com/tier4/tier4_ad_api_adaptor/issues/136>`_)
* Contributors: Ryohsuke Mitsudome

0.41.0 (2025-02-10)
-------------------
* feat: remove autoware_auto messages (`#124 <https://github.com/tier4/tier4_ad_api_adaptor/issues/124>`_)
* fix(autoware_iv_internal_api_adaptor): iv msg types (`#121 <https://github.com/tier4/tier4_ad_api_adaptor/issues/121>`_)
* fix: change the output of ControlMode to autoware_auto_msgs (`#116 <https://github.com/tier4/tier4_ad_api_adaptor/issues/116>`_)
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
* feat(rtc_controller): add new module avoidance_by_lc (`#89 <https://github.com/tier4/tier4_ad_api_adaptor/issues/89>`_)
  * feat(rtc_controller): add new module avoidance_by_lc
  * chore(internal): apply pre-commit
  ---------
* feat: move launch files (`#90 <https://github.com/tier4/tier4_ad_api_adaptor/issues/90>`_)
* feat: apply mrm state to autoware state (`#81 <https://github.com/tier4/tier4_ad_api_adaptor/issues/81>`_)
  feat: apply mrm state
* feat: apply adapi operation mode (`#77 <https://github.com/tier4/tier4_ad_api_adaptor/issues/77>`_)
  * feat: apply adapi routing
  * feat: add auto route clear
  * Update autoware_iv_external_api_adaptor/src/converter/routing.hpp
  Co-authored-by: Kah Hooi Tan <41041286+tkhmy@users.noreply.github.com>
  * feat: apply adapi operation mode
  * feat: change service
  Co-authored-by: Kah Hooi Tan <41041286+tkhmy@users.noreply.github.com>
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
* fix: change initial pose timeout (`#65 <https://github.com/tier4/tier4_ad_api_adaptor/issues/65>`_)
* feat: add option to ignore set engage request when the status is emergency (`#58 <https://github.com/tier4/tier4_ad_api_adaptor/issues/58>`_)
  * feat: add option to ignore set engage request when the status is emergency
  * set default send-engage param to true
  * add null check
  * change default param
* feat(operator): support autoware operation mode service (`#51 <https://github.com/tier4/tier4_ad_api_adaptor/issues/51>`_)
  * feat(operator): support autoware operation mode service
  * change topic name
  * fix return value
  * fix typo
* feat: add external api adaptor for calibration status (`#43 <https://github.com/tier4/tier4_ad_api_adaptor/issues/43>`_)
  * (editting) save current work
  * fix bug and run pre-commit
  * pull the latest tier4/universe
  * pull the latest tier4/univerese
* Contributors: Ryohsuke Mitsudome, Satoshi OTA, Takagi, Isamu, Takamasa Horibe, TakumiKozaka-T4, Tomoya Kimura, Vincent Richard, Yukihiro Saito

0.40.0 (2024-11-25)
-------------------
* feat: remove autoware_auto messages (`#124 <https://github.com/tier4/tier4_ad_api_adaptor/issues/124>`_)
* fix(autoware_iv_internal_api_adaptor): iv msg types (`#121 <https://github.com/tier4/tier4_ad_api_adaptor/issues/121>`_)
* fix: change the output of ControlMode to autoware_auto_msgs (`#116 <https://github.com/tier4/tier4_ad_api_adaptor/issues/116>`_)
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
* feat(rtc_controller): add new module avoidance_by_lc (`#89 <https://github.com/tier4/tier4_ad_api_adaptor/issues/89>`_)
  * feat(rtc_controller): add new module avoidance_by_lc
  * chore(internal): apply pre-commit
  ---------
* feat: move launch files (`#90 <https://github.com/tier4/tier4_ad_api_adaptor/issues/90>`_)
* feat: apply mrm state to autoware state (`#81 <https://github.com/tier4/tier4_ad_api_adaptor/issues/81>`_)
  feat: apply mrm state
* feat: apply adapi operation mode (`#77 <https://github.com/tier4/tier4_ad_api_adaptor/issues/77>`_)
  * feat: apply adapi routing
  * feat: add auto route clear
  * Update autoware_iv_external_api_adaptor/src/converter/routing.hpp
  Co-authored-by: Kah Hooi Tan <41041286+tkhmy@users.noreply.github.com>
  * feat: apply adapi operation mode
  * feat: change service
  Co-authored-by: Kah Hooi Tan <41041286+tkhmy@users.noreply.github.com>
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
* fix: change initial pose timeout (`#65 <https://github.com/tier4/tier4_ad_api_adaptor/issues/65>`_)
* feat: add option to ignore set engage request when the status is emergency (`#58 <https://github.com/tier4/tier4_ad_api_adaptor/issues/58>`_)
  * feat: add option to ignore set engage request when the status is emergency
  * set default send-engage param to true
  * add null check
  * change default param
* feat(operator): support autoware operation mode service (`#51 <https://github.com/tier4/tier4_ad_api_adaptor/issues/51>`_)
  * feat(operator): support autoware operation mode service
  * change topic name
  * fix return value
  * fix typo
* feat: add external api adaptor for calibration status (`#43 <https://github.com/tier4/tier4_ad_api_adaptor/issues/43>`_)
  * (editting) save current work
  * fix bug and run pre-commit
  * pull the latest tier4/universe
  * pull the latest tier4/univerese
* chore: brand guideline (`#26 <https://github.com/tier4/tier4_ad_api_adaptor/issues/26>`_)
  * chore: brand guideline
  * chore: brand guideline
* ci: pre-commit for include guard (`#24 <https://github.com/tier4/tier4_ad_api_adaptor/issues/24>`_)
* chore: sync files (`#14 <https://github.com/tier4/tier4_ad_api_adaptor/issues/14>`_)
  Co-authored-by: Takagi, Isamu <isamu.takagi@tier4.jp>
* Move api pkgs (`#7 <https://github.com/tier4/tier4_ad_api_adaptor/issues/7>`_)
  * Move awapi package
  * WIP
  * Cancel external api adaptor
  * Fix package name
  * Fix package name
  * Move external api msgs
* Contributors: Ryohsuke Mitsudome, Satoshi OTA, Takagi, Isamu, Takamasa Horibe, TakumiKozaka-T4, Tomoya Kimura, Vincent Richard, Yukihiro Saito, tier4-autoware-public-bot[bot]
