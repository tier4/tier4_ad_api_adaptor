#include "tier4_autoware_api_host/tier4_api_host_node.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tier4_api_msgs/msg/crosswalk_status.hpp>
#include <tier4_api_msgs/msg/intersection_status.hpp>

namespace tier4_autoware_api_host
{

namespace
{
bool is_plugin_in_list(const std::string & plugin_name, const char * const * list, size_t count)
{
  for (size_t i = 0; i < count; ++i) {
    if (plugin_name == list[i]) {
      return true;
    }
  }
  return false;
}

bool is_api_mode_lt_2_only_plugin(const std::string & plugin_name)
{
  static const char * plugins[] = {
    "tier4_autoware_api_host::plugin::ExternalDiagnosticsPlugin",
    "tier4_autoware_api_host::plugin::ExternalDoorPlugin",
    "tier4_autoware_api_host::plugin::ExternalEmergencyPlugin",
    "tier4_autoware_api_host::plugin::ExternalEngagePlugin",
    "tier4_autoware_api_host::plugin::ExternalFailSafeStatePlugin",
    "tier4_autoware_api_host::plugin::ExternalInitialPosePlugin",
    "tier4_autoware_api_host::plugin::ExternalOperatorPlugin",
    "tier4_autoware_api_host::plugin::ExternalRoutePlugin",
    "tier4_autoware_api_host::plugin::ExternalStartPlugin",
    "tier4_autoware_api_host::plugin::ExternalVehicleStatusPlugin",
    "tier4_autoware_api_host::plugin::ExternalVelocityPlugin",
    "tier4_autoware_api_host::plugin::InternalIVMsgsPlugin",
    "tier4_autoware_api_host::plugin::InternalOperatorPlugin",
    "tier4_autoware_api_host::plugin::InternalVelocityPlugin",
    "tier4_autoware_api_host::plugin::DeprecatedEngagePlugin",
    "tier4_autoware_api_host::plugin::DeprecatedManualStatusPlugin",
    "tier4_autoware_api_host::plugin::DeprecatedManualControlPlugin",
    "tier4_autoware_api_host::plugin::DeprecatedHazardStatusPlugin",
  };
  return is_plugin_in_list(plugin_name, plugins, sizeof(plugins) / sizeof(plugins[0]));
}

bool is_api_mode_lt_1_only_plugin(const std::string & plugin_name)
{
  static const char * plugins[] = {
    "tier4_autoware_api_host::plugin::AwapiLaneChangePlugin",
    "tier4_autoware_api_host::plugin::AwapiObstacleAvoidancePlugin",
  };
  return is_plugin_in_list(plugin_name, plugins, sizeof(plugins) / sizeof(plugins[0]));
}

bool is_api_mode_gte_2_only_plugin(const std::string & plugin_name)
{
  static const char * plugins[] = {
    "tier4_autoware_api_host::plugin::ExtensionVelocityLimitPlugin",
    "tier4_autoware_api_host::plugin::DeprecatedInitialPosePlugin",
  };
  return is_plugin_in_list(plugin_name, plugins, sizeof(plugins) / sizeof(plugins[0]));
}
}  // namespace

Tier4ApiHostNode::Tier4ApiHostNode(const rclcpp::NodeOptions & options)
: Node("tier4_api_host", options),
  plugin_loader_(
    std::make_unique<pluginlib::ClassLoader<plugin::Tier4ApiAdaptorPluginBase>>(
      "tier4_autoware_api_host",
      "tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase")),
  api_data_(std::make_shared<plugin::ApiData>())
{
  set_up_params();
  initialize_plugins();

  set_param_res_ = add_on_set_parameters_callback(
    std::bind(&Tier4ApiHostNode::on_parameter, this, std::placeholders::_1));

  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  create_subscriptions();
  create_relays();

  timer_ = create_wall_timer(
    std::chrono::milliseconds(get_parameter("timer_period_ms").as_int()),
    std::bind(&Tier4ApiHostNode::on_timer, this));
}

void Tier4ApiHostNode::set_up_params()
{
  declare_parameter<int>("api_mode", 1);
  api_mode_ = get_parameter("api_mode").as_int();
  declare_parameter<int>("timer_period_ms", 200);

  declare_parameter(
    "plugin_names",
    std::vector<std::string>{});
}

void Tier4ApiHostNode::initialize_plugins()
{
  if (initialized_plugins_) {
    return;
  }

  const auto plugin_names = get_parameter("plugin_names").as_string_array();
  for (const auto & plugin_name : plugin_names) {
    load_plugin(plugin_name);
  }
  initialized_plugins_ = true;
}

bool Tier4ApiHostNode::is_plugin_enabled_for_api_mode(const std::string & plugin_name) const
{
  if (api_mode_ >= 2 && is_api_mode_lt_2_only_plugin(plugin_name)) {
    return false;
  }
  if (api_mode_ >= 1 && is_api_mode_lt_1_only_plugin(plugin_name)) {
    return false;
  }
  if (api_mode_ < 2 && is_api_mode_gte_2_only_plugin(plugin_name)) {
    return false;
  }
  return true;
}

void Tier4ApiHostNode::load_plugin(const std::string & plugin_name)
{
  if (!is_plugin_enabled_for_api_mode(plugin_name)) {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Skipping plugin '" << plugin_name << "' (disabled for api_mode=" << api_mode_ << ")");
    return;
  }

  try {
    auto plugin = plugin_loader_->createSharedInstance(plugin_name);
    plugin->initialize(plugin_name, this, api_data_);
    plugins_.push_back(plugin);
    RCLCPP_INFO_STREAM(get_logger(), "Loaded plugin: " << plugin_name);
  } catch (const pluginlib::CreateClassException & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Failed to load plugin '" << plugin_name << "': " << e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Unexpected error loading plugin '" << plugin_name << "': " << e.what());
  }
}

rcl_interfaces::msg::SetParametersResult Tier4ApiHostNode::on_parameter(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : params) {
    if (param.get_name() == "api_mode") {
      api_mode_ = param.as_int();
      RCLCPP_INFO_STREAM(get_logger(), "api_mode updated to: " << api_mode_);
    }
  }

  for (auto & plugin : plugins_) {
    auto plugin_result = plugin->on_parameter(params);
    if (!plugin_result.successful) {
      result.successful = false;
      result.reason = plugin_result.reason;
      return result;
    }
  }

  return result;
}

namespace
{
template <typename MessageT, typename FieldT>
auto make_api_data_callback(
  tier4_autoware_api_host::plugin::ApiData * api_data, FieldT tier4_autoware_api_host::plugin::ApiData::* field)
{
  return [api_data, field](typename MessageT::ConstSharedPtr msg) { api_data->*field = msg; };
}
}  // namespace

void Tier4ApiHostNode::create_subscriptions()
{
  using std::placeholders::_1;

  // --- AWAPI shared subscriptions ---
  create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", 1,
    [this](autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg) {
      api_data_->steer = msg;
    });
  create_subscription<autoware_control_msgs::msg::Control>(
    "/control/command/control_cmd", 1,
    [this](autoware_control_msgs::msg::Control::ConstSharedPtr msg) {
      api_data_->vehicle_cmd = msg;
    });
  create_subscription<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
    "/vehicle/status/turn_indicators_status", 1,
    make_api_data_callback<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
      api_data_.get(), &plugin::ApiData::turn_indicators));
  create_subscription<autoware_vehicle_msgs::msg::HazardLightsReport>(
    "/vehicle/status/hazard_lights_status", 1,
    make_api_data_callback<autoware_vehicle_msgs::msg::HazardLightsReport>(
      api_data_.get(), &plugin::ApiData::hazard_lights));
  create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", 1,
    make_api_data_callback<nav_msgs::msg::Odometry>(api_data_.get(), &plugin::ApiData::odometry));
  create_subscription<autoware_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", 1,
    make_api_data_callback<autoware_vehicle_msgs::msg::GearReport>(
      api_data_.get(), &plugin::ApiData::gear));
  create_subscription<tier4_vehicle_msgs::msg::BatteryStatus>(
    "/vehicle/status/battery_charge", 1,
    make_api_data_callback<tier4_vehicle_msgs::msg::BatteryStatus>(
      api_data_.get(), &plugin::ApiData::battery));
  create_subscription<sensor_msgs::msg::NavSatFix>(
    "/sensing/gnss/ublox/nav_sat_fix", 1,
    make_api_data_callback<sensor_msgs::msg::NavSatFix>(api_data_.get(), &plugin::ApiData::nav_sat));
  create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics_agg", 1,
    make_api_data_callback<diagnostic_msgs::msg::DiagnosticArray>(
      api_data_.get(), &plugin::ApiData::diagnostics_agg));
  create_subscription<autoware_adapi_v1_msgs::msg::MrmState>(
    "/system/fail_safe/mrm_state", 1,
    make_api_data_callback<autoware_adapi_v1_msgs::msg::MrmState>(
      api_data_.get(), &plugin::ApiData::mrm_state));
  create_subscription<autoware_system_msgs::msg::HazardStatusStamped>(
    "/system/emergency/hazard_status", 1,
    make_api_data_callback<autoware_system_msgs::msg::HazardStatusStamped>(
      api_data_.get(), &plugin::ApiData::hazard_status));
  create_subscription<autoware_internal_planning_msgs::msg::VelocityLimit>(
    "/planning/scenario_planning/current_max_velocity", rclcpp::QoS{1}.transient_local(),
    make_api_data_callback<autoware_internal_planning_msgs::msg::VelocityLimit>(
      api_data_.get(), &plugin::ApiData::current_max_velocity));
  create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "/planning/trajectory", 1,
    make_api_data_callback<autoware_planning_msgs::msg::Trajectory>(
      api_data_.get(), &plugin::ApiData::autoware_trajectory));
  create_subscription<autoware_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", 1,
    make_api_data_callback<autoware_vehicle_msgs::msg::ControlModeReport>(
      api_data_.get(), &plugin::ApiData::control_mode));
  create_subscription<tier4_control_msgs::msg::GateMode>(
    "/control/current_gate_mode", rclcpp::QoS{1}.transient_local(),
    make_api_data_callback<tier4_control_msgs::msg::GateMode>(
      api_data_.get(), &plugin::ApiData::gate_mode));

  // --- External API shared subscriptions ---
  create_subscription<autoware_vehicle_msgs::msg::Engage>(
    "/api/autoware/get/engage", 1,
    make_api_data_callback<autoware_vehicle_msgs::msg::Engage>(
      api_data_.get(), &plugin::ApiData::engage_status));
  create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS(1).transient_local(),
    make_api_data_callback<autoware_adapi_v1_msgs::msg::OperationModeState>(
      api_data_.get(), &plugin::ApiData::operation_mode_state));
  create_subscription<autoware_system_msgs::msg::AutowareState>(
    "/autoware/state", 1,
    make_api_data_callback<autoware_system_msgs::msg::AutowareState>(
      api_data_.get(), &plugin::ApiData::autoware_state));
  create_subscription<tier4_external_api_msgs::msg::Emergency>(
    "/api/autoware/get/emergency", 1,
    make_api_data_callback<tier4_external_api_msgs::msg::Emergency>(
      api_data_.get(), &plugin::ApiData::emergency));
  create_subscription<tier4_external_api_msgs::msg::Operator>(
    "/api/autoware/get/operator", 1,
    make_api_data_callback<tier4_external_api_msgs::msg::Operator>(
      api_data_.get(), &plugin::ApiData::external_operator));
  create_subscription<tier4_external_api_msgs::msg::Observer>(
    "/api/autoware/get/observer", 1,
    make_api_data_callback<tier4_external_api_msgs::msg::Observer>(
      api_data_.get(), &plugin::ApiData::external_observer));
  create_subscription<tier4_external_api_msgs::msg::RosbagLoggingMode>(
    "/api/autoware/get/rosbag_logging_mode", 1,
    make_api_data_callback<tier4_external_api_msgs::msg::RosbagLoggingMode>(
      api_data_.get(), &plugin::ApiData::rosbag_logging_mode));
  create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", 1,
    make_api_data_callback<autoware_vehicle_msgs::msg::VelocityReport>(
      api_data_.get(), &plugin::ApiData::velocity));
  create_subscription<autoware_adapi_v1_msgs::msg::Route>(
    "/api/routing/route", 1,
    make_api_data_callback<autoware_adapi_v1_msgs::msg::Route>(
      api_data_.get(), &plugin::ApiData::adapi_route));
  create_subscription<tier4_api_msgs::msg::DoorStatus>(
    "/vehicle/status/door_status", 1,
    make_api_data_callback<tier4_api_msgs::msg::DoorStatus>(
      api_data_.get(), &plugin::ApiData::door_status));

  // --- External API — SystemMonitor ---
  create_subscription<tier4_external_api_msgs::msg::CpuTemperature>(
    "/system/system_monitor/cpu_monitor/cpu_temperature", 1,
    make_api_data_callback<tier4_external_api_msgs::msg::CpuTemperature>(
      api_data_.get(), &plugin::ApiData::cpu_temperature));
  create_subscription<tier4_external_api_msgs::msg::MemoryStatus>(
    "/system/system_monitor/mem_monitor/memory_status", 1,
    make_api_data_callback<tier4_external_api_msgs::msg::MemoryStatus>(
      api_data_.get(), &plugin::ApiData::memory_status));
  create_subscription<tier4_external_api_msgs::msg::GpuStatus>(
    "/system/system_monitor/gpu_monitor/gpu_status", 1,
    make_api_data_callback<tier4_external_api_msgs::msg::GpuStatus>(
      api_data_.get(), &plugin::ApiData::gpu_status));
  create_subscription<tier4_external_api_msgs::msg::NetworkStatus>(
    "/system/system_monitor/net_monitor/network_status", 1,
    make_api_data_callback<tier4_external_api_msgs::msg::NetworkStatus>(
      api_data_.get(), &plugin::ApiData::network_status));
  create_subscription<tier4_external_api_msgs::msg::HddStatus>(
    "/system/system_monitor/hdd_monitor/hdd_status", 1,
    make_api_data_callback<tier4_external_api_msgs::msg::HddStatus>(
      api_data_.get(), &plugin::ApiData::hdd_status));

  // --- External API — cpu_usage ---
  create_subscription<tier4_external_api_msgs::msg::CpuUsage>(
    "/system/system_monitor/cpu_monitor/cpu_usage", 1,
    make_api_data_callback<tier4_external_api_msgs::msg::CpuUsage>(
      api_data_.get(), &plugin::ApiData::cpu_usage));

  // --- External API — calibration_status ---
  create_subscription<tier4_external_api_msgs::msg::CalibrationStatus>(
    "/accel_brake_map_calibrator/output/calibration_status", 1,
    make_api_data_callback<tier4_external_api_msgs::msg::CalibrationStatus>(
      api_data_.get(), &plugin::ApiData::accel_brake_map_status));

  // --- External API — localization_score ---
  create_subscription<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "/localization/pose_estimator/transform_probability", 1,
    make_api_data_callback<autoware_internal_debug_msgs::msg::Float32Stamped>(
      api_data_.get(), &plugin::ApiData::transform_probability));
  create_subscription<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "/localization/pose_estimator/nearest_voxel_transformation_likelihood", 1,
    make_api_data_callback<autoware_internal_debug_msgs::msg::Float32Stamped>(
      api_data_.get(), &plugin::ApiData::nearest_voxel_transformation_likelihood));

  // --- Internal API subscriptions ---
  create_subscription<tier4_control_msgs::msg::ExternalCommandSelectorMode>(
    "/control/external_cmd_selector/current_selector_mode", 1,
    make_api_data_callback<tier4_control_msgs::msg::ExternalCommandSelectorMode>(
      api_data_.get(), &plugin::ApiData::external_select));
  create_subscription<autoware_perception_msgs::msg::TrackedObjects>(
    "/perception/object_recognition/tracking/objects", 1,
    make_api_data_callback<autoware_perception_msgs::msg::TrackedObjects>(
      api_data_.get(), &plugin::ApiData::tracked_objects));

  // --- Tier4 autoware state (internal) ---
  create_subscription<tier4_system_msgs::msg::AutowareState>(
    "/api/iv_msgs/autoware/state", 1,
    make_api_data_callback<tier4_system_msgs::msg::AutowareState>(
      api_data_.get(), &plugin::ApiData::tier4_autoware_state));

  // --- Extension subscriptions ---
  create_subscription<autoware_perception_msgs::msg::TrafficLightGroup>(
    "/planning/scenario_planning/lane_driving/behavior_planning/debug/traffic_signal", 1,
    make_api_data_callback<autoware_perception_msgs::msg::TrafficLightGroup>(
      api_data_.get(), &plugin::ApiData::traffic_light_group));
  create_subscription<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "/tier4_api/utils/path_distance_calculator/distance", 1,
    [this](const autoware_internal_debug_msgs::msg::Float64Stamped::ConstSharedPtr msg) {
      api_data_->route_distance = msg;
    });
  create_subscription<autoware_internal_planning_msgs::msg::PlanningFactorArray>(
    "/planning/planning_factors/traffic_light", 1,
    [this](const autoware_internal_planning_msgs::msg::PlanningFactorArray::ConstSharedPtr msg) {
      api_data_->planning_factors = msg;
    });
}

void Tier4ApiHostNode::create_relays()
{
  // Relay: /api/autoware/set/traffic_signals -> /external/traffic_light_recognition/traffic_signals
  auto sub_traffic_signals = create_subscription<autoware_perception_msgs::msg::TrafficSignalArray>(
    "/api/autoware/set/traffic_signals", 1,
    [this](autoware_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr msg) {
      static auto pub = create_publisher<autoware_perception_msgs::msg::TrafficSignalArray>(
        "/external/traffic_light_recognition/traffic_signals", 1);
      pub->publish(*msg);
    });

  // Relay: /api/autoware/set/intersection_states -> /planning/.../external_intersection_states
  auto sub_intersection = create_subscription<tier4_api_msgs::msg::IntersectionStatus>(
    "/api/autoware/set/intersection_states", 1,
    [this](tier4_api_msgs::msg::IntersectionStatus::ConstSharedPtr msg) {
      static auto pub = create_publisher<tier4_api_msgs::msg::IntersectionStatus>(
        "/planning/scenario_planning/lane_driving/behavior_planning/external_intersection_states", 1);
      pub->publish(*msg);
    });

  // Relay: /api/autoware/set/crosswalk_states -> /planning/.../external_crosswalk_states
  auto sub_crosswalk = create_subscription<tier4_api_msgs::msg::CrosswalkStatus>(
    "/api/autoware/set/crosswalk_states", 1,
    [this](tier4_api_msgs::msg::CrosswalkStatus::ConstSharedPtr msg) {
      static auto pub = create_publisher<tier4_api_msgs::msg::CrosswalkStatus>(
        "/planning/scenario_planning/lane_driving/behavior_planning/external_crosswalk_states", 1);
      pub->publish(*msg);
    });

  // Relay: route distance deprecated -> autoware_api namespace
  auto sub_route_distance = create_subscription<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "/tier4_api/utils/path_distance_calculator/distance", 1,
    [this](autoware_internal_debug_msgs::msg::Float64Stamped::ConstSharedPtr msg) {
      static auto pub = create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
        "/autoware_api/utils/path_distance_calculator/distance", 1);
      pub->publish(*msg);
    });

  // Relay: planning infrastructure commands -> external API get topic
  auto sub_vtl_cmds = create_subscription<tier4_v2x_msgs::msg::InfrastructureCommandArray>(
    "/planning/scenario_planning/status/infrastructure_commands", 1,
    [this](tier4_v2x_msgs::msg::InfrastructureCommandArray::ConstSharedPtr msg) {
      static auto pub = create_publisher<tier4_v2x_msgs::msg::InfrastructureCommandArray>(
        "/api/external/get/virtual_traffic_light/commands", 1);
      pub->publish(*msg);
    });

  // Relay: external API set VTL states -> AWAPI tmp topic
  auto sub_vtl_status = create_subscription<tier4_v2x_msgs::msg::VirtualTrafficLightStateArray>(
    "/api/external/set/virtual_traffic_light/states", 1,
    [this](tier4_v2x_msgs::msg::VirtualTrafficLightStateArray::ConstSharedPtr msg) {
      static auto pub = create_publisher<tier4_v2x_msgs::msg::VirtualTrafficLightStateArray>(
        "/awapi/tmp/virtual_traffic_light_states", 1);
      pub->publish(*msg);
    });
}

void Tier4ApiHostNode::on_timer()
{
  for (auto & plugin : plugins_) {
    if (is_plugin_enabled_for_api_mode(plugin->get_name())) {
      plugin->on_timer();
    }
  }
}

}  // namespace tier4_autoware_api_host

RCLCPP_COMPONENTS_REGISTER_NODE(tier4_autoware_api_host::Tier4ApiHostNode)
