#ifndef TIER4_AUTOWARE_API_HOST__CONVERTER__FAIL_SAFE_STATE_HPP_
#define TIER4_AUTOWARE_API_HOST__CONVERTER__FAIL_SAFE_STATE_HPP_

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <tier4_external_api_msgs/msg/fail_safe_state.hpp>
#include <tier4_external_api_msgs/msg/fail_safe_state_stamped.hpp>

namespace tier4_autoware_api_host::converter
{

using ExternalFailSafeStateStamped = tier4_external_api_msgs::msg::FailSafeStateStamped;
using ExternalFailSafeState = tier4_external_api_msgs::msg::FailSafeState;
using InternalFailSafeState = autoware_adapi_v1_msgs::msg::MrmState;

inline ExternalFailSafeState to_external_state(const InternalFailSafeState & msg)
{
  using External = ExternalFailSafeState;
  using Internal = InternalFailSafeState;

  auto builder = tier4_external_api_msgs::build<External>();
  switch (msg.state) {
    case Internal::NORMAL:
      return builder.state(External::NORMAL);
    case Internal::MRM_OPERATING:
      return builder.state(External::MRM_OPERATING);
    case Internal::MRM_SUCCEEDED:
      return builder.state(External::MRM_SUCCEEDED);
    case Internal::MRM_FAILED:
      return builder.state(External::MRM_FAILED);
  }
  throw std::out_of_range("fail_safe_state=" + std::to_string(msg.state));
}

inline ExternalFailSafeStateStamped to_external(const InternalFailSafeState & msg)
{
  return tier4_external_api_msgs::build<ExternalFailSafeStateStamped>().stamp(msg.stamp).state(
    to_external_state(msg));
}

}  // namespace tier4_autoware_api_host::converter

#endif  // TIER4_AUTOWARE_API_HOST__CONVERTER__FAIL_SAFE_STATE_HPP_
