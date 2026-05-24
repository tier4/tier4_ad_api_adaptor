#ifndef TIER4_AUTOWARE_API_HOST__DEPRECATED_API_UTILS_HPP_
#define TIER4_AUTOWARE_API_HOST__DEPRECATED_API_UTILS_HPP_

#include <tier4_external_api_msgs/msg/response_status.hpp>

#include <chrono>
#include <string>
#include <utility>

namespace tier4_autoware_api_host::deprecated_api
{

using ResponseStatus = tier4_external_api_msgs::msg::ResponseStatus;

inline bool is_error(const ResponseStatus & status)
{
  return status.code == ResponseStatus::ERROR;
}

inline ResponseStatus response_success(const std::string & message = "")
{
  return tier4_external_api_msgs::build<ResponseStatus>()
    .code(ResponseStatus::SUCCESS)
    .message(message);
}

inline ResponseStatus response_error(const std::string & message = "")
{
  return tier4_external_api_msgs::build<ResponseStatus>()
    .code(ResponseStatus::ERROR)
    .message(message);
}

inline ResponseStatus response_ignored(const std::string & message = "")
{
  return tier4_external_api_msgs::build<ResponseStatus>()
    .code(ResponseStatus::IGNORED)
    .message(message);
}

template <typename ServiceT>
std::pair<ResponseStatus, typename ServiceT::Response::SharedPtr> sync_call(
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const typename ServiceT::Request::SharedPtr & request,
  const std::chrono::nanoseconds & timeout = std::chrono::seconds(2))
{
  if (!client->service_is_ready()) {
    return {response_error("Internal service is not available."), nullptr};
  }
  auto future = client->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    return {response_error("Internal service has timed out."), nullptr};
  }
  return {response_success(), future.get()};
}

}  // namespace tier4_autoware_api_host::deprecated_api

#endif  // TIER4_AUTOWARE_API_HOST__DEPRECATED_API_UTILS_HPP_
