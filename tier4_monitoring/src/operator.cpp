// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "operator.hpp"

#include "message.hpp"

#include <string>

namespace tier4_monitoring
{

Operator::Operator(rclcpp::Node & node, const std::string & ns)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  pub_status_ = node.create_publisher<MonitoringStatus>(ns + "/status", rclcpp::QoS(1));
  sub_heartbeat_ = node.create_subscription<MonitoringHeartbeat>(
    ns + "/heartbeat", rclcpp::QoS(1), std::bind(&Operator::on_heartbeat, this, _1));
  srv_change_ = node.create_service<ChangeMonitoringMode>(
    ns + "/change", std::bind(&Operator::on_change, this, _1, _2));

  stamp_ = std::nullopt;
  mode_ = OperatorMode::kUnknown;
}

void Operator::update(rclcpp::Time now)
{
  const auto is_timeout = [this, now]() {
    constexpr double timeout = 1.0;
    if (!stamp_) return true;
    return timeout < (now - stamp_.value()).seconds();
  };
  if (is_timeout()) {
    stamp_ = std::nullopt;
    mode_ = OperatorMode::kTimeout;
  }
}

void Operator::publish(rclcpp::Time now, bool operating)
{
  MonitoringStatus msg;
  msg.stamp = now;
  msg.operating = operating;
  msg.mode = to_msg(mode_);
  pub_status_->publish(msg);
}

OperatorMode Operator::mode() const
{
  return mode_;
}

void Operator::on_heartbeat(const MonitoringHeartbeat::SharedPtr msg)
{
  stamp_ = msg->stamp;
}

void Operator::on_change(
  const ChangeMonitoringMode::Request::SharedPtr req,
  const ChangeMonitoringMode::Response::SharedPtr res)
{
  const auto mode = from_msg(req->mode);
  if (mode == OperatorMode::kUnknown) {
    res->status.code = ResponseStatus::ERROR;
    res->status.message = "unknown mode";
    return;
  }
  mode_ = mode;
  res->status.code = ResponseStatus::SUCCESS;
}

OperatorGroup::OperatorGroup(const std::string & ns) : ns_(ns)
{
  operating = nullptr;
}

void OperatorGroup::create(rclcpp::Node & node, const std::string & name)
{
  operators_.push_back(std::make_unique<Operator>(node, ns_ + "/" + name));
}

void OperatorGroup::update(rclcpp::Time now)
{
  operating = nullptr;

  for (const auto & operator_ : operators_) {
    operator_->update(now);
  }

  for (const auto & operator_ : operators_) {
    if (operator_->mode() == OperatorMode::kOperating) {
      operating = operator_.get();
      break;
    }
  }
}

void OperatorGroup::publish(rclcpp::Time now)
{
  for (const auto & operator_ : operators_) {
    operator_->publish(now, operating == operator_.get());
  }
}

}  // namespace tier4_monitoring
