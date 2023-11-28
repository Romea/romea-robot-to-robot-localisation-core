// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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

// std
#include <memory>

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_common_utils/conversions/pose_and_twist2d_conversions.hpp"
#include "romea_localisation_utils/conversions/localisation_status_conversions.hpp"
#include "romea_robot_to_robot_localisation_core/robot_to_robot_localisation.hpp"
#include "romea_common_utils/publishers/stamped_data_publisher.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
R2RLocalisation<FilterType_>::R2RLocalisation(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("robot_to_robot_localisation", options)),
  filter_(nullptr),
  tf_publisher_(nullptr),
  leader_pose_and_twist_publisher_(nullptr),
  diagnostic_publisher_(nullptr)
{
  declare_debug(node_);
  declare_log_directory(node_);
  declare_base_footprint_frame_id(node_);
  declare_publish_rate(node_);

  make_filter_();
  make_tf_publisher_();
  make_leader_pose_and_twist_publisher_();
  make_diagnostic_publisher_();
  make_status_publisher_();
  make_timer_();
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
R2RLocalisation<FilterType_>::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2RLocalisation<FilterType_>::make_filter_()
{
  filter_ = std::make_unique<R2RLocalisationFilter<FilterType_>>(node_);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2RLocalisation<FilterType_>::make_status_publisher_()
{
  using DataType = core::LocalisationFSMState;
  using MsgType = romea_localisation_msgs::msg::LocalisationStatus;
  status_publisher_ = make_data_publisher<DataType, MsgType>(
    node_, "status", reliable(1), true);
}


//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2RLocalisation<FilterType_>::make_tf_publisher_()
{
  // tf_publisher_ = make_transform_publisher<Pose2D>(
  //   node_,
  //   get_map_frame_id(node_),
  //   get_base_footprint_frame_id(node_),
  //   true);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2RLocalisation<FilterType_>::make_leader_pose_and_twist_publisher_()
{
  leader_pose_and_twist_publisher_ = make_stamped_data_publisher<core::PoseAndTwist2D,
      romea_common_msgs::msg::PoseAndTwist2DStamped>(
    node_,
    "filtered_leader_pose",
    get_base_footprint_frame_id(node_),
    reliable(1),
    true);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2RLocalisation<FilterType_>::make_diagnostic_publisher_()
{
  diagnostic_publisher_ = make_diagnostic_publisher<core::DiagnosticReport>(
    node_, node_->get_fully_qualified_name(), 1.0);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2RLocalisation<FilterType_>::make_timer_()
{
  auto timer_period = core::durationFromSecond(1. / get_publish_rate(node_));
  auto callback = std::bind(&R2RLocalisation::timer_callback_, this);
  timer_ = node_->create_wall_timer(timer_period, callback);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2RLocalisation<FilterType_>::timer_callback_()
{
  auto stamp = node_->get_clock()->now();

  auto fsm_state = filter_->get_fsm_state();
  if (fsm_state == core::LocalisationFSMState::RUNNING) {
    const auto & results = filter_->get_results(to_romea_duration(stamp));
    auto leader_pose_and_twist = results.toLeaderPoseAndBodyTwist2D();

    std::cout << "leader pose " <<
      leader_pose_and_twist.pose.position.x() << " " <<
      leader_pose_and_twist.pose.position.y() << " " <<
      leader_pose_and_twist.pose.yaw << " " <<
      leader_pose_and_twist.twist.linearSpeeds.x() << " " <<
      leader_pose_and_twist.twist.linearSpeeds.y() << " " <<
      leader_pose_and_twist.twist.angularSpeed << std::endl;

    leader_pose_and_twist_publisher_->publish(stamp, leader_pose_and_twist);
    // tf_publisher_->publish(stamp, leader_pose_and_twist.pose);
    //     rviz_.display(results);
  }

  status_publisher_->publish(fsm_state);
  publish_diagnostics_(stamp);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2RLocalisation<FilterType_>::publish_diagnostics_(const rclcpp::Time & stamp)
{
  auto report = filter_->make_diagnostic_report(to_romea_duration(stamp));
  diagnostic_publisher_->publish(stamp, report);
}

template class R2RLocalisation<core::FilterType::KALMAN>;
template class R2RLocalisation<core::FilterType::PARTICLE>;

}  // namespace ros2
}  // namespace romea

//-----------------------------------------------------------------------------
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::R2RKalmanLocalisation)
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::R2RParticleLocalisation)
