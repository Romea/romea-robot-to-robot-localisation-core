// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <memory>

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_common_utils/publishers/stamped_data_publisher.hpp"
#include "romea_common_utils/conversions/pose_and_twist2d_conversions.hpp"
#include "romea_localisation_utils/conversions/localisation_status_conversions.hpp"
#include "romea_robot_to_robot_localisation/robot_to_robot_localisation.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
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
template<FilterType FilterType_>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
R2RLocalisation<FilterType_>::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
void R2RLocalisation<FilterType_>::make_filter_()
{
  filter_ = std::make_unique<R2RLocalisationFilter<FilterType_>>(node_);
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
void R2RLocalisation<FilterType_>::make_status_publisher_()
{
  using DataType = LocalisationFSMState;
  using MsgType = romea_localisation_msgs::msg::LocalisationStatus;
  status_publisher_ = make_data_publisher<DataType, MsgType>(
    node_, "status", reliable(1), true);
}


//-----------------------------------------------------------------------------
template<FilterType FilterType_>
void R2RLocalisation<FilterType_>::make_tf_publisher_()
{
  tf_publisher_ = make_transform_publisher<Pose2D>(
    node_,
    get_map_frame_id(node_),
    get_base_footprint_frame_id(node_),
    true);
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
void R2RLocalisation<FilterType_>::make_leader_pose_and_twist_publisher_()
{
  leader_pose_and_twist_publisher_ = make_stamped_data_publisher<PoseAndTwist2D,
      romea_common_msgs::msg::PoseAndTwist2DStamped>(
    node_,
    "filtered_leader_pose",
    get_base_footprint_frame_id(node_),
    reliable(1),
    true);
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
void R2RLocalisation<FilterType_>::make_diagnostic_publisher_()
{
  diagnostic_publisher_ = make_diagnostic_publisher<DiagnosticReport>(
    node_, "r2r_localisation", 1.0);
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
void R2RLocalisation<FilterType_>::make_timer_()
{
  Duration timer_period = durationFromSecond(1. / get_publish_rate(node_));
  auto callback = std::bind(&R2RLocalisation::timer_callback_, this);
  timer_ = node_->create_wall_timer(timer_period, callback);
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
void R2RLocalisation<FilterType_>::timer_callback_()
{
  auto stamp = node_->get_clock()->now();

  LocalisationFSMState fsm_state = filter_->get_fsm_state();
  if (fsm_state == LocalisationFSMState::RUNNING) {
    const auto & results = filter_->get_results(to_romea_duration(stamp));
    PoseAndTwist2D leader_pose_and_twist = results.toLeaderPoseAndBodyTwist2D();

    std::cout << "leader pose " <<
      leader_pose_and_twist.pose.position.x() << " " <<
      leader_pose_and_twist.pose.position.y() << " " <<
      leader_pose_and_twist.pose.yaw << " " <<
      leader_pose_and_twist.twist.linearSpeeds.x() << " " <<
      leader_pose_and_twist.twist.linearSpeeds.y() << " " <<
      leader_pose_and_twist.twist.angularSpeed << std::endl;

    leader_pose_and_twist_publisher_->publish(stamp, leader_pose_and_twist);
    tf_publisher_->publish(stamp, leader_pose_and_twist.pose);
    //     rviz_.display(results);
  }

  status_publisher_->publish(fsm_state);
  publish_diagnostics_(stamp);
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
void R2RLocalisation<FilterType_>::publish_diagnostics_(const rclcpp::Time & stamp)
{
  auto report = filter_->make_diagnostic_report(to_romea_duration(stamp));
  diagnostic_publisher_->publish(stamp, report);
}

template class R2RLocalisation<FilterType::KALMAN>;
template class R2RLocalisation<FilterType::PARTICLE>;

}  // namespace romea

//-----------------------------------------------------------------------------
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::R2RKalmanLocalisation)
RCLCPP_COMPONENTS_REGISTER_NODE(romea::R2RParticleLocalisation)
