// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_ROBOT_TO_ROBOT_LOCALISATION_CORE__ROBOT_TO_ROBOT_LOCALISATION_HPP_
#define ROMEA_ROBOT_TO_ROBOT_LOCALISATION_CORE__ROBOT_TO_ROBOT_LOCALISATION_HPP_

// std
#include <memory>

// romea
#include "romea_common_utils/publishers/data_publisher.hpp"
#include "romea_common_utils/conversions/pose_and_twist3d_conversions.hpp"
#include "romea_common_utils/publishers/diagnostic_publisher.hpp"
#include "romea_common_utils/publishers/transform_publisher.hpp"
#include "romea_common_utils/publishers/odom_publisher.hpp"

// local
#include "romea_robot_to_robot_localisation_core/robot_to_robot_localisation_filter.hpp"
#include "romea_robot_to_robot_localisation_core/visibility_control.h"

namespace romea
{

template<FilterType FilterType_>
class R2RLocalisation
{
public:
  ROMEA_ROBOT_TO_ROBOT_LOCALISATION_CORE_PUBLIC
  explicit R2RLocalisation(const rclcpp::NodeOptions & options);

  ROMEA_ROBOT_TO_ROBOT_LOCALISATION_CORE_PUBLIC
  virtual ~R2RLocalisation() = default;

  ROMEA_ROBOT_TO_ROBOT_LOCALISATION_CORE_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

private:
  void make_filter_();

  void make_timer_();

  void make_status_publisher_();

  void make_tf_publisher_();

  void make_leader_pose_and_twist_publisher_();

  void make_diagnostic_publisher_();

  void publish_diagnostics_(const rclcpp::Time & stamp);

  void timer_callback_();

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
  std::unique_ptr<R2RLocalisationFilter<FilterType_>> filter_;

  std::shared_ptr<StampedPublisherBase<Pose2D>> tf_publisher_;
  std::shared_ptr<StampedPublisherBase<PoseAndTwist2D>> leader_pose_and_twist_publisher_;
  std::shared_ptr<StampedPublisherBase<DiagnosticReport>> diagnostic_publisher_;
  std::shared_ptr<PublisherBase<LocalisationFSMState>> status_publisher_;
};

using R2RKalmanLocalisation = R2RLocalisation<FilterType::KALMAN>;
using R2RParticleLocalisation = R2RLocalisation<FilterType::PARTICLE>;

}  // namespace romea

#endif  // ROMEA_ROBOT_TO_ROBOT_LOCALISATION_CORE__ROBOT_TO_ROBOT_LOCALISATION_HPP_
