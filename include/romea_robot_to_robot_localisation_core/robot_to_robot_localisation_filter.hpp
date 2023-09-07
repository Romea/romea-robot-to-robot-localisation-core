#ifndef ROMEA_ROBOT_TO_ROBOT_LOCALISATION__ROBOT_TO_ROBOT_LOCALISATION_FILTER_HPP_
#define ROMEA_ROBOT_TO_ROBOT_LOCALISATION__ROBOT_TO_ROBOT_LOCALISATION_FILTER_HPP_

// std
#include <list>
#include <memory>
#include <string>
#include <utility>

// romea
#include "romea_core_localisation/robot_to_robot/R2RLocalisationTraits.hpp"
#include "romea_localisation_utils/filter/localisation_factory.hpp"
#include "romea_localisation_utils/filter/localisation_updater_interface.hpp"


namespace romea
{

template<FilterType FilterType_>
class R2RLocalisationFilter
{
public:
  using Filter = typename  R2RLocalisationTraits<FilterType_>::Filter;
  using Predictor = typename R2RLocalisationTraits<FilterType_>::Predictor;
  using UpdaterLeaderTwist = typename R2RLocalisationTraits<FilterType_>::UpdaterLeaderTwist;
  using UpdaterTwist = typename  R2RLocalisationTraits<FilterType_>::UpdaterTwist;
  using UpdaterRange = typename  R2RLocalisationTraits<FilterType_>::UpdaterRange;
  using UpdaterPose = typename R2RLocalisationTraits<FilterType_>::UpdaterPose;
  using Results = typename R2RLocalisationTraits<FilterType_>::Results;

  using UpdaterPlugin = LocalisationUpdaterInterfaceBase;
  using UpdaterPluginLeaderTwist = LocalisationUpdaterInterface<Filter, UpdaterLeaderTwist,
      romea_localisation_msgs::msg::ObservationTwist2DStamped>;
  using UpdaterPluginTwist = LocalisationUpdaterInterface<Filter, UpdaterTwist,
      romea_localisation_msgs::msg::ObservationTwist2DStamped>;
  using UpdaterPluginRange = LocalisationUpdaterInterface<Filter, UpdaterRange,
      romea_localisation_msgs::msg::ObservationRangeStamped>;
  using UpdaterPluginPose = LocalisationUpdaterInterface<Filter, UpdaterPose,
      romea_localisation_msgs::msg::ObservationPose2DStamped>;

public:
  R2RLocalisationFilter();

  explicit R2RLocalisationFilter(std::shared_ptr<rclcpp::Node> node);

  void reset();

public:
  LocalisationFSMState get_fsm_state();

  const Results & get_results(const Duration & duration);

  DiagnosticReport make_diagnostic_report(const Duration & duration);

private:
  void make_filter_(std::shared_ptr<rclcpp::Node> node);

  void make_results_(std::shared_ptr<rclcpp::Node> node);

  template<typename Interface>
  void add_proprioceptive_updater_interface_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & updater_name);

  template<typename interface>
  void add_exteroceptive_updater_interface_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & updater_name);

private:
  std::shared_ptr<Filter> filter_;
  std::unique_ptr<Results> results_;
  std::list<std::unique_ptr<UpdaterPlugin>> updater_interfaces_;
};


//-----------------------------------------------------------------------------
template<FilterType FilterType_>
R2RLocalisationFilter<FilterType_>::R2RLocalisationFilter(std::shared_ptr<rclcpp::Node> node)
: filter_(nullptr),
  results_(nullptr),
  updater_interfaces_()
{
  RCLCPP_INFO_STREAM(node->get_logger(), " init filter ");
  make_filter_(node);
  add_proprioceptive_updater_interface_<UpdaterPluginTwist>(node, "twist_updater");
  add_proprioceptive_updater_interface_<UpdaterPluginLeaderTwist>(node, "leader_twist_updater");
  add_exteroceptive_updater_interface_<UpdaterPluginPose>(node, "pose_updater");
  add_exteroceptive_updater_interface_<UpdaterPluginRange>(node, "range_updater");
  make_results_(node);
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
LocalisationFSMState R2RLocalisationFilter<FilterType_>::get_fsm_state()
{
  return filter_->getFSMState();
}


//-----------------------------------------------------------------------------
template<FilterType FilterType_>
void R2RLocalisationFilter<FilterType_>::reset()
{
  filter_->reset();
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
void R2RLocalisationFilter<FilterType_>::make_filter_(std::shared_ptr<rclcpp::Node> node)
{
  declare_predictor_parameters(node);
  declare_filter_parameters<FilterType_>(node);
  filter_ = make_filter<Filter, Predictor, FilterType_>(node);
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
void R2RLocalisationFilter<FilterType_>::make_results_(std::shared_ptr<rclcpp::Node> node)
{
  results_ = make_results<Results, FilterType_>(node);
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
template<typename Interface>
void R2RLocalisationFilter<FilterType_>::add_proprioceptive_updater_interface_(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name)
{
  declare_proprioceptive_updater_parameters(node, updater_name);

  if (!get_updater_topic_name(node, updater_name).empty()) {
    RCLCPP_INFO_STREAM(node->get_logger(), " init " + updater_name);

    using Updater = typename Interface::Updater;
    auto updater = make_proprioceptive_updater<Updater>(
      node,
      updater_name);

    auto plugin = make_updater_interface<Interface>(
      node,
      updater_name,
      filter_,
      std::move(updater));

    updater_interfaces_.push_back(std::move(plugin));
  }
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
template<typename Interface>
void R2RLocalisationFilter<FilterType_>::add_exteroceptive_updater_interface_(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name)
{
  declare_exteroceptive_updater_parameters(node, updater_name);

  if (!get_updater_topic_name(node, updater_name).empty()) {
    RCLCPP_INFO_STREAM(node->get_logger(), " init " + updater_name);

    using Updater = typename Interface::Updater;
    auto updater = make_exteroceptive_updater<Updater, FilterType_>(
      node,
      updater_name);
    auto plugin = make_updater_interface<Interface>(
      node,
      updater_name,
      filter_,
      std::move(updater));

    updater_interfaces_.push_back(std::move(plugin));
  }
}


// //-----------------------------------------------------------------------------
// template<FilterType FilterType_>
// template<typename Plugin>
// void R2RLocalisationFilter<FilterType_>::addInputPlugin_(
//   ros::NodeHandle & nh,
//   ros::NodeHandle & private_nh,
//   const std::string & updater_name)
// {
//   if (private_nh.hasParam(updater_name)) {
//     ROS_INFO_STREAM(" init " + updater_name);
//     using Updater = typename Plugin::Updater;
//     auto updater = makeProprioceptiveUpdater<Updater>(
//       private_nh,
//       updater_name);
//     auto plugin = makeUpdaterPlugin<Plugin>(
//       nh,
//       private_nh,
//       updater_name,
//       filter_,
//       std::move(updater));

//     updater_plugins_.push_back(std::move(plugin));
//   }
// }


// //-----------------------------------------------------------------------------
// template<FilterType FilterType_>
// template<typename Plugin>
// void R2RLocalisationFilter<FilterType_>::addUpdaterPlugin_(
//   ros::NodeHandle & nh,
//   ros::NodeHandle & private_nh,
//   const std::string & updater_name)
// {
//   if (private_nh.hasParam(updater_name)) {
//     ROS_INFO_STREAM(" init " + updater_name);
//     using Updater = typename Plugin::Updater;
//     auto updater = makeExteroceptiveUpdater<Updater, FilterType_>(
//       private_nh,
//       updater_name);

//     auto plugin = makeUpdaterPlugin<Plugin>(
//       nh,
//       private_nh,
//       updater_name,
//       filter_,
//       std::move(updater));

//     updater_plugins_.push_back(std::move(plugin));
//   }
// }


//-----------------------------------------------------------------------------
template<FilterType FilterType_>
const typename R2RLocalisationFilter<FilterType_>::Results &
R2RLocalisationFilter<FilterType_>::get_results(const Duration & duration)
{
  results_->setDuration(duration);
  filter_->getCurrentState(duration, results_.get());
  return *results_;
}

//-----------------------------------------------------------------------------
template<FilterType FilterType_>
DiagnosticReport R2RLocalisationFilter<FilterType_>::make_diagnostic_report(const Duration & stamp)
{
  DiagnosticReport report;
  for (auto & interface_ptr : updater_interfaces_) {
    interface_ptr->heartbeat_callback(stamp);
    report += interface_ptr->get_report();
  }

  return report;
}

}  // namespace romea

#endif  // ROMEA_ROBOT_TO_ROBOT_LOCALISATION__ROBOT_TO_ROBOT_LOCALISATION_FILTER_HPP_
