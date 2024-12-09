# romea_ros2_robot_to_robot_localisation_core #

This package provides robot to robot localisation node able to estimate the leader robot pose by a Kalman or particle filter using observations provided by three localisation plugins: odo ([gitlab](https://gitlab.irstea.fr/romea_ros2/algorithms/localisation/romea_localisation_odo_plugin), [github](https://github.com/Romea/romea-ros2-localisation-odo-plugin)), imu ([gitlab](https://gitlab.irstea.fr/romea_ros2/algorithms/localisation/romea_localisation_imu_plugin), [github](https://github.com/Romea/romea-ros2-localisation-imu-plugin)) and  robot to robot rtls ([gitlab](https://gitlab.irstea.fr/romea_ros2/algorithms/localisation/romea_robot_to_robot_localisation_rtls_plugin), [github](https://github.com/Romea/romea-ros2-robot-to-robot-localisation-rtls-plugin)) plugins. This plugins are used to convert data coming from sensors to observations that can be used by both filters. 

# ROS2 core node description #

#### 1) Subscribed Topics ####

- **localisation/twist** (romea_localisation_msgs::msg::ObservationTwist2DStamped)

    This topic is provided by odo localisation plugin node and contains robot twist displacement observation deduced from odometry data coming from controller 

- **localisation/angular_speed** (romea_localisation_msgs::msg::ObservationAngularSpeedStamped)

    This topic is provided by imu localisation plugin node and contains robot angular speed observation deduced from data coming from IMU sensor

- **localisation/leader_twist** (romea_localisation_msgs::msg::ObservationTwist2DStamped)

    This topic is provided by rtls localisation plugin node and contains leader robot twist displacement observation deduced from leader robot odometry data broadcasted during ranging process

- **localisation/pose** (romea_localisation_msgs::msg::ObservationPose2DStamped)

    This topic is provided by rtls localisation plugin node and contains rough estimation of the pose of the leader robot computed by trilateration algorithm base on ranging data between rtls transceivers embedded on the follower and leader robots  

- **localisation/range** (romea_localisation_msgs::msg::ObservationRangeStamped)

    This topic is provided by rtls localisation plugin node and contains range observation deduced from ranging data between rtls transceivers embedded on the follower and leader robots   

#### 2) Published Topics ####

- **leader_pose_filtered** (romea_common_msgs::msg::Pose2DStamped)

  Pose of leader robot in the follower robot reference frame

#### 3) Parameters ####

  **~filter.state_pool_size** (int , default: 1000)

Size of the pool of filter states

  **~predictor.maximal_dead_recknoning_elapsed_time** (double, default: 1.0)

Maximal elapsed time in dead reckoning mode before to stop localisation filter

  **~predictor.maximal_dead_recknoning_travelled_distance** (double, default: 1.0)

Maximal travelled distance in dead reckoning mode before to stop localisation filter

  **~predictor.maximal_position_circular_error_probability** (double, default)

Maximal circular error in dead reckoning mode before to stop localisation filter

 **~twist_updater.minimal_rate** (int, default: 10)

Minimal rate for twist_updater input data (provided by odo plugin), if this rate is equal to 0 twist_updater is not started 

 **~linear_speed_updater.minimal_rate** (int, default: 0)

Minimal rate for linear_speed_updater input data (provided by odo plugin), if this rate is equal to 0 linear_speed_updater is not started 

  **~linear_speeds_updater.minimal_rate** (int, default: 0)

Minimal rate for linear_speeds_updater input data (provided by odo plugin), if this rate is equal to 0 linear_speeds_updater is not started 

  **~angular_updater.minimal_rate** (int, default: 0)

Minimal rate for angular_updater input data (provided by imu plugin), if this rate is equal to 0 angular_updater is not started 

  **~pose_updater.mahalanobis_distance_rejection_threshold** (double, default: 5.0)

Mahalanobis distance taking into account by pose updater to reject outliers 

  **~pose_updater.minimal_rate** (int, default: 1)

Minimal rate for pose_updater input data (provided by rtls plugin), if this rate is equal to 0 pose_updater is not started 

  **~pose_updater.trigger** (string, default: "once")

Update trigger mode when pose data is received. If "once" mode is selected the pose updater will be triggered only one time otherwise the pose updater will be triggerred each time data is received.

  **~range_updater.mahalanobis_distance_rejection_threshold** (double, default: 5.0)

Mahalanobis distance taking into account by range updater to reject outliers 

  **~range_updater.minimal_rate** (int, default: 10)

Minimal rate for position_updater input data (provided by rtls plugin); , if this rate is equal to 0 range_updater is not started  

  **~range_updater.trigger** (string, default: "always")

Update trigger mode when position data is received. If "once" mode is selected the range updater will be triggered only one time otherwise the range updater will be triggerred each time data is received.

  **~base_footprint_frame_id** (string, default: base_footprint):

Name of robot base footprint

  **~publish_rate** (int, default: 10)

Rate at which localisation results are published

  **~log_directory** (string, default: result of rclcpp::get_logging_directory()):

Directory where localisation logs are stored

  ~**debug** (bool, default: false)

Enable debug logs

## **Usage**

  See romea_ros2_localisation_bringup project 

## **Contributing**

If you'd like to contribute to this project, here are some guidelines:

1. Fork the repository.
2. Create a new branch for your changes.
3. Make your changes.
4. Write tests to cover your changes.
5. Run the tests to ensure they pass.
6. Commit your changes.
7. Push your changes to yo

## License

This project is released under the Apache License 2.0. See the LICENSE file for details.

### Authors

 romea_ros2_robot_to_robot_localisation_core project was developed by **Jean Laneurit** in the context of ADAP2E ANR project.

### Contact

If you have any questions or comments about romea_ros2_robot_to_robot_localisation_core project, please contact **[Jean Laneurit](mailto:jean.laneurit@inrae.fr)** 