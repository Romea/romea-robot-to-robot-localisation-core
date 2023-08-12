#ifndef __R2RLocalisationDisplay_HPP__
#define __R2RLocalisationDisplay_HPP__

//ros
#include <visualization_msgs/Marker.h>


//romea
#include "robot_to_robot_localisation_filter.hpp"
#include <romea_common_utils/rviz_display.hpp>

namespace romea{

template <FilterType FilterType_>
class R2RLocalisationDisplay
{

public :

  R2RLocalisationDisplay();

  void configure(const std::string & robot_name);

  void display(const typename R2RLocalisationFilter<FilterType_>::Results &results);

  void enable();

  void disable();

private :

  void displayTrajectories_(const typename R2RLocalisationFilter<FilterType_>::Results & results);

private:

  bool enable_;
  rviz_visual_tools::RvizVisualTools rviz_util_;

};


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
template <FilterType FilterType_>
R2RLocalisationDisplay<FilterType_>::R2RLocalisationDisplay():
  enable_(false),
  rviz_util_("base_footprint","display")
{
}


//-----------------------------------------------------------------------------
template <FilterType FilterType_>
void R2RLocalisationDisplay<FilterType_>::configure(const std::string & robot_name)
{
  rviz_util_.setBaseFrame(robot_name+"/base_footprint");
  rviz_util_.loadMarkerPub();
  rviz_util_.deleteAllMarkers();
  rviz_util_.enableBatchPublishing();
}

//-----------------------------------------------------------------------------
template <FilterType FilterType_>
void R2RLocalisationDisplay<FilterType_>::enable()
{
  enable_=true;
}

//-----------------------------------------------------------------------------
template <FilterType FilterType_>
void R2RLocalisationDisplay<FilterType_>::disable()
{
  enable_=false;
}

//-----------------------------------------------------------------------------
template <FilterType FilterType_>
void R2RLocalisationDisplay<FilterType_>::displayTrajectories_(const typename R2RLocalisationFilter<FilterType_>::Results &results)
{

  const auto & leader_trajectory2d = results.addon.leaderTrajectory.get();
  const auto & follower_trajectory2d = results.addon.robotTrajectory.get();
#if ROS_VERSION_MINIMUM(1,12,0)
  static VectorOfEigenVector3d leader_trajectory3d(100,Eigen::Vector3d::Zero());
  static VectorOfEigenVector3d follower_trajectory3d(100,Eigen::Vector3d::Zero());
#else
  std::vector<Eigen::Vector3d> leader_trajectory3d(1000,Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> follower_trajectory3d(1000,Eigen::Vector3d::Zero());
#endif
  leader_trajectory3d.resize(leader_trajectory2d.size());
  follower_trajectory3d.resize(follower_trajectory2d.size());

  for(size_t n=0; n<leader_trajectory2d.size(); n++)
  {
    leader_trajectory3d[n]<< leader_trajectory2d[n] ,0 ;
    follower_trajectory3d[n]<< follower_trajectory2d[n] , 0;
  }

  rviz_util_.publishSpheres(leader_trajectory3d,rviz_visual_tools::BLUE,rviz_visual_tools::XXLARGE);
  rviz_util_.publishSpheres(follower_trajectory3d,rviz_visual_tools::WHITE,rviz_visual_tools::XXLARGE);

}

//-----------------------------------------------------------------------------
template<>
void R2RLocalisationDisplay<KALMAN>::display(const typename R2RLocalisationFilter<KALMAN>::Results &results)
{
  if(enable_)
  {
    rviz_util_.deleteAllMarkers();
    rviz_util_.enableBatchPublishing();
    displayTrajectories_(results);

    publish(rviz_util_,
            results.toLeaderPose2D(),
            rviz_visual_tools::GREEN);

    rviz_util_.trigger();
  }
}

//-----------------------------------------------------------------------------
template <>
void R2RLocalisationDisplay<PARTICLE>::display(const typename R2RLocalisationFilter<PARTICLE>::Results &results)
{
  if(enable_)
  {
    rviz_util_.deleteAllMarkers();
    rviz_util_.enableBatchPublishing();
    displayTrajectories_(results);

    const auto & particles = results.state.particles;
    static VectorOfEigenVector3d particles3d(particles.cols());
    for(int n=0; n<particles.cols();++n)
    {
      particles3d[n][0]=particles(0,n);
      particles3d[n][1]=particles(1,n);
    }

    rviz_util_.publishSpheres(particles3d,
                              rviz_visual_tools::GREEN,
                              rviz_visual_tools::XXLARGE);

    publish(rviz_util_,
            results.toLeaderPose2D(),
            rviz_visual_tools::GREEN);

    rviz_util_.trigger();
  }
}


}

#endif

