#include <string>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <bitset>
#include <vector>
#include <boost/thread.hpp>
#include <time.h>
#include <fstream>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <stdio.h>
#include <fstream>
#include <cstdlib>
#include <algorithm>


#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/PointCloud2.h"
#include <laser_geometry/laser_geometry.h>//new add
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <visualization_msgs/Marker.h>

#include "DWA.h"

class Navigation{
    public:
      Navigation();
      ~Navigation();
      void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
      void amclposeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
      void goalposeCallback(const geometry_msgs::PoseStamped& msg);
      void timerCallback(const ros::TimerEvent& event);
      void draw(int id,float color_r,float color_g,float color_b,std::vector<Eigen::Vector3f> drawpoint);
      void drawLine(int id,float color_r,float color_g,float color_b,std::vector<Eigen::Vector3f> drawline);
    private:
      ros::NodeHandle node_;
      ros::Subscriber laserSubscriber_;
      ros::Subscriber amclposeSubscriber_;
      ros::Subscriber goalSubscriber_;
      ros::Publisher marke_Publisher_ ;
      ros::Publisher simulation_speed_Publisher_;
      ros::Timer control_timer;
      Eigen::Vector3f goal_pose;
      Eigen::Vector3f robot_pose;
      sensor_msgs::PointCloud laser_point_cloud_1_;
      tf::TransformListener tf_buffer_;
      std::vector<Eigen::Vector3f> dwa_obs_region;
    	std::vector<Eigen::Vector3f> dwa_obs_robot_region;
      std::vector<Eigen::Vector3f> dwa_predict_pose;
    	std::vector<Eigen::Vector3f> dwa_choose_line;
      laser_geometry::LaserProjection projector_1_;
      bool dwa_start;
      DWA use_dwa;
      float tracking_v, tracking_w;
};
