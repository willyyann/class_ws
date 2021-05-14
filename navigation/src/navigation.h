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

class Navigation{
    public:
      Navigation();
      ~Navigation();
      void laserCallback(const sensor_msgs::LaserScan& scan);
      void amclposeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
      void goalposeCallback(const geometry_msgs::PoseStamped& msg);
      void timerCallback(const ros::TimerEvent& event);
    private:
      ros::NodeHandle node_;
      ros::Subscriber laserSubscriber_;
      ros::Subscriber amclposeSubscriber_;
      ros::Subscriber goalSubscriber_;
      ros::Timer control_timer;
};
