#include "navigation.h"

Navigation::Navigation()
{
    laserSubscriber_ = node_.subscribe("scan", 1000, &Navigation::laserCallback, this);
    amclposeSubscriber_ = node_.subscribe("amcl_pose", 1000, &Navigation::amclposeCallback, this);
    goalSubscriber_ = node_.subscribe("/move_base_simple/goal", 1000, &Navigation::goalposeCallback, this);
    control_timer = node_.createTimer(ros::Duration(0.2), &Navigation::timerCallback, this);
}

Navigation::~Navigation()
{

}

void Navigation::laserCallback(const sensor_msgs::LaserScan& scan)
{

}

void Navigation::amclposeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

}

void Navigation::goalposeCallback(const geometry_msgs::PoseStamped& msg)
{

}

void Navigation::timerCallback(const ros::TimerEvent& event)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation");
    ros::Time::init();
    std::cout<<"navigation start"<<std::endl;
    Navigation *navigation_start;
    navigation_start = new Navigation();
    ros::spin();
    return 0;
}
