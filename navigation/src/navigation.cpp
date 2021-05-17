#include "navigation.h"

Navigation::Navigation()
{
    laserSubscriber_ = node_.subscribe("scan", 1000, &Navigation::laserCallback, this);
    amclposeSubscriber_ = node_.subscribe("amcl_pose", 1000, &Navigation::amclposeCallback, this);
    goalSubscriber_ = node_.subscribe("/move_base_simple/goal", 1000, &Navigation::goalposeCallback, this);
    marke_Publisher_ = node_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    simulation_speed_Publisher_ = node_.advertise<geometry_msgs::Twist>("cmd_vel",10);//new add
    control_timer = node_.createTimer(ros::Duration(0.2), &Navigation::timerCallback, this);
    dwa_start = false;
    dwa_obs_region.clear();
  	dwa_obs_robot_region.clear();
    dwa_predict_pose.clear();
    dwa_choose_line.clear();
    robot_pose.x() = 0;
    robot_pose.y() = 0;
    robot_pose.z() = 0;
    tracking_v = 0;
    tracking_w = 0;
}

Navigation::~Navigation()
{

}

void Navigation::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    sensor_msgs::PointCloud2 cloud;
    try
		{
		    projector_1_.transformLaserScanToPointCloud("base_link", *scan, cloud, tf_buffer_);
		    sensor_msgs::convertPointCloud2ToPointCloud(cloud, laser_point_cloud_1_);
		}
		catch(tf::TransformException &ex)
		{

		}
    dwa_obs_region.clear();
		dwa_obs_robot_region.clear();
    for(int i=0;i<laser_point_cloud_1_.points.size();++i)
		{
		    const geometry_msgs::Point32& currPoint(laser_point_cloud_1_.points[i]);
		    Eigen::Vector3f robot_laser_pose(currPoint.x, currPoint.y, 0.0);
		    float real_x, real_y;
		    use_dwa.robot2world(robot_laser_pose, robot_pose, real_x, real_y);
				Eigen::Vector3f laser_pose(real_x, real_y, 0.0);
				if(robot_laser_pose.x() > 0.4 && robot_laser_pose.x() < 2.0 && fabs(robot_laser_pose.y()) < 2.0)
				{
						dwa_obs_region.push_back(laser_pose);
						dwa_obs_robot_region.push_back(robot_laser_pose);
				}
		}
}

void Navigation::amclposeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    robot_pose.x() = msg.pose.pose.position.x;
    robot_pose.y() = msg.pose.pose.position.y;
    robot_pose.z() = tf::getYaw(msg.pose.pose.orientation);
}

void Navigation::goalposeCallback(const geometry_msgs::PoseStamped& msg)
{
    dwa_start = true;
    goal_pose.x() =  msg.pose.position.x;
    goal_pose.y() =  msg.pose.position.y;
    goal_pose.z() = tf::getYaw(msg.pose.orientation);
    std::cout<<"goal.x = "<<goal_pose.x()<<" goal.y = "<<goal_pose.y()<<" goal.z = "<<goal_pose.z();
}

void Navigation::timerCallback(const ros::TimerEvent& event)
{
    static bool arrival_goal = false;
    float evalParam[4] = { 1, 2, 1, 2.0};
    float Kinematic[6] = { 1.0, 0.5, 0.1, 0.1, 0.01, 0.1 };
    float obstacleR = 0.4; //冲突判定用的障碍物半径
    if(dwa_start)
    {
        std::vector<state> cartraj,realtraj; //cartraj累积存储走过的轨迹点的状态值
        state statex = { robot_pose.x(),robot_pose.y(),robot_pose.z(), tracking_v, tracking_w };
        float tdist = sqrt((statex.x - goal_pose.x())*(statex.x - goal_pose.x()) + (statex.y - goal_pose.y())*(statex.y - goal_pose.y()));
        //std::cout<<"dwa start!!"<<std::endl;
        std::cout<<"tdist = "<<tdist<<std::endl;
        if(tdist < 0.05)arrival_goal = true;
        if(!arrival_goal)
        {
            Eigen::Vector3f dwa_choose_pose(0.0, 0.0, 0.0);
            controlU cu = use_dwa.DynamicWindowApproach(statex, &cartraj, Kinematic, goal_pose, evalParam, dwa_obs_region, dwa_obs_robot_region, obstacleR, dwa_predict_pose, dwa_choose_pose);
            tracking_v = cu.vt;
						tracking_w = cu.wt;
            draw(6 , 1.0, 0.1, 0.1, dwa_predict_pose);
						dwa_choose_line.push_back(robot_pose);
						dwa_choose_line.push_back(dwa_choose_pose);
						drawLine(2, 0.0, 0.0, 1.0, dwa_choose_line);
        }
        else
        {
            arrival_goal = false;
            dwa_start = false;
        }
        geometry_msgs::Twist speed_msg;
				speed_msg.linear.x = tracking_v;
				speed_msg.linear.y = 0;
				speed_msg.linear.z = 0;
				speed_msg.angular.x = 0;
				speed_msg.angular.y = 0;
				speed_msg.angular.z = tracking_w;
				simulation_speed_Publisher_.publish(speed_msg);
    }
}

void Navigation::draw(int id,float color_r,float color_g,float color_b,std::vector<Eigen::Vector3f> drawpoint)
{
    //畫出軌跡再rviz
            visualization_msgs::Marker points;
            points.header.frame_id  = "/map";
            points.header.stamp  = ros::Time::now();
            points.ns  = "points";
            points.action  = visualization_msgs::Marker::ADD;
            points.pose.orientation.w  = 1.0;
            points.id = id;


            points.type = visualization_msgs::Marker::POINTS;

            // POINTS markers use x and y scale for width/height respectively
            points.scale.x = 0.2;
            points.scale.y = 0.2;

            // Points are green
            points.color.g = color_g;
            points.color.b = color_b;
            points.color.r = color_r;
            points.color.a = 1.0;

            // Create the vertices for the points and lines
            for(int i=0;i<drawpoint.size() ;i++)
            {

            geometry_msgs::Point p;
            p.x = drawpoint[i].x();
            p.y = drawpoint[i].y();
            p.z = 0;

            points.points.push_back(p);

            }

            // std::cout<<"sub_missonPath_subPoint_buf.sub_missonPath_subPoint.size()  "<<sub_missonPath_subPoint_buf.sub_missonPath_subPoint.size()<<std::endl;
            // std::cout<<"points  "<<points.points.size()<<std::endl;
            //std::cout<<"draw  "<<std::endl;
            marke_Publisher_.publish(points);
            //畫出軌跡再rviz
}

void Navigation::drawLine(int id,float color_r,float color_g,float color_b,std::vector<Eigen::Vector3f> drawline)
{
    //std::cout<<"========drawLine========"<<std::endl;
    //畫出軌跡再rviz
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "lines";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w =1.0;
    line_strip.id = id;



    line_strip.type = visualization_msgs::Marker::LINE_STRIP;


    line_strip.scale.x = 0.1;

    line_strip.color.r = color_r;
    line_strip.color.g = color_g;
    line_strip.color.b = color_b;
    line_strip.color.a = 1.0;


 for(int i=0;i<drawline.size() ;i++)
    {

        geometry_msgs::Point line;
        line.x = drawline[i].x();
        line.y = drawline[i].y();
        line.z = drawline[i].z();
        line_strip.points.push_back(line);

    }
    marke_Publisher_.publish(line_strip);

    //畫出軌跡再rviz
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
