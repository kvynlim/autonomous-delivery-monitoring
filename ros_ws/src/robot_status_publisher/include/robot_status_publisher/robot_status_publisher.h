#ifndef ROBOT_STATUS_PUBLISHER_H_
#define ROBOT_STATUS_PUBLISHER_H_

#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <robot_status_msgs/RobotStatus.h>

class RobotStatusPublisher
{
public:
    RobotStatusPublisher(std::string robot_id);
    ~RobotStatusPublisher();

    void update();

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    nav_msgs::Odometry m_odomData;
    ros::NodeHandle m_nodeHandle;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;
};

#endif