
#include <robot_status_publisher/robot_status_publisher.h>

RobotStatusPublisher::RobotStatusPublisher(std::string robot_id)
{
    ROS_INFO_STREAM("Initialise robot status publisher for robot " + robot_id);
    m_pub = m_nodeHandle.advertise<robot_status_msgs::RobotStatus>("robot_status", 100);
    m_sub = m_nodeHandle.subscribe("odom", 10, &RobotStatusPublisher::odomCallback, this);
}

RobotStatusPublisher::~RobotStatusPublisher()
{
}

void RobotStatusPublisher::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    m_odomData = *msg;
}

void RobotStatusPublisher::update()
{
    robot_status_msgs::RobotStatus robotStatus;
    robotStatus.stamp = ros::Time::now()
                            robotStatus.battery_level = std::rand() % 101;
    robotStatus.pose.position = m_odomData.pose.pose.position;
    robotStatus.pose.orientation = m_odomData.pose.pose.orientation;

    m_pub.publish(robotStatus);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_status_publisher_node");
    RobotStatusPublisher robotStatusPublisher(argv[1]);
    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        robotStatusPublisher.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}