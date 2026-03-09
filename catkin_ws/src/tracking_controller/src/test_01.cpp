#include <ros/ros.h>
#include <Eigen/Dense>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <tracking_controller/Target.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tracking_controller/utils.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/UInt8.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
void odomCB(const nav_msgs::OdometryConstPtr& odom){
    tf2::Quaternion quat;
    tf2::convert(odom->pose.pose.orientation, quat); //把mavros/local_position/pose里的四元数转给tf2::Quaternion quat
    double roll, pitch,yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    double yaw_02;
    yaw_02 = controller::rpy_from_quaternion(odom->pose.pose.orientation);
    printf("yaw_01:%f yaw_02:%f \n",yaw,yaw_02);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "test_node");
	ros::NodeHandle nh;
    ros::Subscriber odomSub_ = nh.subscribe("/mavros/local_position/odom", 10, odomCB);
    ros::Duration(0.5).sleep();

	ros::spin();

	return 0;
}