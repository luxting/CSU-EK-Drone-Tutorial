/*
	FILE: trackingController.h
	-------------------------------
	function definition of px4 tracking controller
*/
#ifndef TRACKING_CONTROLLER_H
#define TRACKING_CONTROLLER_H
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
#include <tracking_controller/PositionCommand.h>
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
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/RCIn.h>
#define VELOCITY2D_CONTROL 0b101111000111
#define VELOCITY2D_CONTROL_CROSS 0b101111000011

using std::cout; using std::endl;
namespace controller{
	class trackingController{
		private:
			ros::NodeHandle nh_;
			ros::Subscriber odomSub_; // Subscribe to odometry
			ros::Subscriber imuSub_; // IMU data subscriber
			ros::Subscriber targetSub_; // subscriber for the tracking target states
			ros::Subscriber obj_sub;
			ros::Subscriber cross_sub;
			//takeoff and land topic
			ros::Publisher mavros_setpoint_pos_pub;
        	ros::ServiceClient arming_client;
        	ros::ServiceClient set_mode_client;
        	ros::ServiceClient ctrl_pwm_client ;
			ros::Subscriber stateSub_;
			ros::Subscriber judgeSub_;//judge   Sub
			ros::Subscriber pos_get;
			ros::Subscriber pose_suber;
			ros::Subscriber rc_suber;
			ros::Publisher cmdPub_; // command publisher
			ros::Publisher accCmdPub_; // acceleration command publisher
			ros::Publisher poseVisPub_; // current pose publisher
			ros::Publisher posePub_;
			ros::Publisher targetVisPub_; // target pose publisher
			ros::Publisher histTrajVisPub_; // history trajectory publisher
			ros::Publisher targetHistTrajVisPub_; // target trajectory publisher
			ros::Publisher velAndAccVisPub_; // velocity and acceleration visualization publisher
			ros::Publisher velocity_pub;
			ros::Publisher traj_judge_pub;
			ros::Timer cmdTimer_; // command timer
			ros::Timer thrustEstimatorTimer_; // thrust estimator timer
			ros::Timer visTimer_; // visualization timer
			
			// parameters
			bool bodyRateControl_ = false;
			bool attitudeControl_ = false;
			bool accControl_ = true;
			bool pose_data_received_ = false;
			Eigen::Vector3d pPos_, iPos_, dPos_;
			Eigen::Vector3d pVel_, iVel_, dVel_;
			double attitudeControlTau_;
			double hoverThrust_;
			bool verbose_;

			// controller data
			bool odomReceived_ = false;
			bool imuReceived_ = false;
			bool thrustReady_ = false;
			bool firstTargetReceived_ = false;
			bool targetReceived_ = false;
			bool objReceived_ = false;
			bool CrossRecived = false;
			bool firstTime_ = true;
			nav_msgs::Odometry odom_;
			sensor_msgs::Imu imuData_;
			mavros_msgs::State state_;
			std_msgs::UInt8 judge_;
			std_msgs::UInt8 traj_judge_data;
			geometry_msgs::PoseStamped pose_data;
			geometry_msgs::PoseStamped local_pos;
			mavros_msgs::RCIn rc_data;
			mavros_msgs::PositionTarget obj_vel;
			geometry_msgs::Twist cross_vel;

			tracking_controller::Target target_;
			ros::Time prevTime_;
			double deltaTime_;
			Eigen::Vector3d posErrorInt_; // integral of position error
			Eigen::Vector3d velErrorInt_; // integral of velocity error
			Eigen::Vector3d deltaPosError_, prevPosError_; // delta of position error
			Eigen::Vector3d deltaVelError_, prevVelError_; // delta of velocity error
			double cmdThrust_;
			ros::Time cmdThrustTime_;

			// kalman filter
			bool kfFirstTime_ = true;
			bool hz_judge = false;
			ros::Time kfStartTime_;  
			double stateVar_ = 0.01;
			double processNoiseVar_ = 0.01;
			double measureNoiseVar_ = 0.02;
			unsigned short velocity_mask = VELOCITY2D_CONTROL;
			std::deque<double> prevEstimateThrusts_;

			// visualization
			geometry_msgs::PoseStamped poseVis_;
			std::deque<geometry_msgs::PoseStamped> histTraj_;
			geometry_msgs::PoseStamped targetPoseVis_;
			std::deque<geometry_msgs::PoseStamped> targetHistTraj_;
			bool velFirstTime_ = true;
			Eigen::Vector3d prevVel_;
			ros::Time velPrevTime_;
			ros::Time last_time;

			//takeoff
			mavros_msgs::PositionTarget setpoint_raw;
			geometry_msgs::PoseStamped hover_pose;
        	mavros_msgs::SetMode offb_set_mode;
			mavros_msgs::CommandBool arm_cmd;
			float position_x;
			float position_y;
			float position_z;
			double current_yaw,yaw;
			bool get_now_pose;

		public:
			trackingController(const ros::NodeHandle& nh);
			void initParam();
			void registerPub();
			void registerCallback();

			// callback functions
			void odomCB(const nav_msgs::OdometryConstPtr& odom);
			void imuCB(const sensor_msgs::ImuConstPtr& imu);
			//stat callback
			void stateCB(const mavros_msgs::State::ConstPtr& msg);
			//judge callback
			void judgeCB(const std_msgs::UInt8::ConstPtr& msg);
			//get_pose callback
			void getposCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
			void localCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
			void rcInCB(const mavros_msgs::RCIn::ConstPtr& msg);
			void ObjCB(const mavros_msgs::PositionTarget::ConstPtr& msg);
			void CrossCB(const geometry_msgs::Twist::ConstPtr& msg);

			//void targetCB(const tracking_controller::TargetConstPtr& target);
			void targetCB(const quadrotor_msgs::PositionCommandConstPtr& target);
			void cmdCB(const ros::TimerEvent&);
			void thrustEstimateCB(const ros::TimerEvent&);
			void visCB(const ros::TimerEvent&);

			void publishCommand(const Eigen::Vector4d& cmd);
			void publishCommand(const Eigen::Vector4d& cmd, const Eigen::Vector3d& accRef);
			void publishCommand(const Eigen::Vector3d& accRef);
			void computeAttitudeAndAccRef(Eigen::Vector4d& attitudeRefQuat, Eigen::Vector3d& accRef);
			void computeBodyRate(const Eigen::Vector4d& attitudeRefQuat, const Eigen::Vector3d& accRef, Eigen::Vector4d& cmd);

			// visualization
			void publishPoseVis();
			void publishHistTraj();
			void publishTargetVis();
			void publishTargetHistTraj();
			void publishVelAndAccVis();
	};
}

#endif