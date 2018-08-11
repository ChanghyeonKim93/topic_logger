#include <ros/ros.h>

#include <TopicLogger.h>

#include <iostream>
#include <sys/time.h>
#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/Imu.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#include <message_filters/sync_policies/approximate_time.h>
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

bool pose_updated  = false;
bool image_updated = false;
bool imu_updated   = false;

typedef Eigen::Matrix<double,7,1> PoseVector;
typedef Eigen::Matrix<double,9,1> ImuVector;
typedef std::string TopicTime;

PoseVector current_pose = PoseVector::Zero();
ImuVector current_imu = ImuVector::Zero();
TopicTime imu_time, image_time, pose_time;

cv::Mat current_image,current_depth;

std::string dtos(double x){
	std::stringstream s;
	s<<std::setprecision(6) << std::fixed << x;
	return s.str();
}

void pose_callback(const geometry_msgs::TransformStamped::ConstPtr& msg){
	//current_pose(0,0) = msg->pose.position.x;
	//current_pose(1,0) = msg->pose.position.y;
	//current_pose(2,0) = msg->pose.position.z;
	//current_pose(3,0) = msg->pose.orientation.x;
	//current_pose(4,0) = msg->pose.orientation.y;
	//current_pose(5,0) = msg->pose.orientation.z;
	//current_pose(6,0) = msg->pose.orientation.w;
	current_pose(0,0) = msg->transform.translation.x;
	current_pose(1,0) = msg->transform.translation.y;
	current_pose(2,0) = msg->transform.translation.z;
	current_pose(3,0) = msg->transform.rotation.x;
        current_pose(4,0) = msg->transform.rotation.y;
	current_pose(5,0) = msg->transform.rotation.z;
	current_pose(6,0) = msg->transform.rotation.w;
	double curr_time = (double)(msg->header.stamp.sec*1e6+msg->header.stamp.nsec/1000)/1000000.0;
	pose_time = dtos(curr_time);
	pose_updated = true;
	ROS_INFO_STREAM("Pose updated");
}


void image_callback(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
	current_image = cv_ptr->image;
  double curr_time = (double)(msg->header.stamp.sec*1e6+msg->header.stamp.nsec/1000)/1000000.0;
	image_time = dtos(curr_time);
	image_updated = true;
	ROS_INFO_STREAM("Image updated");
}


void rgbd_callback(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth){
	cv_bridge::CvImagePtr cv_ptr_rgb, cv_ptr_depth;
	cv_ptr_rgb = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::RGB8);
	cv_ptr_depth = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_32FC1);
	current_image = cv_ptr_rgb->image;
	current_depth = cv_ptr_depth->image;

  	double curr_time = (double)(rgb->header.stamp.sec*1e6+rgb->header.stamp.nsec/1000)/1000000.0;
	image_time = dtos(curr_time);
	image_updated = true;
	ROS_INFO_STREAM("Image updated");
}

void imu_callback(const sensor_msgs::ImuConstPtr& msg){
	current_imu(0,0) = msg->linear_acceleration.x;
	current_imu(1,0) = msg->linear_acceleration.y;
	current_imu(2,0) = msg->linear_acceleration.z;
	current_imu(3,0) = msg->angular_velocity.x;
	current_imu(4,0) = msg->angular_velocity.y;
	current_imu(5,0) = msg->angular_velocity.z;
	double curr_time = (double)(msg->header.stamp.sec*1e6+msg->header.stamp.nsec/1000)/1000000.0;
	imu_time = dtos(curr_time);
	imu_updated = true;
	ROS_INFO_STREAM("Imu updated");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "topic_logger_node");
	ros::NodeHandle nh("~");

	std::string mydir;
	if(ros::param::get("~folder_dir", mydir)==false){
		ROS_ERROR_STREAM("this is toerere\n");
	}
	std::cout<<"Saving folder directory: " <<mydir<<std::endl;
	TopicLogger *topic_logger = new TopicLogger(mydir);

	ros::Subscriber current_pose_sub, current_imu_sub, current_image_sub;



    	std::string image_topic_name = std::string("/rgb/image");
    	std::string depth_topic_name = std::string("/depth/image");
	std::string imu_topic_name   = std::string("/imu");
	std::string pose_topic_name  = std::string("/vicon/wand/wand");

    	ros::param::get("~image_topic_name", image_topic_name);
    	ros::param::get("~depth_topic_name", depth_topic_name);
    	ros::param::get("~imu_topic_name", imu_topic_name);
    	ros::param::get("~pose_topic_name", pose_topic_name);

	message_filters::Subscriber<sensor_msgs::Image> rgbImgSubs(nh,image_topic_name,1);
	message_filters::Subscriber<sensor_msgs::Image> depthImgSubs(nh,depth_topic_name,1);
	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(3), rgbImgSubs, depthImgSubs);
sync.registerCallback(boost::bind(&rgbd_callback,_1,_2));

	current_pose_sub  = nh.subscribe<geometry_msgs::TransformStamped>(pose_topic_name, 10, &pose_callback);
	current_image_sub = nh.subscribe<sensor_msgs::Image>(image_topic_name,10,&image_callback);
	
	current_imu_sub   = nh.subscribe<sensor_msgs::Imu>(imu_topic_name,10,&imu_callback);

	while(ros::ok()){
		ros::spinOnce(); // so fast
		if(imu_updated==true){
			topic_logger->imu_addline(current_imu,imu_time);
			imu_updated = false;
		}
		/*if(image_updated==true){
			topic_logger->image_addline(current_image,image_time);
			image_updated = false;
		}*/

		if(image_updated==true){
			topic_logger->rgbd_addline(current_image,current_depth,image_time);
			image_updated = false;
		}
		if(pose_updated==true){
			topic_logger->pose_addline(current_pose,pose_time);
			pose_updated=false;
		}

	}
	delete topic_logger;
	return 0;
}
