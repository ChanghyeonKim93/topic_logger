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
#include <sensor_msgs/MagneticField.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

bool pose_updated     = false;
bool single_updated   = false;
bool rgbd_updated     = false;
bool stereo_updated   = false;
bool imu_updated      = false;
bool magnetic_updated = false;

typedef Eigen::Matrix<double, 7, 1> PoseVector;
typedef Eigen::Matrix<double, 9, 1> ImuVector;
typedef std::string TopicTime;

PoseVector current_pose = PoseVector::Zero();
ImuVector current_imu = ImuVector::Zero();

TopicTime imu_time;
TopicTime magnetic_time;
TopicTime single_time;
TopicTime stereo_time;
TopicTime rgbd_time;
TopicTime pose_time;

cv::Mat cur_single_img;
cv::Mat cur_rgb_img;
cv::Mat cur_depth_img;
cv::Mat cur_left_img;
cv::Mat cur_right_img;

std::string dtos(double x)
{
	std::stringstream s;
	s << std::setprecision(6) << std::fixed << x;
	return s.str();
}

void pose_callback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
	//current_pose(0,0) = msg->pose.position.x;
	//current_pose(6,0) = msg->pose.orientation.w;
	current_pose(0, 0) = msg->transform.translation.x;
	current_pose(1, 0) = msg->transform.translation.y;
	current_pose(2, 0) = msg->transform.translation.z;
	current_pose(3, 0) = msg->transform.rotation.x;
	current_pose(4, 0) = msg->transform.rotation.y;
	current_pose(5, 0) = msg->transform.rotation.z;
	current_pose(6, 0) = msg->transform.rotation.w;

	double curr_time = (double)(msg->header.stamp.sec * 1e6 + msg->header.stamp.nsec / 1000) / 1000000.0;
	pose_time = dtos(curr_time);
	pose_updated = true;
	ROS_INFO_STREAM("Pose updated");
}

void image_callback(const sensor_msgs::ImageConstPtr &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
	cur_single_img = cv_ptr->image;

	double curr_time = (double)(msg->header.stamp.sec * 1e6 + msg->header.stamp.nsec / 1000) / 1000000.0;
	single_time = dtos(curr_time);
	single_updated = true;
	ROS_INFO_STREAM("Single Image updated");
}

void rgbd_callback(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth)
{
	cv_bridge::CvImagePtr cv_ptr_rgb, cv_ptr_depth;
	cv_ptr_rgb = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::RGB8);
	cv_ptr_depth = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_32FC1);
	cur_rgb_img = cv_ptr_rgb->image;
	cur_depth_img = cv_ptr_depth->image;

	double curr_time = (double)(rgb->header.stamp.sec * 1e6 + rgb->header.stamp.nsec / 1000) / 1000000.0;
	single_time = dtos(curr_time);
	single_updated = true;
	ROS_INFO_STREAM("Rgb-d updated");
}

void stereo_image_callback(const sensor_msgs::ImageConstPtr &left, const sensor_msgs::ImageConstPtr &right)
{
	cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right;
	cv_ptr_left = cv_bridge::toCvCopy(left, sensor_msgs::image_encodings::MONO8);
	cv_ptr_right = cv_bridge::toCvCopy(right, sensor_msgs::image_encodings::MONO8);
	cur_left_img = cv_ptr_left->image;
	cur_right_img = cv_ptr_right->image;

	double curr_time = (double)(left->header.stamp.sec * 1e6 + left->header.stamp.nsec / 1000) / 1000000.0;
	stereo_time = dtos(curr_time);
	stereo_updated = true;
	ROS_INFO_STREAM("Stereo updated");
}

void imu_callback(const sensor_msgs::ImuConstPtr &msg)
{
	current_imu(0, 0) = msg->linear_acceleration.x;
	current_imu(1, 0) = msg->linear_acceleration.y;
	current_imu(2, 0) = msg->linear_acceleration.z;
	current_imu(3, 0) = msg->angular_velocity.x;
	current_imu(4, 0) = msg->angular_velocity.y;
	current_imu(5, 0) = msg->angular_velocity.z;
	double curr_time = (double)(msg->header.stamp.sec * 1e6 + msg->header.stamp.nsec / 1000) / 1000000.0;
	imu_time = dtos(curr_time);
	imu_updated = true;
	ROS_INFO_STREAM("Imu updated");
}

void magnetic_callback(const sensor_msgs::MagneticFieldConstPtr &msg)
{
	current_imu(6, 0) = msg->magnetic_field.x;
	current_imu(7, 0) = msg->magnetic_field.y;
	current_imu(8, 0) = msg->magnetic_field.z;

	double curr_time = (double)(msg->header.stamp.sec * 1e6 + msg->header.stamp.nsec / 1000) / 1000000.0;
	magnetic_time = dtos(curr_time);
	magnetic_updated = true;
	ROS_INFO_STREAM("Magnetic updated");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "topic_logger_node");
	ros::NodeHandle nh("~");

	std::string folder_dir;
	if (ros::param::get("~folder_dir", folder_dir) == false)
	{
		ROS_ERROR_STREAM("this is toerere\n");
	}
	std::cout << "Saving folder directory: " << folder_dir << std::endl;

	// Generate a topic logger class.
	TopicLogger *topic_logger = new TopicLogger(folder_dir);

	ros::Subscriber cur_pose_sub;
	ros::Subscriber cur_imu_sub;
	ros::Subscriber cur_mag_sub;
	ros::Subscriber cur_single_img_sub;

	bool single_on, rgbd_on, stereo_on, imu_on, magnetic_on, pose_on;

	ros::param::get("~single_on", single_on);
	ros::param::get("~rgbd_image_on", rgbd_on);
	ros::param::get("~stereo_image_on", stereo_on);
	ros::param::get("~imu_on", imu_on);
	ros::param::get("~magnetic_on", magnetic_on);
	ros::param::get("~pose_on", pose_on);

	std::string single_topic;
	std::string rgb_topic;
	std::string depth_topic;
	std::string stereo_left_topic;
	std::string stereo_right_topic;
	std::string imu_topic;
	std::string magnetic_topic;
	std::string pose_topic;

	ros::param::get("~single_topic", single_topic);
	ros::param::get("~rgb_topic", rgb_topic);
	ros::param::get("~depth_topic", depth_topic);
	ros::param::get("~stereo_left_topic", stereo_left_topic);
	ros::param::get("~stereo_right_topic", stereo_right_topic);
	ros::param::get("~imu_topic", imu_topic);
	ros::param::get("~magnetic_topic", magnetic_topic);
	ros::param::get("~pose_topic", pose_topic);

	if (rgbd_on == true)
	{
		message_filters::Subscriber<sensor_msgs::Image> rgb_img_sub(nh, rgb_topic, 1);
		message_filters::Subscriber<sensor_msgs::Image> depth_img_sub(nh, depth_topic, 1);

		message_filters::Synchronizer<MySyncPolicy> sync_rgbd(MySyncPolicy(4), rgb_img_sub, depth_img_sub);
		sync_rgbd.registerCallback(boost::bind(&rgbd_callback, _1, _2));
	}

	if (stereo_on == true)
	{
		message_filters::Subscriber<sensor_msgs::Image> left_img_sub(nh, stereo_left_topic, 1);
		message_filters::Subscriber<sensor_msgs::Image> right_img_sub(nh, stereo_right_topic, 1);

		message_filters::Synchronizer<MySyncPolicy> sync_stereo(MySyncPolicy(4), left_img_sub, right_img_sub);
		sync_stereo.registerCallback(boost::bind(&stereo_image_callback, _1, _2));
	}

	if (pose_on == true)
		cur_pose_sub = nh.subscribe<geometry_msgs::TransformStamped>(pose_topic, 10, &pose_callback);
	if (single_on == true)
		cur_single_img_sub = nh.subscribe<sensor_msgs::Image>(single_topic, 10, &image_callback);
	if (imu_on == true)
		cur_imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 10, &imu_callback);
	if (magnetic_on == true)
		cur_mag_sub = nh.subscribe<sensor_msgs::MagneticField>(magnetic_topic, 10, &magnetic_callback);

	while (ros::ok())
	{
		ros::spinOnce(); // so fast
		if (imu_on == true && imu_updated == true)
		{
			topic_logger->imu_addline(current_imu, imu_time);
			imu_updated = false;
		}
		if(single_on == true && single_updated==true){
			topic_logger->single_image_addline(cur_single_img, single_time);
			single_updated = false;
		}

		if(stereo_on == true && stereo_updated==true){
			topic_logger->stereo_image_addline(cur_left_img, cur_right_img, single_time);
			stereo_updated = false;
		}

		if (rgbd_on == true && rgbd_updated == true)
		{
			topic_logger->rgbd_image_addline(cur_rgb_img, cur_depth_img, rgbd_time);
			rgbd_updated = false;
		}

		if (pose_on == true && pose_updated == true)
		{
			topic_logger->pose_addline(current_pose, pose_time);
			pose_updated = false;
		}

		if(magnetic_on == true && magnetic_updated == true)
		{
			topic_logger->imu_addline(current_imu, imu_time);
			magnetic_updated = false;
		}
	}

	delete topic_logger;
	return 0;
}
