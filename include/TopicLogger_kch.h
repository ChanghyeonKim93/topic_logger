#ifndef _TOPIC_LOGGER_H_
#define _TOPIC_LOGGER_H_

#include <iostream>
#include <ros/ros.h>
#include <sys/time.h>
#include <Eigen/Dense>
#include <fstream>
#include <ctime>
#include <string>

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
typedef Eigen::Matrix<double, 7, 1> PoseVector;
typedef Eigen::Matrix<double, 13, 1> ImuVector;
typedef std::string TopicTime;

class TopicLogger
{
public:
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> *left_img_sub;
  message_filters::Subscriber<sensor_msgs::Image> *right_img_sub;
  message_filters::Synchronizer<MySyncPolicy> *sync_stereo;

  message_filters::Subscriber<sensor_msgs::Image> *rgb_img_sub;
  message_filters::Subscriber<sensor_msgs::Image> *depth_img_sub;;
  message_filters::Synchronizer<MySyncPolicy> *sync_rgbd;

  ros::Subscriber cur_pose_sub;
  ros::Subscriber cur_imu_sub;
  ros::Subscriber cur_mag_sub;
  ros::Subscriber cur_single_img_sub;

  TopicLogger();
  ~TopicLogger();
  void pose_addline(const PoseVector &current_pose, const TopicTime &curr_time);
  void imu_addline(const ImuVector &current_imu, const TopicTime &curr_time);
  void single_image_addline(const cv::Mat &single_image, const TopicTime &curr_time);
  void stereo_image_addline(const cv::Mat &current_left_image, const cv::Mat &current_right_image, const TopicTime &image_time);
  void rgbd_image_addline(const cv::Mat &current_rgb_image, const cv::Mat &current_depth_image, const TopicTime &image_time);
  void pose_callback(const geometry_msgs::TransformStamped::ConstPtr &msg);
  void image_callback(const sensor_msgs::ImageConstPtr &msg);
  void rgbd_callback(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth);
  void stereo_image_callback(const sensor_msgs::ImageConstPtr &left, const sensor_msgs::ImageConstPtr &right);
  void imu_callback(const sensor_msgs::ImuConstPtr &msg);
  void magnetic_callback(const sensor_msgs::MagneticFieldConstPtr &msg);

private:
  std::string folder_dir;

  PoseVector current_pose;
  ImuVector current_imu;

  std::ofstream file_single_image, file_stereo_image, file_rgbd_image, file_imu, file_pose;
	
  bool single_on, rgbd_on, stereo_on, imu_on, magnetic_on, pose_on;

  bool pose_updated;
  bool single_updated;
  bool rgbd_updated;
  bool stereo_updated;
  bool imu_updated;
  bool magnetic_updated;

  std::string single_topic;
	std::string rgb_topic;
	std::string depth_topic;
	std::string stereo_left_topic;
	std::string stereo_right_topic;
	std::string imu_topic;
	std::string magnetic_topic;
	std::string pose_topic;

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
};

#endif
