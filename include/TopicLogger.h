#ifndef _TOPIC_LOGGER_H_
#define _TOPIC_LOGGER_H_

#include <iostream>
#include <ros/ros.h>
#include <sys/time.h>
#include <Eigen/Dense>
#include <fstream>
#include <ctime>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef Eigen::Matrix<double,7,1> PoseVector;
typedef Eigen::Matrix<double,13,1> ImuVector;
typedef std::string TopicTime;

class TopicLogger {

public:
  TopicLogger(std::string folder_dir);
  ~TopicLogger();
  void pose_addline(const PoseVector& current_pose, const TopicTime& curr_time);
  void imu_addline(const ImuVector& current_imu, const TopicTime& curr_time);
  void single_image_addline(const cv::Mat& single_image,const TopicTime& curr_time);
  void stereo_image_addline(const cv::Mat& current_left_image, const cv::Mat& current_right_image, const TopicTime& image_time);
  void rgbd_image_addline(const cv::Mat& current_rgb_image, const cv::Mat& current_depth_image, const TopicTime& image_time);

private:
  std::string folder_dir;

  PoseVector current_pose;
  ImuVector current_imu;

  std::ofstream file_single_image, file_stereo_image, file_rgbd_image, file_imu, file_pose;
};

#endif
