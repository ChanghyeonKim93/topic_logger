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
typedef Eigen::Matrix<double,9,1> ImuVector;
typedef std::string TopicTime;

class TopicLogger {

public:
  TopicLogger(std::string folder_dir);
  ~TopicLogger();
  void pose_addline(const PoseVector& current_pose, const TopicTime& curr_time);
  void imu_addline(const ImuVector& current_imu, const TopicTime& curr_time);
  void image_addline(const cv::Mat& img,const TopicTime& curr_time);
  void rgbd_addline(const cv::Mat& current_image, const cv::Mat& current_depth, const TopicTime& image_time);
private:
  std::string folder_dir;

  PoseVector current_pose;
  ImuVector current_imu;

  std::ofstream file_image, file_imu, file_pose;
  std::string file_image_name, file_imu_name, file_pose_name;

};

#endif
