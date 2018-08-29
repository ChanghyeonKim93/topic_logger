#include <TopicLogger.h>

#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>

TopicLogger::TopicLogger(std::string folder_dir_)
{
	std::string folder_create_command, file_name, folder_dir_temp;
	this->folder_dir = folder_dir_;
	folder_dir_temp = this->folder_dir;

	folder_create_command = "rm -rf " + folder_dir_temp;
	system(folder_create_command.c_str());

	folder_create_command = "mkdir " + folder_dir_temp;
	system(folder_create_command.c_str());

	folder_create_command = "mkdir " + folder_dir_temp + "single_image";
	system(folder_create_command.c_str());

	folder_create_command = "mkdir " + folder_dir_temp + "stereo";
	system(folder_create_command.c_str());
	folder_create_command = "mkdir " + folder_dir_temp + "stereo/right";
	system(folder_create_command.c_str());
	folder_create_command = "mkdir " + folder_dir_temp + "stereo/left";
	system(folder_create_command.c_str());

	folder_create_command = "mkdir " + folder_dir_temp + "rgbd";
	system(folder_create_command.c_str());
	folder_create_command = "mkdir " + folder_dir_temp + "rgbd/depth";
	system(folder_create_command.c_str());
	folder_create_command = "mkdir " + folder_dir_temp + "rgbd/rgb";
	system(folder_create_command.c_str());

	file_name = folder_dir_temp + "association_single.txt";
	//std::cout << file_name.c_str() << std::endl;
	file_single_image.open(file_name.c_str(), std::ios::trunc);
	file_single_image << "# time filename";

	file_name = folder_dir_temp + "stereo/association_stereo.txt";
	//std::cout << file_name.c_str() << std::endl;
	file_stereo_image.open(file_name.c_str(), std::ios::trunc);
	file_stereo_image << "# time filename";

	file_name = folder_dir_temp + "rgbd/association_rgbd.txt";
	//std::cout << file_name.c_str() << std::endl;
	file_rgbd_image.open(file_name.c_str(), std::ios::trunc);
	file_rgbd_image << "# time filename";

	file_name = folder_dir_temp + "imu.txt";
	file_imu.open(file_name.c_str());
	file_imu << "# time tx ty tz wx wy wz mx my mz qx qy qz qw\n";

	file_name = folder_dir_temp + "groundtruth.txt";
	file_pose.open(file_name.c_str());
	file_pose << "# time tx ty tz qx qy qz qw\n";
};

TopicLogger::~TopicLogger()
{
	file_single_image.close();
	file_rgbd_image.close();
	file_stereo_image.close();
	file_pose.close();
	file_imu.close();
	ROS_INFO_STREAM("- Topic logger is terminated.\n");
};

void TopicLogger::pose_addline(const PoseVector &current_pose, const TopicTime &curr_time)
{
	file_pose << curr_time << " ";
	file_pose << std::setprecision(13);
	file_pose.unsetf(std::ios::fixed);
	for (int i = 0; i < 7; i++)
	{
		file_pose << current_pose(i, 0) << " ";
	}
	file_pose << "\n";
};

void TopicLogger::imu_addline(const ImuVector &current_imu, const TopicTime &curr_time)
{
	file_imu << curr_time << " ";
	file_imu << std::setprecision(13);
	file_imu.unsetf(std::ios::fixed);
	for (int i = 0; i < 13; i++)
	{
		file_imu << current_imu(i, 0) << " ";
	}
	file_imu << "\n";
};

void TopicLogger::single_image_addline(const cv::Mat &img, const TopicTime &curr_time)
{
	bool static png_param_on = false;
	std::vector<int> static png_parameters;
	if (png_param_on == false)
	{
		png_parameters.push_back(CV_IMWRITE_PNG_COMPRESSION); // We save with no compression for faster processing
		png_parameters.push_back(0);
		png_param_on = true;
	}
	std::string file_name = folder_dir + "single_image/" + curr_time + ".png";
	cv::imwrite(file_name, img, png_parameters);
	file_single_image << curr_time << " " << curr_time << ".png" << "\n"; // association save
};

void TopicLogger::rgbd_image_addline(const cv::Mat &current_rgb_image, const cv::Mat &current_depth_image, const TopicTime &image_time)
{
	bool static png_param_on = false;
	std::vector<int> static png_parameters;
	if (png_param_on == false)
	{
		png_parameters.push_back(CV_IMWRITE_PNG_COMPRESSION); // We save with no compression for faster processing
		png_parameters.push_back(0);
		png_param_on = true;
	}
	std::string file_name = folder_dir + "rgbd/rgb/" + image_time + ".png";
	cv::imwrite(file_name, current_rgb_image, png_parameters);
	std::string file_depth_name = folder_dir + "rgbd/depth/" + image_time + ".png";
	cv::imwrite(file_depth_name, current_depth_image, png_parameters);
	file_rgbd_image << image_time << " " << image_time << ".png" << "\n"; // association save
}

void TopicLogger::stereo_image_addline(const cv::Mat &current_left_image, const cv::Mat &current_right_image, const TopicTime &image_time)
{
	bool static png_param_on = false;
	std::vector<int> static png_parameters;
	if (png_param_on == false)
	{
		png_parameters.push_back(CV_IMWRITE_PNG_COMPRESSION); // We save with no compression for faster processing
		png_parameters.push_back(0);
		png_param_on = true;
	}
	std::string file_name = folder_dir + "stereo/left/" + image_time + ".png";
	//cv::imwrite(file_name, current_left_image, png_parameters);
	std::string file_depth_name = folder_dir + "stereo/right/" + image_time + ".png";
	//cv::imwrite(file_depth_name, current_right_image, png_parameters);
	file_stereo_image << image_time << " " << image_time << ".png" << "\n"; // association save
}
