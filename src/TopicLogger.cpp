#include <TopicLogger.h>

#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>

TopicLogger::TopicLogger(std::string folder_dir_){
	std::string folder_create_command, file_name, folder_dir_temp;
	this->folder_dir = folder_dir_;
  folder_dir_temp = this->folder_dir;

// 1. imu folder
	//if(strcmp(*folder_dir_temp.end(),'/')!=0) folder_dir_temp+='/';

	folder_create_command = "mkdir " + folder_dir_temp;
	system(folder_create_command.c_str());
	folder_create_command = "mkdir " + folder_dir_temp + "image";
	system(folder_create_command.c_str());

	file_name = folder_dir_temp+"association.txt";
	std::cout<<file_name.c_str()<<std::endl;
	file_image.open(file_name.c_str(),std::ios::trunc);
	file_image << "# time filename";

	file_name = folder_dir_temp+"imu.txt";
	file_imu.open(file_name.c_str());
	file_imu << "# time tx ty tz wx wy wz\n";

	file_name = folder_dir_temp+"groundtruth.txt";
	file_pose.open(file_name.c_str());
	file_pose << "# time tx ty tz qx qy qz qw\n";
};

TopicLogger::~TopicLogger(){
	file_pose.close();
	file_imu.close();
	file_image.close();
	ROS_INFO_STREAM("  Logger is terminated.\n");
};

void TopicLogger::pose_addline(const PoseVector& current_pose, const TopicTime& curr_time ){

	file_pose <<curr_time<< "\t";
	file_pose <<std::setprecision(13);
	file_pose.unsetf(std::ios::fixed);
	for(int i=0; i<7; i++)
	{
		file_pose << this->current_pose(i,0) << "\t";
	}
	file_pose << "\n";
};

void TopicLogger::imu_addline(const ImuVector& current_imu, const TopicTime& curr_time){
	file_imu <<curr_time<< "\t";
	file_imu <<std::setprecision(13);
	file_imu.unsetf(std::ios::fixed);
	for(int i=0; i<6; i++){
		file_imu << current_imu(i,0) << "\t";
	}
	file_imu << "\n";
};

void TopicLogger::image_addline(const cv::Mat& img,const TopicTime& curr_time){
	bool static png_param_on=false;
	std::vector<int> static png_parameters;
	if(png_param_on==false){
			png_parameters.push_back( CV_IMWRITE_PNG_COMPRESSION );	// We save with no compression for faster processing
			png_parameters.push_back(0);
			png_param_on = true;
	}
	std::string filne_name = folder_dir+"image/"+curr_time+".png";
	cv::imwrite(filne_name,img,png_parameters);
	file_image << curr_time<<"\t"<< curr_time<<".png"<<"\n";	// association save
};
