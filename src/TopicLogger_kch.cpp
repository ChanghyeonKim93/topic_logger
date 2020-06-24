#include <TopicLogger_kch.h>

#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
typedef Eigen::Matrix<double, 7, 1> PoseVector;
typedef Eigen::Matrix<double, 13, 1> ImuVector;
typedef std::string TopicTime;

std::string dtos(double x)
{
	std::stringstream s;
	s << std::setprecision(6) << std::fixed << x;
	return s.str();
}

TopicLogger::TopicLogger()
{
	if (ros::param::get("~folder_dir", this->folder_dir) == false)
	{
		ROS_ERROR_STREAM("This is not a valid folder directory.\n");
	}
	ROS_INFO_STREAM("Saving folder directory: " << this->folder_dir);

	ros::param::get("~single_on", this->single_on);
	ros::param::get("~rgbd_on", this->rgbd_on);
	ros::param::get("~stereo_on", this->stereo_on);
	ros::param::get("~imu_on", this->imu_on);
	ros::param::get("~magnetic_on", this->magnetic_on);
	ros::param::get("~pose_on", this->pose_on);

	ros::param::get("~single_topic", this->single_topic);
	ros::param::get("~rgb_topic", this->rgb_topic);
	ros::param::get("~depth_topic", this->depth_topic);
	ros::param::get("~stereo_left_topic", this->stereo_left_topic);
	ros::param::get("~stereo_right_topic", this->stereo_right_topic);
	ros::param::get("~imu_topic", this->imu_topic);
	ros::param::get("~magnetic_topic", this->magnetic_topic);
	ros::param::get("~pose_topic", this->pose_topic);

	ROS_INFO_STREAM(" Activated topics - single [" << this->single_on << "], rgbd [" << this->rgbd_on << "], stereo [" << this->stereo_on << "], imu [" << this->imu_on << "], magnetic [" << this->magnetic_on << "], pose [" << this->pose_on << "]");
	// subscribers
	this->left_img_sub = new message_filters::Subscriber<sensor_msgs::Image>(this->nh, this->stereo_left_topic, 10);
	this->right_img_sub = new message_filters::Subscriber<sensor_msgs::Image>(this->nh, this->stereo_right_topic, 10);
	this->sync_stereo = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *left_img_sub, *right_img_sub);

	this->rgb_img_sub = new message_filters::Subscriber<sensor_msgs::Image>(this->nh, this->rgb_topic, 10);
	this->depth_img_sub = new message_filters::Subscriber<sensor_msgs::Image>(this->nh, this->depth_topic, 10);
	this->sync_rgbd = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *rgb_img_sub, *depth_img_sub);

	if (this->rgbd_on == true)
		this->sync_rgbd->registerCallback(boost::bind(&TopicLogger::rgbd_callback,this, _1, _2));
	if (this->stereo_on == true)
		this->sync_stereo->registerCallback(boost::bind(&TopicLogger::stereo_image_callback,this, _1, _2));
	if (this->pose_on == true)
		this->cur_pose_sub = this->nh.subscribe<geometry_msgs::TransformStamped>(this->pose_topic, 200, &TopicLogger::pose_callback,this);
	if (this->single_on == true)
		this->cur_single_img_sub = this->nh.subscribe<sensor_msgs::Image>(this->single_topic, 10, &TopicLogger::image_callback,this);
	if (this->imu_on == true)
		this->cur_imu_sub = this->nh.subscribe<sensor_msgs::Imu>(imu_topic, 200, &TopicLogger::imu_callback,this);
	if (this->magnetic_on == true)
		this->cur_mag_sub = this->nh.subscribe<sensor_msgs::MagneticField>(this->magnetic_topic, 200, &TopicLogger::magnetic_callback,this);

	this->pose_updated = false;
	this->single_updated = false;
	this->rgbd_updated = false;
	this->stereo_updated = false;
	this->imu_updated = false;
	this->magnetic_updated = false;

	this->current_pose = PoseVector::Zero();
	this->current_imu = ImuVector::Zero();

	std::string folder_create_command, file_name, folder_dir_temp;
	folder_dir_temp = this->folder_dir;

	folder_create_command = "rm -rf " + folder_dir_temp;
	system(folder_create_command.c_str());

	folder_create_command = "mkdir " + folder_dir_temp;
	system(folder_create_command.c_str());

	folder_create_command = "mkdir " + folder_dir_temp + "single";
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

	file_name = folder_dir_temp + "single/association_single.txt";
	//std::cout << file_name.c_str() << std::endl;
	this->file_single_image.open(file_name.c_str(), std::ios::trunc);
	this->file_single_image << "#time filename\n";

	file_name = folder_dir_temp + "stereo/association_stereo.txt";
	//std::cout << file_name.c_str() << std::endl;
	this->file_stereo_image.open(file_name.c_str(), std::ios::trunc);
	this->file_stereo_image << "#time filename\n";

	file_name = folder_dir_temp + "rgbd/association_rgbd.txt";
	//std::cout << file_name.c_str() << std::endl;
	this->file_rgbd_image.open(file_name.c_str(), std::ios::trunc);
	this->file_rgbd_image << "#time filename\n";

	file_name = folder_dir_temp + "imu.txt";
	this->file_imu.open(file_name.c_str());
	this->file_imu << "#time tx ty tz wx wy wz mx my mz qw qx qy qz\n";

	file_name = folder_dir_temp + "groundtruth.txt";
	this->file_pose.open(file_name.c_str());
	this->file_pose << "#time tx ty tz qw qx qy qz\n";
};

TopicLogger::~TopicLogger()
{
	this->file_single_image.close();
	this->file_rgbd_image.close();
	this->file_stereo_image.close();
	this->file_pose.close();
	this->file_imu.close();
	//ROS_INFO_STREAM("- Topic logger is terminated.\n");
};

void TopicLogger::pose_addline(const PoseVector &current_pose, const TopicTime &curr_time)
{
	this->file_pose << curr_time << " ";
	this->file_pose << std::setprecision(13);
	this->file_pose.unsetf(std::ios::fixed);
	for (int i = 0; i < 7; i++)
	{
		this->file_pose << current_pose(i, 0) << " ";
	}
	this->file_pose << "\n";
};

void TopicLogger::imu_addline(const ImuVector &current_imu, const TopicTime &curr_time)
{
	this->file_imu << curr_time << " ";
	this->file_imu << std::setprecision(13);
	this->file_imu.unsetf(std::ios::fixed);
	for (int i = 0; i < 13; i++)
	{
		this->file_imu << current_imu(i, 0) << " ";
	}
	this->file_imu << "\n";
	//ROS_INFO_STREAM("imu time : "<<curr_time);
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
	std::string file_name = folder_dir + "single/" + curr_time + ".png";
	cv::imwrite(file_name, img, png_parameters);
	this->file_single_image <<curr_time<<" "<< curr_time << ".png"
							<< "\n"; // association save
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
	std::string file_name = this->folder_dir + "rgbd/rgb/" + image_time + ".png";
	cv::imwrite(file_name, current_rgb_image, png_parameters);
	std::string file_depth_name = this->folder_dir + "rgbd/depth/" + image_time + ".png";
	cv::Mat img_depth_16uc1;
	current_depth_image *= 1000.0f;

    	current_depth_image.convertTo(img_depth_16uc1, CV_16UC1);
	cv::imwrite(file_depth_name, img_depth_16uc1, png_parameters);
	this->file_rgbd_image <<image_time<<" "<< image_time << ".png"
						  << "\n"; // association save
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
	cv::Mat img_temp;

	std::string file_name = this->folder_dir + "stereo/left/" + image_time + ".png";
	cv::cvtColor(current_left_image,img_temp,cv::COLOR_BGR2GRAY);
	cv::imwrite(file_name, img_temp, png_parameters);

	std::string file_depth_name = this->folder_dir + "stereo/right/" + image_time + ".png";
	cv::cvtColor(current_right_image,img_temp,cv::COLOR_BGR2GRAY);
	cv::imwrite(file_depth_name, img_temp, png_parameters);
	file_stereo_image <<image_time<<" "<< image_time << ".png" << "\n"; // association save

	ROS_INFO_STREAM("stereo time :"<<image_time);
}

void TopicLogger::pose_callback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
	//current_pose(0,0) = msg->pose.position.x;
	//current_pose(6,0) = msg->pose.orientation.w;
	this->current_pose(0, 0) = msg->transform.translation.x;
	this->current_pose(1, 0) = msg->transform.translation.y;
	this->current_pose(2, 0) = msg->transform.translation.z;
	this->current_pose(4, 0) = msg->transform.rotation.w;
	this->current_pose(5, 0) = msg->transform.rotation.x;
	this->current_pose(6, 0) = msg->transform.rotation.y;
	this->current_pose(7, 0) = msg->transform.rotation.z;

	double curr_time = (double)(msg->header.stamp.sec * 1e6 + msg->header.stamp.nsec / 1000) / 1000000.0;
	this->pose_time = dtos(curr_time);
	this->pose_updated = true;

	if(this->pose_on == true)
		this->pose_addline(this->current_pose, pose_time);
	ROS_INFO_STREAM("Pose updated");
}

void TopicLogger::image_callback(const sensor_msgs::ImageConstPtr &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	// cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	cur_single_img = cv_ptr->image;

	double curr_time = (double)(msg->header.stamp.sec * 1e6 + msg->header.stamp.nsec / 1000) / 1000000.0;
	this->single_time = dtos(curr_time);

	this->single_updated = true;
	
	if(single_on == true) 
		this->single_image_addline(this->cur_single_img, single_time);

	ROS_INFO_STREAM("  Single Image updated");
}

void TopicLogger::rgbd_callback(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth)
{
	cv_bridge::CvImagePtr cv_ptr_rgb, cv_ptr_depth;
	cv_ptr_rgb = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::RGB8);
	cv_ptr_depth = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_32FC1);
	this->cur_rgb_img = cv_ptr_rgb->image;
	this->cur_depth_img = cv_ptr_depth->image;

	double curr_time = (double)(rgb->header.stamp.sec * 1e6 + rgb->header.stamp.nsec / 1000) / 1000000.0;
	this->rgbd_time = dtos(curr_time);
	this->rgbd_updated = true;

	if(this->rgbd_on == true)
		this->rgbd_image_addline(this->cur_rgb_img, this->cur_depth_img, this->rgbd_time);

	//ROS_INFO_STREAM("    Rgb-d updated");
}

void TopicLogger::stereo_image_callback(const sensor_msgs::ImageConstPtr &left, const sensor_msgs::ImageConstPtr &right)
{
	cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right;
	cv_ptr_left = cv_bridge::toCvCopy(left, sensor_msgs::image_encodings::BGR8);
	cv_ptr_right = cv_bridge::toCvCopy(right, sensor_msgs::image_encodings::BGR8);
	this->cur_left_img = cv_ptr_left->image;
	this->cur_right_img = cv_ptr_right->image;

	double curr_time = (double)(left->header.stamp.sec * 1e6 + left->header.stamp.nsec / 1000) / 1000000.0;
	this->stereo_time = dtos(curr_time);
	this->stereo_updated = true;
	if(this->stereo_on == true)
		this->stereo_image_addline(this->cur_left_img, this->cur_right_img, this->stereo_time);

	//ROS_INFO_STREAM("      Stereo updated");
}

void TopicLogger::imu_callback(const sensor_msgs::ImuConstPtr &msg)
{
	this->current_imu(0, 0) = msg->linear_acceleration.x;
	this->current_imu(1, 0) = msg->linear_acceleration.y;
	this->current_imu(2, 0) = msg->linear_acceleration.z;
	this->current_imu(3, 0) = msg->angular_velocity.x;
	this->current_imu(4, 0) = msg->angular_velocity.y;
	this->current_imu(5, 0) = msg->angular_velocity.z;

	this->current_imu(9, 0)  = msg->orientation.w;
	this->current_imu(10, 0) = msg->orientation.x;
	this->current_imu(11, 0) = msg->orientation.y;
	this->current_imu(12, 0) = msg->orientation.z;


	double curr_time = (double)(msg->header.stamp.sec * 1e6 + msg->header.stamp.nsec / 1000) / 1000000.0;
	this->imu_time = dtos(curr_time);
	this->imu_updated = true;

	if(this->imu_on == true)
		this->imu_addline(this->current_imu, this->imu_time);

	//ROS_INFO_STREAM("        Imu updated");
}

void TopicLogger::magnetic_callback(const sensor_msgs::MagneticFieldConstPtr &msg)
{
	this->current_imu(6, 0) = msg->magnetic_field.x;
	this->current_imu(7, 0) = msg->magnetic_field.y;
	this->current_imu(8, 0) = msg->magnetic_field.z;

	double curr_time = (double)(msg->header.stamp.sec * 1e6 + msg->header.stamp.nsec / 1000) / 1000000.0;
	this->magnetic_time = dtos(curr_time);
	this->magnetic_updated = true;
	//ROS_INFO_STREAM("        Magnetic updated");
}
