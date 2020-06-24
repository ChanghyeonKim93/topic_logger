
#include <ros/ros.h>

#include <TopicLogger_kch.h>

#include <iostream>
#include <sys/time.h>
#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "topic_logger_node");
	
	// Generate a topic logger class.
	TopicLogger *topic_logger = new TopicLogger();

    while (ros::ok())
		ros::spinOnce(); // so fast

	delete topic_logger;
	return 0;
}
