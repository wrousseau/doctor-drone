#include "photographing.hpp"
#include "utils.hpp"
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


#include <string>

void Photographing::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	std::stringstream ss;
	ss << "../../photos/" << Utils::getExperimentTimestamp() << "/" << Utils::getPictureNumber() << ".jpg";
	std::string path = ss.str();
    try
    {
      	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv::imwrite( path, cv_ptr->image );
		hasTakenPicture = true;
    }
    catch (cv_bridge::Exception& e)
    {
      	ROS_ERROR("cv_bridge exception: %s", e.what());
      	return;
    }
}
