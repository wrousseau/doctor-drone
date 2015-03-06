#include "flying.hpp"

bool Flying::isThereAWindow(int i)
{
	if (i < 50)
		return false;
	return inspector->hasWindow(currentImage);
}

geometry_msgs::Twist Flying::computeTwist(double vxTarget, double vyTarget, double vzTarget, double K)
{
		geometry_msgs::Twist twistMsgGen;
	
		twistMsgGen.linear.x = K*(vxTarget - vx);
		twistMsgGen.linear.y = K*(vyTarget - vy); 
		twistMsgGen.linear.z = K*(vzTarget - vz);
		twistMsgGen.angular.x = 0.0; 
		twistMsgGen.angular.y = 0.0;
		twistMsgGen.angular.z = 0.0;
		return twistMsgGen;
}

void Flying::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      currentImage = cv_ptr->image;
      //grayscaleImage = cv::Mat::zeros(cv_ptr->image.size(), CV_8U);
      //cv::cvtColor(cv_ptr->image, grayscaleImage, CV_BGR2GRAY);
      //double gradient = opticalFlowEqualizer->getOpticalFlowGradient(grayscaleImage);
      //ROS_INFO("gradient : %f", gradient);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

void Flying::navigationCallback(const ardrone_autonomy::Navdata& msg)
{
	vx = msg.vx * 0.001;
	vy = msg.vy * 0.001;	
	vz = msg.vz * 0.001;	
}