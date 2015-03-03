#ifndef OPTICAL_FLOW_ENGINE_H
#define OPTICAL_FLOW_ENGINE_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


class OpticalFlowEngine
{

  cv::Rect m_roi;
  cv::Size m_size;

  cv::Mat m_currentImage;
  cv::Mat m_previousImage;

  cv::Mat m_opticalFlow;
  //cv::Mat m_opticalFlowImage;

public :
  OpticalFlowEngine(const cv::Rect & roi);
  ~OpticalFlowEngine();

  void updateCurrentImage(const cv::Mat & image);
  void computeOpticalFlow();

  double getMeanValue();

  static cv::Point2f cart2Polar(const cv::Point2f &cartPoint);

};


#endif /* OPTICAL_FLOW_ENGINE_H */
