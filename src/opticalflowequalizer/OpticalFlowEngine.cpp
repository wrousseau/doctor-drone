#include "OpticalFlowEngine.h"


OpticalFlowEngine::OpticalFlowEngine(const cv::Rect & roi) : m_roi(roi)
{
  m_currentImage = cv::Mat::zeros(m_roi.height, m_roi.width, CV_8U);
}

OpticalFlowEngine::~OpticalFlowEngine()
{

}

void OpticalFlowEngine::updateCurrentImage(const cv::Mat & image)
{
  m_previousImage = m_currentImage;
  m_currentImage = image(m_roi).clone();
}

void OpticalFlowEngine::computeOpticalFlow()
{
  cv::calcOpticalFlowFarneback( m_previousImage, m_currentImage, m_opticalFlow, 0.1, 1, 21, 30, 5, 1.1, cv::OPTFLOW_FARNEBACK_GAUSSIAN );
}

double OpticalFlowEngine::getMeanValue()
{

  cv::Point2f flowPoint;
  double mean = 0.0;
  for (unsigned int y = 0; y < m_opticalFlow.rows; ++y)
  {
    for (unsigned int x = 0; x < m_opticalFlow.cols; ++x)
    {
      try
      {
        flowPoint = OpticalFlowEngine::cart2Polar(m_opticalFlow.at<cv::Point2f>(y,x));
      }
      catch  (const cv::Exception &e)
      {
        std::cout << e.what() << std::endl;
      }
      mean += flowPoint.x;
    }
  }

  mean /= (double)m_opticalFlow.size().area();
  return mean;

}

cv::Point2f OpticalFlowEngine::cart2Polar(const cv::Point2f &cartPoint)
{
  cv::Point2f polarPoint;
  polarPoint.x = cartPoint.x * cartPoint.x;
  polarPoint.x += cartPoint.y * cartPoint.y;
  polarPoint.x = sqrt(polarPoint.x);

  polarPoint.y = atan2(cartPoint.y, cartPoint.x);

  return polarPoint;
}
