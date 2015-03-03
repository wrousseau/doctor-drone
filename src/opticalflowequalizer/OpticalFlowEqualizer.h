#ifndef OPTICAL_FLOW_EQUALIZER_H
#define OPTICAL_FLOW_EQUALIZER_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "OpticalFlowEngine.h"


class OpticalFlowEqualizer
{

  OpticalFlowEngine *mp_leftOFEngine;
  OpticalFlowEngine *mp_rightOFEngine;

public :
  OpticalFlowEqualizer(const cv::Size & imageSize);
  ~OpticalFlowEqualizer();

  double getOpticalFlowGradient(const cv::Mat & nextImage);

};


#endif /* OPTICAL_FLOW_EQUALIZER_H */
