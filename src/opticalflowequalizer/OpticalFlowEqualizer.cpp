#include "OpticalFlowEqualizer.h"

#include <stdio.h>
#include <iostream>



OpticalFlowEqualizer::OpticalFlowEqualizer(const cv::Size & imageSize)
{

  mp_leftOFEngine = new OpticalFlowEngine( cv::Rect(0, 0, imageSize.width/4, imageSize.height) );
  mp_rightOFEngine = new OpticalFlowEngine( cv::Rect(3*imageSize.width/4, 0, imageSize.width/4, imageSize.height) );

}

OpticalFlowEqualizer::~OpticalFlowEqualizer()
{
  delete mp_leftOFEngine;
  delete mp_rightOFEngine;
}

double OpticalFlowEqualizer::getOpticalFlowGradient(const cv::Mat & nextImage)
{
  mp_leftOFEngine->updateCurrentImage(nextImage);
  mp_rightOFEngine->updateCurrentImage(nextImage);

  mp_leftOFEngine->computeOpticalFlow();
  mp_rightOFEngine->computeOpticalFlow();

  return fabs(mp_leftOFEngine->getMeanValue() - mp_rightOFEngine->getMeanValue());
}
