#ifndef WINDOW_INSPECTOR_H
#define WINDOW_INSPECTOR_H

#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "WindowInspectorCoach.h"

enum InspectionMethod { CONTOURS, SVM_COACHED };

class WindowInspector
{
  InspectionMethod m_method;
  cv::Mat m_image;
  WindowInspectorCoach *m_coach;

public :
  WindowInspector(InspectionMethod method = CONTOURS);
  ~WindowInspector();

  bool hasWindow(cv::Mat & image);

};


#endif /* WINDOW_INSPECTOR_H */
