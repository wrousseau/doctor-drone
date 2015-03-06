#ifndef WINDOW_INSPECTOR_H
#define WINDOW_INSPECTOR_H

#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


class WindowInspector
{
public:
  cv::Mat m_image;

public :
  WindowInspector();
  ~WindowInspector();

  bool hasWindow(cv::Mat & image);

};


#endif /* WINDOW_INSPECTOR_H */
