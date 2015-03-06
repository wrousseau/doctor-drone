#include "WindowInspector.h"


using namespace std;
using namespace cv;

#define MIN_WIN_AREA 25000
#define MAX_WIN_AREA 55000

WindowInspector::WindowInspector(InspectionMethod method) :
m_method(method)
{
  if(m_method == SVM_COACHED){
    m_coach = new WindowInspectorCoach();
  }
}

WindowInspector::~WindowInspector()
{

}

bool WindowInspector::hasWindow(cv::Mat & image)
{
  if (m_method == SVM_COACHED) {
    /* Support Vector Machine Method */

    m_coach->predictWindowPresence(image);
    return false;
  }
  else
  {
    /* Find window contours method */

    // Binarize image
    Mat grayImage;
    Mat bImage;
    cvtColor(image, grayImage, CV_BGR2GRAY);
    Canny(grayImage, bImage, 0, 50, 5);

    // Detect contours
    vector<vector<Point> > contours;
    findContours(bImage.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (unsigned int k = 0; k < contours.size(); ++k)
    {
      RotatedRect rect = minAreaRect(contours[k]);
      if (rect.size.area() < MIN_WIN_AREA || rect.size.area() > MAX_WIN_AREA || rect.size.height > rect.size.width || rect.angle > -45 ) {
        contours.erase(contours.begin() + k);
      } else {
        rectangle(image, rect.boundingRect(), Scalar(0,0,255));
        return true;
      }
    }

    return false;
  }
}
