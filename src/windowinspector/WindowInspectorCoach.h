#ifndef WINDOW_INSPECTOR_COACH_H
#define WINDOW_INSPECTOR_COACH_H

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/ocl/ocl.hpp"

class WindowInspectorCoach
{
  enum TrainingDataStatus { POSITIVE, NEGATIVE };

  std::string m_dirName;

  cv::Size m_imageSize;
  unsigned int m_descriptorsSize;
  cv::Mat m_trainingData;
  cv::Mat m_trainingLabels;

  cv::HOGDescriptor *m_hog;
  std::vector<float> m_descriptors;
  CvSVM m_svm;
  bool m_trained;

public :
  WindowInspectorCoach();
  ~WindowInspectorCoach();

  void initTrainingDataset(TrainingDataStatus status);
  bool train();
  bool predictWindowPresence(cv::Mat & image);

};


#endif /* WINDOW_INSPECTOR_COACH_H */
