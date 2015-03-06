#include "WindowInspectorCoach.h"


#include <dirent.h>

using namespace std;
using namespace cv;

WindowInspectorCoach::WindowInspectorCoach() :
m_dirName("../../data/training_data"),
m_trained(false)
{
  m_hog = new HOGDescriptor(cvSize(9,9),cvSize(9,9),cvSize(9,9),cvSize(9,9),8);

  initTrainingDataset(POSITIVE);
  initTrainingDataset(NEGATIVE);
  cout << "Training data size : " << m_trainingData.size() << endl;
  cout << "Training labels size : " << m_trainingLabels.size() << endl;

  cout << "Training SVM ..."<< endl;
  train();
}

WindowInspectorCoach::~WindowInspectorCoach()
{
  delete m_hog;
}

void WindowInspectorCoach::initTrainingDataset(TrainingDataStatus status)
{
  string dirName = m_dirName;
  int label;
  switch(status)
  {
    case POSITIVE: dirName.append("/positives"); label = 1; break;
    case NEGATIVE: dirName.append("/negatives"); label = -1; break;
    default: break;
  }

  DIR *dir = opendir(dirName.c_str());
  string filePathName;
  struct dirent *ent;
  Mat trainingDataBuffer;

  Mat labels;
  m_descriptorsSize = 0;


  if (dir != NULL) {
    while ((ent = readdir (dir)) != NULL) {
        filePathName = dirName;
        filePathName.append("/");
        filePathName.append(ent->d_name);
        Mat image = imread(filePathName);
        if (!image.empty())
        {
          if (m_imageSize.area() == 0)
            m_imageSize = image.size();
          cvtColor(image,image,CV_RGB2GRAY);
          resize(image, image, m_imageSize);

          // Compute descriptors
          m_descriptors.clear();

          m_hog->compute(image, m_descriptors, Size(1,1), Size(0,0));
          if (m_descriptorsSize == 0)
            m_descriptorsSize = m_descriptors.size();
          // Set labels and training data
          labels = Mat(1, m_descriptorsSize, CV_32F, (float)label);
          Mat features = Mat( 1, m_descriptorsSize, CV_32F, m_descriptors.data() );
          m_trainingData.push_back(features);
          m_trainingLabels.push_back(label);
        }
      }
      closedir (dir);
  } else {
      cout << "Unable to open directory named : " << dirName << endl;
  }
  cout << "Input images loaded"<< endl;
}

bool WindowInspectorCoach::train()
{
  m_trained = m_svm.train(m_trainingData, m_trainingLabels);
  if (m_trained)
      cout << "Support Vector Machine trained !" << endl;
  else
    cout << "Error: Failed to train Support Vector Machine" << endl;

  return m_trained;
}

bool WindowInspectorCoach::predictWindowPresence(Mat & image)
{
  if (!m_trained)
    train();

  if (!m_trained)
  {
    cout << "Error: Failed to train Support Vector Machine" << endl;
    return false;
  }


  Mat grayImage;
  cvtColor(image, grayImage, CV_BGR2GRAY);
  resize(grayImage, grayImage, m_imageSize);
  m_descriptors.clear();
  m_hog->compute(grayImage, m_descriptors, cv::Size(1,1), cv::Size(0,0));
  Mat features = cv::Mat( 1, m_descriptors.size(), CV_32F, m_descriptors.data() );
  return m_svm.predict(features) == 1;
}
