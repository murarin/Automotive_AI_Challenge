#ifndef EFFDET_DETECTOR_H_
#define EFFDET_DETECTOR_H_

#include <algorithm>
#include <iomanip>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>

//#include <caffe/caffe.hpp>
#include <torch/torch.h>
#include <torch/script.h>
//#include <nms/nms.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "rect_class_score.h"

namespace EFFDET
{
	enum EFFDETDetectorClasses
	{
		BACKGROUND,
		PLANE, BICYCLE, BIRD, BOAT,
		BOTTLE, BUS, CAR, CAT, CHAIR,
		COW, TABLE, DOG, HORSE,
		MOTORBIKE, PERSON, PLANT,
		SHEEP, SOFA, TRAIN, TV, NUM_CLASSES
	};
}

class EFFDETDetector
{
public:
	EFFDETDetector(const std::string& in_pre_trained_model_file, const float score_threshold, bool in_use_gpu);

	std::vector <  RectClassScore<float>  > Detect(const cv::Mat& img);

private:
  //void SetMean(const cv::Scalar &in_mean_value);

  //void WrapInputLayer(std::vector<cv::Mat> *input_channels);

  void Preprocess(const cv::Mat &img, torch::Tensor* input_channels);

private:
  //boost::shared_ptr <caffe::Net<float> > net_;
  torch::jit::script::Module network;

  //cv::Size input_geometry_;
  //int num_channels_;
  //cv::Scalar mean_;

  float cls_threshold;

  const int COMMON_SIZE = 512;
};

#endif //EFFDET_DETECTOR_H
