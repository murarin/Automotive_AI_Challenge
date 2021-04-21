#include "vision_effDet_detect.h"
#include <nms/nms.h>


EFFDETDetector::EFFDETDetector(const std::string& in_pre_trained_model_file, const float score_threshold, bool in_use_gpu)
{

  cls_threshold = score_threshold;

  //Load the network
  network = torch::jit::load(in_pre_trained_model_file);

  if (in_use_gpu)
  {
    network.to(at::kCUDA);
  }

  //SetMean(in_mean_value);
}

std::vector <  RectClassScore<float>  > EFFDETDetector::Detect(const cv::Mat& img)
{
  //caffe::Blob<float> *input_layer = net_->input_blobs()[0];
  //input_layer->Reshape(1, num_channels_, input_geometry_.height,
                       //input_geometry_.width);
  /* Forward dimension change to all layers. */

  //net_->Reshape();

  //std::vector <cv::Mat> input_channels;
  torch::Tensor inputs;
  //WrapInputLayer(&input_channels);

  Preprocess(img, &inputs);
  //ROS_INFO("%s", inputs.sizes());

  //std::cout << inputs.sizes() << std::endl;

  //net_->Forward();
  auto output = network.forward({inputs}).toTuple();

  torch::Tensor classification = output->elements()[0].toTensor();
  torch::Tensor transformed_anchors = output->elements()[1].toTensor();
  torch::Tensor scores_over_thresh = output->elements()[2].toTensor();
  torch::Tensor scores = output->elements()[3].toTensor();

  std::vector <RectClassScore<float> > detections;

  if (scores_over_thresh.sum().item<int16_t>() != 0)
  {
    torch::Tensor indices = scores_over_thresh.nonzero().view(-1);

    classification = classification.index_select(1, indices);
    transformed_anchors = transformed_anchors.index_select(1, indices);
    scores = scores.index_select(1, indices);

    torch::Tensor anchors_nms_idx = nms(transformed_anchors[0].squeeze(-1), scores[0].squeeze(-1), 0.5);

    auto last_scores_labels = classification[0].index_select(0, anchors_nms_idx).max(1);

    torch::Tensor l_scores = std::get<0>(last_scores_labels);
    torch::Tensor l_labels = std::get<1>(last_scores_labels);

    torch::Tensor boxes = transformed_anchors[0].index_select(0, anchors_nms_idx);

    boxes /= 0.711111;

    for (int box_id = 0; box_id < boxes.sizes()[0]; box_id++)
    {

      float pred_prob = l_scores[box_id].item<float>();

      //std::cout << pred_prob << std::endl;

      if (pred_prob < cls_threshold)
      {
        break;
      }

      // int pred_label = l_labels[box_id].item<int16_t>();
      // int xmin = std::round(boxes[box_id][0].item<float>());
      // int ymin = std::round(boxes[box_id][1].item<float>());
      // int xmax = std::round(boxes[box_id][2].item<float>());
      // int ymax = std::round(boxes[box_id][3].item<float>());

      RectClassScore<float> detection;
      detection.class_type = l_labels[box_id].item<int16_t>();
      detection.score = pred_prob;
      detection.x = std::round(boxes[box_id][0].item<float>());
      detection.y = std::round(boxes[box_id][1].item<float>());
      detection.w = std::round(boxes[box_id][2].item<float>()) - detection.x;
      detection.h = std::round(boxes[box_id][3].item<float>()) - detection.y;

      detection.enabled = true;

      detections.push_back(detection);
    }

  }

  return detections;
}


void EFFDETDetector::Preprocess(const cv::Mat& img,
		torch::Tensor* input_channels)
{

  cv::Mat input_data;
  cv::cvtColor(img, input_data, cv::COLOR_BGR2RGB);

  int w = input_data.cols;
  int h = input_data.rows;
  int resized_w = COMMON_SIZE;
  int resized_h = COMMON_SIZE;
  float scale = 1.0;

  if (h > w) {
    scale = (float)COMMON_SIZE / h;
    resized_h = COMMON_SIZE;
    resized_w = (int)(w * scale);

  }else {
    scale = (float)COMMON_SIZE / w;
    resized_h = (int)(h * scale);
    resized_w = COMMON_SIZE;

  }

  cv::resize(input_data, input_data, cv::Size(resized_w, resized_h));

  cv::Mat brank = cv::Mat::zeros(COMMON_SIZE, COMMON_SIZE, CV_8UC3);

  cv::Mat roi = brank(cv::Rect(0, 0, resized_w, resized_h));
  input_data.copyTo(roi);

  brank.convertTo(input_data, CV_32FC3, 1.0f/255.0f);

  *input_channels = torch::from_blob(input_data.data, {1, input_data.rows, input_data.cols, 3}, at::kFloat).cuda();
  *input_channels = at::transpose(*input_channels, 1, 2);
  *input_channels = at::transpose(*input_channels, 1, 3);

  (*input_channels)[0][0] = (*input_channels)[0][0].sub_(0.485).div_(0.229);
  (*input_channels)[0][1] = (*input_channels)[0][1].sub_(0.456).div_(0.224);
  (*input_channels)[0][2] = (*input_channels)[0][2].sub_(0.406).div_(0.225);

}
