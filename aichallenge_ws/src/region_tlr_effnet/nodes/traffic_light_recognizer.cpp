#include "traffic_light_recognizer.h"

#include <string>
#include <vector>

TrafficLightRecognizer::TrafficLightRecognizer()
{
}  // TrafficLightRecognizer::TrafficLightRecognizer()


TrafficLightRecognizer::~TrafficLightRecognizer()
{
}  // TrafficLightRecognizer::~TrafficLightRecognizer()


void TrafficLightRecognizer::Init(const std::string& pretrained_model_file_name, const bool use_gpu)
{

  // Load the network
  network = torch::jit::load(pretrained_model_file_name);

  if (use_gpu)
  {
    network.to(at::kCUDA);
  }

  // SetMean(kPixelMean_);
}  // void TrafficLightRecognizer::Init()

LightState TrafficLightRecognizer::RecognizeLightState(const cv::Mat& image)
{

  at::Tensor inputs;

  Preprocess(image, &inputs);

  // Run EFFNET recognition
  at::Tensor output = at::softmax(network.forward({inputs}).toTensor(), 1);
  auto pred = at::argmax(output, 1, true);
  int label = pred.item<int64_t>();
  float prob = output[0][label].item<float_t>();

  float max_score = 0;
  LightState result = LightState::UNDEFINED;

  if (max_score < prob)
  {
    result = static_cast<LightState>(label);
  }

  return result;

}  // void TrafficLightRecognizer::RecognizeLightState()


void TrafficLightRecognizer::Preprocess(const cv::Mat& image, at::Tensor* input_channels)
{

  cv::Mat input_data;
  cv::cvtColor(image, input_data, cv::COLOR_BGR2RGB);

  int w = input_data.cols;
  int h = input_data.rows;

  if (h > HEIGHT)
  {
    cv::resize(input_data, input_data, cv::Size(), (float)HEIGHT / h, (float)HEIGHT / h);
    w = input_data.cols;
    h = input_data.rows;

  }

  if (w > WIDTH)
  {
    cv::resize(input_data, input_data, cv::Size(), (float)WIDTH / w, (float)WIDTH / w);
    w = input_data.cols;
    h = input_data.rows;

  }

  if (h < HEIGHT || w < WIDTH)
  {
    cv::Mat brank = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    int left_x = std::nearbyint(WIDTH / 2.0 - w / 2.0);
    int left_y = std::nearbyint(HEIGHT / 2.0 - h / 2.0);

    cv::Mat roi_roi = brank(cv::Rect(left_x, left_y, input_data.cols, input_data.rows));
    input_data.copyTo(roi_roi);

    input_data = brank;

  }

  input_data.convertTo(input_data, CV_32FC3, 1.0f / 255.0f);

  input_data = (input_data - 0.5) / 0.5;

  *input_channels = torch::from_blob(input_data.data, {1, input_data.rows, input_data.cols, 3}, at::kFloat).cuda();
  *input_channels = at::transpose(*input_channels, 1, 2);
  *input_channels = at::transpose(*input_channels, 1, 3);

}  // void TrafficLightRecognizer::Preprocess()
