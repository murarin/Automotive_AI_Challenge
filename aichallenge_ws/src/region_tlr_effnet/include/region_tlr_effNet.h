#ifndef TRAFFICLIGHT_RECOGNIZER_REGION_TLR_EFFNET_REGION_TLR_EFFNET_H
#define TRAFFICLIGHT_RECOGNIZER_REGION_TLR_EFFNET_REGION_TLR_EFFNET_H

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <autoware_msgs/Signals.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "context.h"
#include "traffic_light_recognizer.h"

class RegionTLREFFNETROSNode
{
public:
  RegionTLREFFNETROSNode();
  ~RegionTLREFFNETROSNode();

  void RunRecognition();
  void ImageRawCallback(const sensor_msgs::Image& image);
  void ROISignalCallback(const autoware_msgs::Signals::ConstPtr& extracted_pos);

  std::vector<Context> contexts_;

private:

  /* Light state transition*/
  const LightState kStateTransitionMatrix[4][4] =
  {
    /* current: */
    /* GREEN   , RED    , UNDEFINED    , YELLOW  */
    /* -------------------------------------------  */
    { GREEN, YELLOW, GREEN, YELLOW },   /* | previous = GREEN */
    { GREEN, RED, RED, RED },           /* | previous = RED */
    { GREEN, RED, UNDEFINED, YELLOW },  /* | previous = UNDEFINED */
    { UNDEFINED, RED, YELLOW, YELLOW }, /* | previous = YELLOW */
  };


  void GetROSParam();
  void StartSubscribersAndPublishers();
  LightState DetermineState(LightState previous_state, LightState current_state, int* state_judge_count);
  void PublishTrafficLight(std::vector<Context> contexts);
  void PublishString(std::vector<Context> contexts);
  void PublishMarkerArray(std::vector<Context> contexts);
  void PublishImage(std::vector<Context> contexts);

  std::string image_topic_name_;
  std::string network_definition_file_name_;
  std::string pretrained_model_file_name_;
  bool use_gpu_;
  int gpu_id_;

  // Subscribers
  ros::Subscriber image_subscriber;
  ros::Subscriber roi_signal_subscriber;

  // Publishers
  ros::Publisher signal_state_publisher;
  ros::Publisher signal_state_string_publisher;
  ros::Publisher marker_publisher;
  ros::Publisher superimpose_image_publisher;


  bool kAdvertiseInLatch_;

  cv::Mat frame_;

  std_msgs::Header frame_header_;

  TrafficLightRecognizer recognizer;

  const int kChangeStateThreshold = 10;

  const int32_t kTrafficLightRed;
  const int32_t kTrafficLightGreen;
  const int32_t kTrafficLightUnknown;
  const std::string kStringRed;
  const std::string kStringGreen;
  const std::string kStringUnknown;
};

#endif  // TRAFFICLIGHT_RECOGNIZER_REGION_TLR_EFFNET_REGION_TLR_EFFNET_H
