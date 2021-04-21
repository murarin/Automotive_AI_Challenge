#include "region_tlr_effNet.h"
#include "context.h"

#include <string>
#include <vector>

#include <autoware_msgs/TrafficLight.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

RegionTLREFFNETROSNode::RegionTLREFFNETROSNode()
  : image_topic_name_("/image_raw")
  , network_definition_file_name_("")
  , pretrained_model_file_name_("")
  , use_gpu_(false)
  , gpu_id_(0)
  , kAdvertiseInLatch_(true)
  , kTrafficLightRed(0)
  , kTrafficLightGreen(1)
  , kTrafficLightUnknown(2)
  , kStringRed("red signal")
  , kStringGreen("green signal")
  , kStringUnknown("")
{
}  // RegionTLREFFNETROSNode::RegionTLREFFNETROSNode()

RegionTLREFFNETROSNode::~RegionTLREFFNETROSNode()
{
}  // RegionTLREFFNETROSNode::~RegionTLREFFNETROSNode()

void RegionTLREFFNETROSNode::RunRecognition()
{
  GetROSParam();

  recognizer.Init(pretrained_model_file_name_, use_gpu_);

  StartSubscribersAndPublishers();
  ros::spin();
}  // RegionTLREFFNETROSNode::RunRecognition()


void RegionTLREFFNETROSNode::ImageRawCallback(const sensor_msgs::Image& image)
{
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  frame_ = cv_image->image.clone();

  frame_header_ = image.header;
}


void RegionTLREFFNETROSNode::ROISignalCallback(const autoware_msgs::Signals::ConstPtr& extracted_pos)
{
  static ros::Time previous_timestamp;
  if (frame_.empty() || frame_header_.stamp == previous_timestamp)
  {
    std::cout << "No Image" << std::endl;
    return;
  }
  // std::cout << "rois: " << extracted_pos->Signals.size() << std::endl;

  Context::SetContexts(&contexts_, extracted_pos, frame_.rows, frame_.cols);

  for (Context& context : contexts_)
  {
    // for (unsigned int i = 0; i < contexts_.size(); i++) {
    //   Context& context = contexts_.at(i);
    if (context.topLeft.x > context.botRight.x)
    {
      continue;
    }

    // std::cout << "roi inside: " << cv::Rect(context.topLeft, context.botRight) << std::endl;
    // extract region of interest from input image
    cv::Mat roi = frame_(cv::Rect(context.topLeft, context.botRight)).clone();

    // cv::imshow("effNet_tlr", roi);
    //  cv::waitKey(200);

    LightState current_state = recognizer.RecognizeLightState(roi);

    context.lightState =
        DetermineState(context.lightState,           // previous state
                       current_state,                // current state
                       &(context.stateJudgeCount));  // counter to record how many times does state recognized
  }

  PublishTrafficLight(contexts_);
  PublishString(contexts_);
  PublishMarkerArray(contexts_);
  PublishImage(contexts_);

  previous_timestamp = frame_header_.stamp;
}


void RegionTLREFFNETROSNode::GetROSParam()
{
  ros::NodeHandle private_node_handle("~");

  private_node_handle.param<std::string>("image_raw_topic", image_topic_name_, "/image_raw");
  private_node_handle.param<std::string>("pretrained_model_file", pretrained_model_file_name_, "");
  private_node_handle.param<bool>("use_gpu", use_gpu_, false);

  if (pretrained_model_file_name_.empty())
  {
    ROS_FATAL("No Pretrained Model File was specified. Terminate program... ");
    exit(EXIT_FAILURE);
  }
}  // RegionTLREFFNETROSNode::ProcessROSParam()


void RegionTLREFFNETROSNode::StartSubscribersAndPublishers()
{
  ros::NodeHandle node_handle;

  // subscribers
  image_subscriber = node_handle.subscribe(image_topic_name_, 1, &RegionTLREFFNETROSNode::ImageRawCallback, this);
  roi_signal_subscriber = node_handle.subscribe("/roi_signal", 1, &RegionTLREFFNETROSNode::ROISignalCallback, this);

  // publishers
  signal_state_publisher = node_handle.advertise<autoware_msgs::TrafficLight>("light_color", 1);
  signal_state_string_publisher = node_handle.advertise<std_msgs::String>("/sound_player", 1);
  marker_publisher = node_handle.advertise<visualization_msgs::MarkerArray>("tlr_result", 1, kAdvertiseInLatch_);
  superimpose_image_publisher = node_handle.advertise<sensor_msgs::Image>("tlr_superimpose_image", 1);
}  // RegionTLREFFNETROSNode::StartSubscribersAndPublishers()


LightState RegionTLREFFNETROSNode::DetermineState(LightState previous_state, LightState current_state,
                                               int* state_judge_count)
{
  LightState transition_candidate = kStateTransitionMatrix[previous_state][current_state];

  //TODO Review
  if (*state_judge_count > kChangeStateThreshold)
  {
    *state_judge_count = 0;
    return transition_candidate;
  }
  else
  {
    if (transition_candidate != previous_state)
    {
      (*state_judge_count)++;
    }
    return previous_state;
  }


}  // LightState RegionTLREFFNETROSNode::DetermineState()

void RegionTLREFFNETROSNode::PublishTrafficLight(std::vector<Context> contexts)
{
  autoware_msgs::TrafficLight topic;
  static int32_t previous_state = kTrafficLightUnknown;
  topic.traffic_light = kTrafficLightUnknown;
  for (const auto ctx : contexts)
  {
    switch (ctx.lightState)
    {
      case GREEN:
        topic.traffic_light = kTrafficLightGreen;
        break;
      case YELLOW:
      case RED:
        topic.traffic_light = kTrafficLightRed;
        break;
      case UNDEFINED:
        topic.traffic_light = kTrafficLightUnknown;
        break;
    }

    if (topic.traffic_light != kTrafficLightUnknown)
    {
      break;
    }
  }

  if (topic.traffic_light != previous_state)
  {
    signal_state_publisher.publish(topic);
    previous_state = topic.traffic_light;
  }
}  // void RegionTLREFFNETROSNode::PublishTrafficLight()


void RegionTLREFFNETROSNode::PublishString(std::vector<Context> contexts)
{
  std_msgs::String topic;
  static std::string previous_state = kStringUnknown;
  topic.data = kStringUnknown;
  for (const auto ctx : contexts)
  {
    switch (ctx.lightState)
    {
      case GREEN:
        topic.data = kStringGreen;
        break;
      case YELLOW:
      case RED:
        topic.data = kStringRed;
        break;
      case UNDEFINED:
        topic.data = kStringUnknown;
        break;
    }


    if (topic.data != kStringUnknown)
    {
      break;
    }
  }

  if (topic.data != previous_state)
  {
    signal_state_string_publisher.publish(topic);
    previous_state = topic.data;
  }
}  // void RegionTLREFFNETROSNode::PublishString()


void RegionTLREFFNETROSNode::PublishMarkerArray(std::vector<Context> contexts)
{
  // Define color constants
  std_msgs::ColorRGBA color_black;
  color_black.r = 0.0f;
  color_black.g = 0.0f;
  color_black.b = 0.0f;
  color_black.a = 1.0f;

  std_msgs::ColorRGBA color_red;
  color_red.r = 1.0f;
  color_red.g = 0.0f;
  color_red.b = 0.0f;
  color_red.a = 1.0f;

  std_msgs::ColorRGBA color_yellow;
  color_yellow.r = 1.0f;
  color_yellow.g = 1.0f;
  color_yellow.b = 0.0f;
  color_yellow.a = 1.0f;

  std_msgs::ColorRGBA color_green;
  color_green.r = 0.0f;
  color_green.g = 1.0f;
  color_green.b = 0.0f;
  color_green.a = 1.0f;

  for (const auto ctx : contexts)
  {
    visualization_msgs::MarkerArray signal_set;
    visualization_msgs::Marker red_light, yellow_light, green_light;

    red_light.header.frame_id = "map";
    yellow_light.header.frame_id = "map";
    green_light.header.frame_id = "map";

    red_light.ns = "tlr_result_red";
    red_light.id = ctx.signalID;

    yellow_light.ns = "tlr_result_yellow";
    yellow_light.id = ctx.signalID;

    green_light.ns = "tlr_result_green";
    green_light.id = ctx.signalID;

    red_light.type = visualization_msgs::Marker::SPHERE;
    yellow_light.type = visualization_msgs::Marker::SPHERE;
    green_light.type = visualization_msgs::Marker::SPHERE;

    red_light.pose.position.x = ctx.redCenter3d.x;
    red_light.pose.position.y = ctx.redCenter3d.y;
    red_light.pose.position.z = ctx.redCenter3d.z;
    red_light.pose.orientation.x = 0.0;
    red_light.pose.orientation.y = 0.0;
    red_light.pose.orientation.z = 0.0;
    red_light.pose.orientation.w = 0.0;

    yellow_light.pose.position.x = ctx.yellowCenter3d.x;
    yellow_light.pose.position.y = ctx.yellowCenter3d.y;
    yellow_light.pose.position.z = ctx.yellowCenter3d.z;
    yellow_light.pose.orientation.x = 0.0;
    yellow_light.pose.orientation.y = 0.0;
    yellow_light.pose.orientation.z = 0.0;
    yellow_light.pose.orientation.w = 0.0;

    green_light.pose.position.x = ctx.greenCenter3d.x;
    green_light.pose.position.y = ctx.greenCenter3d.y;
    green_light.pose.position.z = ctx.greenCenter3d.z;
    green_light.pose.orientation.x = 0.0;
    green_light.pose.orientation.y = 0.0;
    green_light.pose.orientation.z = 0.0;
    green_light.pose.orientation.w = 0.0;

    red_light.scale.x = 0.3;
    red_light.scale.y = 0.3;
    red_light.scale.z = 0.3;

    yellow_light.scale.x = 0.3;
    yellow_light.scale.y = 0.3;
    yellow_light.scale.z = 0.3;

    green_light.scale.x = 0.3;
    green_light.scale.y = 0.3;
    green_light.scale.z = 0.3;

    switch (ctx.lightState)
    {
      case GREEN:
        red_light.color = color_black;
        yellow_light.color = color_black;
        green_light.color = color_green;
        break;
      case YELLOW:
        red_light.color = color_black;
        yellow_light.color = color_yellow;
        green_light.color = color_black;
        break;
      case RED:
        red_light.color = color_red;
        yellow_light.color = color_black;
        green_light.color = color_black;
        break;
      case UNDEFINED:
        red_light.color = color_black;
        yellow_light.color = color_black;
        green_light.color = color_black;
        break;
    }

    red_light.lifetime = ros::Duration(0.1);
    yellow_light.lifetime = ros::Duration(0.1);
    green_light.lifetime = ros::Duration(0.1);

    // Pack each light marker into one
    signal_set.markers.push_back(red_light);
    signal_set.markers.push_back(yellow_light);
    signal_set.markers.push_back(green_light);

    // Publish
    marker_publisher.publish(signal_set);
  }
}  // void RegionTLREFFNETROSNode::PublishMarkerArray()

void RegionTLREFFNETROSNode::PublishImage(std::vector<Context> contexts)
{

  cv::Mat result_image = frame_.clone();

  std::string label;
  const int kFontFace = cv::FONT_HERSHEY_COMPLEX_SMALL;
  const double kFontScale = 0.8;
  int font_baseline = 0;
  CvScalar label_color;

  for (const auto ctx : contexts_)
  {
    // Draw superimpose result
    circle(result_image, ctx.redCenter, ctx.lampRadius, CV_RGB(255, 0, 0), 1, 0);
    circle(result_image, ctx.yellowCenter, ctx.lampRadius, CV_RGB(255, 255, 0), 1, 0);
    circle(result_image, ctx.greenCenter, ctx.lampRadius, CV_RGB(0, 255, 0), 1, 0);

    // Draw recognition result
    switch (ctx.lightState)
    {
      case GREEN:
        label = "GREEN";
        label_color = CV_RGB(0, 255, 0);
        break;
      case YELLOW:
        label = "YELLOW";
        label_color = CV_RGB(255, 255, 0);
        break;
      case RED:
        label = "RED";
        label_color = CV_RGB(255, 0, 0);
        break;
      case UNDEFINED:
        label = "UNKNOWN";
        label_color = CV_RGB(0, 0, 0);
    }

    if (ctx.leftTurnSignal)
    {
      label += " LEFT";
    }
    if (ctx.rightTurnSignal)
    {
      label += " RIGHT";
    }
    label += " " + std::to_string(ctx.closestLaneId);

    cv::Point label_origin = cv::Point(ctx.topLeft.x, ctx.botRight.y + font_baseline);

    cv::putText(result_image, label, label_origin, kFontFace, kFontScale, label_color);
  }

  // Publish　result image
  cv_bridge::CvImage converter;
  converter.header = frame_header_;
  converter.encoding = sensor_msgs::image_encodings::BGR8;
  converter.image = result_image;
  superimpose_image_publisher.publish(converter.toImageMsg());
}  // void RegionTLREFFNETROSNode::PublishImage()


int main(int argc, char* argv[])
{
  // Initialize node
  ros::init(argc, argv, "region_tlr_effNet");

  RegionTLREFFNETROSNode region_tlr_effNet_ros_node;

  // recognition
  region_tlr_effNet_ros_node.RunRecognition();

  return 0;
}  // main()
