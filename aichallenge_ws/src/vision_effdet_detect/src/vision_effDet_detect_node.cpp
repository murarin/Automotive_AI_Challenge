#include <string>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
//#include "autoware_config_msgs/ConfigSSD.h"
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

#include <rect_class_score.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#if (CV_MAJOR_VERSION <= 2)

#include <opencv2/contrib/contrib.hpp>

#else
//#include "gencolors.cpp"
#endif

#include "vision_effDet_detect.h"

class ROSEFFDETApp
{
  ros::Subscriber subscriber_image_raw_;
  //ros::Subscriber subscriber_ssd_config_;
  ros::Publisher publisher_detected_objects_;
  ros::NodeHandle node_handle_;

  //cv::Scalar pixel_mean_;
  //std::vector<cv::Scalar> colors_;

  EFFDETDetector* effDet_detector_;

  float score_threshold_;

  //unsigned int gpu_device_id_;

  bool use_gpu_;

  std::vector<unsigned int> detect_classes_;

  void convert_rect_to_detected_object(std::vector< RectClassScore<float> >& in_objects,
                                       autoware_msgs::DetectedObjectArray &out_message,
                                       cv::Mat& in_image)
  {
    for (unsigned int i = 0; i < in_objects.size(); ++i)
    {
      //if (in_objects[i].score >= score_threshold_)
      //{
      autoware_msgs::DetectedObject obj;
      obj.header = out_message.header;
      obj.label = in_objects[i].GetClassString();
      obj.score = in_objects[i].score;

      obj.color.r = 1.0f;
      obj.color.g = 0.0f;
      obj.color.b = 0.0f;
      obj.color.a = 1.0f;

      obj.image_frame = out_message.header.frame_id;
      obj.x = in_objects[i].x;
      obj.y = in_objects[i].y;
      obj.width = in_objects[i].w;
      obj.height = in_objects[i].h;
      obj.valid = true;

      //obj.roi_image = in_image(cv::Rect(obj.x, obj.y, obj.width, obj.height));

      out_message.objects.push_back(obj);
      //}
    }
  }

  void image_callback(const sensor_msgs::Image &image_source)
  {
    //Receive Image, convert it to OpenCV Mat
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source,
                                                         "bgr8");//toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_image->image;

    //Detect Object in image
    std::vector<RectClassScore < float> > detections;
    //cv::TickMeter timer; timer.start();
    //std::cout << "score:" << score_threshold_ << " slices:" << image_slices_ << " slices overlap:" << slices_overlap_ << "nms" << group_threshold_ << std::endl;
    detections = effDet_detector_->Detect(image);
    //timer.stop();
    //std::cout << "Detection took: " << timer.getTimeMilli() << std::endl;

    //timer.reset(); timer.start();
    autoware_msgs::DetectedObjectArray output_detected_message;
    output_detected_message.header = image_source.header;
    convert_rect_to_detected_object(detections, output_detected_message, image);

    publisher_detected_objects_.publish(output_detected_message);
  }


  //void config_cb(const autoware_config_msgs::ConfigSSD::ConstPtr& param)
  //{
  //  score_threshold_ 	= param->score_threshold;
  //}

public:
  void Run()
  {
    //ROS STUFF
    ros::NodeHandle private_node_handle("~");//to receive args

    //RECEIVE IMAGE TOPIC NAME
    std::string image_raw_topic_str;
    if (private_node_handle.getParam("image_raw_node", image_raw_topic_str))
    {
      ROS_INFO("Setting image node to %s", image_raw_topic_str.c_str());
    } else
    {
      ROS_INFO("No image node received, defaulting to /image_raw, you can use _image_raw_node:=YOUR_TOPIC");
      image_raw_topic_str = "/image_raw";
    }

    std::string pretrained_model_file;
    if (private_node_handle.getParam("pretrained_model_file", pretrained_model_file))
    {
      ROS_INFO("Pretrained Model File: %s", pretrained_model_file.c_str());
    } else
    {
      ROS_INFO("No Pretrained Model File was received. Finishing execution.");
      return;
    }

    if (private_node_handle.getParam("score_threshold", score_threshold_))
    {
      ROS_INFO("Score Threshold: %f", score_threshold_);
    }

    if (private_node_handle.getParam("use_gpu", use_gpu_))
    {
      ROS_INFO("GPU Mode: %d", use_gpu_);
    }

    //EFFDET STUFF
    effDet_detector_ = new EFFDETDetector(pretrained_model_file, score_threshold_, use_gpu_);

    if (NULL == effDet_detector_)
    {
      ROS_INFO("Error while creating effDet Object");
      return;
    }
    ROS_INFO("effDet Detector initialized.");

// #if (CV_MAJOR_VERSION <= 2)
//     cv::generateColors(colors_, 20);
// #else
//     generateColors(colors_, 20);
// #endif

    publisher_detected_objects_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>(
      "/detection/image_detector/objects", 1);

    ROS_INFO("Subscribing to... %s", image_raw_topic_str.c_str());
    subscriber_image_raw_ = node_handle_.subscribe(image_raw_topic_str, 1, &ROSEFFDETApp::image_callback, this);

    //std::string config_topic("/config");
    //config_topic += "/effDet";
    //subscriber_effDet_config_ = node_handle_.subscribe(config_topic, 1, &ROSEFFDETApp::config_cb, this);

    ros::spin();
    ROS_INFO("END effDet");

  }

  ~ROSEFFDETApp()
  {
    if (NULL != effDet_detector_)
      delete effDet_detector_;
  }

  ROSEFFDETApp()
  {
    effDet_detector_ 	= NULL;
    score_threshold_= 0.5;
    use_gpu_ 		= false;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "effDet_unc");

  ROSEFFDETApp app;

	app.Run();

  return 0;
}
