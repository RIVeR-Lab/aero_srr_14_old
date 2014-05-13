#ifndef CASCADE_CLASSIFIER_H_
#define CASCADE_CLASSIFIER_H_

//#define USE_GPU 1

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#ifdef USE_GPU
#include <opencv2/gpu/gpu.hpp>
#endif

#include <time.h>
#include <image_geometry/stereo_camera_model.h>
#include <stereo_msgs/DisparityImage.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/shared_ptr.hpp>

#include <tf/transform_listener.h>

namespace vision{

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;
using namespace boost;

#ifdef USE_GPU
  typedef cv::gpu::GpuMat InternalMat;
#else
  typedef cv::Mat InternalMat;
#endif

class CascadeClassifier{
 private:
  typedef ApproximateTime<Image, Image, CameraInfo, CameraInfo> ImageSyncPolicy;
  typedef message_filters::Synchronizer<ImageSyncPolicy> ImageSynchronizer;

 public:
  CascadeClassifier(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~CascadeClassifier();

 private:
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter sub_l_image_;
  image_transport::SubscriberFilter sub_r_image_;
  message_filters::Subscriber<CameraInfo> sub_l_info_;
  message_filters::Subscriber<CameraInfo> sub_r_info_;
  shared_ptr<ImageSynchronizer> image_synchronizer_;

  image_geometry::StereoCameraModel stereo_model_;

  tf::TransformListener tf_listener_;

  ros::Publisher disparity_pub_;

  ros::Publisher object_detection_pub_;
  image_transport::Publisher object_detection_disparity_pub_;
  image_transport::Publisher object_detection_image_pub_;

#ifdef USE_GPU
  cv::gpu::CascadeClassifier_GPU cascade_classifier_;
  cv::gpu::StereoBM_GPU block_matcher_;
#else
  cv::CascadeClassifier cascade_classifier_;
  cv::StereoBM block_matcher_;
#endif

  bool show_windows_;

  std::string robot_frame_;
  double scale_factor_;
  int min_neighbors_;
  cv::Size min_size_;
  cv::Size max_size_;

  inline int window_size(){
#ifdef USE_GPU
    return block_matcher_.winSize;
#else
    return block_matcher_.state->SADWindowSize;
#endif
  }
  inline int num_disparities(){
#ifdef USE_GPU
    return block_matcher_.ndisp;
#else
    return block_matcher_.state->numberOfDisparities;
#endif
  }
  inline int min_disparity(){
#ifdef USE_GPU
    return 0;
#else
    return block_matcher_.state->minDisparity;
#endif
  }


  void imageCb(const ImageConstPtr& l_image_msg,
	       const ImageConstPtr& r_image_msg,
	       const CameraInfoConstPtr& l_info_msg,
	       const CameraInfoConstPtr& r_info_msg);


  void compute_disparity(const InternalMat& left, const InternalMat& right, cv::Mat& d_image);

  void detect(const std::string& l_camera_frame, const ros::Time& time, const InternalMat& l_image, const cv::Mat& d_image, cv::Mat& l_feedback_image, cv::Mat& d_feedback_image);
  float average_disparity(const cv::Mat& disp, const cv::Point2d& pt, int width, int height);
};


}

#endif
