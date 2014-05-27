#ifndef CASCADE_CLASSIFIER_H_
#define CASCADE_CLASSIFIER_H_

#define USE_GPU 1

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

class CascadeClassifier{

  typedef ApproximateTime<Image, DisparityImage> ImageSyncPolicy;
  typedef message_filters::Synchronizer<ImageSyncPolicy> ImageSynchronizer;

  typedef ApproximateTime<CameraInfo, CameraInfo> InfoSyncPolicy;
  typedef message_filters::Synchronizer<InfoSyncPolicy> InfoSynchronizer;
 public:
  CascadeClassifier(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~CascadeClassifier();

 private:
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter sub_l_image_;
  message_filters::Subscriber<DisparityImage> sub_d_image_;
  shared_ptr<ImageSynchronizer> image_synchronizer_;

  message_filters::Subscriber<CameraInfo> sub_l_info_;
  message_filters::Subscriber<CameraInfo> sub_r_info_;
  shared_ptr<InfoSynchronizer> info_synchronizer_;

  boost::mutex stereo_model_mutex_;
  bool stereo_model_init_;
  image_geometry::StereoCameraModel stereo_model_;

  tf::TransformListener tf_listener_;

  ros::Publisher object_location_pub_;
  image_transport::Publisher disparity_pub_;
  image_transport::Publisher image_pub_;

#ifdef USE_GPU
  cv::gpu::CascadeClassifier_GPU cascade_classifier_;
#else
  cv::CascadeClassifier cascade_classifier_;
#endif

  bool show_windows_;

  std::string robot_frame_;
  double scale_factor_;
  int min_neighbors_;
  cv::Size min_size_;
  cv::Size max_size_;
  double object_radius_;
  float max_disparity_for_detection_;


  void imageCb(const ImageConstPtr& l_image_msg,
	       const DisparityImageConstPtr& d_image_msg);

  void infoCb(const CameraInfoConstPtr& l_info_msg,
	      const CameraInfoConstPtr& r_info_msg);

  void detect_and_publish(const std::string& l_camera_frame, const ros::Time& time, cv::Mat& l_mat, cv::Mat& d_image);
  float average_disparity(const cv::Mat& disp, const cv::Point2d& pt, int width, int height);
};


}

#endif
