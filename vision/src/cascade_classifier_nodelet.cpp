#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <vision/cascade_classifier.h>

namespace vision{

  class CascadeClassifierNodelet : public nodelet::Nodelet
  {

  public:
    CascadeClassifierNodelet(){}

  private:
    void onInit(){
      detector_.reset(new CascadeClassifier(getNodeHandle(), getPrivateNodeHandle()));
    }
    boost::shared_ptr<CascadeClassifier> detector_;

  };

}

PLUGINLIB_DECLARE_CLASS(vision, CascadeClassifierNodelet, vision::CascadeClassifierNodelet, nodelet::Nodelet);
