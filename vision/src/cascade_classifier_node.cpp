#include <vision/cascade_classifier.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "cascade_classifier");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  vision::CascadeClassifier classifier(nh, pnh);
  ros::spin();
}
