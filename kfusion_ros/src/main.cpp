#include <ros/ros.h>
#include <kfusion_ros/kfusion_wrapper.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "kfusion_node");

  ros::NodeHandle nh, nh_private("~");

  kfusion_ros::KFusionWrapper kfusion(nh, nh_private);

  ros::spin();

  return 0;
}
