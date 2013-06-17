#ifndef KFUSION_WRAPPER_H_
#define KFUSION_WRAPPER_H_

#include <kfusion.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

namespace kfusion_ros
{

class KFusionWrapper
{
public:
  KFusionWrapper(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
  virtual ~KFusionWrapper();

private:
  ros::NodeHandle &nh_, &nh_private_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber depth_subscriber_;

  ros::Publisher pointcloud_publisher_, bounding_box_publisher_;
  tf::TransformBroadcaster tf_;

  KFusionConfig configuration_;
  KFusion kfusion_;
  Image<uint16_t, HostDevice> input_depth_img_;

  bool first_;
  int failures_count_;

  void init(const sensor_msgs::CameraInfo& info);

  void onDepth(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);

  void publishBoundingBox(const std_msgs::Header& header);

  void publishPointCloud(const std_msgs::Header& header);

  void publishTransforms(const std_msgs::Header& header);
};

} /* namespace kfusion_ros */
#endif /* KFUSION_WRAPPER_H_ */
