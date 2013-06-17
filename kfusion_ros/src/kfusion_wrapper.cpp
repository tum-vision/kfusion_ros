#include <kfusion_ros/kfusion_wrapper.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <visualization_msgs/Marker.h>

#include <opencv2/opencv.hpp>

namespace kfusion_ros
{

template<typename T>
struct CudaTypeTrait { static int CvType() { return 0; } };

template<> struct CudaTypeTrait<float> { static int CvType() { return CV_32FC1; } };
template<> struct CudaTypeTrait<float3> { static int CvType() { return CV_32FC3; } };

template<typename T, typename Allocator>
void toImageMessage(Image<T, Allocator>& img, sensor_msgs::Image& img_msg)
{
  img_msg.width = img.size.x;
  img_msg.height = img.size.y;
  img_msg.step = img.size.x * sizeof(T);
  img_msg.data.resize(img.size.x * img.size.y * sizeof(T));

  std::copy((uchar*)(img.data()), (uchar*)(img.data() + (img.size.x * img.size.y)), img_msg.data.data());
}

template<typename T>
void toImageMessage(Image<T, Device>& img, sensor_msgs::Image& img_msg)
{
  Image<T, Host> tmp;
  tmp.alloc(img.size);
  tmp = img;

  toImageMessage(tmp, img_msg);
}

template<typename T, typename Allocator>
void toCvMat(Image<T, Allocator>& img, cv::Mat& mat)
{
  mat.create(img.size.y, img.size.x, CudaTypeTrait<T>::CvType());

  std::copy((uchar*)(img.data()), (uchar*)(img.data() + (img.size.x * img.size.y)), mat.ptr<uchar>());
}

template<typename T>
void toCvMat(Image<T, Device>& img, cv::Mat& mat)
{
  Image<T, Host> tmp;
  tmp.alloc(img.size);
  tmp = img;

  toCvMat(tmp, mat);
}

void createBoundingBoxMarker(const float3& volume, visualization_msgs::Marker& bb)
{
  bb.type = visualization_msgs::Marker::POINTS;

  geometry_msgs::Point p1;

  for(int x = 0; x < 2; x++)
    for(int y = 0; y < 2; y++)
      for(int z = 0; z < 2; z++)
      {
        p1.x = x * volume.x;
        p1.y = y * volume.y;
        p1.z = z * volume.z;

        bb.points.push_back(p1);
      }

}

KFusionWrapper::KFusionWrapper(ros::NodeHandle &nh, ros::NodeHandle &nh_private) :
    nh_(nh),
    nh_private_(nh_private),
    it_(nh),
    first_(true),
    failures_count_(0)
{
  pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("kfusion_pointcloud", 1);
  bounding_box_publisher_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

  depth_subscriber_ = it_.subscribeCamera("camera/depth_registered/image_raw", 1, &KFusionWrapper::onDepth, this);
}

KFusionWrapper::~KFusionWrapper()
{
}

void KFusionWrapper::init(const sensor_msgs::CameraInfo& info)
{
  configuration_.inputSize.x = info.width;
  configuration_.inputSize.y = info.height;
  configuration_.volumeDimensions = make_float3(3.0);
  configuration_.volumeSize = make_uint3(256);

  configuration_.camera.x = info.K[0]; // fx
  configuration_.camera.y = info.K[4]; // fy
  configuration_.camera.z = info.K[2]; // ox
  configuration_.camera.w = info.K[5]; // oy

  Matrix4 pose;
  pose.data[0] = make_float4(1.0f, 0.0f, 0.0f, configuration_.volumeDimensions.x / 2.0f);
  pose.data[1] = make_float4(0.0f, 1.0f, 0.0f, configuration_.volumeDimensions.y / 2.0f);
  pose.data[2] = make_float4(0.0f, 0.0f, 1.0f, 0.0f);
  pose.data[3] = make_float4(0.0f, 0.0f, 0.0f, 1.0f);

  input_depth_img_.alloc(make_uint2(info.width, info.height));

  kfusion_.Init(configuration_);
  kfusion_.setPose(pose);

  // create bounding box visualization
  visualization_msgs::Marker bb;
  bb.header.frame_id = "/kfusion_volume";
  bb.header.stamp = ros::Time::now();
  bb.ns = "kfusion/boundingbox";
  bb.id = 1;
  bb.color.a = bb.color.r = 1.0;
  bb.scale.x = bb.scale.y = bb.scale.z = 0.05;
  createBoundingBoxMarker(configuration_.volumeDimensions, bb);
  bounding_box_publisher_.publish(bb);
}

void KFusionWrapper::onDepth(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  if(failures_count_ > 60)
  {
    failures_count_ = 0;
    first_ = true;
  }

  ROS_WARN_COND(first_, "Initializing volume and tracking!");

  if(first_)
  {
    init(*info_msg);
  }

  uint16_t const* data = (uint16_t const*)(img_msg->data.data());
  std::copy(data, data + (input_depth_img_.size.x * input_depth_img_.size.y), input_depth_img_.data());

  kfusion_.setKinectDeviceDepth(input_depth_img_.getDeviceImage());

  if(first_ || kfusion_.Track())
  {
      kfusion_.Integrate();
      kfusion_.Raycast();

      publishPointCloud(img_msg->header);
      publishTransforms(img_msg->header);
  }
  else
  {
    failures_count_++;
  }

  first_ = false;
}

void KFusionWrapper::publishPointCloud(const std_msgs::Header& header)
{
  cv::Mat vertices;
  toCvMat(kfusion_.vertex, vertices);

  sensor_msgs::PointField pfx;
  pfx.name = "x";
  pfx.datatype = sensor_msgs::PointField::FLOAT32;
  pfx.count = 1;
  pfx.offset = 4;
  sensor_msgs::PointField pfy;
  pfy.name = "y";
  pfy.datatype = sensor_msgs::PointField::FLOAT32;
  pfy.count = 1;
  pfy.offset = 8;
  sensor_msgs::PointField pfz;
  pfz.name = "z";
  pfz.datatype = sensor_msgs::PointField::FLOAT32;
  pfz.count = 1;
  pfz.offset = 12;

  sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
  cloud_msg->header = header;
  cloud_msg->header.frame_id = "/kfusion_volume";
  cloud_msg->width = kfusion_.vertex.size.x;
  cloud_msg->height = kfusion_.vertex.size.y;
  cloud_msg->is_dense = false;
  cloud_msg->point_step = sizeof(float) * 3;
  cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
  cloud_msg->fields.push_back(pfx);
  cloud_msg->fields.push_back(pfy);
  cloud_msg->fields.push_back(pfz);
  cloud_msg->data.resize(cloud_msg->row_step * cloud_msg->height);

  std::copy(vertices.ptr<uchar>(), vertices.ptr<uchar>(vertices.rows - 1, vertices.cols - 1), cloud_msg->data.data());

  pointcloud_publisher_.publish(cloud_msg);
}

void KFusionWrapper::publishTransforms(const std_msgs::Header& header)
{
  //TODO: fix this stuff :(
  tf::Transform world_volume, volume_camera;
  world_volume.getBasis().setEulerZYX(0, 0, 0);
  world_volume.getOrigin().setValue(0, 0, 0);

  volume_camera.getBasis().setValue(
      kfusion_.pose.data[0].x, kfusion_.pose.data[0].y, kfusion_.pose.data[0].z,
      kfusion_.pose.data[1].x, kfusion_.pose.data[1].y, kfusion_.pose.data[1].z,
      kfusion_.pose.data[2].x, kfusion_.pose.data[2].y, kfusion_.pose.data[2].z
  );
  volume_camera.getOrigin().setValue(kfusion_.pose.data[0].w, kfusion_.pose.data[1].w, kfusion_.pose.data[2].w);

  tf_.sendTransform(tf::StampedTransform(world_volume, header.stamp, "/world", "/kfusion_volume"));
  tf_.sendTransform(tf::StampedTransform(volume_camera, header.stamp, "/kfusion_volume", "/camera"));
}

} /* namespace kfusion_ros */
