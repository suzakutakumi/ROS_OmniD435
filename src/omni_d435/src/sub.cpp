#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void pointcloudCallback(const sensor_msgs::PointCloud2 &pcl_msg)
{
  static int cnt=1;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(pcl_msg, cloud);
  auto now = ros::Time::now().sec;
  auto filename = "/home/omni-d435/Downloads/pcd/merged" + std::to_string(now) + ".pcd";

  ROS_INFO("subscribe: %s(SIZE: %d)", filename.c_str(), cloud.size());
  // pcl::io::savePCDFileBinary(filename, cloud);
  pcl::PCDWriter writer;
  writer.write(filename, cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_pcl_practice_sub");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("pointcloud", 10, pointcloudCallback);

  ros::spin();
  return 0;
}