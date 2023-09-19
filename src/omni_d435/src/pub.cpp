#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "d435.hpp"
#include "PointCloud.hpp"

enum Direction
{
  Front,
  Right,
  Left,
  Back,
};

int main(int argc, char **argv)
try
{
  constexpr int num_of_using_device = 4;
  const std::string serial_numbers[num_of_using_device] = {
      "102422073987", "102422070478", "102122072472", "102422071935"};
  D435 *cameras[num_of_using_device];
  for (int i = 0; i < num_of_using_device; i++)
  {
    cameras[i] = new D435(serial_numbers[i]);
  }

  // Initialize ROS
  ros::init(argc, argv, "omni_d435_pub");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);

  while (ros::ok())
  {
    PointCloud pcs[4];
    for (int i = 0; i < num_of_using_device; i++)
    {
      cameras[i]->update();
      auto points = cameras[i]->get_points();
      auto color = cameras[i]->get_color();
      pcs[i] = PointCloud(points, color);

      switch (i)
      {
      case Direction::Front:
        pcs[i].filter([](pcl::PointXYZRGB &p)
                      { p.z += 0.080; });
        break;
      case Direction::Back:
        pcs[i].filter([](pcl::PointXYZRGB &p)
                      {
                p.x=-p.x;
                p.z=-p.z;
                p.z-=0.080; });
        break;
      case Direction::Right:
        pcs[i].filter([](pcl::PointXYZRGB &p)
                      {
                float x=p.x;
                p.x=p.z;
                p.z=-x;
                p.x+=0.080; });
        break;
      case Direction::Left:
        pcs[i].filter([](pcl::PointXYZRGB &p)
                      {
                float x=p.x;
                p.x=-p.z;
                p.z=x;
                p.x-=0.080; });
        break;
      }
    }

    PointCloud merged;
    for (auto &p : pcs)
    {
      merged.extended(p);
    }
    sensor_msgs::PointCloud2 pcl_msg;
    pcl::toROSMsg(*merged.get_cloud(), pcl_msg);
    ROS_INFO("publish:%d", merged.get_cloud()->size());
    pub.publish(pcl_msg);

    ros::spinOnce();
  }

  return 0;
}
catch (const rs2::error &e)
{
  std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
}
catch (const std::exception &e)
{
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}