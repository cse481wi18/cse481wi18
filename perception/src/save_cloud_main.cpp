#include <iostream>
#include <string>

#include "pcl_ros/transforms.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"

void print_usage() {
  std::cout << "Saves a point cloud on head_kinect/depth_registered/points to "
               "NAME.bag in the current directory."
            << std::endl;
  std::cout << "Usage: rosrun perception save_cloud NAME" << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "save_cloud_main");
  if (argc < 2) {
    print_usage();
    return 1;
  }
  std::string name(argv[1]);

  sensor_msgs::PointCloud2ConstPtr cloud =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
          "head_camera/depth_registered/points");

  tf::TransformListener tf_listener;
  tf_listener.waitForTransform("base_link", cloud->header.frame_id,
                               ros::Time(0), ros::Duration(5.0));
  tf::StampedTransform transform;
  try {
    tf_listener.lookupTransform("base_link", cloud->header.frame_id,
                                ros::Time(0), transform);
  } catch (tf::LookupException& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  } catch (tf::ExtrapolationException& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  sensor_msgs::PointCloud2 cloud_out;
  pcl_ros::transformPointCloud("base_link", transform, *cloud, cloud_out);

  std::string filename(name + ".bag");
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Write);
  bag.write("head_camera/depth_registered/points", ros::Time::now(), cloud_out);
  bag.close();

  return 0;
}
