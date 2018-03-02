#include "perception/segmentation.h"

#include <limits.h>
#include <math.h>
#include <sstream>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/PointIndices.h"
#include "pcl/common/angles.h"
#include "pcl/common/common.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "shape_msgs/SolidPrimitive.h"
#include "visualization_msgs/Marker.h"

#include "perception/box_fitter.h"
#include "perception/object_recognizer.h"
#include "perception/typedefs.h"

using geometry_msgs::Pose;
using geometry_msgs::Vector3;

namespace perception {
void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices,
                    pcl::ModelCoefficients::Ptr coeff) {
  pcl::PointIndices indices_internal;
  pcl::SACSegmentation<PointC> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);
  Eigen::Vector3f axis;
  axis << 0, 0, 1;
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(10.0));

  seg.segment(indices_internal, *coeff);

  double distance_above_plane;
  ros::param::param("distance_above_plane", distance_above_plane, 0.005);

  // Build custom indices that ignores points above the plane.
  for (size_t i = 0; i < cloud->size(); ++i) {
    const PointC& pt = cloud->points[i];
    float val = coeff->values[0] * pt.x + coeff->values[1] * pt.y +
                coeff->values[2] * pt.z + coeff->values[3];
    if (val <= distance_above_plane) {
      indices->indices.push_back(i);
    }
  }

  if (indices->indices.size() == 0) {
    ROS_ERROR("Unable to find surface.");
    return;
  }
}

void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices) {
  pcl::ExtractIndices<PointC> extract;

  pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
  extract.setInputCloud(cloud);
  extract.setIndices(surface_indices);
  extract.setNegative(true);
  extract.filter(above_surface_indices->indices);

  double cluster_tolerance;
  int min_cluster_size, max_cluster_size;
  ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
  ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
  ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

  pcl::EuclideanClusterExtraction<PointC> euclid;
  euclid.setInputCloud(cloud);
  euclid.setIndices(above_surface_indices);
  euclid.setClusterTolerance(cluster_tolerance);
  euclid.setMinClusterSize(min_cluster_size);
  euclid.setMaxClusterSize(max_cluster_size);
  euclid.extract(*object_indices);

  size_t min_size = std::numeric_limits<size_t>::max();
  size_t max_size = std::numeric_limits<size_t>::min();
  for (size_t i = 0; i < object_indices->size(); ++i) {
    size_t cluster_size = (*object_indices)[i].indices.size();
    if (cluster_size < min_size) {
      min_size = cluster_size;
    }
    if (cluster_size > max_size) {
      max_size = cluster_size;
    }
  }

  ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
           object_indices->size(), min_size, max_size);
}

void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions) {
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);

  pose->position.x = (max_pt.x() + min_pt.x()) / 2;
  pose->position.y = (max_pt.y() + min_pt.y()) / 2;
  pose->position.z = (max_pt.z() + min_pt.z()) / 2;
  pose->orientation.w = 1;

  dimensions->x = max_pt.x() - min_pt.x();
  dimensions->y = max_pt.y() - min_pt.y();
  dimensions->z = max_pt.z() - min_pt.z();
}

void SegmentTabletopScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          std::vector<Object>* objects) {
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
  SegmentSurface(cloud, table_inliers, coeff);

  PointCloudC::Ptr cloud_out(new PointCloudC());
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(table_inliers);
  extract.setNegative(false);
  extract.filter(*cloud_out);

  std::vector<pcl::PointIndices> object_indices;
  SegmentSurfaceObjects(cloud, table_inliers, &object_indices);

  extract.setInputCloud(cloud);
  extract.setNegative(false);
  for (size_t i = 0; i < object_indices.size(); ++i) {
    // Reify indices into a point cloud of the object.
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = object_indices[i];
    extract.setIndices(indices);
    PointCloudC::Ptr object_cloud(new PointCloudC());
    extract.filter(*object_cloud);

    PointCloudC::Ptr extract_out(new PointCloudC());
    shape_msgs::SolidPrimitive shape;
    geometry_msgs::Pose obj_pose;
    FitBox(*object_cloud, coeff, *extract_out, shape, obj_pose);

    Object obj;
    obj.cloud = object_cloud;
    obj.pose = obj_pose;
    obj.dimensions.x = shape.dimensions[0];
    obj.dimensions.y = shape.dimensions[1];
    obj.dimensions.z = shape.dimensions[2];
    objects->push_back(obj);
  }
}

Segmenter::Segmenter(const ros::Publisher& surface_points_pub,
                     const ros::Publisher& above_surface_pub,
                     const ros::Publisher& marker_pub,
                     const ObjectRecognizer& recognizer)
    : surface_points_pub_(surface_points_pub),
      above_surface_pub_(above_surface_pub),
      marker_pub_(marker_pub),
      recognizer_(recognizer) {}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud_unfiltered(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud_unfiltered);
  PointCloudC::Ptr cloud(new PointCloudC());
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, index);

  std::vector<Object> objects;
  SegmentTabletopScene(cloud, &objects);

  for (size_t i = 0; i < objects.size(); ++i) {
    const Object& object = objects[i];

    // Publish a bounding box around it.
    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = i;
    object_marker.header.frame_id = "base_link";
    object_marker.type = visualization_msgs::Marker::CUBE;
    object_marker.pose = object.pose;
    object_marker.scale = object.dimensions;
    object_marker.color.g = 1;
    object_marker.color.a = 0.3;
    marker_pub_.publish(object_marker);

    // Recognize the object.
    std::string name;
    double confidence;
    recognizer_.Recognize(object, &name, &confidence);
    confidence = round(1000 * confidence) / 1000;

    std::stringstream ss;
    ss << name << " (" << confidence << ")";

    // Publish the recognition result.
    visualization_msgs::Marker name_marker;
    name_marker.ns = "recognition";
    name_marker.id = i;
    name_marker.header.frame_id = "base_link";
    name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    name_marker.pose.position = object.pose.position;
    name_marker.pose.position.z += 0.1;
    name_marker.pose.orientation.w = 1;
    name_marker.scale.x = 0.025;
    name_marker.scale.y = 0.025;
    name_marker.scale.z = 0.025;
    name_marker.color.r = 0;
    name_marker.color.g = 0;
    name_marker.color.b = 1.0;
    name_marker.color.a = 1.0;
    name_marker.text = ss.str();
    marker_pub_.publish(name_marker);
  }
}
}  // namespace perception
