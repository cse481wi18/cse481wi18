#include "perception/box_fitter.h"

#include "geometry_msgs/Pose.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/surface/convex_hull.h"
#include "shape_msgs/SolidPrimitive.h"

#include "Eigen/Eigen"

namespace perception {
bool FitBox(const pcl::PointCloud<pcl::PointXYZRGB>& input,
            const pcl::ModelCoefficients::Ptr model,
            pcl::PointCloud<pcl::PointXYZRGB>& output,
            shape_msgs::SolidPrimitive& shape, geometry_msgs::Pose& pose) {
  // Used to decide between various shapes
  double min_volume = 1000.0;      // the minimum volume shape found thus far.
  Eigen::Matrix3f transformation;  // the transformation for the best-fit shape

  // Compute z height as maximum distance from planes
  double height = 0.0;
  for (size_t i = 0; i < input.size(); ++i) {
    Eigen::Vector4f pp(input[i].x, input[i].y, input[i].z, 1);
    Eigen::Vector4f m(model->values[0], model->values[1], model->values[2],
                      model->values[3]);
    double distance_to_plane = fabs(pp.dot(m));
    if (distance_to_plane > height) height = distance_to_plane;
  }

  // Project object into 2d, using plane model coefficients
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr flat(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ProjectInliers<pcl::PointXYZRGB> projection;
  projection.setModelType(pcl::SACMODEL_PLANE);
  projection.setInputCloud(input.makeShared());  // stupid API
  projection.setModelCoefficients(model);
  projection.filter(*flat);

  // Rotate plane so that Z=0
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr flat_projected(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  Eigen::Vector3f normal(model->values[0], model->values[1], model->values[2]);
  Eigen::Quaternionf qz;
  qz.setFromTwoVectors(normal, Eigen::Vector3f::UnitZ());
  Eigen::Matrix3f plane_rotation = qz.toRotationMatrix();
  Eigen::Matrix3f inv_plane_rotation = plane_rotation.inverse();

  for (size_t i = 0; i < flat->size(); ++i) {
    pcl::PointXYZRGB p;
    p.getVector3fMap() = plane_rotation * (*flat)[i].getVector3fMap();
    flat_projected->push_back(p);
  }

  // Find the convex hull
  pcl::PointCloud<pcl::PointXYZRGB> hull;
  pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;
  convex_hull.setInputCloud(flat_projected);
  convex_hull.setDimension(2);
  convex_hull.reconstruct(hull);

  // Try fitting a rectangle
  shape_msgs::SolidPrimitive rect;  // the best-fit rectangle
  rect.type = rect.BOX;
  rect.dimensions.resize(3);
  for (size_t i = 0; i < hull.size() - 1; ++i) {
    // For each pair of hull points, determine the angle
    double rise = hull[i + 1].y - hull[i].y;
    double run = hull[i + 1].x - hull[i].x;
    // and normalize..
    {
      double l = sqrt((rise * rise) + (run * run));
      rise = rise / l;
      run = run / l;
    }

    // Build rotation matrix from change of basis
    Eigen::Matrix3f rotation;
    rotation(0, 0) = run;
    rotation(0, 1) = rise;
    rotation(0, 2) = 0.0;
    rotation(1, 0) = -rise;
    rotation(1, 1) = run;
    rotation(1, 2) = 0.0;
    rotation(2, 0) = 0.0;
    rotation(2, 1) = 0.0;
    rotation(2, 2) = 1.0;
    Eigen::Matrix3f inv_rotation = rotation.inverse();

    // Project hull to new coordinate system
    pcl::PointCloud<pcl::PointXYZRGB> projected_cloud;
    for (size_t j = 0; j < hull.size(); ++j) {
      pcl::PointXYZRGB p;
      p.getVector3fMap() = rotation * hull[j].getVector3fMap();
      projected_cloud.push_back(p);
    }

    // Compute min/max
    double x_min = 1000.0;
    double x_max = -1000.0;
    double y_min = 1000.0;
    double y_max = -1000.0;
    for (size_t j = 0; j < projected_cloud.size(); ++j) {
      if (projected_cloud[j].x < x_min) x_min = projected_cloud[j].x;
      if (projected_cloud[j].x > x_max) x_max = projected_cloud[j].x;

      if (projected_cloud[j].y < y_min) y_min = projected_cloud[j].y;
      if (projected_cloud[j].y > y_max) y_max = projected_cloud[j].y;
    }

    // Is this the best estimate?
    double area = (x_max - x_min) * (y_max - y_min);
    if (area * height < min_volume) {
      transformation = inv_plane_rotation * inv_rotation;

      rect.dimensions[0] = (x_max - x_min);
      rect.dimensions[1] = (y_max - y_min);
      rect.dimensions[2] = height;

      Eigen::Vector3f pose3f((x_max + x_min) / 2.0, (y_max + y_min) / 2.0,
                             projected_cloud[0].z + height / 2.0);
      pose3f = transformation * pose3f;
      pose.position.x = pose3f(0);
      pose.position.y = pose3f(1);
      pose.position.z = pose3f(2);

      Eigen::Quaternionf q(transformation);
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();

      min_volume = area * height;
      shape = rect;
    }
  }

  // Project input to new frame
  Eigen::Vector3f origin(pose.position.x, pose.position.y, pose.position.z);
  for (size_t j = 0; j < input.size(); ++j) {
    pcl::PointXYZRGB p;
    p.getVector3fMap() = transformation * (input[j].getVector3fMap() - origin);
    output.push_back(p);
  }
  return true;
}
}  // namespace perception
