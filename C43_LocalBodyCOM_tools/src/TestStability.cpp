// SVN $HeadURL$
// SVN $Id$

/*
 * hrl_kinematics - a kinematics library for humanoid robots based on KDL
 *
 * Copyright 2011-2012 Armin Hornung, University of Freiburg
 * License: BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "C43_LocalBodyCOM/TestStability.h"
#include <pcl/surface/convex_hull.h>
#include <pcl/point_types.h>

using robot_state_publisher::SegmentPair;
using boost::shared_ptr;

namespace C43_LocalBodyCOM {

TestStability::TestStability()
: Kinematics(), rfoot_mesh_link_name("r_foot")//"r_talus" Yuval changed from:rfoot_mesh_link_name("RAnkleRoll_link")
{

  initFootPolygon();
}

TestStability::~TestStability() {

}

bool TestStability::isPoseStable(const std::map<std::string, double>& joint_positions, FootSupport support_mode){
  tf::Vector3 normal(0.0, 0.0, 1.0);
  return isPoseStable(joint_positions, support_mode, normal);
}

bool TestStability::isPoseStable(const std::map<std::string, double>& joint_positions,
                                 FootSupport support_mode, const tf::Vector3& normal_vector)
{

  tf::Vector3 upright_vector(0.0, 0.0, 1.0);
  double angle = acos(upright_vector.dot(normal_vector.normalized()));

  tf::Quaternion q;
  if (std::abs(angle) < SIMD_EPSILON)
    q=tf::createIdentityQuaternion();
  else{
    tf::Vector3 axis = upright_vector.cross(normal_vector).normalized();
    q = tf::Quaternion(axis, angle);
  }
  tf::Transform rotate_plane(q, tf::Vector3(0,0,0));

  tf::Point com; // center of mass in root frame
  double m;

  // transforms from root to left and right foot:
  tf::Transform tf_right_foot, tf_left_foot;
  computeCOM(joint_positions, com, m, tf_right_foot, tf_left_foot);

  tf::Transform tf_to_support;

  // com_ = com; //Yuval add
  if (com_Arr_.empty()){ // init vector
      com_Arr_.push_back(com); //Yuval add
      com_Arr_.push_back(tf_right_foot.inverse() * com); //Yuval add
      com_Arr_.push_back(tf_left_foot.inverse() * com); //Yuval add
  }else {
      com_Arr_[0] = com; //Yuval add
      com_Arr_[1] = tf_right_foot.inverse() * com; //Yuval add
      com_Arr_[2] = tf_left_foot.inverse() * com; //Yuval add
  }

  if (support_mode == SUPPORT_SINGLE_LEFT){
    support_polygon_ = foot_support_polygon_left_;
    tf_to_support_ = tf_left_foot;
  } else { // RIGHT or DOUBLE
    support_polygon_ = foot_support_polygon_right_;
    tf_to_support_ = tf_right_foot;

  }

  // rotate and project down
  for (unsigned i = 0; i < support_polygon_.size(); ++i){
    support_polygon_[i] = rotate_plane * support_polygon_[i];
  }


  if (support_mode == SUPPORT_DOUBLE){
    // append left if double support:
    tf::Transform tf_right_to_left = tf_right_foot.inverseTimes(tf_left_foot);
    for (unsigned i = 0; i < foot_support_polygon_left_.size(); ++i){
      support_polygon_.push_back(rotate_plane * tf_right_to_left * foot_support_polygon_left_[i]);
    }
    // Yuval changed from: support_polygon_ = convexHull(support_polygon_);
    foot_support_polygon_double_ = convexHull(support_polygon_);
    // Yuval: add center of support polygon to last point of support polygon vector
    foot_support_polygon_double_.push_back(GetCenterOfPolygon(foot_support_polygon_double_)); // Yuval added
    support_polygon_ = foot_support_polygon_double_; // Yuval added
  }
  if (support_polygon_.size() <= 2)
    return false;

  // projected com in support frame, rotated around support plane:
  p_com_ = rotate_plane * tf_to_support_.inverse() * com;
  p_com_.setZ(0.0);

  tf::Point center_polygonP = support_polygon_[support_polygon_.size()];
  err_pCoM2SupportCenter_ = p_com_ - center_polygonP;

  // Yuval: removing center point of polygon to check stability
  std::vector<tf::Point> support_polygon_without_center = support_polygon_;
  support_polygon_without_center.pop_back();

  return pointInConvexHull(p_com_, support_polygon_without_center);
}

/* // Yuval Comm:
geometry_msgs::PolygonStamped TestStability::getSupportPolygon() const{
  geometry_msgs::PolygonStamped footprint_poly;
  footprint_poly.header.frame_id = root_link_name_;
  footprint_poly.header.stamp = ros::Time::now();
  for (unsigned i=0; i < support_polygon_.size(); ++i){
    geometry_msgs::Point32 p;
    tf::Point tfP = tf_to_support_ * support_polygon_[i];
    p.x = tfP.x();
    p.y = tfP.y();
    p.z = tfP.z();
    footprint_poly.polygon.points.push_back(p);
  }

  return footprint_poly;
} */

visualization_msgs::Marker TestStability::getSupportPolygon() const{

  visualization_msgs::Marker footprint_poly;
  footprint_poly.header.stamp = ros::Time::now();
  footprint_poly.header.frame_id = root_link_name_;
  footprint_poly.action = visualization_msgs::Marker::ADD;
  footprint_poly.type = visualization_msgs::Marker::LINE_STRIP;//Yuval added
  for (unsigned i=0; i < support_polygon_.size(); ++i){
    geometry_msgs::Point p;
    tf::pointTFToMsg(tf_to_support_ * support_polygon_[i], p);
    footprint_poly.points.push_back(p);
  }

  footprint_poly.scale.x = 0.01;
  footprint_poly.color.a = 0.5; //1.0
  footprint_poly.color.g = 0.5; //1.0
  footprint_poly.color.r = 1.0; //0.0

  return footprint_poly;
}

visualization_msgs::Marker TestStability::getCOMMarker() const{

  visualization_msgs::Marker com_marker;
  com_marker.header.stamp = ros::Time::now();
  com_marker.header.frame_id = root_link_name_;
  com_marker.action = visualization_msgs::Marker::ADD;
  //com_marker.type = visualization_msgs::Marker::SPHERE;//Yuval Comm
  com_marker.type = visualization_msgs::Marker::ARROW;//Yuval added
  geometry_msgs::Point p_start,p_end; //Yuval added
  //tf::pointTFToMsg(com_, p); //Yuval added
  p_start.x = com_Arr_[0].x(); //Yuval added
  p_start.y = com_Arr_[0].y(); //Yuval added
  p_start.z = com_Arr_[0].z(); //Yuval added
  com_marker.points.push_back(p_start); //Yuval added
  // tf::pointTFToMsg(tf_to_support_ * p_com_, com_marker.pose.position); //Yuval Comm
  // tf::quaternionTFToMsg(tf_to_support_.getRotation(), com_marker.pose.orientation); //Yuval Comm
  tf::pointTFToMsg(tf_to_support_ * p_com_, p_end);
  com_marker.points.push_back(p_end); //Yuval added
  com_marker.scale.x = 0.03;
  com_marker.scale.y = 0.03;
  com_marker.scale.z = -0.09;
  com_marker.color.a = 1.0;
  com_marker.color.g = 0.5; //1.0
  com_marker.color.r = 1.0; //0.0


  return com_marker;
}
/* //Yuval added:
geometry_msgs::Point TestStability::getCOM() const{

  geometry_msgs::Point com_point;
  com_point.x = com_.x();
  com_point.y = com_.y();
  com_point.z = com_.z();


  return com_point;
} */ //Yuval added

//Yuval added:
CoM_Array_msg TestStability::getCOM(const ros::Time time_stamp, bool& published) const{

  CoM_Array_msg com_point;
  com_point.header.stamp = time_stamp;
  if (!com_Arr_.empty()) {
      for (unsigned i = 0; i < 3; ++i){
          com_point.x[i] = com_Arr_[i].x();
          com_point.y[i] = com_Arr_[i].y();
          com_point.z[i] = com_Arr_[i].z();
      }
      published = 1;
  }
  return com_point;
} //Yuval added

//Yuval added:
pCoM_err_msg TestStability::get_pCOM_err(const ros::Time time_stamp) const{

  pCoM_err_msg err_vec;
  err_vec.header.stamp = time_stamp;
  err_vec.x = err_pCoM2SupportCenter_.x();
  err_vec.y = err_pCoM2SupportCenter_.y();
  err_vec.z = err_pCoM2SupportCenter_.z();

  return err_vec;
} //Yuval added

std::vector<tf::Point> TestStability::convexHull(const std::vector<tf::Point>& points) const{
  std::vector<tf::Point> hull;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> chull_points;
  pcl::ConvexHull<pcl::PointXYZ> chull;

  if (points.empty()){
    ROS_ERROR("convexHull on empty set of points!");
    return hull;
  }

  for (unsigned i = 0; i < points.size(); ++i){
    pcl_points->points.push_back(pcl::PointXYZ(points[i].x(), points[i].y(), 0.0));
  }

#if ROS_VERSION_MINIMUM(1,8,0) // test for Fuerte (newer PCL)
  chull.setDimension(2);
#else

#endif

  chull.setInputCloud(pcl_points);
  std::vector<pcl::Vertices> polygons;
  chull.reconstruct(chull_points, polygons);

  if (polygons.size() == 0){
    ROS_ERROR("Convex hull polygons are empty");
    return hull;
  } else if (polygons.size() > 1){
    ROS_WARN("Convex hull polygons are larger than 1");
  }

  for (unsigned i = 0; i < polygons[0].vertices.size(); ++i){
    int idx = polygons[0].vertices[i];
    tf::Point p(chull_points.points[idx].x,
                chull_points.points[idx].y,
                chull_points.points[idx].z);
    hull.push_back(p);
  }

  return hull;
}

bool TestStability::pointInConvexHull(const tf::Point& point, const std::vector<tf::Point>& polygon) const{
  assert(polygon.size() >=3);
  int positive_direction = 0;
  for (unsigned i = 0; i < polygon.size(); ++i){
    int i2 = (i+1)% (polygon.size());
    double dx = polygon[i2].getX() - polygon[i].getX();
    double dy = polygon[i2].getY() - polygon[i].getY();
    if (dx == 0.0 && dy == 0.0){
      ROS_DEBUG("Skipping polygon connection [%d-%d] (identical points)", i, i2);
      continue;
    }
    double line_test = (point.y() - polygon[i].getY())*dx - (point.x() - polygon[i].getX())*dy;
    if (i == 0)
      positive_direction = (line_test > 0.0);
    ROS_DEBUG("Line test [%d-%d] from (%f,%f) to (%f,%f): %f", i, i2, polygon[i].getX(), polygon[i].getY(),
              polygon[i2].getX(), polygon[i2].getY(), line_test);
    if ((line_test > 0.0) != positive_direction)
      return false;

  }

  return true;
}

void TestStability::initFootPolygon(){
  // TODO: param?
  if (!loadFootPolygon()){
    ROS_WARN("Could not load foot mesh, using default points");

    foot_support_polygon_right_.push_back(tf::Point(0.07f, 0.023f, 0.0));
    foot_support_polygon_right_.push_back(tf::Point(0.07f, -0.03f, 0.0));
    foot_support_polygon_right_.push_back(tf::Point(-0.03f, -0.03f, 0.0));
    foot_support_polygon_right_.push_back(tf::Point(-0.03f, 0.02, 0.0));
  }


  // mirror for left:
  foot_support_polygon_left_ = foot_support_polygon_right_;
  for (unsigned i=0; i < foot_support_polygon_left_.size(); ++i){
    foot_support_polygon_left_[i] *= tf::Point(1.0, -1.0, 1.0);
  }
  // restore order of polygon
  foot_support_polygon_left_ = convexHull(foot_support_polygon_left_);

  // Yuval: add center of support polygon to last point of support polygon vector
  foot_support_polygon_left_.push_back(GetCenterOfPolygon(foot_support_polygon_left_)); // Yuval added

}

bool TestStability::loadFootPolygon(){
  boost::shared_ptr<const urdf::Link> foot_link =  urdf_model_.getLink(rfoot_mesh_link_name);
  assert(foot_link);
  boost::shared_ptr<const urdf::Geometry> geom;
  urdf::Pose geom_pose;
  if (foot_link->collision && foot_link->collision->geometry){
    geom = foot_link->collision->geometry;
    geom_pose = foot_link->collision->origin;
  } else if (foot_link->visual && foot_link->visual->geometry){
    geom = foot_link->visual->geometry;
    geom_pose = foot_link->visual->origin;
  } else{
    ROS_ERROR_STREAM("No geometry for link "<< rfoot_mesh_link_name << " available");
    return false;
  }

  tf::Pose geom_origin = tf::Pose(btTransform(btQuaternion(geom_pose.rotation.x, geom_pose.rotation.y, geom_pose.rotation.z, geom_pose.rotation.w),
                                              btVector3(geom_pose.position.x, geom_pose.position.y, geom_pose.position.z)));


  if (geom->type != urdf::Geometry::MESH){
    ROS_ERROR_STREAM("Geometry for link "<< rfoot_mesh_link_name << " is not a mesh");
    return false;
  }
  else {
    shared_ptr<const urdf::Mesh> mesh = boost::dynamic_pointer_cast<const urdf::Mesh>(geom);

    //// arm_navigation (Electric / Fuerte)
    tf::Vector3 scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
    shapes::Mesh* shape_mesh = shapes::createMeshFromFilename(mesh->filename, &scale);
    size_t vertex_count = shape_mesh->vertexCount;

    //// MoveIt:
    //const Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
    //shapes::Mesh* shape_mesh = shapes::createMeshFromFilename(mesh->filename, scale);
    //size_t vertex_count = shape_mesh->vertex_count;

    for (unsigned int i = 0 ; i < vertex_count ; ++i)
    {
      unsigned int i3 = i * 3;

      tf::Point p(shape_mesh->vertices[i3], shape_mesh->vertices[i3 + 1], shape_mesh->vertices[i3 + 2]); // proj down (z=0)
      tf::Point projectedP = geom_origin*p;
      projectedP.setZ(0.0);
      // transform into local foot frame:
      foot_support_polygon_right_.push_back(projectedP);
    }

    foot_support_polygon_right_ = convexHull(foot_support_polygon_right_);

    // Yuval: add center of support polygon to last point of support polygon vector
    foot_support_polygon_right_.push_back(GetCenterOfPolygon(foot_support_polygon_right_)); // Yuval added

  }

  ROS_INFO("Foot polygon loaded with %zu points", foot_support_polygon_right_.size());

  return true;

}

// Yuval added function:
// Calculate center of support polygon
tf::Point TestStability::GetCenterOfPolygon(const std::vector<tf::Point>& foot_support_polygon) const{
    size_t support_polygon_count = foot_support_polygon.size();
    tf::Point center_support_polygonP = foot_support_polygon[0];
    tf::Point last_vertexP = foot_support_polygon[0];
    int countV = 1;
    // center_support_polygonP.setZero();
    for (unsigned int i = 1 ; i < support_polygon_count ; ++i) // calc. "center mass" of points
    {
        // we take points that are from a certain distance from each other, so we don't give more wieght to curves
        if (last_vertexP.distance(foot_support_polygon[i]) > 0.05){
            center_support_polygonP += foot_support_polygon[i];
            countV += 1;
            last_vertexP = foot_support_polygon[i];
        }

    }
    return (center_support_polygonP/countV);

}

} /* namespace hrl_kinematics */
