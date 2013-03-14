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


#ifndef HRL_KINEMATICS_TESTSTABILITY_H_
#define HRL_KINEMATICS_TESTSTABILITY_H_

#include <hrl_kinematics/Kinematics.h>

#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/PolygonStamped.h>
#include <hrl_kinematics/SupportLegs_Status.h> // Yuval added
#include <hrl_kinematics/CoM_Array_msg.h> // Yuval added
#include <hrl_kinematics/pCoM_err_msg.h> // Yuval added

#include <vector>

namespace hrl_kinematics {

  /**
   * Class to test whether the kinematic state of a (humanoid) robot
   * is statically stable, given its joint configuration, support
   * mode and the normal vector of the supporting plane.
   * @see isPoseStable()
   */
class TestStability : public Kinematics{
public:
  TestStability();
  virtual ~TestStability();

  /**
   * Test stability based on center of mass and support polygon, assuming
   * the robot is standing on a horizontal supporting plane.
   *
   * @param joint_positions given robot configuration
   * @param support_mode which foot the robot is standing on (SUPPORT_SINGLE_RIGHT, SUPPORT_SINGLE_LEFT, SUPPORT_DOUBLE)
   * @return true if pose is stable
   */
  bool isPoseStable(const std::map<std::string, double>& joint_positions, FootSupport support_mode);

  /**
   * Test stability based on center of mass and support polygon, assuming
   * the robot is standing on an arbitrary supporting plane (given by normal_vector)
   *
   * @param joint_positions given robot configuration
   * @param support_mode which foot the robot is standing on (SUPPORT_SINGLE_RIGHT, SUPPORT_SINGLE_LEFT, SUPPORT_DOUBLE)
   * @param normal_vector normal vector of the supporting plane 
   * @return true if pose is stable
   */
  bool isPoseStable(const std::map<std::string, double>& joint_positions, FootSupport support_mode, const tf::Vector3& normal_vector);

  /// @return the support polygon (for visualization)
  // geometry_msgs::PolygonStamped getSupportPolygon() const; //Yuval Comm
  visualization_msgs::Marker getSupportPolygon() const; // Yuval added
  /// @return the center of mass (for visualization)
  visualization_msgs::Marker getCOMMarker() const;
  /// @return the center of mass
  CoM_Array_msg getCOM(const ros::Time time_stamp, bool& published) const; //Yuval add
  // geometry_msgs::Point getCOM() const; //Yuval add
  tf::Point get_pCOM() const {return p_com_;} //Yuval changed from: tf::Point getCOM() const {return p_com_;}
  //tf::Point getCOM() const {return com_;} //Yuval added
  pCoM_err_msg get_pCOM_err(const ros::Time time_stamp) const; //Yuval added

protected:
  void initFootPolygon();
  bool loadFootPolygon();
  std::vector<tf::Point> convexHull(const std::vector<tf::Point>& points) const;
  /// tests if point is in polygon in 2D. point and polygon will be projected down to z=0 (z values will be ignored)
  bool pointInConvexHull(const tf::Point& point, const std::vector<tf::Point>& polygon) const;
  /// Calc. center of support polygon
  tf::Point GetCenterOfPolygon(const std::vector<tf::Point>& polygon) const; // Yuval added
  /// Support polygon (x,y) for the right foot
  std::vector<tf::Point> foot_support_polygon_right_;
  /// Support polygon (x,y) for the left foot (mirrored from right)
  std::vector<tf::Point> foot_support_polygon_left_;
  /// Support polygon (x,y) for the two feet
  std::vector<tf::Point>foot_support_polygon_double_; // Yuval added

  tf::Point p_com_; //, com_;//Yuval add: com_ // center of mass to publish in root frame ("pelvis")
  std::vector<tf::Point> com_Arr_; // center of mass to publish in root frame ("pelvis"), Right Support and Left Support
  tf::Point err_pCoM2SupportCenter_; // Error between projected CoM to center of support polygon
  std::vector<tf::Point> support_polygon_;
  tf::Transform tf_to_support_;
  std::string rfoot_mesh_link_name;

};

} /* namespace hrl_kinematics */
#endif
