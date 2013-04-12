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

// #include <C43_LocalBodyCOM/TestStability.h>
#include <C43_LocalBodyCOM/Kinematics.h>

#include <C43_LocalBodyCOM/CoM_Array_msg.h> // Yuval added
#include <ros/ros.h>

using robot_state_publisher::SegmentPair;

namespace C43_LocalBodyCOM {
  
/**
 * Simple node to use the Kinematics class.
 * Based on the current kinematic state, it will output Center of Mass
 */
class CoM_Node_class { // TestStabilityNode {
public:
  //TestStabilityNode(Kinematics::FootSupport support = Kinematics::SUPPORT_DOUBLE);
  //virtual ~TestStabilityNode();

  CoM_Node_class(int stam);//(Kinematics::FootSupport support = Kinematics::SUPPORT_DOUBLE);
  virtual ~CoM_Node_class();
  void jointStateCb(const sensor_msgs::JointStateConstPtr& state);
  CoM_Array_msg getCOM(const std::map<std::string, double>& joint_positions, const ros::Time time_stamp, bool& published);// const;
  /*bool FootSupport_func(C43_LocalBodyCOM::SupportLegs_Status::Request  &req,
                        C43_LocalBodyCOM::SupportLegs_Status::Response &res);
  */

protected:
  ros::NodeHandle nh_; //,nh2_; //Yuval added: nh2_
  ros::Subscriber joint_state_sub_;
  //ros::ServiceServer support_mode_srv_; // Yuval added
  ros::Publisher com_pub_; // visualization_pub_, support_polygon_pub_, pcom_pub_, com_pub_, pcom_err_pub_; //Yuval added: com_pub_, pcom_err_pub_
  //TestStability test_stability_;
  //Kinematics::FootSupport support_mode_;//, support_mode_status_; //Yuval added: support_mode_status_
  ros::Time joints_state_time_stamp_; // Yuval added
  bool published_CoM_; // Yuval added
  tf::TransformBroadcaster br_;
  tf::Transform transform_;
  Kinematics Kinematics_;
  std::vector<tf::Point> com_Arr_; // center of mass to publish in root frame ("pelvis"), Right Support and Left Support

};

//TestStabilityNode::TestStabilityNode(Kinematics::FootSupport support)
//: support_mode_ (support), support_mode_status_ (support)
CoM_Node_class::CoM_Node_class(int stam_get_value)//(Kinematics::FootSupport support)
    //: support_mode_ (support)
{
  ros::NodeHandle nh_private("~");
  /* visualization_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>("stability_visualization", 1); //
  // support_polygon_pub_ = nh_private.advertise<geometry_msgs::PolygonStamped>("support_polygon", 10); // Yuval Comm
  support_polygon_pub_ = nh_private.advertise<visualization_msgs::Marker>("support_polygon", 10); // Yuval added
  pcom_pub_ = nh_private.advertise<visualization_msgs::Marker>("projected_com", 10);//
  // com_pub_ = nh_private.advertise<geometry_msgs::Point>("CoM", 1); //Yuval added
  com_pub_ = nh_private.advertise<CoM_Array_msg>("CoM", 1000); //Yuval added
  pcom_err_pub_ = nh_private.advertise<pCoM_err_msg>("pCoM_err", 1000); //Yuval added

  joint_state_sub_ = nh_.subscribe("/atlas/joint_states", 1000, &TestStabilityNode::jointStateCb, this);
  support_mode_srv_ = nh2_.advertiseService("support_legs", &TestStabilityNode::FootSupport_func, this);  //Yuval added
  published_CoM_ = 1; // Yuval added
  */

  com_pub_ = nh_private.advertise<CoM_Array_msg>("com", 1000); //Yuval added
  joint_state_sub_ = nh_.subscribe("/atlas/joint_states", 1000, &CoM_Node_class::jointStateCb, this);
  published_CoM_ = 1; // Yuval added
}

//TestStabilityNode::~TestStabilityNode(){
//}

CoM_Node_class::~CoM_Node_class()
{
}


void CoM_Node_class::jointStateCb(const sensor_msgs::JointStateConstPtr& state){
  size_t num_joints = state->position.size();
  ROS_DEBUG("Received JointState with %zu joints", num_joints);

  // TODO: need to obtain current support mode
  //delete: support_mode_ = support_mode_status_;//Yuval added

  // get joint positions from state message
  std::map<std::string, double> joint_positions;
  for (unsigned int i=0; i<state->name.size(); i++)
    joint_positions.insert(make_pair(state->name[i], state->position[i]));

  // get time stamp of joint positions state message
  if (published_CoM_) {
      joints_state_time_stamp_ = state->header.stamp;
      published_CoM_ = 0;
  }

  tf::Vector3 normal_vector(0.0, 0.0, 1.0); // planar support
  // for testing, normal vector of arbitrary plane:
  //tf::Vector3 normal_vector(0.5, 0.0, 1.0);
  normal_vector.normalize();

  // bool stable = test_stability_.isPoseStable(joint_positions, support_mode_, normal_vector);

  // print info
  //delete: tf::Point com = test_stability_.get_pCOM();
  //if (stable)
  //  ROS_INFO("Pose is stable, pCOM at %f %f", com.x(), com.y());
  //else
  //  ROS_INFO("Pose is NOT stable, pCOM at %f %f", com.x(), com.y());

  // publish visualization:

  // TODO link coms:
//  visualization_msgs::MarkerArray com_vis_markers;
//
//  visualization_pub_.publish(vis_markers);
//  visualizatiTestStabilityNodeon_msgs::Marker marker;
//  createCoGMarker(kdl_tree_.getRootSegment()->first, "torso", 0.04, com, marker);
//
//  com_vis_markers_.markers.push_back(marker);
//
//
//  visualization_pub_.publish(com_vis_markers_);


  CoM_Array_msg com2bup;
  com2bup = CoM_Node_class::getCOM(joint_positions,joints_state_time_stamp_, published_CoM_);

  transform_.setOrigin( tf::Vector3(com2bup.x[0], com2bup.y[0], com2bup.z[0]) );
  transform_.setRotation( tf::Quaternion(0, 0, 0) );
  br_.sendTransform(tf::StampedTransform(transform_, com2bup.header.stamp, "pelvis", "com"));


  com_pub_.publish(com2bup); //Yuval added

  /*delete:pcom_err_pub_.publish(test_stability_.get_pCOM_err(joints_state_time_stamp_)); //Yuval added


  support_polygon_pub_.publish(test_stability_.getSupportPolygon());
  pcom_pub_.publish(test_stability_.getCOMMarker());
  */

}


CoM_Array_msg CoM_Node_class::getCOM(const std::map<std::string, double>& joint_positions, const ros::Time time_stamp, bool& published) //const
{
tf::Point com; // center of mass in root frame
double m;

// transforms from root to left and right foot:
tf::Transform tf_right_foot, tf_left_foot;
Kinematics_.computeCOM(joint_positions, com, m, tf_right_foot, tf_left_foot);

//tf::Transform tf_to_support;

// com_ = com; //Yuval add
if (com_Arr_.empty())
    { // init vector
    com_Arr_.push_back(com); //Yuval add
    com_Arr_.push_back(tf_right_foot.inverse() * com); //Yuval add
    com_Arr_.push_back(tf_left_foot.inverse() * com); //Yuval add
    }
else{
    com_Arr_[0] = com; //Yuval add
    com_Arr_[1] = tf_right_foot.inverse() * com; //Yuval add
    com_Arr_[2] = tf_left_foot.inverse() * com; //Yuval add
    }

CoM_Array_msg com_point;
com_point.header.stamp = time_stamp;
if (!com_Arr_.empty())
    {
    for (unsigned i = 0; i < 3; ++i)
        {
        com_point.x[i] = com_Arr_[i].x();
        com_point.y[i] = com_Arr_[i].y();
        com_point.z[i] = com_Arr_[i].z();
        }
    published = 1;
    }
return com_point;
}


/*//Yuval added: to use service for Support Legs
bool TestStabilityNode::FootSupport_func(C43_LocalBodyCOM::SupportLegs_Status::Request  &req,
                                         C43_LocalBodyCOM::SupportLegs_Status::Response &res){
    switch ( req.FootSupport_CMD )
    {
    case 0:  //req.SUPPORT_DOUBLE:
        support_mode_status_ = Kinematics::SUPPORT_DOUBLE;
        ROS_INFO("request: SUPPORT_DOUBLE");
        res.FootSupport_Status = support_mode_status_;
        break;
    case 1:  //req.SUPPORT_SINGLE_RIGHT:
        support_mode_status_ = Kinematics::SUPPORT_SINGLE_RIGHT;
        ROS_INFO("request: SUPPORT_SINGLE_RIGH");
        res.FootSupport_Status = support_mode_status_;
        break;
    case 2:  //req.SUPPORT_SINGLE_LEFT:
        support_mode_status_ = Kinematics::SUPPORT_SINGLE_LEFT;
        ROS_INFO("request: SUPPORT_SINGLE_LEFT");
        res.FootSupport_Status = support_mode_status_;
        break;
    case 3:  //req.GET_SUPPORT_STATUS:
        ROS_INFO("request: Get support foot status");
        res.FootSupport_Status = support_mode_status_;
        break;
    default:
        ROS_INFO("request ERROR: Invalid FootSupport_CMD");
        res.FootSupport_Status = req.FootSupport_CMD;
    }
    ROS_INFO("sending back response: [%d]", (int)res.FootSupport_Status);
    return true;
} */

}
using namespace C43_LocalBodyCOM;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "c43_local_body");

 /*Kinematics::FootSupport support = Kinematics::SUPPORT_DOUBLE;

  if (argc == 2){
    int i = atoi(argv[1]);
    if (i >= 0 && i <= 2)
      support = Kinematics::FootSupport(i);

  } */

  try
  {
    ros::spinOnce();
    // C43_LocalBodyCOM::TestStabilityNode StabilityNode(support);
    C43_LocalBodyCOM::CoM_Node_class CoM_Node(1);//(support);

    ros::spin();
  }
  catch(C43_LocalBodyCOM::Kinematics::InitFailed &e)
  {
    std::cerr << "Could not initialize kinematics node: " << e.what() << std::endl;
    // TODO: does not work?
    //ROS_INFO("Could not initialize kinematics node: %s\n", e.what());
    return 1;
  }

  return 0;

}

