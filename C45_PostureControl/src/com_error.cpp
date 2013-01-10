#include "com_error.h"

//namespace hrl_kinematics{

	com_error_node::com_error_node(){
	//: support_mode_ (support), support_mode_status_ (support){
		  ros::NodeHandle nh_private("~");

		  /*
		  joint_state_sub_ = nh_.subscribe("joint_states", 1, &com_error_node::jointStateCb, this);*/

		  pcom_sub_ = nh_.subscribe("projected_com", 1, &com_error_node::get_com_from_hrl_kinematics, this);
		  support_polygon_sub_ = nh_.subscribe("support_polygon", 1, &com_error_node::get_support_polygon_from_hrl_kinematics, this);


		  com_error_srv_ = nh3_.advertiseService("com_error", &com_error_node::result_com_error, this);

		  //support_mode_srv_ = nh2_.advertiseService("support_legs", &com_error_node::FootSupport_func, this);
	}
	com_error_node::~com_error_node(){

	}

	void com_error_node::get_com_from_hrl_kinematics(const visualization_msgs::MarkerConstPtr& comMarker){
		  ROS_INFO("# of points: %d", (int) comMarker->points.size());
		  //this->_CoM.x = com.x();
		  //this->_CoM.y = com.y();
	}

	void com_error_node::get_support_polygon_from_hrl_kinematics(const geometry_msgs::PolygonStampedConstPtr& sp){
		  double avg_x=0, avg_y=0;
		  for(unsigned i=0; i < sp->polygon.points.size(); i++){
			  avg_x += sp->polygon.points.at(i).x;
			  avg_y += sp->polygon.points.at(i).y;
		  }
		  avg_x = avg_x / sp->polygon.points.size();
		  avg_y = avg_y / sp->polygon.points.size();
		  this->_SP.x = avg_x;
		  this->_SP.y = avg_y;
	}

	bool com_error_node::result_com_error(C45_PostureControl::com_error::Request &req, C45_PostureControl::com_error::Response &res)
	{
		//joint_state_sub_ = nh_.subscribe("joint_states", 1, &com_error_node::jointStateCb, this);
		res.x_error = this->_CoM.x - this->_SP.x;
		res.y_error = this->_CoM.y - this->_SP.y;
	  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
	  return true;
	}

	/*void com_error_node::jointStateCb(const sensor_msgs::JointStateConstPtr& state){
	  size_t num_joints = state->position.size();
	  ROS_DEBUG("Received JointState with %zu joints", num_joints);

	  // TODO: need to obtain current support mode
	  support_mode_ = support_mode_status_;//Yuval added

	  // get joint positions from state message
	  std::map<std::string, double> joint_positions;
	  for (unsigned int i=0; i<state->name.size(); i++)
	    joint_positions.insert(make_pair(state->name[i], state->position[i]));

	  tf::Vector3 normal_vector(0.0, 0.0, 1.0); // planar support
	  // for testing, normal vector of arbitrary plane:
	  //tf::Vector3 normal_vector(0.5, 0.0, 1.0);
	  normal_vector.normalize();

	  bool stable = test_stability_.isPoseStable(joint_positions, support_mode_, normal_vector);

//	   //print info
	  tf::Point com1 = test_stability_.getCOM();
	  if (stable)
	    ROS_INFO("Pose is stable, pCOM at %f %f", com1.x(), com1.y());
	  else
	    ROS_INFO("Pose is NOT stable, pCOM at %f %f", com1.x(), com1.y());

	  geometry_msgs::PolygonStamped sp(test_stability_.getSupportPolygon());
	  double avg_x=0, avg_y=0;
	  for(unsigned i=0; i < sp.polygon.points.size(); i++){
		  avg_x += sp.polygon.points.at(i).x;
		  avg_y += sp.polygon.points.at(i).y;
	  }
	  avg_x = avg_x / sp.polygon.points.size();
	  avg_y = avg_y / sp.polygon.points.size();
	  this->_SP.x = avg_x;
	  this->_SP.y = avg_y;
	  ROS_INFO("Got support polygon average: x=%f, y=%f", this->_SP.x, this->_SP.y);

	  tf::Point com(test_stability_.getCOM());
	  		this->_CoM.x = com.x();
	  		this->_CoM.y = com.y();
	  ROS_INFO("Got COM: x=%f, y=%f", this->_CoM.x, this->_CoM.y);
	}*/

	/*bool com_error_node::FootSupport_func(hrl_kinematics::SupportLegs_Status::Request &req,
	                                         hrl_kinematics::SupportLegs_Status::Response &res){
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
	        break;
	    }
	    ROS_INFO("sending back response: [%d]", (int)res.FootSupport_Status);
	    return true;
	}*/
//}

//using namespace hrl_kinematics;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "calculate_com_error_server");
	/*Kinematics::FootSupport support = Kinematics::SUPPORT_DOUBLE;

	if (argc == 2){
		int i = atoi(argv[1]);
		if (i >= 0 && i <= 2)
			support = Kinematics::FootSupport(i);
	}

	try{*/
		com_error_node c = com_error_node();

		ros::spin();
	/*}
	catch(hrl_kinematics::Kinematics::InitFailed &e)
	{
	    std::cerr << "Could not initialize kinematics node: " << e.what() << std::endl;
	    return 1;
	}
	catch(...){
		ROS_INFO("Unknown error");
	}*/

  return 0;
}
