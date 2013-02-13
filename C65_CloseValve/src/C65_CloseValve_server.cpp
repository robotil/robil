#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Float32.h>
#include <C65_CloseValve/C65_CloseValveAction.h>

class C65_CloseValve_Server
{
public:
	C65_CloseValve_Server(std::string name) :
	   as_(nh_, name, false),
	   action_name_(name)
	  {
	    //register the goal and feeback callbacks
	    as_.registerGoalCallback(boost::bind(&C65_CloseValve_Server::goalCB, this));
	    as_.registerPreemptCallback(boost::bind(&C65_CloseValve_Server::preemptCB,this));
	    ROS_INFO("%s: Running", action_name_.c_str());

	    //subscribe to the data topic of interest
	    sub_ = nh_.subscribe("/random_number", 1, &C65_CloseValve_Server::analysisCB, this);
	    as_.start();
	  }

	  ~C65_CloseValve_Server(void)
	  {
	  }

	  void goalCB()
	  {
	    // reset helper variables
	    data_count_ = 0;
	    sum_ = 0;
	    sum_sq_ = 0;
	    // accept the new goal
	    goal_ = as_.acceptNewGoal()->max;
	    feedback_.location[0] = 0;
	    feedback_.location[1] = 0;
	    ROS_INFO("Accepted goal: %d", goal_);
	  }

	  void preemptCB()
	  {
	    ROS_INFO("%s: Preempted", action_name_.c_str());
	    // set the action state to preempted
	    as_.setPreempted();
	  }

	  void analysisCB(const std_msgs::Float32::ConstPtr& msg)
	  {
	    // make sure that the action hasn't been canceled
	    if (!as_.isActive())
	      return;

	    feedback_.location[0] = feedback_.location[0] + msg->data;
	    feedback_.location[1] = feedback_.location[1] + (msg->data / 2.0);
	    data_count_++;
        ROS_INFO("Feedback: %f.04 %f.04", feedback_.location[0], feedback_.location[1]);
	    as_.publishFeedback(feedback_);

	    if(data_count_ > goal_)
	    {
	    	result_.success = true;
	      if(result_.success == false)
	      {
	        ROS_INFO("%s: Aborted", action_name_.c_str());
	        //set the action state to aborted
	        as_.setAborted(result_);
	      }
	      else
	      {
	        ROS_INFO("%s: Succeeded", action_name_.c_str());
	        // set the action state to succeeded
	        as_.setSucceeded(result_);
	      }
	    }
	  }

	protected:

	  ros::NodeHandle nh_;
	  actionlib::SimpleActionServer<C65_CloseValve::C65_CloseValveAction> as_;
	  std::string action_name_;
	  int data_count_;
	  int32_t goal_;
	  float sum_, sum_sq_;
	  C65_CloseValve::C65_CloseValveFeedback feedback_;
	  C65_CloseValve::C65_CloseValveResult result_;
	  ros::Subscriber sub_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "C65_CloseValve");

  C65_CloseValve_Server a(ros::this_node::getName());
  ros::spin();

  return 0;
}
