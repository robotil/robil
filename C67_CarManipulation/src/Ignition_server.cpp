/*
 * Ignition_server.cpp
 *
 *  Created on: Feb 4, 2013
 *      Author: gotesdyner
 */
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <C67_CarManipulation/IgnitionAction.h>

class IgnitionAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<C67_CarManipulation::IgnitionAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  C67_CarManipulation::IgnitionFeedback feedback_;
  C67_CarManipulation::IgnitionResult result_;

public:

  IgnitionAction(std::string name) :
    as_(nh_, name, boost::bind(&IgnitionAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~IgnitionAction(void)
  {
  }

  void executeCB(const C67_CarManipulation::IgnitionGoalConstPtr &goal)
  {
    // helper variables
	const unsigned int rate = 10;
	const unsigned int timeout = rate*10;
	unsigned int i = 0;
    ros::Rate r(rate);
    bool success = true;


    // push_back the seeds for the fibonacci sequence
//    feedback_.sequence.clear();
//    feedback_.sequence.push_back(0);
//    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    //ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    ROS_INFO("%s: Executing, target turn Ignition %s", action_name_.c_str(), (goal->operation == 1? "ON":"OFF"));
    // start executing the action
    // set busy state
    feedback_.busy = 1;
    // check that preempt has not been requested by the client
    while (ros::ok())
    {
    	// out of limit check
    	if ((goal->operation < 1)||(goal->operation > 2))
    	{
			ROS_INFO("%s: Goal out of range", action_name_.c_str());
			// set the action state to preempted
			as_.setAborted();
			success = false;
			break;
    	}
    	// preempting check
    	if (as_.isPreemptRequested())
        {
        	ROS_INFO("%s: Preempted", action_name_.c_str());
        	// set the action state to preempted
        	as_.setPreempted();
        	success = false;
        	break;
        }
        // check timeout
        i++;
        if (i > timeout)
        {
        	ROS_INFO("%s: timeout", action_name_.c_str());
			// set the action state to timeout
			as_.setAborted();
			success = false;
			break;
        }
        // success state to be removed
        if (i == 3*rate)
        {
        	feedback_.busy  = false;
        	as_.publishFeedback(feedback_);
        	success = true;
        	break;
        }

		// publish the feedback
		as_.publishFeedback(feedback_);
		// this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
		r.sleep();
    }
    result_.success = success;
    if(success)
    {
		//result_.sequence = feedback_.sequence;

		ROS_INFO("%s: Succeeded", action_name_.c_str());
		// set the action state to succeeded
		as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Ignition");

  IgnitionAction ignition(ros::this_node::getName());
  ros::spin();

  return 0;
}



