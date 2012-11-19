#include "C11_Node.h"

C11_Node::C11_Node()
{
}

C11_Node::C11_Node(int argc, char** argv ):
       init_argc(argc),
       init_argv(argv)
{

}

C11_Node::~C11_Node() {
  delete nh_;
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
        wait();
}

bool C11_Node::proccess(C11_OperatorControl::C11::Request  &req,
    C11_OperatorControl::C11::Response &res )
          {
                  ROS_INFO("recived request, tying to fetch data\n");

                  return true;
          }

bool C11_Node::init(const std::string &master_url, const std::string &host_url) {
 //       service = nh_.advertiseService("C11", &C11_Node::proccess, this); //Specifying what to do when a service is requested
        ROS_INFO("service on\n");
        std::map<std::string,std::string> remappings;
        remappings["__master"] = master_url;
        remappings["__hostname"] = host_url;
        ros::init(remappings,"C11_Node");
        nh_ = new ros::NodeHandle();
        service = nh_->advertiseService("C11", &C11_Node::proccess, this); //Specifying what to do when a service is requested
        if ( ! ros::master::check() ) {
                return false;
        }
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;
        // Add your ros communications here.
 //       chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
        start();
        return true;
}

bool C11_Node::init() {
        ros::init(init_argc,init_argv,"robilGui");
        if ( ! ros::master::check() ) {
                return false;
        }
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;
        start();
        return true;
}

void C11_Node::run() {
        ros::Rate loop_rate(1);
        int count = 0;
        while ( ros::ok() ) {


                ros::spinOnce();
                loop_rate.sleep();
                ++count;
        }
        std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
        Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

/*int main(int argc, char **argv)
{
  ros::init(argc, argv, "C11_OperatorControl");
  C11_Node my_node();
  while(ros::ok()){
	  ros::spin();
  }
  return 0;
}*/

