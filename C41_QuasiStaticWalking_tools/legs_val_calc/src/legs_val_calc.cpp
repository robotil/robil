#include <ros/ros.h>
#include <math.h>
#include <Eigen/Dense>
#include <legs_val_calc/legs_val_calc.h>
#include <sensor_msgs/JointState.h>

class legs_val_calculator{
private:
	double q0_l,q1_l,q2_l,q3_l,q4_l,q5_l;
	double q0_r,q1_r,q2_r,q3_r,q4_r,q5_r;
	Eigen::MatrixXf l_jc,r_jc;
	Eigen::MatrixXf l_l_jk;
	ros::NodeHandle nh_,nh2_;
	ros::Subscriber joint_states;
	ros::ServiceServer legs_val_calc_service_;
	ros::ServiceServer LeftFoot2Pelvis_jacobian_service_;
	ros::ServiceServer RightFoot2Pelvis_jacobian_service_;
	ros::ServiceServer walk_val_calc_service_;
	std::map <std::string, int> joints;
	std::vector<double> positions;


public:
	legs_val_calculator(){

		l_jc.resize(6,6),r_jc.resize(6,6);
		l_l_jk.resize(6,12);
		joint_states = nh_.subscribe("/atlas/joint_states",100,&legs_val_calculator::joint_states_CB,this);
		//legs_val_calc_service_= nh_.advertiseService("legs_val_calc_srv",&legs_val_calculator::val_calc_srv_CB,this);
		LeftFoot2Pelvis_jacobian_service_= nh2_.advertiseService("LeftFoot2Pelvis_jacobian_srv",&legs_val_calculator::LeftFoot2Pelvis_jacobian_CB,this);
		RightFoot2Pelvis_jacobian_service_= nh2_.advertiseService("RightFoot2Pelvis_jacobian_srv",&legs_val_calculator::RightFoot2Pelvis_jacobian_CB,this);
		walk_val_calc_service_= nh2_.advertiseService("walk_legs_val_calc_srv",&legs_val_calculator::walk_legs_val_calc_srv_CB,this);


	}

	~legs_val_calculator(){}

	void joint_states_CB(const sensor_msgs::JointStateConstPtr &state){
		for(unsigned int i=0; i < state->name.size(); i++){
			joints[state->name[i]] = i;
		}
		q0_l=state->position[joints["l_leg_uhz"]];
		q1_l=state->position[joints["l_leg_mhx"]];
		q2_l=state->position[joints["l_leg_lhy"]];
		q3_l=state->position[joints["l_leg_kny"]];
		q4_l=state->position[joints["l_leg_uay"]];
		q5_l=state->position[joints["l_leg_lax"]];
		q0_r=state->position[joints["r_leg_uhz"]];
		q1_r=state->position[joints["r_leg_mhx"]];
		q2_r=state->position[joints["r_leg_lhy"]];
		q3_r=state->position[joints["r_leg_kny"]];
		q4_r=state->position[joints["r_leg_uay"]];
		q5_r=state->position[joints["r_leg_lax"]];

	}

	bool walk_legs_val_calc_srv_CB(legs_val_calc::legs_val_calcRequest &req,legs_val_calc::legs_val_calcResponse &res){

		/*left leg jacobian*/
		double c_q0_l=cos(q0_l);
		double s_q0_l=sin(q0_l);
		double c_q1_l=cos(q1_l);
		double s_q1_l=sin(q1_l);
		double c_q2_l=cos(q2_l);
		double s_q2_l=sin(q2_l);
		double c_q3_l=cos(q3_l);
		double s_q3_l=sin(q3_l);
		double c_q4_l=cos(q4_l);
		double s_q4_l=sin(q4_l);
		double c_q5_l=cos(q5_l);
		double s_q5_l=sin(q5_l);



		double temp1 = (s_q1_l*s_q5_l - c_q5_l*(c_q4_l*(c_q1_l*c_q2_l*c_q3_l - c_q1_l*s_q2_l*s_q3_l) - s_q4_l*(c_q1_l*c_q2_l*s_q3_l + c_q1_l*c_q3_l*s_q2_l)));
		double temp2 = (c_q5_l*s_q1_l + s_q5_l*(c_q4_l*(c_q1_l*c_q2_l*c_q3_l - c_q1_l*s_q2_l*s_q3_l) - s_q4_l*(c_q1_l*c_q2_l*s_q3_l + c_q1_l*c_q3_l*s_q2_l)));
		double temp3 = (c_q4_l*(c_q1_l*c_q2_l*s_q3_l + c_q1_l*c_q3_l*s_q2_l) + s_q4_l*(c_q1_l*c_q2_l*c_q3_l - c_q1_l*s_q2_l*s_q3_l));
		double temp4 = (s_q4_l*(c_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l) + s_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l)) - c_q4_l*(c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l) - s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l)));
		double temp5 = (s_q4_l*(c_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l) + s_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l)) - c_q4_l*(c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l) - s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l)));

		l_jc(0,0)=(c_q2_l*s_q0_l)/20 - (c_q0_l*s_q1_l)/20 - s_q0_l/20 + (187*s_q0_l*s_q2_l)/500 + (211*c_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l))/500 + (211*s_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l))/500 - (187*c_q0_l*c_q2_l*s_q1_l)/500 + (c_q0_l*s_q1_l*s_q2_l)/20;
		l_jc(0,1)=(c_q1_l*s_q0_l*s_q2_l)/20 - (187*c_q1_l*c_q2_l*s_q0_l)/500 - (c_q1_l*s_q0_l)/20 - (211*c_q1_l*c_q2_l*c_q3_l*s_q0_l)/500 + (211*c_q1_l*s_q0_l*s_q2_l*s_q3_l)/500;
		l_jc(0,2)=(c_q0_l*s_q2_l)/20 - (187*c_q0_l*c_q2_l)/500 - (211*c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l))/500 + (211*s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l))/500 + (187*s_q0_l*s_q1_l*s_q2_l)/500 + (c_q2_l*s_q0_l*s_q1_l)/20;
		l_jc(0,3)=(211*s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l))/500 - (211*c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l))/500;
		l_jc(0,4)=0;
		l_jc(0,5)=0;
		l_jc(1,0)=c_q0_l/20 - (c_q0_l*c_q2_l)/20 - (187*c_q0_l*s_q2_l)/500 - (s_q0_l*s_q1_l)/20 - (211*c_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l))/500 - (211*s_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l))/500 + (s_q0_l*s_q1_l*s_q2_l)/20 - (187*c_q2_l*s_q0_l*s_q1_l)/500;
		l_jc(1,1)=(c_q0_l*c_q1_l)/20 + (187*c_q0_l*c_q1_l*c_q2_l)/500 - (c_q0_l*c_q1_l*s_q2_l)/20 + (211*c_q0_l*c_q1_l*c_q2_l*c_q3_l)/500 - (211*c_q0_l*c_q1_l*s_q2_l*s_q3_l)/500;
		l_jc(1,2)=(s_q0_l*s_q2_l)/20 - (187*c_q2_l*s_q0_l)/500 - (211*c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l))/500 + (211*s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l))/500 - (c_q0_l*c_q2_l*s_q1_l)/20 - (187*c_q0_l*s_q1_l*s_q2_l)/500;
		l_jc(1,3)=(211*s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l))/500 - (211*c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l))/500;
		l_jc(1,4)=0;
		l_jc(1,5)=0;
		l_jc(2,0)=0;
		l_jc(2,1)=(s_q1_l*(211*cos(q2_l + q3_l) + 187*c_q2_l - 25*s_q2_l + 25))/500;
		l_jc(2,2)=(c_q1_l*(211*sin(q2_l + q3_l) + 25*c_q2_l + 187*s_q2_l))/500;
		l_jc(2,3)=(211*sin(q2_l + q3_l)*c_q1_l)/500;
		l_jc(2,4)=0;
		l_jc(2,5)=0;
		l_jc(3,0)=0;
		l_jc(3,1)=-((c_q1_l*c_q5_l + s_q5_l*(c_q4_l*(s_q1_l*s_q2_l*s_q3_l - c_q2_l*c_q3_l*s_q1_l) + s_q4_l*(c_q2_l*s_q1_l*s_q3_l + c_q3_l*s_q1_l*s_q2_l)))/temp1 - (temp2*(c_q1_l*s_q5_l - c_q5_l*(c_q4_l*(s_q1_l*s_q2_l*s_q3_l - c_q2_l*c_q3_l*s_q1_l) + s_q4_l*(c_q2_l*s_q1_l*s_q3_l + c_q3_l*s_q1_l*s_q2_l))))/pow(temp1,2))/(pow(temp2,2)/pow(temp1,2) + 1);
		l_jc(3,2)=((s_q5_l*temp3)/temp1 + (c_q5_l*temp2*temp3)/pow(temp1,2))/(pow(temp2,2)/pow(temp1,2) + 1);
		l_jc(3,3)=((s_q5_l*temp3)/temp1 + (c_q5_l*temp2*temp3)/pow(temp1,2))/(pow(temp2,2)/pow(temp1,2) + 1);
		l_jc(3,4)=((s_q5_l*temp3)/temp1 + (c_q5_l*temp2*temp3)/pow(temp1,2))/(pow(temp2,2)/pow(temp1,2) + 1);
		l_jc(3,5)=1;
		l_jc(4,0)=0;
		l_jc(4,1)=-((c_q4_l*(c_q2_l*s_q1_l*s_q3_l + c_q3_l*s_q1_l*s_q2_l) - s_q4_l*(s_q1_l*s_q2_l*s_q3_l - c_q2_l*c_q3_l*s_q1_l))/pow(pow(temp2,2) + pow(temp1,2),0.5) + (temp3*(2*temp2*(c_q1_l*c_q5_l + s_q5_l*(c_q4_l*(s_q1_l*s_q2_l*s_q3_l - c_q2_l*c_q3_l*s_q1_l) + s_q4_l*(c_q2_l*s_q1_l*s_q3_l + c_q3_l*s_q1_l*s_q2_l))) + 2*temp1*(c_q1_l*s_q5_l - c_q5_l*(c_q4_l*(s_q1_l*s_q2_l*s_q3_l - c_q2_l*c_q3_l*s_q1_l) + s_q4_l*(c_q2_l*s_q1_l*s_q3_l + c_q3_l*s_q1_l*s_q2_l)))))/(2*pow(pow(temp2,2) + pow(temp1,2),1.5)))/(pow(temp3,2)/(pow(temp2,2) + pow(temp1,2)) + 1);
		l_jc(4,2)=((c_q4_l*(c_q1_l*c_q2_l*c_q3_l - c_q1_l*s_q2_l*s_q3_l) - s_q4_l*(c_q1_l*c_q2_l*s_q3_l + c_q1_l*c_q3_l*s_q2_l))/pow(pow(temp2,2) + pow(temp1,2),0.5) - (temp3*(2*c_q5_l*temp1*temp3 - 2*s_q5_l*temp2*temp3))/(2*pow(pow(temp2,2) + pow(temp1,2),1.5)))/(pow(temp3,2)/(pow(temp2,2) + pow(temp1,2)) + 1);
		l_jc(4,3)=((c_q4_l*(c_q1_l*c_q2_l*c_q3_l - c_q1_l*s_q2_l*s_q3_l) - s_q4_l*(c_q1_l*c_q2_l*s_q3_l + c_q1_l*c_q3_l*s_q2_l))/pow(pow(temp2,2) + pow(temp1,2),0.5) - (temp3*(2*c_q5_l*temp1*temp3 - 2*s_q5_l*temp2*temp3))/(2*pow(pow(temp2,2) + pow(temp1,2),1.5)))/(pow(temp3,2)/(pow(temp2,2) + pow(temp1,2)) + 1);
		l_jc(4,4)=((c_q4_l*(c_q1_l*c_q2_l*c_q3_l - c_q1_l*s_q2_l*s_q3_l) - s_q4_l*(c_q1_l*c_q2_l*s_q3_l + c_q1_l*c_q3_l*s_q2_l))/pow(pow(temp2,2) + pow(temp1,2),0.5) - (temp3*(2*c_q5_l*temp1*temp3 - 2*s_q5_l*temp2*temp3))/(2*pow(pow(temp2,2) + pow(temp1,2),1.5)))/(pow(temp3,2)/(pow(temp2,2) + pow(temp1,2)) + 1);
		l_jc(4,5)=0;
		l_jc(5,0)=1;
		l_jc(5,1)=-((c_q4_l*(c_q0_l*c_q1_l*c_q2_l*s_q3_l + c_q0_l*c_q1_l*c_q3_l*s_q2_l) + s_q4_l*(c_q0_l*c_q1_l*c_q2_l*c_q3_l - c_q0_l*c_q1_l*s_q2_l*s_q3_l))/temp4 + (temp5*(c_q4_l*(c_q1_l*c_q2_l*s_q0_l*s_q3_l + c_q1_l*c_q3_l*s_q0_l*s_q2_l) + s_q4_l*(c_q1_l*c_q2_l*c_q3_l*s_q0_l - c_q1_l*s_q0_l*s_q2_l*s_q3_l)))/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		l_jc(5,2)=((s_q4_l*(c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l) - s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l)) + c_q4_l*(c_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l) + s_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l)))/temp4 - ((s_q4_l*(c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l) - s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l)) + c_q4_l*(c_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l) + s_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l)))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		l_jc(5,3)=((s_q4_l*(c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l) - s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l)) + c_q4_l*(c_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l) + s_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l)))/temp4 - ((s_q4_l*(c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l) - s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l)) + c_q4_l*(c_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l) + s_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l)))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		l_jc(5,4)=((s_q4_l*(c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l) - s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l)) + c_q4_l*(c_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l) + s_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l)))/temp4 - ((s_q4_l*(c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l) - s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l)) + c_q4_l*(c_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l) + s_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l)))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		l_jc(5,5)=0;









/*
		double temp1 = (s_q1_l*s_q5_l - c_q5_l*(c_q4_l*(c_q1_l*c_q2_l*c_q3_l - c_q1_l*s_q2_l*s_q3_l) - s_q4_l*(c_q1_l*c_q2_l*s_q3_l + c_q1_l*c_q3_l*s_q2_l)));
		double temp2 = (c_q4_l*(c_q1_l*c_q2_l*s_q3_l + c_q1_l*c_q3_l*s_q2_l) + s_q4_l*(c_q1_l*c_q2_l*c_q3_l - c_q1_l*s_q2_l*s_q3_l));
		double temp3 = (c_q5_l*s_q1_l + s_q5_l*(c_q4_l*(c_q1_l*c_q2_l*c_q3_l - c_q1_l*s_q2_l*s_q3_l) - s_q4_l*(c_q1_l*c_q2_l*s_q3_l + c_q1_l*c_q3_l*s_q2_l)));
		double temp4 = (s_q4_l*(c_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l) + s_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l)) - c_q4_l*(c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l) - s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l)));
		double temp5 = (s_q4_l*(c_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l) + s_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l)) - c_q4_l*(c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l) - s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l)));

		l_jc(0,0)=(s_q4_l*(c_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l) + s_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l)))/5 - s_q0_l/20 - (c_q0_l*s_q1_l)/20 + (c_q2_l*s_q0_l)/20 + (187*s_q0_l*s_q2_l)/500 - (c_q5_l*(s_q4_l*(c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l) - s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l)) + c_q4_l*(c_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l) + s_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l))))/20 + (211*c_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l))/500 + (211*s_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l))/500 - (c_q4_l*(c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l) - s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l)))/5 - (187*c_q0_l*c_q2_l*s_q1_l)/500 + (c_q0_l*c_q1_l*s_q5_l)/20 + (c_q0_l*s_q1_l*s_q2_l)/20;
		l_jc(0,1)=(c_q5_l*(c_q4_l*(c_q1_l*c_q2_l*c_q3_l*s_q0_l - c_q1_l*s_q0_l*s_q2_l*s_q3_l) - s_q4_l*(c_q1_l*c_q2_l*s_q0_l*s_q3_l + c_q1_l*c_q3_l*s_q0_l*s_q2_l)))/20 - (c_q1_l*s_q0_l)/20 - (c_q4_l*(c_q1_l*c_q2_l*s_q0_l*s_q3_l + c_q1_l*c_q3_l*s_q0_l*s_q2_l))/5 - (s_q4_l*(c_q1_l*c_q2_l*c_q3_l*s_q0_l - c_q1_l*s_q0_l*s_q2_l*s_q3_l))/5 - (s_q0_l*s_q1_l*s_q5_l)/20 - (187*c_q1_l*c_q2_l*s_q0_l)/500 + (c_q1_l*s_q0_l*s_q2_l)/20 - (211*c_q1_l*c_q2_l*c_q3_l*s_q0_l)/500 + (211*c_q1_l*s_q0_l*s_q2_l*s_q3_l)/500;
		l_jc(0,2)=(c_q0_l*s_q2_l)/20 - (187*c_q0_l*c_q2_l)/500 - (s_q4_l*(c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l) - s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l)))/5 - (c_q5_l*temp4)/20 - (211*c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l))/500 + (211*s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l))/500 - (c_q4_l*(c_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l) + s_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l)))/5 + (187*s_q0_l*s_q1_l*s_q2_l)/500 + (c_q2_l*s_q0_l*s_q1_l)/20;
		l_jc(0,3)=(211*s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l))/500 - (c_q5_l*temp4)/20 - (211*c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l))/500 - (s_q4_l*(c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l) - s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l)))/5 - (c_q4_l*(c_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l) + s_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l)))/5;
		l_jc(0,4)=- (s_q4_l*(c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l) - s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l)))/5 - (c_q5_l*temp4)/20 - (c_q4_l*(c_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l) + s_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l)))/5;
		l_jc(0,5)=(c_q1_l*c_q5_l*s_q0_l)/20 - (s_q5_l*(s_q4_l*(c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l) - s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l)) + c_q4_l*(c_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l) + s_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l))))/20;
		l_jc(1,0)=c_q0_l/20 - (s_q4_l*(c_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l) + s_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l)))/5 - (c_q0_l*c_q2_l)/20 - (187*c_q0_l*s_q2_l)/500 - (s_q0_l*s_q1_l)/20 + (c_q5_l*(s_q4_l*(c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l) - s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l)) + c_q4_l*(c_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l) + s_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l))))/20 - (211*c_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l))/500 - (211*s_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l))/500 + (c_q4_l*(c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l) - s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l)))/5 + (s_q0_l*s_q1_l*s_q2_l)/20 - (187*c_q2_l*s_q0_l*s_q1_l)/500 + (c_q1_l*s_q0_l*s_q5_l)/20;
		l_jc(1,1)=(c_q0_l*c_q1_l)/20 + (c_q4_l*(c_q0_l*c_q1_l*c_q2_l*s_q3_l + c_q0_l*c_q1_l*c_q3_l*s_q2_l))/5 + (s_q4_l*(c_q0_l*c_q1_l*c_q2_l*c_q3_l - c_q0_l*c_q1_l*s_q2_l*s_q3_l))/5 - (c_q5_l*(c_q4_l*(c_q0_l*c_q1_l*c_q2_l*c_q3_l - c_q0_l*c_q1_l*s_q2_l*s_q3_l) - s_q4_l*(c_q0_l*c_q1_l*c_q2_l*s_q3_l + c_q0_l*c_q1_l*c_q3_l*s_q2_l)))/20 + (187*c_q0_l*c_q1_l*c_q2_l)/500 - (c_q0_l*c_q1_l*s_q2_l)/20 + (c_q0_l*s_q1_l*s_q5_l)/20 + (211*c_q0_l*c_q1_l*c_q2_l*c_q3_l)/500 - (211*c_q0_l*c_q1_l*s_q2_l*s_q3_l)/500;
		l_jc(1,2)=(s_q0_l*s_q2_l)/20 - (187*c_q2_l*s_q0_l)/500 - (s_q4_l*(c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l) - s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l)))/5 - (c_q5_l*temp5)/20 - (211*c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l))/500 + (211*s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l))/500 - (c_q4_l*(c_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l) + s_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l)))/5 - (c_q0_l*c_q2_l*s_q1_l)/20 - (187*c_q0_l*s_q1_l*s_q2_l)/500;
		l_jc(1,3)=(211*s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l))/500 - (c_q5_l*temp5)/20 - (211*c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l))/500 - (s_q4_l*(c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l) - s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l)))/5 - (c_q4_l*(c_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l) + s_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l)))/5;
		l_jc(1,4)=- (s_q4_l*(c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l) - s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l)))/5 - (c_q5_l*temp5)/20 - (c_q4_l*(c_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l) + s_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l)))/5;
		l_jc(1,5)=- (s_q5_l*(s_q4_l*(c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l) - s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l)) + c_q4_l*(c_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l) + s_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l))))/20 - (c_q0_l*c_q1_l*c_q5_l)/20;
		l_jc(2,0)=0;
		l_jc(2,1)=s_q1_l/20 + (187*c_q2_l*s_q1_l)/500 - (c_q1_l*s_q5_l)/20 - (s_q1_l*s_q2_l)/20 + (c_q4_l*(c_q2_l*s_q1_l*s_q3_l + c_q3_l*s_q1_l*s_q2_l))/5 - (s_q4_l*(s_q1_l*s_q2_l*s_q3_l - c_q2_l*c_q3_l*s_q1_l))/5 + (c_q5_l*(c_q4_l*(s_q1_l*s_q2_l*s_q3_l - c_q2_l*c_q3_l*s_q1_l) + s_q4_l*(c_q2_l*s_q1_l*s_q3_l + c_q3_l*s_q1_l*s_q2_l)))/20 - (211*s_q1_l*s_q2_l*s_q3_l)/500 + (211*c_q2_l*c_q3_l*s_q1_l)/500;
		l_jc(2,2)=(c_q1_l*c_q2_l)/20 + (187*c_q1_l*s_q2_l)/500 - (c_q4_l*(c_q1_l*c_q2_l*c_q3_l - c_q1_l*s_q2_l*s_q3_l))/5 + (s_q4_l*(c_q1_l*c_q2_l*s_q3_l + c_q1_l*c_q3_l*s_q2_l))/5 - (c_q5_l*temp2)/20 + (211*c_q1_l*c_q2_l*s_q3_l)/500 + (211*c_q1_l*c_q3_l*s_q2_l)/500;
		l_jc(2,3)=(s_q4_l*(c_q1_l*c_q2_l*s_q3_l + c_q1_l*c_q3_l*s_q2_l))/5 - (c_q4_l*(c_q1_l*c_q2_l*c_q3_l - c_q1_l*s_q2_l*s_q3_l))/5 - (c_q5_l*temp2)/20 + (211*c_q1_l*c_q2_l*s_q3_l)/500 + (211*c_q1_l*c_q3_l*s_q2_l)/500;
		l_jc(2,4)=(s_q4_l*(c_q1_l*c_q2_l*s_q3_l + c_q1_l*c_q3_l*s_q2_l))/5 - (c_q4_l*(c_q1_l*c_q2_l*c_q3_l - c_q1_l*s_q2_l*s_q3_l))/5 - (c_q5_l*temp2)/20;
		l_jc(2,5)=- (c_q5_l*s_q1_l)/20 - (s_q5_l*(c_q4_l*(c_q1_l*c_q2_l*c_q3_l - c_q1_l*s_q2_l*s_q3_l) - s_q4_l*(c_q1_l*c_q2_l*s_q3_l + c_q1_l*c_q3_l*s_q2_l)))/20;
		l_jc(3,0)=0;
		l_jc(3,1)=-((c_q1_l*c_q5_l + s_q5_l*(c_q4_l*(s_q1_l*s_q2_l*s_q3_l - c_q2_l*c_q3_l*s_q1_l) + s_q4_l*(c_q2_l*s_q1_l*s_q3_l + c_q3_l*s_q1_l*s_q2_l)))/temp1 - (temp3*(c_q1_l*s_q5_l - c_q5_l*(c_q4_l*(s_q1_l*s_q2_l*s_q3_l - c_q2_l*c_q3_l*s_q1_l) + s_q4_l*(c_q2_l*s_q1_l*s_q3_l + c_q3_l*s_q1_l*s_q2_l))))/pow(temp1,2))/(pow(temp3,2)/pow(temp1,2) + 1);
		l_jc(3,2)=((s_q5_l*temp2)/temp1 + (c_q5_l*temp3*temp2)/pow(temp1,2))/(pow(temp3,2)/pow(temp1,2) + 1);
		l_jc(3,3)=((s_q5_l*temp2)/temp1 + (c_q5_l*temp3*temp2)/pow(temp1,2))/(pow(temp3,2)/pow(temp1,2) + 1);
		l_jc(3,4)=((s_q5_l*temp2)/temp1 + (c_q5_l*temp3*temp2)/pow(temp1,2))/(pow(temp3,2)/pow(temp1,2) + 1);
		l_jc(3,5)=1;

		l_jc(4,0)=0;
		l_jc(4,1)=-((c_q4_l*(c_q2_l*s_q1_l*s_q3_l + c_q3_l*s_q1_l*s_q2_l) - s_q4_l*(s_q1_l*s_q2_l*s_q3_l - c_q2_l*c_q3_l*s_q1_l))/pow(pow(temp3,2) + pow(temp1,2),0.5) + (temp2*(2*temp3*(c_q1_l*c_q5_l + s_q5_l*(c_q4_l*(s_q1_l*s_q2_l*s_q3_l - c_q2_l*c_q3_l*s_q1_l) + s_q4_l*(c_q2_l*s_q1_l*s_q3_l + c_q3_l*s_q1_l*s_q2_l))) + 2*temp1*(c_q1_l*s_q5_l - c_q5_l*(c_q4_l*(s_q1_l*s_q2_l*s_q3_l - c_q2_l*c_q3_l*s_q1_l) + s_q4_l*(c_q2_l*s_q1_l*s_q3_l + c_q3_l*s_q1_l*s_q2_l)))))/(2*pow(pow(temp3,2) + pow(temp1,2),1.5)))/(pow(temp2,2)/(pow(temp3,2) + pow(temp1,2)) + 1);
		l_jc(4,2)=((c_q4_l*(c_q1_l*c_q2_l*c_q3_l - c_q1_l*s_q2_l*s_q3_l) - s_q4_l*(c_q1_l*c_q2_l*s_q3_l + c_q1_l*c_q3_l*s_q2_l))/pow(pow(temp3,2) + pow(temp1,2),0.5) - (temp2*(2*c_q5_l*temp1*temp2 - 2*s_q5_l*temp3*temp2))/(2*pow(pow(temp3,2) + pow(temp1,2),1.5)))/(pow(temp2,2)/(pow(temp3,2) + pow(temp1,2)) + 1);
		l_jc(4,3)=((c_q4_l*(c_q1_l*c_q2_l*c_q3_l - c_q1_l*s_q2_l*s_q3_l) - s_q4_l*(c_q1_l*c_q2_l*s_q3_l + c_q1_l*c_q3_l*s_q2_l))/pow(pow(temp3,2) + pow(temp1,2),0.5) - (temp2*(2*c_q5_l*temp1*temp2 - 2*s_q5_l*temp3*temp2))/(2*pow(pow(temp3,2) + pow(temp1,2),1.5)))/(pow(temp2,2)/(pow(temp3,2) + pow(temp1,2)) + 1);
		l_jc(4,4)=((c_q4_l*(c_q1_l*c_q2_l*c_q3_l - c_q1_l*s_q2_l*s_q3_l) - s_q4_l*(c_q1_l*c_q2_l*s_q3_l + c_q1_l*c_q3_l*s_q2_l))/pow(pow(temp3,2) + pow(temp1,2),0.5) - (temp2*(2*c_q5_l*temp1*temp2 - 2*s_q5_l*temp3*temp2))/(2*pow(pow(temp3,2) + pow(temp1,2),1.5)))/(pow(temp2,2)/(pow(temp3,2) + pow(temp1,2)) + 1);
		l_jc(4,5)=0;

		l_jc(5,0)=1;
		l_jc(5,1)=-((c_q4_l*(c_q0_l*c_q1_l*c_q2_l*s_q3_l + c_q0_l*c_q1_l*c_q3_l*s_q2_l) + s_q4_l*(c_q0_l*c_q1_l*c_q2_l*c_q3_l - c_q0_l*c_q1_l*s_q2_l*s_q3_l))/temp4 + (temp5*(c_q4_l*(c_q1_l*c_q2_l*s_q0_l*s_q3_l + c_q1_l*c_q3_l*s_q0_l*s_q2_l) + s_q4_l*(c_q1_l*c_q2_l*c_q3_l*s_q0_l - c_q1_l*s_q0_l*s_q2_l*s_q3_l)))/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		l_jc(5,2)=((s_q4_l*(c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l) - s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l)) + c_q4_l*(c_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l) + s_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l)))/temp4 - ((s_q4_l*(c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l) - s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l)) + c_q4_l*(c_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l) + s_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l)))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		l_jc(5,3)=((s_q4_l*(c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l) - s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l)) + c_q4_l*(c_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l) + s_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l)))/temp4 - ((s_q4_l*(c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l) - s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l)) + c_q4_l*(c_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l) + s_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l)))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		l_jc(5,4)=((s_q4_l*(c_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l) - s_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l)) + c_q4_l*(c_q3_l*(s_q0_l*s_q2_l - c_q0_l*c_q2_l*s_q1_l) + s_q3_l*(c_q2_l*s_q0_l + c_q0_l*s_q1_l*s_q2_l)))/temp4 - ((s_q4_l*(c_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l) - s_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l)) + c_q4_l*(c_q3_l*(c_q0_l*s_q2_l + c_q2_l*s_q0_l*s_q1_l) + s_q3_l*(c_q0_l*c_q2_l - s_q0_l*s_q1_l*s_q2_l)))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		l_jc(5,5)=0;
*/

		/*Right leg jacobian*/

		double c_q0_r=cos(q0_r);
		double s_q0_r=sin(q0_r);
		double c_q1_r=cos(q1_r);
		double s_q1_r=sin(q1_r);
		double c_q2_r=cos(q2_r);
		double s_q2_r=sin(q2_r);
		double c_q3_r=cos(q3_r);
		double s_q3_r=sin(q3_r);
		double c_q4_r=cos(q4_r);
		double s_q4_r=sin(q4_r);
		double c_q5_r=cos(q5_r);
		double s_q5_r=sin(q5_r);


		temp1 = (s_q1_r*s_q5_r - c_q5_r*(c_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r) - s_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r)));
		temp2 = (c_q5_r*s_q1_r + s_q5_r*(c_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r) - s_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r)));
		temp3 = (c_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r) + s_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r));
		temp4 = (s_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r)) - c_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)));
		temp5 = (s_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)) - c_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)));

		r_jc(0,0)=(c_q2_r*s_q0_r)/20 - (c_q0_r*s_q1_r)/20 - s_q0_r/20 + (187*s_q0_r*s_q2_r)/500 + (211*c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r))/500 + (211*s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r))/500 - (187*c_q0_r*c_q2_r*s_q1_r)/500 + (c_q0_r*s_q1_r*s_q2_r)/20;
		r_jc(0,1)=(c_q1_r*s_q0_r*s_q2_r)/20 - (187*c_q1_r*c_q2_r*s_q0_r)/500 - (c_q1_r*s_q0_r)/20 - (211*c_q1_r*c_q2_r*c_q3_r*s_q0_r)/500 + (211*c_q1_r*s_q0_r*s_q2_r*s_q3_r)/500;
		r_jc(0,2)=(c_q0_r*s_q2_r)/20 - (187*c_q0_r*c_q2_r)/500 - (211*c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r))/500 + (211*s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r))/500 + (187*s_q0_r*s_q1_r*s_q2_r)/500 + (c_q2_r*s_q0_r*s_q1_r)/20;
		r_jc(0,3)=(211*s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r))/500 - (211*c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r))/500;
		r_jc(0,4)=0;
		r_jc(0,5)=0;
		r_jc(1,0)=c_q0_r/20 - (c_q0_r*c_q2_r)/20 - (187*c_q0_r*s_q2_r)/500 - (s_q0_r*s_q1_r)/20 - (211*c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r))/500 - (211*s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r))/500 + (s_q0_r*s_q1_r*s_q2_r)/20 - (187*c_q2_r*s_q0_r*s_q1_r)/500;
		r_jc(1,1)=(c_q0_r*c_q1_r)/20 + (187*c_q0_r*c_q1_r*c_q2_r)/500 - (c_q0_r*c_q1_r*s_q2_r)/20 + (211*c_q0_r*c_q1_r*c_q2_r*c_q3_r)/500 - (211*c_q0_r*c_q1_r*s_q2_r*s_q3_r)/500;
		r_jc(1,2)=(s_q0_r*s_q2_r)/20 - (187*c_q2_r*s_q0_r)/500 - (211*c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r))/500 + (211*s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r))/500 - (c_q0_r*c_q2_r*s_q1_r)/20 - (187*c_q0_r*s_q1_r*s_q2_r)/500;
		r_jc(1,3)=(211*s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r))/500 - (211*c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r))/500;
		r_jc(1,4)=0;
		r_jc(1,5)=0;
		r_jc(2,0)=0;
		r_jc(2,1)=(s_q1_r*(211*cos(q2_r + q3_r) + 187*c_q2_r - 25*s_q2_r + 25))/500;
		r_jc(2,2)=(c_q1_r*(211*sin(q2_r + q3_r) + 25*c_q2_r + 187*s_q2_r))/500;
		r_jc(2,3)=(211*sin(q2_r + q3_r)*c_q1_r)/500;
		r_jc(2,4)=0;
		r_jc(2,5)=0;
		r_jc(3,0)=0;
		r_jc(3,1)=-((c_q1_r*c_q5_r + s_q5_r*(c_q4_r*(s_q1_r*s_q2_r*s_q3_r - c_q2_r*c_q3_r*s_q1_r) + s_q4_r*(c_q2_r*s_q1_r*s_q3_r + c_q3_r*s_q1_r*s_q2_r)))/temp1 - (temp2*(c_q1_r*s_q5_r - c_q5_r*(c_q4_r*(s_q1_r*s_q2_r*s_q3_r - c_q2_r*c_q3_r*s_q1_r) + s_q4_r*(c_q2_r*s_q1_r*s_q3_r + c_q3_r*s_q1_r*s_q2_r))))/pow(temp1,2))/(pow(temp2,2)/pow(temp1,2) + 1);
		r_jc(3,2)=((s_q5_r*temp3)/temp1 + (c_q5_r*temp2*temp3)/pow(temp1,2))/(pow(temp2,2)/pow(temp1,2) + 1);
		r_jc(3,3)=((s_q5_r*temp3)/temp1 + (c_q5_r*temp2*temp3)/pow(temp1,2))/(pow(temp2,2)/pow(temp1,2) + 1);
		r_jc(3,4)=((s_q5_r*temp3)/temp1 + (c_q5_r*temp2*temp3)/pow(temp1,2))/(pow(temp2,2)/pow(temp1,2) + 1);
		r_jc(3,5)=1;
		r_jc(4,0)=0;
		r_jc(4,1)=-((c_q4_r*(c_q2_r*s_q1_r*s_q3_r + c_q3_r*s_q1_r*s_q2_r) - s_q4_r*(s_q1_r*s_q2_r*s_q3_r - c_q2_r*c_q3_r*s_q1_r))/pow(pow(temp2,2) + pow(temp1,2),0.5) + (temp3*(2*temp2*(c_q1_r*c_q5_r + s_q5_r*(c_q4_r*(s_q1_r*s_q2_r*s_q3_r - c_q2_r*c_q3_r*s_q1_r) + s_q4_r*(c_q2_r*s_q1_r*s_q3_r + c_q3_r*s_q1_r*s_q2_r))) + 2*temp1*(c_q1_r*s_q5_r - c_q5_r*(c_q4_r*(s_q1_r*s_q2_r*s_q3_r - c_q2_r*c_q3_r*s_q1_r) + s_q4_r*(c_q2_r*s_q1_r*s_q3_r + c_q3_r*s_q1_r*s_q2_r)))))/(2*pow(pow(temp2,2) + pow(temp1,2),1.5)))/(pow(temp3,2)/(pow(temp2,2) + pow(temp1,2)) + 1);
		r_jc(4,2)=((c_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r) - s_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r))/pow(pow(temp2,2) + pow(temp1,2),0.5) - (temp3*(2*c_q5_r*temp1*temp3 - 2*s_q5_r*temp2*temp3))/(2*pow(pow(temp2,2) + pow(temp1,2),1.5)))/(pow(temp3,2)/(pow(temp2,2) + pow(temp1,2)) + 1);
		r_jc(4,3)=((c_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r) - s_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r))/pow(pow(temp2,2) + pow(temp1,2),0.5) - (temp3*(2*c_q5_r*temp1*temp3 - 2*s_q5_r*temp2*temp3))/(2*pow(pow(temp2,2) + pow(temp1,2),1.5)))/(pow(temp3,2)/(pow(temp2,2) + pow(temp1,2)) + 1);
		r_jc(4,4)=((c_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r) - s_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r))/pow(pow(temp2,2) + pow(temp1,2),0.5) - (temp3*(2*c_q5_r*temp1*temp3 - 2*s_q5_r*temp2*temp3))/(2*pow(pow(temp2,2) + pow(temp1,2),1.5)))/(pow(temp3,2)/(pow(temp2,2) + pow(temp1,2)) + 1);
		r_jc(4,5)=0;
		r_jc(5,0)=1;
		r_jc(5,1)=-((c_q4_r*(c_q0_r*c_q1_r*c_q2_r*s_q3_r + c_q0_r*c_q1_r*c_q3_r*s_q2_r) + s_q4_r*(c_q0_r*c_q1_r*c_q2_r*c_q3_r - c_q0_r*c_q1_r*s_q2_r*s_q3_r))/temp4 + (temp5*(c_q4_r*(c_q1_r*c_q2_r*s_q0_r*s_q3_r + c_q1_r*c_q3_r*s_q0_r*s_q2_r) + s_q4_r*(c_q1_r*c_q2_r*c_q3_r*s_q0_r - c_q1_r*s_q0_r*s_q2_r*s_q3_r)))/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		r_jc(5,2)=((s_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)) + c_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)))/temp4 - ((s_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)) + c_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r)))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		r_jc(5,3)=((s_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)) + c_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)))/temp4 - ((s_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)) + c_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r)))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		r_jc(5,4)=((s_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)) + c_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)))/temp4 - ((s_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)) + c_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r)))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		r_jc(5,5)=0;

/*
		temp1 = (s_q1_r*s_q5_r - c_q5_r*(c_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r) - s_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r)));
		temp2 = (c_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r) + s_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r));
		temp3 = (c_q5_r*s_q1_r + s_q5_r*(c_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r) - s_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r)));
		temp4 = (s_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r)) - c_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)));
		temp5 = (s_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)) - c_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)));

		r_jc(0,0)=(s_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)))/5 - s_q0_r/20 - (c_q0_r*s_q1_r)/20 + (c_q2_r*s_q0_r)/20 + (187*s_q0_r*s_q2_r)/500 - (c_q5_r*(s_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)) + c_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r))))/20 + (211*c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r))/500 + (211*s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r))/500 - (c_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)))/5 - (187*c_q0_r*c_q2_r*s_q1_r)/500 + (c_q0_r*c_q1_r*s_q5_r)/20 + (c_q0_r*s_q1_r*s_q2_r)/20;
		r_jc(0,1)=(c_q5_r*(c_q4_r*(c_q1_r*c_q2_r*c_q3_r*s_q0_r - c_q1_r*s_q0_r*s_q2_r*s_q3_r) - s_q4_r*(c_q1_r*c_q2_r*s_q0_r*s_q3_r + c_q1_r*c_q3_r*s_q0_r*s_q2_r)))/20 - (c_q1_r*s_q0_r)/20 - (c_q4_r*(c_q1_r*c_q2_r*s_q0_r*s_q3_r + c_q1_r*c_q3_r*s_q0_r*s_q2_r))/5 - (s_q4_r*(c_q1_r*c_q2_r*c_q3_r*s_q0_r - c_q1_r*s_q0_r*s_q2_r*s_q3_r))/5 - (s_q0_r*s_q1_r*s_q5_r)/20 - (187*c_q1_r*c_q2_r*s_q0_r)/500 + (c_q1_r*s_q0_r*s_q2_r)/20 - (211*c_q1_r*c_q2_r*c_q3_r*s_q0_r)/500 + (211*c_q1_r*s_q0_r*s_q2_r*s_q3_r)/500;
		r_jc(0,2)=(c_q0_r*s_q2_r)/20 - (187*c_q0_r*c_q2_r)/500 - (s_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)))/5 - (c_q5_r*temp4)/20 - (211*c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r))/500 + (211*s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r))/500 - (c_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r)))/5 + (187*s_q0_r*s_q1_r*s_q2_r)/500 + (c_q2_r*s_q0_r*s_q1_r)/20;
		r_jc(0,3)=(211*s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r))/500 - (c_q5_r*temp4)/20 - (211*c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r))/500 - (s_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)))/5 - (c_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r)))/5;
		r_jc(0,4)=- (s_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)))/5 - (c_q5_r*temp4)/20 - (c_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r)))/5;
		r_jc(0,5)=(c_q1_r*c_q5_r*s_q0_r)/20 - (s_q5_r*(s_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)) + c_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r))))/20;
		r_jc(1,0)=c_q0_r/20 - (s_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r)))/5 - (c_q0_r*c_q2_r)/20 - (187*c_q0_r*s_q2_r)/500 - (s_q0_r*s_q1_r)/20 + (c_q5_r*(s_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)) + c_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r))))/20 - (211*c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r))/500 - (211*s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r))/500 + (c_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)))/5 + (s_q0_r*s_q1_r*s_q2_r)/20 - (187*c_q2_r*s_q0_r*s_q1_r)/500 + (c_q1_r*s_q0_r*s_q5_r)/20;
		r_jc(1,1)=(c_q0_r*c_q1_r)/20 + (c_q4_r*(c_q0_r*c_q1_r*c_q2_r*s_q3_r + c_q0_r*c_q1_r*c_q3_r*s_q2_r))/5 + (s_q4_r*(c_q0_r*c_q1_r*c_q2_r*c_q3_r - c_q0_r*c_q1_r*s_q2_r*s_q3_r))/5 - (c_q5_r*(c_q4_r*(c_q0_r*c_q1_r*c_q2_r*c_q3_r - c_q0_r*c_q1_r*s_q2_r*s_q3_r) - s_q4_r*(c_q0_r*c_q1_r*c_q2_r*s_q3_r + c_q0_r*c_q1_r*c_q3_r*s_q2_r)))/20 + (187*c_q0_r*c_q1_r*c_q2_r)/500 - (c_q0_r*c_q1_r*s_q2_r)/20 + (c_q0_r*s_q1_r*s_q5_r)/20 + (211*c_q0_r*c_q1_r*c_q2_r*c_q3_r)/500 - (211*c_q0_r*c_q1_r*s_q2_r*s_q3_r)/500;
		r_jc(1,2)=(s_q0_r*s_q2_r)/20 - (187*c_q2_r*s_q0_r)/500 - (s_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)))/5 - (c_q5_r*temp5)/20 - (211*c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r))/500 + (211*s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r))/500 - (c_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)))/5 - (c_q0_r*c_q2_r*s_q1_r)/20 - (187*c_q0_r*s_q1_r*s_q2_r)/500;
		r_jc(1,3)=(211*s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r))/500 - (c_q5_r*temp5)/20 - (211*c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r))/500 - (s_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)))/5 - (c_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)))/5;
		r_jc(1,4)=- (s_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)))/5 - (c_q5_r*temp5)/20 - (c_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)))/5;
		r_jc(1,5)=- (s_q5_r*(s_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)) + c_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r))))/20 - (c_q0_r*c_q1_r*c_q5_r)/20;
		r_jc(2,0)=0;
		r_jc(2,1)=s_q1_r/20 + (187*c_q2_r*s_q1_r)/500 - (c_q1_r*s_q5_r)/20 - (s_q1_r*s_q2_r)/20 + (c_q4_r*(c_q2_r*s_q1_r*s_q3_r + c_q3_r*s_q1_r*s_q2_r))/5 - (s_q4_r*(s_q1_r*s_q2_r*s_q3_r - c_q2_r*c_q3_r*s_q1_r))/5 + (c_q5_r*(c_q4_r*(s_q1_r*s_q2_r*s_q3_r - c_q2_r*c_q3_r*s_q1_r) + s_q4_r*(c_q2_r*s_q1_r*s_q3_r + c_q3_r*s_q1_r*s_q2_r)))/20 - (211*s_q1_r*s_q2_r*s_q3_r)/500 + (211*c_q2_r*c_q3_r*s_q1_r)/500;
		r_jc(2,2)=(c_q1_r*c_q2_r)/20 + (187*c_q1_r*s_q2_r)/500 - (c_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r))/5 + (s_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r))/5 - (c_q5_r*temp2)/20 + (211*c_q1_r*c_q2_r*s_q3_r)/500 + (211*c_q1_r*c_q3_r*s_q2_r)/500;
		r_jc(2,3)=(s_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r))/5 - (c_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r))/5 - (c_q5_r*temp2)/20 + (211*c_q1_r*c_q2_r*s_q3_r)/500 + (211*c_q1_r*c_q3_r*s_q2_r)/500;
		r_jc(2,4)=(s_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r))/5 - (c_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r))/5 - (c_q5_r*temp2)/20;
		r_jc(2,5)=- (c_q5_r*s_q1_r)/20 - (s_q5_r*(c_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r) - s_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r)))/20;
		r_jc(3,0)=0;
		r_jc(3,1)=-((c_q1_r*c_q5_r + s_q5_r*(c_q4_r*(s_q1_r*s_q2_r*s_q3_r - c_q2_r*c_q3_r*s_q1_r) + s_q4_r*(c_q2_r*s_q1_r*s_q3_r + c_q3_r*s_q1_r*s_q2_r)))/temp1 - (temp3*(c_q1_r*s_q5_r - c_q5_r*(c_q4_r*(s_q1_r*s_q2_r*s_q3_r - c_q2_r*c_q3_r*s_q1_r) + s_q4_r*(c_q2_r*s_q1_r*s_q3_r + c_q3_r*s_q1_r*s_q2_r))))/pow(temp1,2))/(pow(temp3,2)/pow(temp1,2) + 1);
		r_jc(3,2)=((s_q5_r*temp2)/temp1 + (c_q5_r*temp3*temp2)/pow(temp1,2))/(pow(temp3,2)/pow(temp1,2) + 1);
		r_jc(3,3)=((s_q5_r*temp2)/temp1 + (c_q5_r*temp3*temp2)/pow(temp1,2))/(pow(temp3,2)/pow(temp1,2) + 1);
		r_jc(3,4)=((s_q5_r*temp2)/temp1 + (c_q5_r*temp3*temp2)/pow(temp1,2))/(pow(temp3,2)/pow(temp1,2) + 1);
		r_jc(3,5)=1;

		r_jc(4,0)=0;
		r_jc(4,1)=-((c_q4_r*(c_q2_r*s_q1_r*s_q3_r + c_q3_r*s_q1_r*s_q2_r) - s_q4_r*(s_q1_r*s_q2_r*s_q3_r - c_q2_r*c_q3_r*s_q1_r))/pow(pow(temp3,2) + pow(temp1,2),0.5) + (temp2*(2*temp3*(c_q1_r*c_q5_r + s_q5_r*(c_q4_r*(s_q1_r*s_q2_r*s_q3_r - c_q2_r*c_q3_r*s_q1_r) + s_q4_r*(c_q2_r*s_q1_r*s_q3_r + c_q3_r*s_q1_r*s_q2_r))) + 2*temp1*(c_q1_r*s_q5_r - c_q5_r*(c_q4_r*(s_q1_r*s_q2_r*s_q3_r - c_q2_r*c_q3_r*s_q1_r) + s_q4_r*(c_q2_r*s_q1_r*s_q3_r + c_q3_r*s_q1_r*s_q2_r)))))/(2*pow(pow(temp3,2) + pow(temp1,2),1.5)))/(pow(temp2,2)/(pow(temp3,2) + pow(temp1,2)) + 1);
		r_jc(4,2)=((c_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r) - s_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r))/pow(pow(temp3,2) + pow(temp1,2),0.5) - (temp2*(2*c_q5_r*temp1*temp2 - 2*s_q5_r*temp3*temp2))/(2*pow(pow(temp3,2) + pow(temp1,2),1.5)))/(pow(temp2,2)/(pow(temp3,2) + pow(temp1,2)) + 1);
		r_jc(4,3)=((c_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r) - s_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r))/pow(pow(temp3,2) + pow(temp1,2),0.5) - (temp2*(2*c_q5_r*temp1*temp2 - 2*s_q5_r*temp3*temp2))/(2*pow(pow(temp3,2) + pow(temp1,2),1.5)))/(pow(temp2,2)/(pow(temp3,2) + pow(temp1,2)) + 1);
		r_jc(4,4)=((c_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r) - s_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r))/pow(pow(temp3,2) + pow(temp1,2),0.5) - (temp2*(2*c_q5_r*temp1*temp2 - 2*s_q5_r*temp3*temp2))/(2*pow(pow(temp3,2) + pow(temp1,2),1.5)))/(pow(temp2,2)/(pow(temp3,2) + pow(temp1,2)) + 1);
		r_jc(4,5)=0;

		r_jc(5,0)=1;
		r_jc(5,1)=-((c_q4_r*(c_q0_r*c_q1_r*c_q2_r*s_q3_r + c_q0_r*c_q1_r*c_q3_r*s_q2_r) + s_q4_r*(c_q0_r*c_q1_r*c_q2_r*c_q3_r - c_q0_r*c_q1_r*s_q2_r*s_q3_r))/temp4 + (temp5*(c_q4_r*(c_q1_r*c_q2_r*s_q0_r*s_q3_r + c_q1_r*c_q3_r*s_q0_r*s_q2_r) + s_q4_r*(c_q1_r*c_q2_r*c_q3_r*s_q0_r - c_q1_r*s_q0_r*s_q2_r*s_q3_r)))/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		r_jc(5,2)=((s_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)) + c_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)))/temp4 - ((s_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)) + c_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r)))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		r_jc(5,3)=((s_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)) + c_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)))/temp4 - ((s_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)) + c_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r)))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		r_jc(5,4)=((s_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)) + c_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)))/temp4 - ((s_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)) + c_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r)))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		r_jc(5,5)=0;

*/

		/*input vector to Eigen vector*/
		Eigen::VectorXf desired_vel(6);
		desired_vel(0)=req.x_dot;
		desired_vel(1)=req.y_dot;
		desired_vel(2)=req.z_dot;
		desired_vel(3)=req.roll_dot;
		desired_vel(4)=req.pitch_dot;
		desired_vel(5)=req.yaw_dot;

		/*inverse calc*/
		Eigen::MatrixXf l_jc_inverse(6,6),r_jc_inverse(6,6);
		l_jc_inverse=l_jc.inverse();
		r_jc_inverse=r_jc.inverse();

		/*matrix dot vector*/
		Eigen::VectorXf output_left_vel(6);
		Eigen::VectorXf output_right_vel(6);
		output_left_vel=l_jc_inverse*desired_vel;
		output_right_vel=r_jc_inverse*desired_vel;

		/*Eigen vector to output vector*/
		res.q_left_dot[0]=output_left_vel(0);
		res.q_left_dot[1]=output_left_vel(1);
		res.q_left_dot[2]=output_left_vel(2);
		res.q_left_dot[3]=output_left_vel(3);
		res.q_left_dot[4]=output_left_vel(4);
		res.q_left_dot[5]=output_left_vel(5);

		res.q_right_dot[0]=output_right_vel(0);
		res.q_right_dot[1]=output_right_vel(1);
		res.q_right_dot[2]=output_right_vel(2);
		res.q_right_dot[3]=output_right_vel(3);
		res.q_right_dot[4]=output_right_vel(4);
		res.q_right_dot[5]=output_right_vel(5);

		return true;
	}




	bool LeftFoot2Pelvis_jacobian_CB(legs_val_calc::legs_val_calc::Request &req,legs_val_calc::legs_val_calc::Response &res){

		/*left leg jacobian*/
		double c_q0_l=cos(q0_l);
		double s_q0_l=sin(q0_l);
		double c_q1_l=cos(q1_l);
		double s_q1_l=sin(q1_l);
		double c_q2_l=cos(q2_l);
		double s_q2_l=sin(q2_l);
		double c_q3_l=cos(q3_l);
		double s_q3_l=sin(q3_l);
		double c_q4_l=cos(q4_l);
		double s_q4_l=sin(q4_l);
		double c_q5_l=cos(q5_l);
		double s_q5_l=sin(q5_l);

		double temp1 = (cos(q5_l)*(sin(q4_l)*(cos(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l)) - sin(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l))) + cos(q4_l)*(cos(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l)) + sin(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l)))) - cos(q0_l)*cos(q1_l)*sin(q5_l));
		double temp2 = (cos(q5_l)*(sin(q4_l)*(cos(q3_l)*(cos(q0_l)*cos(q2_l) - sin(q0_l)*sin(q1_l)*sin(q2_l)) - sin(q3_l)*(cos(q0_l)*sin(q2_l) + cos(q2_l)*sin(q0_l)*sin(q1_l))) + cos(q4_l)*(cos(q3_l)*(cos(q0_l)*sin(q2_l) + cos(q2_l)*sin(q0_l)*sin(q1_l)) + sin(q3_l)*(cos(q0_l)*cos(q2_l) - sin(q0_l)*sin(q1_l)*sin(q2_l)))) + cos(q1_l)*sin(q0_l)*sin(q5_l));
		double temp3 = (sin(q1_l)*sin(q5_l) - cos(q5_l)*(cos(q4_l)*(cos(q1_l)*cos(q2_l)*cos(q3_l) - cos(q1_l)*sin(q2_l)*sin(q3_l)) - sin(q4_l)*(cos(q1_l)*cos(q2_l)*sin(q3_l) + cos(q1_l)*cos(q3_l)*sin(q2_l))));
		double temp4 = (sin(q4_l)*(cos(q3_l)*(cos(q0_l)*sin(q2_l) + cos(q2_l)*sin(q0_l)*sin(q1_l)) + sin(q3_l)*(cos(q0_l)*cos(q2_l) - sin(q0_l)*sin(q1_l)*sin(q2_l))) - cos(q4_l)*(cos(q3_l)*(cos(q0_l)*cos(q2_l) - sin(q0_l)*sin(q1_l)*sin(q2_l)) - sin(q3_l)*(cos(q0_l)*sin(q2_l) + cos(q2_l)*sin(q0_l)*sin(q1_l))));
		double temp5 = (sin(q5_l)*(sin(q4_l)*(cos(q3_l)*(cos(q0_l)*cos(q2_l) - sin(q0_l)*sin(q1_l)*sin(q2_l)) - sin(q3_l)*(cos(q0_l)*sin(q2_l) + cos(q2_l)*sin(q0_l)*sin(q1_l))) + cos(q4_l)*(cos(q3_l)*(cos(q0_l)*sin(q2_l) + cos(q2_l)*sin(q0_l)*sin(q1_l)) + sin(q3_l)*(cos(q0_l)*cos(q2_l) - sin(q0_l)*sin(q1_l)*sin(q2_l)))) - cos(q1_l)*cos(q5_l)*sin(q0_l));


		l_jc(0,0)=(89*sin(q0_l + q1_l - q2_l - q3_l - q4_l))/4000 - (89*sin(q0_l + q1_l + q2_l + q3_l + q4_l))/4000 + (89*sin(q0_l - q1_l + q2_l + q3_l + q4_l))/4000 - (89*sin(q0_l - q1_l - q2_l - q3_l - q4_l))/4000 - (89*cos(q0_l + q2_l + q3_l + q4_l))/2000 - (89*cos(q0_l - q2_l - q3_l - q4_l))/2000;
		l_jc(0,1)=(89*sin(q0_l + q1_l - q2_l - q3_l - q4_l))/4000 - (89*sin(q0_l + q1_l + q2_l + q3_l + q4_l))/4000 - (89*sin(q0_l - q1_l + q2_l + q3_l + q4_l))/4000 + (89*sin(q0_l - q1_l - q2_l - q3_l - q4_l))/4000;
		l_jc(0,2)=sin(q2_l + q3_l + q4_l)/20 - cos(q2_l + q3_l + q4_l)/20 - (89*sin(q0_l + q1_l + q2_l + q3_l + q4_l))/4000 - (89*sin(q0_l + q1_l - q2_l - q3_l - q4_l))/4000 + (89*sin(q0_l - q1_l + q2_l + q3_l + q4_l))/4000 + (89*sin(q0_l - q1_l - q2_l - q3_l - q4_l))/4000 - (89*cos(q0_l + q2_l + q3_l + q4_l))/2000 + (89*cos(q0_l - q2_l - q3_l - q4_l))/2000;
		l_jc(0,3)=sin(q2_l + q3_l + q4_l)/20 - cos(q2_l + q3_l + q4_l)/20 - (89*sin(q0_l + q1_l + q2_l + q3_l + q4_l))/4000 - (89*sin(q0_l + q1_l - q2_l - q3_l - q4_l))/4000 + (89*sin(q0_l - q1_l + q2_l + q3_l + q4_l))/4000 + (89*sin(q0_l - q1_l - q2_l - q3_l - q4_l))/4000 - (89*cos(q0_l + q2_l + q3_l + q4_l))/2000 - (187*cos(q3_l + q4_l))/500 + (89*cos(q0_l - q2_l - q3_l - q4_l))/2000 - sin(q3_l + q4_l)/20;
		l_jc(0,4)=sin(q2_l + q3_l + q4_l)/20 - cos(q2_l + q3_l + q4_l)/20 - (89*sin(q0_l + q1_l + q2_l + q3_l + q4_l))/4000 - (89*sin(q0_l + q1_l - q2_l - q3_l - q4_l))/4000 + (89*sin(q0_l - q1_l + q2_l + q3_l + q4_l))/4000 + (89*sin(q0_l - q1_l - q2_l - q3_l - q4_l))/4000 - (89*cos(q0_l + q2_l + q3_l + q4_l))/2000 - (187*cos(q3_l + q4_l))/500 + (89*cos(q0_l - q2_l - q3_l - q4_l))/2000 - sin(q3_l + q4_l)/20 - (211*cos(q4_l))/500;
		l_jc(0,5)=0;
		l_jc(1,0)=(89*cos(q1_l)*cos(q5_l)*sin(q0_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q3_l)*sin(q4_l)*sin(q5_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q4_l)*sin(q3_l)*sin(q5_l))/1000 - (89*cos(q0_l)*cos(q3_l)*cos(q4_l)*sin(q2_l)*sin(q5_l))/1000 + (89*cos(q0_l)*sin(q2_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/1000 - (89*cos(q2_l)*cos(q3_l)*cos(q4_l)*sin(q0_l)*sin(q1_l)*sin(q5_l))/1000 + (89*cos(q2_l)*sin(q0_l)*sin(q1_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/1000 + (89*cos(q3_l)*sin(q0_l)*sin(q1_l)*sin(q2_l)*sin(q4_l)*sin(q5_l))/1000 + (89*cos(q4_l)*sin(q0_l)*sin(q1_l)*sin(q2_l)*sin(q3_l)*sin(q5_l))/1000;
		l_jc(1,1)=(89*cos(q0_l)*cos(q5_l)*sin(q1_l))/1000 + (89*cos(q0_l)*cos(q1_l)*cos(q2_l)*cos(q3_l)*cos(q4_l)*sin(q5_l))/1000 - (89*cos(q0_l)*cos(q1_l)*cos(q2_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/1000 - (89*cos(q0_l)*cos(q1_l)*cos(q3_l)*sin(q2_l)*sin(q4_l)*sin(q5_l))/1000 - (89*cos(q0_l)*cos(q1_l)*cos(q4_l)*sin(q2_l)*sin(q3_l)*sin(q5_l))/1000;
		l_jc(1,2)=(cos(q2_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/20 - (cos(q2_l)*cos(q3_l)*sin(q4_l)*sin(q5_l))/20 - (cos(q2_l)*cos(q4_l)*sin(q3_l)*sin(q5_l))/20 - (cos(q3_l)*cos(q4_l)*sin(q2_l)*sin(q5_l))/20 - (cos(q2_l)*cos(q3_l)*cos(q4_l)*sin(q5_l))/20 + (cos(q3_l)*sin(q2_l)*sin(q4_l)*sin(q5_l))/20 + (cos(q4_l)*sin(q2_l)*sin(q3_l)*sin(q5_l))/20 + (sin(q2_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/20 - (89*cos(q2_l)*cos(q3_l)*cos(q4_l)*sin(q0_l)*sin(q5_l))/1000 + (89*cos(q2_l)*sin(q0_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/1000 + (89*cos(q3_l)*sin(q0_l)*sin(q2_l)*sin(q4_l)*sin(q5_l))/1000 + (89*cos(q4_l)*sin(q0_l)*sin(q2_l)*sin(q3_l)*sin(q5_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q3_l)*sin(q1_l)*sin(q4_l)*sin(q5_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q4_l)*sin(q1_l)*sin(q3_l)*sin(q5_l))/1000 - (89*cos(q0_l)*cos(q3_l)*cos(q4_l)*sin(q1_l)*sin(q2_l)*sin(q5_l))/1000 + (89*cos(q0_l)*sin(q1_l)*sin(q2_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/1000;
		l_jc(1,3)=(cos(q3_l)*cos(q4_l)*sin(q5_l))/20 - (sin(q3_l)*sin(q4_l)*sin(q5_l))/20 - (187*cos(q3_l)*sin(q4_l)*sin(q5_l))/500 - (187*cos(q4_l)*sin(q3_l)*sin(q5_l))/500 - (cos(q2_l)*cos(q3_l)*cos(q4_l)*sin(q5_l))/20 - (cos(q2_l)*cos(q3_l)*sin(q4_l)*sin(q5_l))/20 - (cos(q2_l)*cos(q4_l)*sin(q3_l)*sin(q5_l))/20 - (cos(q3_l)*cos(q4_l)*sin(q2_l)*sin(q5_l))/20 + (cos(q2_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/20 + (cos(q3_l)*sin(q2_l)*sin(q4_l)*sin(q5_l))/20 + (cos(q4_l)*sin(q2_l)*sin(q3_l)*sin(q5_l))/20 + (sin(q2_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/20 - (89*cos(q2_l)*cos(q3_l)*cos(q4_l)*sin(q0_l)*sin(q5_l))/1000 + (89*cos(q2_l)*sin(q0_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/1000 + (89*cos(q3_l)*sin(q0_l)*sin(q2_l)*sin(q4_l)*sin(q5_l))/1000 + (89*cos(q4_l)*sin(q0_l)*sin(q2_l)*sin(q3_l)*sin(q5_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q3_l)*sin(q1_l)*sin(q4_l)*sin(q5_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q4_l)*sin(q1_l)*sin(q3_l)*sin(q5_l))/1000 - (89*cos(q0_l)*cos(q3_l)*cos(q4_l)*sin(q1_l)*sin(q2_l)*sin(q5_l))/1000 + (89*cos(q0_l)*sin(q1_l)*sin(q2_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/1000;
		l_jc(1,4)=(cos(q3_l)*cos(q4_l)*sin(q5_l))/20 - (sin(q3_l)*sin(q4_l)*sin(q5_l))/20 - (211*sin(q4_l)*sin(q5_l))/500 - (187*cos(q3_l)*sin(q4_l)*sin(q5_l))/500 - (187*cos(q4_l)*sin(q3_l)*sin(q5_l))/500 - (cos(q2_l)*cos(q3_l)*cos(q4_l)*sin(q5_l))/20 - (cos(q2_l)*cos(q3_l)*sin(q4_l)*sin(q5_l))/20 - (cos(q2_l)*cos(q4_l)*sin(q3_l)*sin(q5_l))/20 - (cos(q3_l)*cos(q4_l)*sin(q2_l)*sin(q5_l))/20 + (cos(q2_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/20 + (cos(q3_l)*sin(q2_l)*sin(q4_l)*sin(q5_l))/20 + (cos(q4_l)*sin(q2_l)*sin(q3_l)*sin(q5_l))/20 + (sin(q2_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/20 - (89*cos(q2_l)*cos(q3_l)*cos(q4_l)*sin(q0_l)*sin(q5_l))/1000 + (89*cos(q2_l)*sin(q0_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/1000 + (89*cos(q3_l)*sin(q0_l)*sin(q2_l)*sin(q4_l)*sin(q5_l))/1000 + (89*cos(q4_l)*sin(q0_l)*sin(q2_l)*sin(q3_l)*sin(q5_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q3_l)*sin(q1_l)*sin(q4_l)*sin(q5_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q4_l)*sin(q1_l)*sin(q3_l)*sin(q5_l))/1000 - (89*cos(q0_l)*cos(q3_l)*cos(q4_l)*sin(q1_l)*sin(q2_l)*sin(q5_l))/1000 + (89*cos(q0_l)*sin(q1_l)*sin(q2_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/1000;
		l_jc(1,5)=(211*cos(q4_l)*cos(q5_l))/500 + (187*cos(q3_l)*cos(q4_l)*cos(q5_l))/500 + (89*cos(q0_l)*cos(q1_l)*sin(q5_l))/1000 + (cos(q3_l)*cos(q5_l)*sin(q4_l))/20 + (cos(q4_l)*cos(q5_l)*sin(q3_l))/20 - (187*cos(q5_l)*sin(q3_l)*sin(q4_l))/500 + (cos(q2_l)*cos(q3_l)*cos(q4_l)*cos(q5_l))/20 - (cos(q2_l)*cos(q3_l)*cos(q5_l)*sin(q4_l))/20 - (cos(q2_l)*cos(q4_l)*cos(q5_l)*sin(q3_l))/20 - (cos(q3_l)*cos(q4_l)*cos(q5_l)*sin(q2_l))/20 - (cos(q2_l)*cos(q5_l)*sin(q3_l)*sin(q4_l))/20 - (cos(q3_l)*cos(q5_l)*sin(q2_l)*sin(q4_l))/20 - (cos(q4_l)*cos(q5_l)*sin(q2_l)*sin(q3_l))/20 + (cos(q5_l)*sin(q2_l)*sin(q3_l)*sin(q4_l))/20 - (89*cos(q2_l)*cos(q3_l)*cos(q5_l)*sin(q0_l)*sin(q4_l))/1000 - (89*cos(q2_l)*cos(q4_l)*cos(q5_l)*sin(q0_l)*sin(q3_l))/1000 - (89*cos(q3_l)*cos(q4_l)*cos(q5_l)*sin(q0_l)*sin(q2_l))/1000 + (89*cos(q5_l)*sin(q0_l)*sin(q2_l)*sin(q3_l)*sin(q4_l))/1000 + (89*cos(q0_l)*cos(q2_l)*cos(q3_l)*cos(q4_l)*cos(q5_l)*sin(q1_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q5_l)*sin(q1_l)*sin(q3_l)*sin(q4_l))/1000 - (89*cos(q0_l)*cos(q3_l)*cos(q5_l)*sin(q1_l)*sin(q2_l)*sin(q4_l))/1000 - (89*cos(q0_l)*cos(q4_l)*cos(q5_l)*sin(q1_l)*sin(q2_l)*sin(q3_l))/1000;
		l_jc(2,0)=(89*cos(q0_l)*cos(q5_l)*sin(q2_l)*sin(q3_l)*sin(q4_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q3_l)*cos(q5_l)*sin(q4_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q4_l)*cos(q5_l)*sin(q3_l))/1000 - (89*cos(q0_l)*cos(q3_l)*cos(q4_l)*cos(q5_l)*sin(q2_l))/1000 - (89*cos(q1_l)*sin(q0_l)*sin(q5_l))/1000 - (89*cos(q2_l)*cos(q3_l)*cos(q4_l)*cos(q5_l)*sin(q0_l)*sin(q1_l))/1000 + (89*cos(q2_l)*cos(q5_l)*sin(q0_l)*sin(q1_l)*sin(q3_l)*sin(q4_l))/1000 + (89*cos(q3_l)*cos(q5_l)*sin(q0_l)*sin(q1_l)*sin(q2_l)*sin(q4_l))/1000 + (89*cos(q4_l)*cos(q5_l)*sin(q0_l)*sin(q1_l)*sin(q2_l)*sin(q3_l))/1000;
		l_jc(2,1)=(89*cos(q0_l)*cos(q1_l)*cos(q2_l)*cos(q3_l)*cos(q4_l)*cos(q5_l))/1000 - (89*cos(q0_l)*sin(q1_l)*sin(q5_l))/1000 - (89*cos(q0_l)*cos(q1_l)*cos(q2_l)*cos(q5_l)*sin(q3_l)*sin(q4_l))/1000 - (89*cos(q0_l)*cos(q1_l)*cos(q3_l)*cos(q5_l)*sin(q2_l)*sin(q4_l))/1000 - (89*cos(q0_l)*cos(q1_l)*cos(q4_l)*cos(q5_l)*sin(q2_l)*sin(q3_l))/1000;
		l_jc(2,2)=(cos(q2_l)*cos(q5_l)*sin(q3_l)*sin(q4_l))/20 - (cos(q2_l)*cos(q3_l)*cos(q5_l)*sin(q4_l))/20 - (cos(q2_l)*cos(q4_l)*cos(q5_l)*sin(q3_l))/20 - (cos(q3_l)*cos(q4_l)*cos(q5_l)*sin(q2_l))/20 - (cos(q2_l)*cos(q3_l)*cos(q4_l)*cos(q5_l))/20 + (cos(q3_l)*cos(q5_l)*sin(q2_l)*sin(q4_l))/20 + (cos(q4_l)*cos(q5_l)*sin(q2_l)*sin(q3_l))/20 + (cos(q5_l)*sin(q2_l)*sin(q3_l)*sin(q4_l))/20 - (89*cos(q2_l)*cos(q3_l)*cos(q4_l)*cos(q5_l)*sin(q0_l))/1000 + (89*cos(q2_l)*cos(q5_l)*sin(q0_l)*sin(q3_l)*sin(q4_l))/1000 + (89*cos(q3_l)*cos(q5_l)*sin(q0_l)*sin(q2_l)*sin(q4_l))/1000 + (89*cos(q4_l)*cos(q5_l)*sin(q0_l)*sin(q2_l)*sin(q3_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q3_l)*cos(q5_l)*sin(q1_l)*sin(q4_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q4_l)*cos(q5_l)*sin(q1_l)*sin(q3_l))/1000 - (89*cos(q0_l)*cos(q3_l)*cos(q4_l)*cos(q5_l)*sin(q1_l)*sin(q2_l))/1000 + (89*cos(q0_l)*cos(q5_l)*sin(q1_l)*sin(q2_l)*sin(q3_l)*sin(q4_l))/1000;
		l_jc(2,3)=(cos(q3_l)*cos(q4_l)*cos(q5_l))/20 - (187*cos(q3_l)*cos(q5_l)*sin(q4_l))/500 - (187*cos(q4_l)*cos(q5_l)*sin(q3_l))/500 - (cos(q5_l)*sin(q3_l)*sin(q4_l))/20 - (cos(q2_l)*cos(q3_l)*cos(q4_l)*cos(q5_l))/20 - (cos(q2_l)*cos(q3_l)*cos(q5_l)*sin(q4_l))/20 - (cos(q2_l)*cos(q4_l)*cos(q5_l)*sin(q3_l))/20 - (cos(q3_l)*cos(q4_l)*cos(q5_l)*sin(q2_l))/20 + (cos(q2_l)*cos(q5_l)*sin(q3_l)*sin(q4_l))/20 + (cos(q3_l)*cos(q5_l)*sin(q2_l)*sin(q4_l))/20 + (cos(q4_l)*cos(q5_l)*sin(q2_l)*sin(q3_l))/20 + (cos(q5_l)*sin(q2_l)*sin(q3_l)*sin(q4_l))/20 - (89*cos(q2_l)*cos(q3_l)*cos(q4_l)*cos(q5_l)*sin(q0_l))/1000 + (89*cos(q2_l)*cos(q5_l)*sin(q0_l)*sin(q3_l)*sin(q4_l))/1000 + (89*cos(q3_l)*cos(q5_l)*sin(q0_l)*sin(q2_l)*sin(q4_l))/1000 + (89*cos(q4_l)*cos(q5_l)*sin(q0_l)*sin(q2_l)*sin(q3_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q3_l)*cos(q5_l)*sin(q1_l)*sin(q4_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q4_l)*cos(q5_l)*sin(q1_l)*sin(q3_l))/1000 - (89*cos(q0_l)*cos(q3_l)*cos(q4_l)*cos(q5_l)*sin(q1_l)*sin(q2_l))/1000 + (89*cos(q0_l)*cos(q5_l)*sin(q1_l)*sin(q2_l)*sin(q3_l)*sin(q4_l))/1000;
		l_jc(2,4)=(cos(q3_l)*cos(q4_l)*cos(q5_l))/20 - (211*cos(q5_l)*sin(q4_l))/500 - (187*cos(q3_l)*cos(q5_l)*sin(q4_l))/500 - (187*cos(q4_l)*cos(q5_l)*sin(q3_l))/500 - (cos(q5_l)*sin(q3_l)*sin(q4_l))/20 - (cos(q2_l)*cos(q3_l)*cos(q4_l)*cos(q5_l))/20 - (cos(q2_l)*cos(q3_l)*cos(q5_l)*sin(q4_l))/20 - (cos(q2_l)*cos(q4_l)*cos(q5_l)*sin(q3_l))/20 - (cos(q3_l)*cos(q4_l)*cos(q5_l)*sin(q2_l))/20 + (cos(q2_l)*cos(q5_l)*sin(q3_l)*sin(q4_l))/20 + (cos(q3_l)*cos(q5_l)*sin(q2_l)*sin(q4_l))/20 + (cos(q4_l)*cos(q5_l)*sin(q2_l)*sin(q3_l))/20 + (cos(q5_l)*sin(q2_l)*sin(q3_l)*sin(q4_l))/20 - (89*cos(q2_l)*cos(q3_l)*cos(q4_l)*cos(q5_l)*sin(q0_l))/1000 + (89*cos(q2_l)*cos(q5_l)*sin(q0_l)*sin(q3_l)*sin(q4_l))/1000 + (89*cos(q3_l)*cos(q5_l)*sin(q0_l)*sin(q2_l)*sin(q4_l))/1000 + (89*cos(q4_l)*cos(q5_l)*sin(q0_l)*sin(q2_l)*sin(q3_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q3_l)*cos(q5_l)*sin(q1_l)*sin(q4_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q4_l)*cos(q5_l)*sin(q1_l)*sin(q3_l))/1000 - (89*cos(q0_l)*cos(q3_l)*cos(q4_l)*cos(q5_l)*sin(q1_l)*sin(q2_l))/1000 + (89*cos(q0_l)*cos(q5_l)*sin(q1_l)*sin(q2_l)*sin(q3_l)*sin(q4_l))/1000;
		l_jc(2,5)=(187*sin(q3_l)*sin(q4_l)*sin(q5_l))/500 - (211*cos(q4_l)*sin(q5_l))/500 + (89*cos(q0_l)*cos(q1_l)*cos(q5_l))/1000 - (187*cos(q3_l)*cos(q4_l)*sin(q5_l))/500 - (cos(q3_l)*sin(q4_l)*sin(q5_l))/20 - (cos(q4_l)*sin(q3_l)*sin(q5_l))/20 - (cos(q2_l)*cos(q3_l)*cos(q4_l)*sin(q5_l))/20 + (cos(q2_l)*cos(q3_l)*sin(q4_l)*sin(q5_l))/20 + (cos(q2_l)*cos(q4_l)*sin(q3_l)*sin(q5_l))/20 + (cos(q3_l)*cos(q4_l)*sin(q2_l)*sin(q5_l))/20 + (cos(q2_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/20 + (cos(q3_l)*sin(q2_l)*sin(q4_l)*sin(q5_l))/20 + (cos(q4_l)*sin(q2_l)*sin(q3_l)*sin(q5_l))/20 - (sin(q2_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/20 + (89*cos(q2_l)*cos(q3_l)*sin(q0_l)*sin(q4_l)*sin(q5_l))/1000 + (89*cos(q2_l)*cos(q4_l)*sin(q0_l)*sin(q3_l)*sin(q5_l))/1000 + (89*cos(q3_l)*cos(q4_l)*sin(q0_l)*sin(q2_l)*sin(q5_l))/1000 - (89*sin(q0_l)*sin(q2_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/1000 - (89*cos(q0_l)*cos(q2_l)*cos(q3_l)*cos(q4_l)*sin(q1_l)*sin(q5_l))/1000 + (89*cos(q0_l)*cos(q2_l)*sin(q1_l)*sin(q3_l)*sin(q4_l)*sin(q5_l))/1000 + (89*cos(q0_l)*cos(q3_l)*sin(q1_l)*sin(q2_l)*sin(q4_l)*sin(q5_l))/1000 + (89*cos(q0_l)*cos(q4_l)*sin(q1_l)*sin(q2_l)*sin(q3_l)*sin(q5_l))/1000;
		l_jc(3,0)=-temp2/(temp3*(pow(temp1,2)/pow(temp3,2) + 1));
		l_jc(3,1)=((cos(q5_l)*(cos(q4_l)*(cos(q0_l)*cos(q1_l)*cos(q2_l)*cos(q3_l) - cos(q0_l)*cos(q1_l)*sin(q2_l)*sin(q3_l)) - sin(q4_l)*(cos(q0_l)*cos(q1_l)*cos(q2_l)*sin(q3_l) + cos(q0_l)*cos(q1_l)*cos(q3_l)*sin(q2_l))) - cos(q0_l)*sin(q1_l)*sin(q5_l))/temp3 + ((cos(q1_l)*sin(q5_l) - cos(q5_l)*(cos(q4_l)*(sin(q1_l)*sin(q2_l)*sin(q3_l) - cos(q2_l)*cos(q3_l)*sin(q1_l)) + sin(q4_l)*(cos(q2_l)*sin(q1_l)*sin(q3_l) + cos(q3_l)*sin(q1_l)*sin(q2_l))))*temp1)/pow(temp3,2))/(pow(temp1,2)/pow(temp3,2) + 1);
		l_jc(3,2)=((cos(q5_l)*(sin(q4_l)*(cos(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l)) + sin(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l))) - cos(q4_l)*(cos(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l)) - sin(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l)))))/temp3 + (cos(q5_l)*(cos(q4_l)*(cos(q1_l)*cos(q2_l)*sin(q3_l) + cos(q1_l)*cos(q3_l)*sin(q2_l)) + sin(q4_l)*(cos(q1_l)*cos(q2_l)*cos(q3_l) - cos(q1_l)*sin(q2_l)*sin(q3_l)))*temp1)/pow(temp3,2))/(pow(temp1,2)/pow(temp3,2) + 1);
		l_jc(3,3)=((cos(q5_l)*(sin(q4_l)*(cos(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l)) + sin(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l))) - cos(q4_l)*(cos(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l)) - sin(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l)))))/temp3 + (cos(q5_l)*(cos(q4_l)*(cos(q1_l)*cos(q2_l)*sin(q3_l) + cos(q1_l)*cos(q3_l)*sin(q2_l)) + sin(q4_l)*(cos(q1_l)*cos(q2_l)*cos(q3_l) - cos(q1_l)*sin(q2_l)*sin(q3_l)))*temp1)/pow(temp3,2))/(pow(temp1,2)/pow(temp3,2) + 1);
		l_jc(3,4)=((cos(q5_l)*(sin(q4_l)*(cos(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l)) + sin(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l))) - cos(q4_l)*(cos(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l)) - sin(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l)))))/temp3 + (cos(q5_l)*(cos(q4_l)*(cos(q1_l)*cos(q2_l)*sin(q3_l) + cos(q1_l)*cos(q3_l)*sin(q2_l)) + sin(q4_l)*(cos(q1_l)*cos(q2_l)*cos(q3_l) - cos(q1_l)*sin(q2_l)*sin(q3_l)))*temp1)/pow(temp3,2))/(pow(temp1,2)/pow(temp3,2) + 1);
		l_jc(3,5)=((sin(q5_l)*(sin(q4_l)*(cos(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l)) - sin(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l))) + cos(q4_l)*(cos(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l)) + sin(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l)))) + cos(q0_l)*cos(q1_l)*cos(q5_l))/temp3 + ((cos(q5_l)*sin(q1_l) + sin(q5_l)*(cos(q4_l)*(cos(q1_l)*cos(q2_l)*cos(q3_l) - cos(q1_l)*sin(q2_l)*sin(q3_l)) - sin(q4_l)*(cos(q1_l)*cos(q2_l)*sin(q3_l) + cos(q1_l)*cos(q3_l)*sin(q2_l))))*temp1)/pow(temp3,2))/(pow(temp1,2)/pow(temp3,2) + 1);
		l_jc(4,0)=(temp1/pow(pow(temp1,2) + pow(temp3,2),0.5) + (temp1*pow(temp2,2))/pow(pow(temp1,2) + pow(temp3,2),1.5))/(pow(temp2,2)/(pow(temp1,2) + pow(temp3,2)) + 1);
		l_jc(4,1)=-((cos(q5_l)*(cos(q4_l)*(cos(q1_l)*cos(q2_l)*cos(q3_l)*sin(q0_l) - cos(q1_l)*sin(q0_l)*sin(q2_l)*sin(q3_l)) - sin(q4_l)*(cos(q1_l)*cos(q2_l)*sin(q0_l)*sin(q3_l) + cos(q1_l)*cos(q3_l)*sin(q0_l)*sin(q2_l))) - sin(q0_l)*sin(q1_l)*sin(q5_l))/pow(pow(temp1,2) + pow(temp3,2),0.5) + ((2*(cos(q5_l)*(cos(q4_l)*(cos(q0_l)*cos(q1_l)*cos(q2_l)*cos(q3_l) - cos(q0_l)*cos(q1_l)*sin(q2_l)*sin(q3_l)) - sin(q4_l)*(cos(q0_l)*cos(q1_l)*cos(q2_l)*sin(q3_l) + cos(q0_l)*cos(q1_l)*cos(q3_l)*sin(q2_l))) - cos(q0_l)*sin(q1_l)*sin(q5_l))*temp1 - 2*temp3*(cos(q1_l)*sin(q5_l) - cos(q5_l)*(cos(q4_l)*(sin(q1_l)*sin(q2_l)*sin(q3_l) - cos(q2_l)*cos(q3_l)*sin(q1_l)) + sin(q4_l)*(cos(q2_l)*sin(q1_l)*sin(q3_l) + cos(q3_l)*sin(q1_l)*sin(q2_l)))))*temp2)/(2*pow(pow(temp1,2) + pow(temp3,2),1.5)))/(pow(temp2,2)/(pow(temp1,2) + pow(temp3,2)) + 1);
		l_jc(4,2)=-(((2*cos(q5_l)*(sin(q4_l)*(cos(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l)) + sin(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l))) - cos(q4_l)*(cos(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l)) - sin(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l))))*temp1 - 2*cos(q5_l)*temp3*(cos(q4_l)*(cos(q1_l)*cos(q2_l)*sin(q3_l) + cos(q1_l)*cos(q3_l)*sin(q2_l)) + sin(q4_l)*(cos(q1_l)*cos(q2_l)*cos(q3_l) - cos(q1_l)*sin(q2_l)*sin(q3_l))))*temp2)/(2*pow(pow(temp1,2) + pow(temp3,2),1.5)) - (cos(q5_l)*temp4)/pow(pow(temp1,2) + pow(temp3,2),0.5))/(pow(temp2,2)/(pow(temp1,2) + pow(temp3,2)) + 1);
		l_jc(4,3)=-(((2*cos(q5_l)*(sin(q4_l)*(cos(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l)) + sin(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l))) - cos(q4_l)*(cos(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l)) - sin(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l))))*temp1 - 2*cos(q5_l)*temp3*(cos(q4_l)*(cos(q1_l)*cos(q2_l)*sin(q3_l) + cos(q1_l)*cos(q3_l)*sin(q2_l)) + sin(q4_l)*(cos(q1_l)*cos(q2_l)*cos(q3_l) - cos(q1_l)*sin(q2_l)*sin(q3_l))))*temp2)/(2*pow(pow(temp1,2) + pow(temp3,2),1.5)) - (cos(q5_l)*temp4)/pow(pow(temp1,2) + pow(temp3,2),0.5))/(pow(temp2,2)/(pow(temp1,2) + pow(temp3,2)) + 1);
		l_jc(4,4)=-(((2*cos(q5_l)*(sin(q4_l)*(cos(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l)) + sin(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l))) - cos(q4_l)*(cos(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l)) - sin(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l))))*temp1 - 2*cos(q5_l)*temp3*(cos(q4_l)*(cos(q1_l)*cos(q2_l)*sin(q3_l) + cos(q1_l)*cos(q3_l)*sin(q2_l)) + sin(q4_l)*(cos(q1_l)*cos(q2_l)*cos(q3_l) - cos(q1_l)*sin(q2_l)*sin(q3_l))))*temp2)/(2*pow(pow(temp1,2) + pow(temp3,2),1.5)) - (cos(q5_l)*temp4)/pow(pow(temp1,2) + pow(temp3,2),0.5))/(pow(temp2,2)/(pow(temp1,2) + pow(temp3,2)) + 1);
		l_jc(4,5)=(temp5/pow(pow(temp1,2) + pow(temp3,2),0.5) - ((2*(sin(q5_l)*(sin(q4_l)*(cos(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l)) - sin(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l))) + cos(q4_l)*(cos(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l)) + sin(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l)))) + cos(q0_l)*cos(q1_l)*cos(q5_l))*temp1 - 2*(cos(q5_l)*sin(q1_l) + sin(q5_l)*(cos(q4_l)*(cos(q1_l)*cos(q2_l)*cos(q3_l) - cos(q1_l)*sin(q2_l)*sin(q3_l)) - sin(q4_l)*(cos(q1_l)*cos(q2_l)*sin(q3_l) + cos(q1_l)*cos(q3_l)*sin(q2_l))))*temp3)*temp2)/(2*pow(pow(temp1,2) + pow(temp3,2),1.5)))/(pow(temp2,2)/(pow(temp1,2) + pow(temp3,2)) + 1);
		l_jc(5,0)=((sin(q5_l)*(sin(q4_l)*(cos(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l)) - sin(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l))) + cos(q4_l)*(cos(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l)) + sin(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l)))) + cos(q0_l)*cos(q1_l)*cos(q5_l))/temp4 - ((sin(q4_l)*(cos(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l)) + sin(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l))) - cos(q4_l)*(cos(q3_l)*(cos(q2_l)*sin(q0_l) + cos(q0_l)*sin(q1_l)*sin(q2_l)) - sin(q3_l)*(sin(q0_l)*sin(q2_l) - cos(q0_l)*cos(q2_l)*sin(q1_l))))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		l_jc(5,1)=-((sin(q5_l)*(cos(q4_l)*(cos(q1_l)*cos(q2_l)*cos(q3_l)*sin(q0_l) - cos(q1_l)*sin(q0_l)*sin(q2_l)*sin(q3_l)) - sin(q4_l)*(cos(q1_l)*cos(q2_l)*sin(q0_l)*sin(q3_l) + cos(q1_l)*cos(q3_l)*sin(q0_l)*sin(q2_l))) + cos(q5_l)*sin(q0_l)*sin(q1_l))/temp4 - ((cos(q4_l)*(cos(q1_l)*cos(q2_l)*sin(q0_l)*sin(q3_l) + cos(q1_l)*cos(q3_l)*sin(q0_l)*sin(q2_l)) + sin(q4_l)*(cos(q1_l)*cos(q2_l)*cos(q3_l)*sin(q0_l) - cos(q1_l)*sin(q0_l)*sin(q2_l)*sin(q3_l)))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		l_jc(5,2)=(sin(q5_l) + ((sin(q4_l)*(cos(q3_l)*(cos(q0_l)*cos(q2_l) - sin(q0_l)*sin(q1_l)*sin(q2_l)) - sin(q3_l)*(cos(q0_l)*sin(q2_l) + cos(q2_l)*sin(q0_l)*sin(q1_l))) + cos(q4_l)*(cos(q3_l)*(cos(q0_l)*sin(q2_l) + cos(q2_l)*sin(q0_l)*sin(q1_l)) + sin(q3_l)*(cos(q0_l)*cos(q2_l) - sin(q0_l)*sin(q1_l)*sin(q2_l))))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		l_jc(5,3)=(sin(q5_l) + ((sin(q4_l)*(cos(q3_l)*(cos(q0_l)*cos(q2_l) - sin(q0_l)*sin(q1_l)*sin(q2_l)) - sin(q3_l)*(cos(q0_l)*sin(q2_l) + cos(q2_l)*sin(q0_l)*sin(q1_l))) + cos(q4_l)*(cos(q3_l)*(cos(q0_l)*sin(q2_l) + cos(q2_l)*sin(q0_l)*sin(q1_l)) + sin(q3_l)*(cos(q0_l)*cos(q2_l) - sin(q0_l)*sin(q1_l)*sin(q2_l))))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		l_jc(5,4)=(sin(q5_l) + ((sin(q4_l)*(cos(q3_l)*(cos(q0_l)*cos(q2_l) - sin(q0_l)*sin(q1_l)*sin(q2_l)) - sin(q3_l)*(cos(q0_l)*sin(q2_l) + cos(q2_l)*sin(q0_l)*sin(q1_l))) + cos(q4_l)*(cos(q3_l)*(cos(q0_l)*sin(q2_l) + cos(q2_l)*sin(q0_l)*sin(q1_l)) + sin(q3_l)*(cos(q0_l)*cos(q2_l) - sin(q0_l)*sin(q1_l)*sin(q2_l))))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		l_jc(5,5)=-temp2/(temp4*(pow(temp5,2)/pow(temp4,2) + 1));


		/*input vector to Eigen vector*/
		Eigen::VectorXf desired_vel(6);
		desired_vel(0)=req.x_dot;
		desired_vel(1)=req.y_dot;
		desired_vel(2)=req.z_dot;
		desired_vel(3)=req.roll_dot;
		desired_vel(4)=req.pitch_dot;
		desired_vel(5)=req.yaw_dot;

		/*inverse calc*/
		Eigen::MatrixXf l_jc_inverse(6,6);
		l_jc_inverse=l_jc.inverse();

		/*matrix dot vector*/
		Eigen::VectorXf output_left_vel(6);
		Eigen::VectorXf output_right_vel(6);
		output_left_vel=l_jc_inverse*desired_vel;

		/*Eigen vector to output vector*/
		res.q_left_dot[0]=output_left_vel(0);
		res.q_left_dot[1]=output_left_vel(1);
		res.q_left_dot[2]=output_left_vel(2);
		res.q_left_dot[3]=output_left_vel(3);
		res.q_left_dot[4]=output_left_vel(4);
		res.q_left_dot[5]=output_left_vel(5);

		return true;
	}


	bool RightFoot2Pelvis_jacobian_CB(legs_val_calc::legs_val_calc::Request &req,legs_val_calc::legs_val_calc::Response &res){

		/*Right leg jacobian*/

		double c_q0_r=cos(q0_r);
		double s_q0_r=sin(q0_r);
		double c_q1_r=cos(q1_r);
		double s_q1_r=sin(q1_r);
		double c_q2_r=cos(q2_r);
		double s_q2_r=sin(q2_r);
		double c_q3_r=cos(q3_r);
		double s_q3_r=sin(q3_r);
		double c_q4_r=cos(q4_r);
		double s_q4_r=sin(q4_r);
		double c_q5_r=cos(q5_r);
		double s_q5_r=sin(q5_r);

		double temp1 = (c_q5_r*(s_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)) + c_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r))) - c_q0_r*c_q1_r*s_q5_r);
		double temp2 = (c_q5_r*(s_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)) + c_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r))) + c_q1_r*s_q0_r*s_q5_r);
		double temp3 = (s_q1_r*s_q5_r - c_q5_r*(c_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r) - s_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r)));
		double temp4 = (s_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r)) - c_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)));
		double temp5 = (s_q5_r*(s_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)) + c_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r))) - c_q1_r*c_q5_r*s_q0_r);

		r_jc(0,0)=(89*sin(q0_r + q1_r + q2_r + q3_r + q4_r))/4000 - (89*sin(q0_r + q1_r - q2_r - q3_r - q4_r))/4000 - (89*sin(q0_r - q1_r + q2_r + q3_r + q4_r))/4000 + (89*sin(q0_r - q1_r - q2_r - q3_r - q4_r))/4000 + (89*cos(q0_r + q2_r + q3_r + q4_r))/2000 + (89*cos(q0_r - q2_r - q3_r - q4_r))/2000;
		r_jc(0,1)=(89*sin(q0_r + q1_r + q2_r + q3_r + q4_r))/4000 - (89*sin(q0_r + q1_r - q2_r - q3_r - q4_r))/4000 + (89*sin(q0_r - q1_r + q2_r + q3_r + q4_r))/4000 - (89*sin(q0_r - q1_r - q2_r - q3_r - q4_r))/4000;
		r_jc(0,2)=(89*sin(q0_r + q1_r + q2_r + q3_r + q4_r))/4000 - cos(q2_r + q3_r + q4_r)/20 + sin(q2_r + q3_r + q4_r)/20 + (89*sin(q0_r + q1_r - q2_r - q3_r - q4_r))/4000 - (89*sin(q0_r - q1_r + q2_r + q3_r + q4_r))/4000 - (89*sin(q0_r - q1_r - q2_r - q3_r - q4_r))/4000 + (89*cos(q0_r + q2_r + q3_r + q4_r))/2000 - (89*cos(q0_r - q2_r - q3_r - q4_r))/2000;
		r_jc(0,3)=(89*sin(q0_r + q1_r + q2_r + q3_r + q4_r))/4000 - cos(q2_r + q3_r + q4_r)/20 + sin(q2_r + q3_r + q4_r)/20 + (89*sin(q0_r + q1_r - q2_r - q3_r - q4_r))/4000 - (89*sin(q0_r - q1_r + q2_r + q3_r + q4_r))/4000 - (89*sin(q0_r - q1_r - q2_r - q3_r - q4_r))/4000 + (89*cos(q0_r + q2_r + q3_r + q4_r))/2000 - (187*cos(q3_r + q4_r))/500 - (89*cos(q0_r - q2_r - q3_r - q4_r))/2000 - sin(q3_r + q4_r)/20;
		r_jc(0,4)=(89*sin(q0_r + q1_r + q2_r + q3_r + q4_r))/4000 - cos(q2_r + q3_r + q4_r)/20 + sin(q2_r + q3_r + q4_r)/20 + (89*sin(q0_r + q1_r - q2_r - q3_r - q4_r))/4000 - (89*sin(q0_r - q1_r + q2_r + q3_r + q4_r))/4000 - (89*sin(q0_r - q1_r - q2_r - q3_r - q4_r))/4000 + (89*cos(q0_r + q2_r + q3_r + q4_r))/2000 - (187*cos(q3_r + q4_r))/500 - (89*cos(q0_r - q2_r - q3_r - q4_r))/2000 - sin(q3_r + q4_r)/20 - (211*c_q4_r)/500;
		r_jc(0,5)=0;
		r_jc(1,0)=(89*c_q0_r*c_q2_r*c_q3_r*s_q4_r*s_q5_r)/1000 - (89*c_q1_r*c_q5_r*s_q0_r)/1000 + (89*c_q0_r*c_q2_r*c_q4_r*s_q3_r*s_q5_r)/1000 + (89*c_q0_r*c_q3_r*c_q4_r*s_q2_r*s_q5_r)/1000 - (89*c_q0_r*s_q2_r*s_q3_r*s_q4_r*s_q5_r)/1000 + (89*c_q2_r*c_q3_r*c_q4_r*s_q0_r*s_q1_r*s_q5_r)/1000 - (89*c_q2_r*s_q0_r*s_q1_r*s_q3_r*s_q4_r*s_q5_r)/1000 - (89*c_q3_r*s_q0_r*s_q1_r*s_q2_r*s_q4_r*s_q5_r)/1000 - (89*c_q4_r*s_q0_r*s_q1_r*s_q2_r*s_q3_r*s_q5_r)/1000;
		r_jc(1,1)=(89*c_q0_r*c_q1_r*c_q2_r*s_q3_r*s_q4_r*s_q5_r)/1000 - (89*c_q0_r*c_q1_r*c_q2_r*c_q3_r*c_q4_r*s_q5_r)/1000 - (89*c_q0_r*c_q5_r*s_q1_r)/1000 + (89*c_q0_r*c_q1_r*c_q3_r*s_q2_r*s_q4_r*s_q5_r)/1000 + (89*c_q0_r*c_q1_r*c_q4_r*s_q2_r*s_q3_r*s_q5_r)/1000;
		r_jc(1,2)=(c_q2_r*s_q3_r*s_q4_r*s_q5_r)/20 - (c_q2_r*c_q3_r*s_q4_r*s_q5_r)/20 - (c_q2_r*c_q4_r*s_q3_r*s_q5_r)/20 - (c_q3_r*c_q4_r*s_q2_r*s_q5_r)/20 - (c_q2_r*c_q3_r*c_q4_r*s_q5_r)/20 + (c_q3_r*s_q2_r*s_q4_r*s_q5_r)/20 + (c_q4_r*s_q2_r*s_q3_r*s_q5_r)/20 + (s_q2_r*s_q3_r*s_q4_r*s_q5_r)/20 + (89*c_q2_r*c_q3_r*c_q4_r*s_q0_r*s_q5_r)/1000 - (89*c_q2_r*s_q0_r*s_q3_r*s_q4_r*s_q5_r)/1000 - (89*c_q3_r*s_q0_r*s_q2_r*s_q4_r*s_q5_r)/1000 - (89*c_q4_r*s_q0_r*s_q2_r*s_q3_r*s_q5_r)/1000 + (89*c_q0_r*c_q2_r*c_q3_r*s_q1_r*s_q4_r*s_q5_r)/1000 + (89*c_q0_r*c_q2_r*c_q4_r*s_q1_r*s_q3_r*s_q5_r)/1000 + (89*c_q0_r*c_q3_r*c_q4_r*s_q1_r*s_q2_r*s_q5_r)/1000 - (89*c_q0_r*s_q1_r*s_q2_r*s_q3_r*s_q4_r*s_q5_r)/1000;
		r_jc(1,3)=(c_q3_r*c_q4_r*s_q5_r)/20 - (s_q3_r*s_q4_r*s_q5_r)/20 - (187*c_q3_r*s_q4_r*s_q5_r)/500 - (187*c_q4_r*s_q3_r*s_q5_r)/500 - (c_q2_r*c_q3_r*c_q4_r*s_q5_r)/20 - (c_q2_r*c_q3_r*s_q4_r*s_q5_r)/20 - (c_q2_r*c_q4_r*s_q3_r*s_q5_r)/20 - (c_q3_r*c_q4_r*s_q2_r*s_q5_r)/20 + (c_q2_r*s_q3_r*s_q4_r*s_q5_r)/20 + (c_q3_r*s_q2_r*s_q4_r*s_q5_r)/20 + (c_q4_r*s_q2_r*s_q3_r*s_q5_r)/20 + (s_q2_r*s_q3_r*s_q4_r*s_q5_r)/20 + (89*c_q2_r*c_q3_r*c_q4_r*s_q0_r*s_q5_r)/1000 - (89*c_q2_r*s_q0_r*s_q3_r*s_q4_r*s_q5_r)/1000 - (89*c_q3_r*s_q0_r*s_q2_r*s_q4_r*s_q5_r)/1000 - (89*c_q4_r*s_q0_r*s_q2_r*s_q3_r*s_q5_r)/1000 + (89*c_q0_r*c_q2_r*c_q3_r*s_q1_r*s_q4_r*s_q5_r)/1000 + (89*c_q0_r*c_q2_r*c_q4_r*s_q1_r*s_q3_r*s_q5_r)/1000 + (89*c_q0_r*c_q3_r*c_q4_r*s_q1_r*s_q2_r*s_q5_r)/1000 - (89*c_q0_r*s_q1_r*s_q2_r*s_q3_r*s_q4_r*s_q5_r)/1000;
		r_jc(1,4)=(c_q3_r*c_q4_r*s_q5_r)/20 - (s_q3_r*s_q4_r*s_q5_r)/20 - (211*s_q4_r*s_q5_r)/500 - (187*c_q3_r*s_q4_r*s_q5_r)/500 - (187*c_q4_r*s_q3_r*s_q5_r)/500 - (c_q2_r*c_q3_r*c_q4_r*s_q5_r)/20 - (c_q2_r*c_q3_r*s_q4_r*s_q5_r)/20 - (c_q2_r*c_q4_r*s_q3_r*s_q5_r)/20 - (c_q3_r*c_q4_r*s_q2_r*s_q5_r)/20 + (c_q2_r*s_q3_r*s_q4_r*s_q5_r)/20 + (c_q3_r*s_q2_r*s_q4_r*s_q5_r)/20 + (c_q4_r*s_q2_r*s_q3_r*s_q5_r)/20 + (s_q2_r*s_q3_r*s_q4_r*s_q5_r)/20 + (89*c_q2_r*c_q3_r*c_q4_r*s_q0_r*s_q5_r)/1000 - (89*c_q2_r*s_q0_r*s_q3_r*s_q4_r*s_q5_r)/1000 - (89*c_q3_r*s_q0_r*s_q2_r*s_q4_r*s_q5_r)/1000 - (89*c_q4_r*s_q0_r*s_q2_r*s_q3_r*s_q5_r)/1000 + (89*c_q0_r*c_q2_r*c_q3_r*s_q1_r*s_q4_r*s_q5_r)/1000 + (89*c_q0_r*c_q2_r*c_q4_r*s_q1_r*s_q3_r*s_q5_r)/1000 + (89*c_q0_r*c_q3_r*c_q4_r*s_q1_r*s_q2_r*s_q5_r)/1000 - (89*c_q0_r*s_q1_r*s_q2_r*s_q3_r*s_q4_r*s_q5_r)/1000;
		r_jc(1,5)=(211*c_q4_r*c_q5_r)/500 + (187*c_q3_r*c_q4_r*c_q5_r)/500 - (89*c_q0_r*c_q1_r*s_q5_r)/1000 + (c_q3_r*c_q5_r*s_q4_r)/20 + (c_q4_r*c_q5_r*s_q3_r)/20 - (187*c_q5_r*s_q3_r*s_q4_r)/500 + (c_q2_r*c_q3_r*c_q4_r*c_q5_r)/20 - (c_q2_r*c_q3_r*c_q5_r*s_q4_r)/20 - (c_q2_r*c_q4_r*c_q5_r*s_q3_r)/20 - (c_q3_r*c_q4_r*c_q5_r*s_q2_r)/20 - (c_q2_r*c_q5_r*s_q3_r*s_q4_r)/20 - (c_q3_r*c_q5_r*s_q2_r*s_q4_r)/20 - (c_q4_r*c_q5_r*s_q2_r*s_q3_r)/20 + (c_q5_r*s_q2_r*s_q3_r*s_q4_r)/20 + (89*c_q2_r*c_q3_r*c_q5_r*s_q0_r*s_q4_r)/1000 + (89*c_q2_r*c_q4_r*c_q5_r*s_q0_r*s_q3_r)/1000 + (89*c_q3_r*c_q4_r*c_q5_r*s_q0_r*s_q2_r)/1000 - (89*c_q5_r*s_q0_r*s_q2_r*s_q3_r*s_q4_r)/1000 - (89*c_q0_r*c_q2_r*c_q3_r*c_q4_r*c_q5_r*s_q1_r)/1000 + (89*c_q0_r*c_q2_r*c_q5_r*s_q1_r*s_q3_r*s_q4_r)/1000 + (89*c_q0_r*c_q3_r*c_q5_r*s_q1_r*s_q2_r*s_q4_r)/1000 + (89*c_q0_r*c_q4_r*c_q5_r*s_q1_r*s_q2_r*s_q3_r)/1000;
		r_jc(2,0)=(89*c_q1_r*s_q0_r*s_q5_r)/1000 + (89*c_q0_r*c_q2_r*c_q3_r*c_q5_r*s_q4_r)/1000 + (89*c_q0_r*c_q2_r*c_q4_r*c_q5_r*s_q3_r)/1000 + (89*c_q0_r*c_q3_r*c_q4_r*c_q5_r*s_q2_r)/1000 - (89*c_q0_r*c_q5_r*s_q2_r*s_q3_r*s_q4_r)/1000 + (89*c_q2_r*c_q3_r*c_q4_r*c_q5_r*s_q0_r*s_q1_r)/1000 - (89*c_q2_r*c_q5_r*s_q0_r*s_q1_r*s_q3_r*s_q4_r)/1000 - (89*c_q3_r*c_q5_r*s_q0_r*s_q1_r*s_q2_r*s_q4_r)/1000 - (89*c_q4_r*c_q5_r*s_q0_r*s_q1_r*s_q2_r*s_q3_r)/1000;
		r_jc(2,1)=(89*c_q0_r*s_q1_r*s_q5_r)/1000 - (89*c_q0_r*c_q1_r*c_q2_r*c_q3_r*c_q4_r*c_q5_r)/1000 + (89*c_q0_r*c_q1_r*c_q2_r*c_q5_r*s_q3_r*s_q4_r)/1000 + (89*c_q0_r*c_q1_r*c_q3_r*c_q5_r*s_q2_r*s_q4_r)/1000 + (89*c_q0_r*c_q1_r*c_q4_r*c_q5_r*s_q2_r*s_q3_r)/1000;
		r_jc(2,2)=(c_q2_r*c_q5_r*s_q3_r*s_q4_r)/20 - (c_q2_r*c_q3_r*c_q5_r*s_q4_r)/20 - (c_q2_r*c_q4_r*c_q5_r*s_q3_r)/20 - (c_q3_r*c_q4_r*c_q5_r*s_q2_r)/20 - (c_q2_r*c_q3_r*c_q4_r*c_q5_r)/20 + (c_q3_r*c_q5_r*s_q2_r*s_q4_r)/20 + (c_q4_r*c_q5_r*s_q2_r*s_q3_r)/20 + (c_q5_r*s_q2_r*s_q3_r*s_q4_r)/20 + (89*c_q2_r*c_q3_r*c_q4_r*c_q5_r*s_q0_r)/1000 - (89*c_q2_r*c_q5_r*s_q0_r*s_q3_r*s_q4_r)/1000 - (89*c_q3_r*c_q5_r*s_q0_r*s_q2_r*s_q4_r)/1000 - (89*c_q4_r*c_q5_r*s_q0_r*s_q2_r*s_q3_r)/1000 + (89*c_q0_r*c_q2_r*c_q3_r*c_q5_r*s_q1_r*s_q4_r)/1000 + (89*c_q0_r*c_q2_r*c_q4_r*c_q5_r*s_q1_r*s_q3_r)/1000 + (89*c_q0_r*c_q3_r*c_q4_r*c_q5_r*s_q1_r*s_q2_r)/1000 - (89*c_q0_r*c_q5_r*s_q1_r*s_q2_r*s_q3_r*s_q4_r)/1000;
		r_jc(2,3)=(c_q3_r*c_q4_r*c_q5_r)/20 - (187*c_q3_r*c_q5_r*s_q4_r)/500 - (187*c_q4_r*c_q5_r*s_q3_r)/500 - (c_q5_r*s_q3_r*s_q4_r)/20 - (c_q2_r*c_q3_r*c_q4_r*c_q5_r)/20 - (c_q2_r*c_q3_r*c_q5_r*s_q4_r)/20 - (c_q2_r*c_q4_r*c_q5_r*s_q3_r)/20 - (c_q3_r*c_q4_r*c_q5_r*s_q2_r)/20 + (c_q2_r*c_q5_r*s_q3_r*s_q4_r)/20 + (c_q3_r*c_q5_r*s_q2_r*s_q4_r)/20 + (c_q4_r*c_q5_r*s_q2_r*s_q3_r)/20 + (c_q5_r*s_q2_r*s_q3_r*s_q4_r)/20 + (89*c_q2_r*c_q3_r*c_q4_r*c_q5_r*s_q0_r)/1000 - (89*c_q2_r*c_q5_r*s_q0_r*s_q3_r*s_q4_r)/1000 - (89*c_q3_r*c_q5_r*s_q0_r*s_q2_r*s_q4_r)/1000 - (89*c_q4_r*c_q5_r*s_q0_r*s_q2_r*s_q3_r)/1000 + (89*c_q0_r*c_q2_r*c_q3_r*c_q5_r*s_q1_r*s_q4_r)/1000 + (89*c_q0_r*c_q2_r*c_q4_r*c_q5_r*s_q1_r*s_q3_r)/1000 + (89*c_q0_r*c_q3_r*c_q4_r*c_q5_r*s_q1_r*s_q2_r)/1000 - (89*c_q0_r*c_q5_r*s_q1_r*s_q2_r*s_q3_r*s_q4_r)/1000;
		r_jc(2,4)=(c_q3_r*c_q4_r*c_q5_r)/20 - (211*c_q5_r*s_q4_r)/500 - (187*c_q3_r*c_q5_r*s_q4_r)/500 - (187*c_q4_r*c_q5_r*s_q3_r)/500 - (c_q5_r*s_q3_r*s_q4_r)/20 - (c_q2_r*c_q3_r*c_q4_r*c_q5_r)/20 - (c_q2_r*c_q3_r*c_q5_r*s_q4_r)/20 - (c_q2_r*c_q4_r*c_q5_r*s_q3_r)/20 - (c_q3_r*c_q4_r*c_q5_r*s_q2_r)/20 + (c_q2_r*c_q5_r*s_q3_r*s_q4_r)/20 + (c_q3_r*c_q5_r*s_q2_r*s_q4_r)/20 + (c_q4_r*c_q5_r*s_q2_r*s_q3_r)/20 + (c_q5_r*s_q2_r*s_q3_r*s_q4_r)/20 + (89*c_q2_r*c_q3_r*c_q4_r*c_q5_r*s_q0_r)/1000 - (89*c_q2_r*c_q5_r*s_q0_r*s_q3_r*s_q4_r)/1000 - (89*c_q3_r*c_q5_r*s_q0_r*s_q2_r*s_q4_r)/1000 - (89*c_q4_r*c_q5_r*s_q0_r*s_q2_r*s_q3_r)/1000 + (89*c_q0_r*c_q2_r*c_q3_r*c_q5_r*s_q1_r*s_q4_r)/1000 + (89*c_q0_r*c_q2_r*c_q4_r*c_q5_r*s_q1_r*s_q3_r)/1000 + (89*c_q0_r*c_q3_r*c_q4_r*c_q5_r*s_q1_r*s_q2_r)/1000 - (89*c_q0_r*c_q5_r*s_q1_r*s_q2_r*s_q3_r*s_q4_r)/1000;
		r_jc(2,5)=(187*s_q3_r*s_q4_r*s_q5_r)/500 - (211*c_q4_r*s_q5_r)/500 - (89*c_q0_r*c_q1_r*c_q5_r)/1000 - (187*c_q3_r*c_q4_r*s_q5_r)/500 - (c_q3_r*s_q4_r*s_q5_r)/20 - (c_q4_r*s_q3_r*s_q5_r)/20 - (c_q2_r*c_q3_r*c_q4_r*s_q5_r)/20 + (c_q2_r*c_q3_r*s_q4_r*s_q5_r)/20 + (c_q2_r*c_q4_r*s_q3_r*s_q5_r)/20 + (c_q3_r*c_q4_r*s_q2_r*s_q5_r)/20 + (c_q2_r*s_q3_r*s_q4_r*s_q5_r)/20 + (c_q3_r*s_q2_r*s_q4_r*s_q5_r)/20 + (c_q4_r*s_q2_r*s_q3_r*s_q5_r)/20 - (s_q2_r*s_q3_r*s_q4_r*s_q5_r)/20 - (89*c_q2_r*c_q3_r*s_q0_r*s_q4_r*s_q5_r)/1000 - (89*c_q2_r*c_q4_r*s_q0_r*s_q3_r*s_q5_r)/1000 - (89*c_q3_r*c_q4_r*s_q0_r*s_q2_r*s_q5_r)/1000 + (89*s_q0_r*s_q2_r*s_q3_r*s_q4_r*s_q5_r)/1000 + (89*c_q0_r*c_q2_r*c_q3_r*c_q4_r*s_q1_r*s_q5_r)/1000 - (89*c_q0_r*c_q2_r*s_q1_r*s_q3_r*s_q4_r*s_q5_r)/1000 - (89*c_q0_r*c_q3_r*s_q1_r*s_q2_r*s_q4_r*s_q5_r)/1000 - (89*c_q0_r*c_q4_r*s_q1_r*s_q2_r*s_q3_r*s_q5_r)/1000;
		r_jc(3,0)=-temp2/(temp3*(pow(temp1,2)/pow(temp3,2) + 1));
		r_jc(3,1)=((c_q5_r*(c_q4_r*(c_q0_r*c_q1_r*c_q2_r*c_q3_r - c_q0_r*c_q1_r*s_q2_r*s_q3_r) - s_q4_r*(c_q0_r*c_q1_r*c_q2_r*s_q3_r + c_q0_r*c_q1_r*c_q3_r*s_q2_r)) - c_q0_r*s_q1_r*s_q5_r)/temp3 + ((c_q1_r*s_q5_r - c_q5_r*(c_q4_r*(s_q1_r*s_q2_r*s_q3_r - c_q2_r*c_q3_r*s_q1_r) + s_q4_r*(c_q2_r*s_q1_r*s_q3_r + c_q3_r*s_q1_r*s_q2_r)))*temp1)/pow(temp3,2))/(pow(temp1,2)/pow(temp3,2) + 1);
		r_jc(3,2)=((c_q5_r*(s_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)) - c_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r))))/temp3 + (c_q5_r*(c_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r) + s_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r))*temp1)/pow(temp3,2))/(pow(temp1,2)/pow(temp3,2) + 1);
		r_jc(3,3)=((c_q5_r*(s_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)) - c_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r))))/temp3 + (c_q5_r*(c_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r) + s_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r))*temp1)/pow(temp3,2))/(pow(temp1,2)/pow(temp3,2) + 1);
		r_jc(3,4)=((c_q5_r*(s_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)) - c_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r))))/temp3 + (c_q5_r*(c_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r) + s_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r))*temp1)/pow(temp3,2))/(pow(temp1,2)/pow(temp3,2) + 1);
		r_jc(3,5)=((s_q5_r*(s_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)) + c_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r))) + c_q0_r*c_q1_r*c_q5_r)/temp3 + ((c_q5_r*s_q1_r + s_q5_r*(c_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r) - s_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r)))*temp1)/pow(temp3,2))/(pow(temp1,2)/pow(temp3,2) + 1);
		r_jc(4,0)=(temp1/pow(pow(temp1,2) + pow(temp3,2),0.5) + (temp1*pow(temp2,2))/pow(pow(temp1,2) + pow(temp3,2),1.5))/(pow(temp2,2)/(pow(temp1,2) + pow(temp3,2)) + 1);
		r_jc(4,1)=-((c_q5_r*(c_q4_r*(c_q1_r*c_q2_r*c_q3_r*s_q0_r - c_q1_r*s_q0_r*s_q2_r*s_q3_r) - s_q4_r*(c_q1_r*c_q2_r*s_q0_r*s_q3_r + c_q1_r*c_q3_r*s_q0_r*s_q2_r)) - s_q0_r*s_q1_r*s_q5_r)/pow(pow(temp1,2) + pow(temp3,2),0.5) + ((2*(c_q5_r*(c_q4_r*(c_q0_r*c_q1_r*c_q2_r*c_q3_r - c_q0_r*c_q1_r*s_q2_r*s_q3_r) - s_q4_r*(c_q0_r*c_q1_r*c_q2_r*s_q3_r + c_q0_r*c_q1_r*c_q3_r*s_q2_r)) - c_q0_r*s_q1_r*s_q5_r)*temp1 - 2*temp3*(c_q1_r*s_q5_r - c_q5_r*(c_q4_r*(s_q1_r*s_q2_r*s_q3_r - c_q2_r*c_q3_r*s_q1_r) + s_q4_r*(c_q2_r*s_q1_r*s_q3_r + c_q3_r*s_q1_r*s_q2_r))))*temp2)/(2*pow(pow(temp1,2) + pow(temp3,2),1.5)))/(pow(temp2,2)/(pow(temp1,2) + pow(temp3,2)) + 1);
		r_jc(4,2)=-(((2*c_q5_r*(s_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)) - c_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)))*temp1 - 2*c_q5_r*temp3*(c_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r) + s_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r)))*temp2)/(2*pow(pow(temp1,2) + pow(temp3,2),1.5)) - (c_q5_r*temp4)/pow(pow(temp1,2) + pow(temp3,2),0.5))/(pow(temp2,2)/(pow(temp1,2) + pow(temp3,2)) + 1);
		r_jc(4,3)=-(((2*c_q5_r*(s_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)) - c_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)))*temp1 - 2*c_q5_r*temp3*(c_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r) + s_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r)))*temp2)/(2*pow(pow(temp1,2) + pow(temp3,2),1.5)) - (c_q5_r*temp4)/pow(pow(temp1,2) + pow(temp3,2),0.5))/(pow(temp2,2)/(pow(temp1,2) + pow(temp3,2)) + 1);
		r_jc(4,4)=-(((2*c_q5_r*(s_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)) - c_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)))*temp1 - 2*c_q5_r*temp3*(c_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r) + s_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r)))*temp2)/(2*pow(pow(temp1,2) + pow(temp3,2),1.5)) - (c_q5_r*temp4)/pow(pow(temp1,2) + pow(temp3,2),0.5))/(pow(temp2,2)/(pow(temp1,2) + pow(temp3,2)) + 1);
		r_jc(4,5)=(temp5/pow(pow(temp1,2) + pow(temp3,2),0.5) - ((2*(s_q5_r*(s_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)) + c_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r))) + c_q0_r*c_q1_r*c_q5_r)*temp1 - 2*(c_q5_r*s_q1_r + s_q5_r*(c_q4_r*(c_q1_r*c_q2_r*c_q3_r - c_q1_r*s_q2_r*s_q3_r) - s_q4_r*(c_q1_r*c_q2_r*s_q3_r + c_q1_r*c_q3_r*s_q2_r)))*temp3)*temp2)/(2*pow(pow(temp1,2) + pow(temp3,2),1.5)))/(pow(temp2,2)/(pow(temp1,2) + pow(temp3,2)) + 1);
		r_jc(5,0)=((s_q5_r*(s_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)) + c_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r))) + c_q0_r*c_q1_r*c_q5_r)/temp4 - ((s_q4_r*(c_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r) + s_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r)) - c_q4_r*(c_q3_r*(c_q2_r*s_q0_r + c_q0_r*s_q1_r*s_q2_r) - s_q3_r*(s_q0_r*s_q2_r - c_q0_r*c_q2_r*s_q1_r)))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		r_jc(5,1)=-((s_q5_r*(c_q4_r*(c_q1_r*c_q2_r*c_q3_r*s_q0_r - c_q1_r*s_q0_r*s_q2_r*s_q3_r) - s_q4_r*(c_q1_r*c_q2_r*s_q0_r*s_q3_r + c_q1_r*c_q3_r*s_q0_r*s_q2_r)) + c_q5_r*s_q0_r*s_q1_r)/temp4 - ((c_q4_r*(c_q1_r*c_q2_r*s_q0_r*s_q3_r + c_q1_r*c_q3_r*s_q0_r*s_q2_r) + s_q4_r*(c_q1_r*c_q2_r*c_q3_r*s_q0_r - c_q1_r*s_q0_r*s_q2_r*s_q3_r))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		r_jc(5,2)=(s_q5_r + ((s_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)) + c_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r)))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		r_jc(5,3)=(s_q5_r + ((s_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)) + c_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r)))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		r_jc(5,4)=(s_q5_r + ((s_q4_r*(c_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r) - s_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r)) + c_q4_r*(c_q3_r*(c_q0_r*s_q2_r + c_q2_r*s_q0_r*s_q1_r) + s_q3_r*(c_q0_r*c_q2_r - s_q0_r*s_q1_r*s_q2_r)))*temp5)/pow(temp4,2))/(pow(temp5,2)/pow(temp4,2) + 1);
		r_jc(5,5)=-temp2/(temp4*(pow(temp5,2)/pow(temp4,2) + 1));




		/*input vector to Eigen vector*/
		Eigen::VectorXf desired_vel(6);
		desired_vel(0)=req.x_dot;
		desired_vel(1)=req.y_dot;
		desired_vel(2)=req.z_dot;
		desired_vel(3)=req.roll_dot;
		desired_vel(4)=req.pitch_dot;
		desired_vel(5)=req.yaw_dot;

		/*inverse calc*/
		Eigen::MatrixXf r_jc_inverse(6,6);
		r_jc_inverse=r_jc.inverse();

		/*matrix dot vector*/
		Eigen::VectorXf output_right_vel(6);
		output_right_vel=r_jc_inverse*desired_vel;

		/*Eigen vector to output vector*/
		res.q_right_dot[0]=output_right_vel(0);
		res.q_right_dot[1]=output_right_vel(1);
		res.q_right_dot[2]=output_right_vel(2);
		res.q_right_dot[3]=output_right_vel(3);
		res.q_right_dot[4]=output_right_vel(4);
		res.q_right_dot[5]=output_right_vel(5);

		return true;
	}
};



int main(int argc, char** argv){
	ros::init(argc, argv, "legs_val_calculator_service");
	legs_val_calculator legs_calc_objct =legs_val_calculator();
	ROS_INFO("ready");
	ros::spin();
	return 0;
}
