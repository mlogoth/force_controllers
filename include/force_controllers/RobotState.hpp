#ifndef ROBOTSTATE_HPP
#define ROBOTSTATE_HPP

#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/WrenchStamped.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <force_controllers/time_derivative.hpp>


class RobotState
{
protected:
	ros::NodeHandle nhd;
	int _ndof, _num_of_other_states;
	ros::Subscriber sub_state, sub_ft, sub_ft_sx, sub_ft_sy;
	bool _isSubscribed;
	std::vector<timeDerivative<double,double>> vel_dt, vel_slip_dt;


public:
	/*
		 * Arm Torso Joint Variables
		 * 1. q_pos : arm_torso joint position
		 * 2. q_vel : arm_torso joint velocity
		 * 3. q_eff : arm_torso Joint Effort
		 * 4. q_name : arm_torso Joint Names
		 * 5. jp_head: head joint position
		 */
	Eigen::VectorXd q_pos, q_vel, q_eff, q_acc, ft, ft_sx, ft_sy,slip,slipdot,slipddot;
	std::vector<std::string> q_name,slipname;

	/* Initialize Robot State class
		 * @param ROS node handler n_
		 * @param robotnamespace for the topics
		*/
	RobotState(ros::NodeHandle n_, int ndof_, std::string namespace_, std::string ft_topic_name="ft_sensor");	
	RobotState(ros::NodeHandle n_,std::vector<std::string> _joint_names , std::string namespace_="", std::string ft_topic_name="ft_sensor");

	RobotState(ros::NodeHandle n_, int ndof_);

	// Read Joint States Function (Listener)
	void getJointState(const sensor_msgs::JointState data);

	// Get the Force Torque Measurements
	void getFtSensor(const geometry_msgs::WrenchStamped data);

	// Get the Force Torque Measurements
	void getSxFtSensor(const geometry_msgs::WrenchStamped data);

	// Get the Force Torque Measurements
	void getSyFtSensor(const geometry_msgs::WrenchStamped data);

	bool isSubscribed() { return _isSubscribed; }

	// Read Marker States Function (Listener)
	//void getMarkerPose(const geometry_msgs::PoseStamped data);
};

#endif