//#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <stdio.h>

#include "sensor_msgs/JointState.h"
#include <geometry_msgs/WrenchStamped.h>

#include <Eigen/Dense>

#include <force_controllers/RobotState.hpp>

RobotState::RobotState(ros::NodeHandle n_, int ndof_, std::string namespace_ = "")
{
	nhd = n_;
	_ndof = ndof_;
	_num_of_other_states = 0;

	q_pos.resize(_ndof);
	q_vel.resize(_ndof);
	q_eff.resize(_ndof);
	q_name.resize(_ndof);
	q_acc.resize(_ndof);
	vel_dt.resize(_ndof);
	vel_slip_dt.resize(3);

	
	ft.resize(6);
	ft_sx.resize(6);
	ft_sy.resize(6);

	std::cout << "Initialize Robot State Class...\n"
			  << std::endl;

	/* Initialize Subscriber*/
	sub_state = nhd.subscribe(namespace_ + "/joint_states", 10, &RobotState::getJointState, this);
	sub_ft = nhd.subscribe(namespace_ + "/ft_sensor", 10, &RobotState::getFtSensor, this);
	sub_ft_sx = nhd.subscribe(namespace_ + "/s_x_ft_sensor", 10, &RobotState::getSxFtSensor, this);
	sub_ft_sy = nhd.subscribe(namespace_ + "/s_y_ft_sensor", 10, &RobotState::getSyFtSensor, this);

	/* Initialize Variables */
	for (int i = 0; i < _ndof; i++)
	{
		q_pos[i] = 0.0;
		q_name[i] = " ";
		q_vel[i] = 0.0;
		q_eff[i] = 0.0;
	}

	std::cout << "Initialized...OK \n"
			  << std::endl;
	
	_isSubscribed = false;
	
}


RobotState::RobotState(ros::NodeHandle n_, int ndof_)
{
	nhd = n_;
	_ndof = ndof_;
	_num_of_other_states = 0;

	q_pos.resize(_ndof);
	q_vel.resize(_ndof);
	q_eff.resize(_ndof);
	q_name.resize(_ndof);
	q_acc.resize(_ndof);
	vel_dt.resize(_ndof);
	vel_slip_dt.resize(3);

	ft.resize(6);
	ft_sx.resize(6);
	ft_sy.resize(6);

	std::cout << "Initialize Robot State Class...\n"
			  << std::endl;

	/* Initialize Subscriber*/
	sub_state = nhd.subscribe("/joint_states", 10, &RobotState::getJointState, this);
	sub_ft = nhd.subscribe( "/ft_sensor", 10, &RobotState::getFtSensor, this);
	sub_ft_sx = nhd.subscribe("/s_x_ft_sensor", 10, &RobotState::getSxFtSensor, this);
	sub_ft_sy = nhd.subscribe("/s_y_ft_sensor", 10, &RobotState::getSyFtSensor, this);

	/* Initialize Variables */
	for (int i = 0; i < _ndof; i++)
	{
		q_pos[i] = 0.0;
		q_name[i] = " ";
		q_vel[i] = 0.0;
		q_eff[i] = 0.0;
	}

	std::cout << "Initialized...OK \n"
			  << std::endl;
	
	_isSubscribed = false;
}

// Read Joint States
void RobotState::getJointState(const sensor_msgs::JointState data)
{
	if (_ndof != data.position.size() && _ndof>data.position.size())
	{
		ROS_ERROR("RobotState Class not well initialized: Defined number of DoFs not equal or less than joint_state topic Read variables size!");
		std::exit(-1);
	}

	if (data.position.size()>_ndof){
		_num_of_other_states = data.position.size()-_ndof;
		slip.resize(_num_of_other_states);
		slipdot.resize(_num_of_other_states);
		slipname.resize(_num_of_other_states);
		slipddot.resize(_num_of_other_states);
		
	}

	for (int i = 0; i < _ndof; i++)
	{
		q_pos[i] = data.position[i];
		q_name[i] = data.name[i];
		q_vel[i] = data.velocity[i];
		q_eff[i] = data.effort[i];
		q_acc[i] = vel_dt[i].getDerivative(q_vel[i],data.header.stamp.toSec());
	}

	for(unsigned int i=0;i<_num_of_other_states;i++){
		slip[i] =  data.position[i+_ndof];
		slipdot[i] = data.velocity[i+_ndof];
		slipddot[i] = vel_slip_dt[i].getDerivative(slipdot[i],data.header.stamp.toSec());
		slipname[i]	= data.name[i+_ndof];
		}
	_isSubscribed = true;
}



// Read FT Sensor
void RobotState::getFtSensor(const geometry_msgs::WrenchStamped data)
{

	ft << data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x,data.wrench.torque.y, data.wrench.torque.z;
}

// Read FT Sensor
void RobotState::getSxFtSensor(const geometry_msgs::WrenchStamped data)
{

	ft_sx << data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x,data.wrench.torque.y, data.wrench.torque.z;
}


// Read FT Sensor
void RobotState::getSyFtSensor(const geometry_msgs::WrenchStamped data)
{

	ft_sy << data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x,data.wrench.torque.y, data.wrench.torque.z;
}
