

#ifndef IMPEDANCE_CONTROLLER_HPP
#define IMPEDANCE_CONTROLLER_HPP

#include <cstddef>
#include <vector>
#include <string>
#include <fstream>

// Boost LibrariesW
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <stdio.h>
#include <math.h>
#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/String.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <pluginlib/class_list_macros.h>

#include <kdl_wrapper_ros/kdl_wrapper.hpp>
#include <force_controllers/time_derivative.hpp>
#include <force_controllers/RobotState.hpp>

#include "Iir.h"

namespace force_controllers
{

class impedance_controller : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{

public:
    impedance_controller(void);
    ~impedance_controller(void);

    
    void read_xml_robot_params(const ros::NodeHandle &_node, std::string _ns);
    void read_xml_controller_params(const ros::NodeHandle &_node, std::string _ns);
    
    // Initialization Function
    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
    // Starting Routine
    void starting(const ros::Time &time);
    // Update Routine
    void update(const ros::Time &time, const ros::Duration &duration);

    void stopping(const ros::Time &time);

    // Function that converts strings to lowercase.
    std::string str_tolower(std::string s);

    int JointLimitDumpimgMat(const Eigen::VectorXd & q, Eigen::MatrixXd &Wjl);
    void ComputeOrientationError(const Eigen::Matrix3d &Reed, const Eigen::Matrix3d &Ree, Eigen::Vector3d &eo);
    void publishJointError(const Eigen::VectorXd &eqpos, const Eigen::VectorXd &eqvel, const Eigen::VectorXd& qpdes);
    void publisEndEffectorError(const Eigen::VectorXd &ep, const Eigen::VectorXd &eo,const Eigen::VectorXd &evl, const Eigen::VectorXd &eva);


private:
    std::vector<hardware_interface::JointHandle> joints_;
    hardware_interface::EffortJointInterface *robot_;
    //hardware_interface::ForceTorqueSensorHandle ft_;

    // Ros Publishers
    ros::Publisher pub_eo, pub_ep, pub_evl, pub_eva, pub_eq;

    // ROS Variables
    ros::NodeHandle node_;
    double begin;

    // Size of Gains
    int _gains_vector_size = 6;

    // Trajectory Variables
    Eigen::VectorXd At;
    Eigen::VectorXd omega;

    // Variables
    // Desired Joint Space Impedance Parameters
    Eigen::MatrixXd Mdq, Kdq, Ddq,Mdc, Kdc, Ddc,Wq;

    Eigen::VectorXd v, G, F;
    Eigen::MatrixXd Mr, Cr;
    Eigen::VectorXd Uimp, Ucrt, Uvel, vel;
    Eigen::Matrix<double, 6, 1> vc;

    // Joint Limits Avoidance
    Eigen::VectorXd qlim_lower, qlim_upper, W0;
    double activation_percentage;

    // Robot Desired 
    Eigen::VectorXd qdpos, qdvel, qdacc, qdeff;
    Eigen::Vector3d  peed;
    Eigen::Matrix3d  Reed;
    Eigen::VectorXd veed, veeddot; 

    
    // Robot State Variables
    std::vector<std::string> _joint_names,qnam;
    Eigen::VectorXd qpos, qvel, qacc, qeff;
    Eigen::VectorXd ft;
    const double *mfrc;
    const double *mtrq;
    std::vector<timeDerivative<double, double>> vel_dt; // derivatives of joints velocity

    // KDL Wrapper 
    KDLWrapper *kdl, *kdl_ft;
    
    // Robot State Wrapper
    RobotState *state;
    double JacW8;

    int jnt;

    double mycommands[7];
    std::string jnt_CM[7];

    std::string _robot_description_name, _control_type, _control_space;

    std::list<std::string> type_of_controllers = {"classical", "variable", "gravity", "pd"};
    std::list<std::string> type_of_control_space = {"joint", "cartesian"};
};

} // namespace force_controllers
#endif