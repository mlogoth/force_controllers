

#ifndef ADMITTANCE_CONTROLLER_HPP
#define ADMITTANCE_CONTROLLER_HPP

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
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/String.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <pluginlib/class_list_macros.h>

#include <kdl_wrapper_ros/kdl_wrapper.hpp>
#include <force_controllers/RobotState.hpp>
#include <force_controllers/admittance_controller.hpp>
#include <boost/algorithm/string.hpp>

#include "Iir.h"

namespace force_controllers
{

    class admittance_controller : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
    {

    public:
        admittance_controller(void);
        ~admittance_controller(void);

        void read_xml_robot_params(const ros::NodeHandle &_node, std::string _ns);
        void read_xml_controller_params(const ros::NodeHandle &_node, std::string _ns);

        // Initialization Function
        bool init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n);
        // Starting Routine
        void starting(const ros::Time &time);
        // Update Routine
        void update(const ros::Time &time, const ros::Duration &duration);

        void stopping(const ros::Time &time);

        // Function that converts strings to lowercase.
        std::string str_tolower(std::string s);
        
        // template <class MatT>
        // Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> pinv(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4});

        int JointLimitDumpimgMat(const Eigen::VectorXd &q, Eigen::MatrixXd &Wjl);
        void ComputeOrientationError(const Eigen::Matrix3d &Reed, const Eigen::Matrix3d &Ree, Eigen::Vector3d &eo);
        void publishJointError(const Eigen::VectorXd &eqpos, const Eigen::VectorXd &eqvel, const Eigen::VectorXd &qpdes);
        void publisEndEffectorError(const Eigen::VectorXd &ep, const Eigen::VectorXd &eo, const Eigen::VectorXd &evl, const Eigen::VectorXd &eva);
        void publisCorrectedFT(const Eigen::VectorXd &ep, const std::string &frame_id);
        void publisCorrectedFTDot(const Eigen::VectorXd &ep, const std::string &frame_id);
        void printVector(Eigen::VectorXd vec, std::string name_of_vector = "vector");
        void getGripperCompensationWrench(Eigen::VectorXd &w);
        
        // get wrench measured by f/t sensor wrt to 'wrt_frame' and with/without gripper compensation.
        void getEndEffectorWrench(Eigen::VectorXd &_ft, std::string wrt_frame = "world", bool compensate_gripper = true);
   
   private:
        std::vector<hardware_interface::JointHandle> joints_;
        hardware_interface::EffortJointInterface *robot_;
        //hardware_interface::ForceTorqueSensorHandle ft_;

        // Ros Publishers
        ros::Publisher  pub_ftdot, pub_ft, pub_eo, pub_ep, pub_evl, pub_eva, pub_eq, pub_platform_commands;

        // ROS Variables
        ros::NodeHandle node_;
        double begin, all_joints_interface;

        // Size of Gains
        int _gains_vector_size = 6, platform_joints = 3;

        // Trajectory Variables
        Eigen::VectorXd At;
        Eigen::VectorXd omega;
        Eigen::VectorXd gripper_wrench ;

        // Variables
        // Desired Joint Space Impedance Parameters
        Eigen::MatrixXd Mdq, Kdq, Ddq, Mdc, Kdc, Ddc, Wq;

        Eigen::VectorXd v, G, F;
        Eigen::MatrixXd Mr, Cr;
        Eigen::VectorXd Uimp, Ucrt, Uvel, vel;
        Eigen::Matrix<double, 6, 1> vc;

        // Joint Limits Avoidance
        Eigen::VectorXd qlim_lower, qlim_upper, W0;
        double activation_percentage;

        // Robot Desired
        Eigen::VectorXd qdpos, qdvel, qdacc, qdeff;
        Eigen::Vector3d peed;
        Eigen::Matrix3d Reed;
        Eigen::VectorXd veed, veeddot;

        // Robot State Variables
        std::vector<std::string> _joint_names, qnam;
        Eigen::VectorXd qpos, qvel, qacc, qeff, old_command;
        Eigen::VectorXd ft,ftdot,ft_prev;
        const double *mfrc;
        const double *mtrq;
        double _forceGain;
        std::vector<timeDerivative<double, double>> vel_dt; // derivatives of joints velocity
        timeDerivative<Eigen::VectorXd,double> wrench_time_derivative; // wrench time derivative
        // compute transformation matrices for each segment;
        std::vector<std::vector<Eigen::Matrix<double, 4, 4>>> fingers_link_pose;
        std::vector<std::vector<double>> fingers_link_mass;

        // KDL Wrapper
        KDLWrapper *kdl, *kdl_ft, *kdl_finger_2, *kdl_finger_1;
        std::vector<KDLWrapper> fingers;
        // Robot State Wrapper
        RobotState *state;
        double JacW8;

        int jnt;

        double mycommands[7];
        std::string jnt_CM[7];

        std::string _robot_tip_link, _robot_root_link, _ft_link, _robot_description_name, _control_type, _control_space;
        std::string _ft_link_name, _ft_topic_name, _robot_finger1_link, _robot_finger2_link;
        std::list<std::string> type_of_controllers = {"classical", "variable", "gravity", "pd","force"};
        std::list<std::string> type_of_control_space = {"joint", "cartesian"};
    };

} // namespace force_controllers
#endif