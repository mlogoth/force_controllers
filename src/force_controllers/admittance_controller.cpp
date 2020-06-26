#include <force_controllers/admittance_controller.hpp>

Eigen::VectorXd constrainAngle(const Eigen::VectorXd vec)
{
    Eigen::VectorXd x = vec;
    std::cout<<"initial: "<<x<<std::endl;
    std::cout<<"Constraint Angle: "<<std::endl;
    
    for (unsigned int i = 0; i < x.size(); i++)
    {
        x[i] = fmod(x[i] + M_PI, 2 * M_PI);
        if (x[i] < 0)
            x[i] += 2.0 * M_PI;
        x[i] -= M_PI;
        std::cout<<x[i]<<" |";

    }
    std::cout<<std::endl;
    return x;
}

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i)
    {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

namespace force_controllers
{

    admittance_controller::admittance_controller(void)
    {
        ROS_ERROR("admittance_controller initialization");
        node_ = ros::NodeHandle();
    }

    admittance_controller::~admittance_controller(void){};

    bool admittance_controller::init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n)
    {

        node_ = n;
        std::string ns_prefix = n.getNamespace();

        pub_eo = n.advertise<geometry_msgs::Vector3Stamped>(ns_prefix + "/error_orientation", 1000);
        pub_ep = n.advertise<geometry_msgs::Vector3Stamped>(ns_prefix + "/error_position", 1000);
        pub_evl = n.advertise<geometry_msgs::Vector3Stamped>(ns_prefix + "/error_linear_velocity", 1000);
        pub_eva = n.advertise<geometry_msgs::Vector3Stamped>(ns_prefix + "/error_angular_velocity", 1000);
        pub_eq = n.advertise<sensor_msgs::JointState>(ns_prefix + "/error_joints", 1000);
        pub_ft = n.advertise<geometry_msgs::WrenchStamped>(ns_prefix + "/ft_corrected", 1000);
        pub_ftdot = n.advertise<geometry_msgs::WrenchStamped>(ns_prefix + "/ftdot_corrected", 1000);

        // ROS Print Namespace
        ROS_INFO("Namespace: ");
        std::cout << ns_prefix << std::endl;

        // Get Parameters
        admittance_controller::read_xml_robot_params(node_, ns_prefix);

        // Resize State Variables
        qpos.resize(_joint_names.size());
        qvel.resize(_joint_names.size());
        qacc.resize(_joint_names.size());
        qeff.resize(_joint_names.size());
        qnam.resize(_joint_names.size());
        vel_dt.resize(_joint_names.size());
        ft.resize(6);

        // Initialize KDL Wrapper

        ROS_INFO("Initialize KDL");

        // to tip link
        kdl = new KDLWrapper(_robot_description_name, node_);
        kdl->init(_robot_root_link, _robot_tip_link, true);
        int num_of_segments_world_tip = kdl->getChain().getNrOfSegments();

        // to ft link
        kdl_ft = new KDLWrapper(_robot_description_name, node_);
        // init kdl with end effector and base link
        kdl_ft->init(_robot_root_link, _ft_link_name, true);
        int num_of_segments_world_ft = kdl_ft->getChain().getNrOfSegments();

        // force torque link to finger 1 link
        kdl_finger_1 = new KDLWrapper(_robot_description_name, node_);
        kdl_finger_1->init(_ft_link_name, _robot_finger1_link, true);
        int num_of_segments_ft_fig1 = kdl_finger_1->getChain().getNrOfSegments();
        fingers.push_back(*kdl_finger_1);

        // force torque link to finger 2 link
        kdl_finger_2 = new KDLWrapper(_robot_description_name, node_);
        kdl_finger_2->init(_ft_link_name, _robot_finger2_link, true);
        int num_of_segments_ft_fig2 = kdl_finger_2->getChain().getNrOfSegments();
        fingers.push_back(*kdl_finger_2);

        // force/torque sensro to end_effector_link
        ROS_INFO(" ----------- Initialize KDL FT -> TIP  ------------------");

        //for each finger calculate transformations and masses if masses > 0
        for (auto kdl_finger = fingers.begin(); kdl_finger != fingers.end(); ++kdl_finger)
        {

            // std::cout << "========== FINGER - "<<fingers_link_pose.size()+1<<" ===========" << std::endl;

            std::vector<Eigen::Matrix<double, 4, 4>> finger_segments_transformation_matrices;
            std::vector<double> finger_segments_mass;

            for (unsigned int i = 0; i < kdl_finger->getChain().getNrOfSegments(); i++)
            {

                if (kdl_finger->getChain().getSegment(i).getInertia().getMass() > 0)
                {

                    Eigen::MatrixXd mat(4, 4);
                    kdl_finger->KDLFrame2EigenVec(kdl_finger->getChain().getSegment(i).getFrameToTip(), mat);
                    // express each segment transformation to ft link
                    for (auto elt = 0; elt < i; elt++)
                    {
                        Eigen::MatrixXd m(4, 4);
                        kdl_finger->KDLFrame2EigenVec(kdl_finger->getChain().getSegment(elt).getFrameToTip(), m);
                        mat = m * mat;
                    }
                    finger_segments_transformation_matrices.push_back(mat);
                    finger_segments_mass.push_back(kdl_finger->getChain().getSegment(i).getInertia().getMass());
                }
            }

            // add transformations and mass for each finger
            fingers_link_pose.push_back(finger_segments_transformation_matrices);
            fingers_link_mass.push_back(finger_segments_mass);
        }

        // get gripper compensation wrench
        gripper_wrench.resize(6);
        gripper_wrench << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        admittance_controller::getGripperCompensationWrench(gripper_wrench);

        std::cout << "kdl last segment: " << kdl->getChain().getSegment(num_of_segments_world_tip - 1).getName() << std::endl;
        std::cout << "kdl ft last segment: " << kdl_ft->getChain().getSegment(num_of_segments_world_ft - 1).getName() << std::endl;

        // Initialize Robot State
        state = new RobotState(node_, std::vector<std::string>(_joint_names.begin(), _joint_names.begin() + 3), "/mm1", _ft_topic_name);

        //robot_ = robot->get<hardware_interface::EffortJointInterface>();
        for (unsigned int i = 0; i < _joint_names.size(); i++)
        {
            // std::cout << "i: " << i << " | _joint_names[i]: " << _joint_names[i] << std::endl;
            hardware_interface::JointHandle j = robot->getHandle(_joint_names[i]);
            joints_.push_back(j);
            // std::cout << "Joint Handle Interface: Name: " << joints_[i - 0].getName() << std::endl;
        }

        if (_joint_names.size() == joints_.size())
        {
            all_joints_interface = true;
            platform_joints = 0;
        }
        // Force Torque Sensor Handler
        //ft_ = ft_sensor->getHandle("ft_sensor");
        return true;
    }

    void admittance_controller::starting(const ros::Time &time)
    {
        ROS_INFO("Starting Admittance Controller");

        std::string ns_prefix = node_.getNamespace();

        admittance_controller::read_xml_controller_params(node_, ns_prefix);

        ros::spinOnce();

        qdpos.resize(_joint_names.size());
        qdvel.resize(_joint_names.size());
        qdacc.resize(_joint_names.size());
        qdeff.resize(_joint_names.size());

        // Get Time Now
        begin = ros::Time::now().toSec();

        // get platform state
        for (unsigned int i = 0; i < platform_joints; i++)
        {
            qnam[i] = state->q_name[i];
            qpos[i] = state->q_pos[i];
            qvel[i] = state->q_vel[i];
            qeff[i] = state->q_eff[i];
            // qacc[i] = vel_dt[i].getDerivative(qvel[i], time.now().toSec());
        }
        std::cout << "Read Variables in Starting" << std::endl;
        for (unsigned int i = 0; i < joints_.size(); i++)
        {
            qnam[i + platform_joints] = joints_[i].getName();
            qpos[i + platform_joints] = joints_[i].getPosition();
            qvel[i + platform_joints] = joints_[i].getVelocity();
            qeff[i + platform_joints] = joints_[i].getEffort();
            // qacc[i+3] = vel_dt[i+3].getDerivative(qvel[i+3], time.now().toSec());
        }

        qpos = constrainAngle(qpos);

        while (state->ft.size() < 6)
        {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }

        //TODO: Desired Joints The First
        qdpos = qpos;
        qdvel = Eigen::VectorXd::Zero(qvel.size());
        qdacc = Eigen::VectorXd::Zero(qacc.size());

        // Desired End Effector Position
        kdl_ft->ComputeFK(qpos, peed, Reed);
        veed = Eigen::VectorXd::Zero(6);
        veeddot = Eigen::VectorXd::Zero(6);

        // Get ft sensor measurements - express to world frame with gripper compensation routine
        admittance_controller::getEndEffectorWrench(ft, "world", true);

        ft_prev = ft;
        ftdot.resize(6);
        ftdot.Zero(6);
        // Get wrench time derivative
        //ftdot = wrench_time_derivative.getDerivative(ft,time.now().toSec());

        // Robot Forward Kinematics
        std::cout << "FK: \n"
                  << peed << std::endl
                  << Reed << std::endl;

        // Init Send Command Zeros ///(SHOULD NOT BE ZERO COMMAND, USE GRAVITY COMPENASATION INSTEAD)///
        ROS_INFO("Send Commands");
        for (unsigned int i = 0; i < joints_.size(); i++)
            joints_[i].setCommand(0);
    }

    void admittance_controller::update(const ros::Time &time, const ros::Duration &duration)
    {

        ros::spinOnce();
        std::cout <<" *****************  "<<node_.getNamespace()<<" ***************** \n" <<"===========================================" << std::endl;

        // Time Now
        double t = ros::Time::now().toSec() - begin;

        //Control Input Vector
        Eigen::VectorXd u_, ac_, u_g;
        u_.setZero(_joint_names.size());
        u_g.setZero(_joint_names.size());

        for (unsigned int i = 0; i < platform_joints; i++)
        {
            qnam[i] = state->q_name[i];
            qpos[i] = state->q_pos[i];
            qvel[i] = state->q_vel[i];
            qeff[i] = state->q_eff[i];
            // qacc[i] = vel_dt[i].getDerivative(qvel[i], time.now().toSec());
        }

        /* Read Variables */
        for (unsigned int i = 0; i < joints_.size(); i++)
        {
            qnam[i + platform_joints] = joints_[i].getName();
            qpos[i + platform_joints] = joints_[i].getPosition();
            qvel[i + platform_joints] = joints_[i].getVelocity();
            qeff[i + platform_joints] = joints_[i].getEffort();
            // qacc[i + platform_joints] = vel_dt[i + platform_joints].getDerivative(qvel[i + platform_joints], t);
        }

        qpos = constrainAngle(qpos); 

        /*  Get ft sensor measurements - express to base frame */
        admittance_controller::getEndEffectorWrench(ft, "world", true); // expressed in world frame and compensate gripper
        // Get wrench time derivative
        //ftdot = wrench_time_derivative.getDerivative(ft,time.now().toSec());
        //std::cout << "FDOT:\n" << ftdot<<std::endl;
        ftdot = (ft - ft_prev) / duration.toSec();
        ft_prev = ft;
        // Publish wrench time derivative
        admittance_controller::publisCorrectedFTDot(ftdot, "world");


        //////////////////////////////////////////////////////////////////////
        Eigen::MatrixXd Jac(6, joints_.size()), Jacdot(6, joints_.size());

        // Controller Type to Gravity Compesation
        if (_control_type == "gravity")
        {
            ac_ = u_g;
        }

        else if (_control_type == "force")
        {
            std::cout << " -----  CONTROLLER FORCE ---- " << std::endl;

            if (_control_space == "joint")
            {
                Eigen::VectorXd fdw(6);
                fdw.setZero(6);

                admittance_controller::getEndEffectorWrench(ft, "world", true);
                for (size_t i = 0; i < 3; i++)
                {
                    fdw[i] = At[0];
                }

                std::cout << "--- Error Force ---" << std::endl;
                Eigen::VectorXd f_error = (fdw - ft);
                std::cout << Eigen::Map<Eigen::RowVectorXd>(f_error.data(), f_error.size()) << std::endl;

                admittance_controller::publishJointError(f_error, qdvel - qvel, fdw);

                //command in joint space -> acceleration (need integration)
                std::cout << "--- Compute Acceleration ---" << std::endl;
                Eigen::MatrixXd Jacpinv;
                kdl_ft->pinv(Jac, Jacpinv);
                Eigen::VectorXd ac = Jacpinv * (Kdc * (f_error)); // Eigen::VectorXd::Zero(6);//(Kdc * (f_error) );//+ Ddc*(-ftdot));
                std::cout << "Acceleration: " << Eigen::Map<Eigen::RowVectorXd>(ac.data(), ac.size()) << std::endl;

                ac_ = ac * duration.toSec() + qvel;
            }
        }

        else if (_control_type == "pd")
        {
            std::cout << " -----  CONTROLLER PD -------" << std::endl;

            if (_control_space == "joint")
            {
                Eigen::VectorXd qdpos2 = Eigen::VectorXd::Zero(qdpos.size());
                for (size_t i = 0; i < qdpos.size(); i++)
                {
                    qdpos2[i] = qdpos[i] + At[0] * sin(omega[0] * t);
                }

                admittance_controller::getEndEffectorWrench(ft, "world", true);

                std::cout << "--- Error Joionts ---" << std::endl;
                Eigen::VectorXd q_error = qdpos2 - qpos;
                std::cout << Eigen::Map<Eigen::RowVectorXd>(q_error.data(), q_error.size()) << std::endl;

                // std::cout << "Publish Errors" << std::endl;
                admittance_controller::publishJointError(qdpos2 - qpos, qdvel - qvel, qdpos2);
                //command in joint space -> acceleration (need integration)
                std::cout << "--- Compute Acceleration ---" << std::endl;
                // std::cout <<"Kdq size: "<<Kdq.rows()<<","<<Kdq.cols()<<std::endl;
                // std::cout <<"Ddq size: "<<Ddq.rows()<<","<<Ddq.cols()<<std::endl;

                Eigen::VectorXd ac = Kdq * (qdpos2 - qpos) + Ddq * (qdvel - qvel);
                std::cout << "Acceleration: " << Eigen::Map<Eigen::RowVectorXd>(ac.data(), ac.size()) << std::endl;
                std::cout << "Kdq: \n"
                          << Kdq << std::endl;
                // std::cout<<"Duration: "<<duration.toSec()<<std::endl;
                ac_ = ac * duration.toSec() + qvel;
            }

            else
            {

                Eigen::VectorXd vee = Eigen::VectorXd::Zero(6);
                Eigen::Vector3d pee, eo;
                Eigen::Matrix3d Ree;
                Eigen::MatrixXd Jac(6, qpos.size()), Jac_dot(6, qpos.size()), Jac_inv(qpos.size(), 6);
                // Compute Position and Velocity
                kdl_ft->ComputeFKVel(qpos, qvel, vee, pee, Ree);

                // Compute Orientation Error
                Eigen::Matrix3d Reed2;
                Reed2 = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-0.0 * M_PI, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(0.0 * M_PI, Eigen::Vector3d::UnitZ());
                admittance_controller::ComputeOrientationError(Reed * Reed2, Ree, eo);

                // Euler Tranformation Matrix
                Eigen::Matrix3d Tf;
                kdl_ft->eulerTf(eo, Tf);
                Tf = Ree * Tf;
                eo = eo;
                //Kdc.block<3,3>(3,3) = Tf*Kdc.block<3,3>(3,3);

                kdl_ft->ComputeJac(qpos, Jac);
                kdl_ft->ComputeJacDot(qpos, qvel, Jac_dot);
                kdl_ft->pinv(Jac, Jac_inv);

                Eigen::VectorXd e(6), ep(3), peed2(3);

                peed2[0] = peed[0] + At[0] * sin(omega[0] * t); //At[0];//
                peed2[1] = peed[1] + At[1] * cos(omega[1] * t); //At[1]*cos(omega[1]*t);
                peed2[2] = peed[2] + At[2] * sin(omega[2] * t);

                ep = peed2 - pee;
                e << ep, -eo;

                admittance_controller::publisEndEffectorError(ep, -eo, veed.head(3) - vee.head(3), veed.tail<3>() - vee.tail<3>());

                Eigen::VectorXd qc;
                qc = (qlim_upper + qlim_lower) / 2.0;

                // command in cartesian space : accelaration -> integration needed
                Eigen::VectorXd ac = 5.0 * (Kdc * e + Ddc * (veed - vee)); //Eigen::VectorXd::Zero(6);//
                // command in joint space : accelaration -> integration needed
                Eigen::MatrixXd Wc;
                admittance_controller::JointLimitDumpimgMat(qpos, Wc);
                Eigen::MatrixXd JacInvDamped = (Jac.transpose() * Jac + Wc).inverse() * Jac.transpose();
                //ac_ = JacInvDamped*(ac-Jac_dot*qvel);
                ac_ = Jac_inv * (ac - Jac_dot * qvel) + (Eigen::MatrixXd::Identity(qvel.size(), qvel.size()) - Jac_inv * Jac) * Wc * (qc - qpos);
            }
        }

        // IMPEDANCE CONTROL
        else if (_control_type == "classical")
        {
            std::cout << " -----  CLASSICAL CONTROLLER ADMITTANCE -------" << std::endl;

            // JOINT SPACE
            if (_control_space == "joint")
            {

                // compute jacobian of ft link
                Eigen::MatrixXd Jac(6, qpos.size());
                kdl_ft->ComputeJac(qpos, Jac);

                // create a trajectory for joints
                Eigen::VectorXd qdpos2 = Eigen::VectorXd::Zero(qdpos.size());
                for (size_t i = 0; i < qdpos.size(); i++)
                {
                    qdpos2[i] = qdpos[i] + At[0] * sin(omega[0] * t);
                }

                Eigen::VectorXd eq =  qdpos2 - qpos;
                Eigen::VectorXd eqdot =  qdvel - qvel;
                Eigen::VectorXd admQ = Kdq * (eq) + Ddq * (eqdot);
                Eigen::VectorXd admT = _forceGain*Jac.transpose() * ft ;
                Eigen::VectorXd ac = Mdq.inverse() * (admQ + admT);
                // std::cout<<"Kdq: \n"<<Kdq<<std::endl;
                // std::cout<<"Jacobian\n"<<Jac.transpose()<<std::endl;
                // std::cout<<"Jacobian norm:" <<Jac.norm()<<std::endl;
                // std::cout<<"Jacobian norm:\n" <<pseudoinverse(Jac)<<std::endl;
                //if(ft.norm()>1000){exit(-1);}

                admittance_controller::publishJointError(qdpos2 - qpos, qdvel - qvel, admT);
    
               std::cout << "Kq+Dqdot:\n"
                          << Eigen::Map<Eigen::RowVectorXd>(admQ.data(),admQ.size()) << std::endl;
                std::cout << "Jt*F:\n"
                          << Eigen::Map<Eigen::RowVectorXd>(admT.data(),admT.size()) << std::endl;
                std::cout<<"Md.inv*(Kdq * (qdpos2 - qpos) + Ddq * (qdvel - qvel) + Jac.transpose() * ft): \n"<< Eigen::Map<Eigen::RowVectorXd>(ac.data(),ac.size()) <<std::endl;
                std::cout<<"ac*dt: \n"<<ac*duration.toSec() <<std::endl;
                // command in joint space : accelaration -> integration needed
                ac_ = ac * duration.toSec() + qvel;
                //std::cout<<"Force Torque: \n"<<Eigen::Map<Eigen::RowVectorXd>(ft.data(),ft.size())<<std::endl;
                // std::cout<<"Force Torque in joints: \n"<<Eigen::Map<Eigen::RowVectorXd>(fq.data(),fq.size())<<std::endl;
                //std::cout<< "Impedance Commands:\n"<<Eigen::Map<Eigen::RowVectorXd>(ac.data(),ac.size())<<std::endl;
            }

            // CARTESIAN SPACE
            else
            {

                Eigen::VectorXd vee = Eigen::VectorXd::Zero(6);
                Eigen::Vector3d pee, eo;
                Eigen::Matrix3d Ree;
                Eigen::MatrixXd Jac(6, qpos.size()), Jac_dot(6, qpos.size()), Jac_inv(qpos.size(), 6);

                // Compute Position and Velocity
                kdl_ft->ComputeFKVel(qpos, qvel, vee, pee, Ree);

                // Compute  Orientation Error
                Eigen::Matrix3d Reed2;

                Reed2 = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-0.2 * M_PI, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(0.0 * M_PI, Eigen::Vector3d::UnitZ());

                admittance_controller::ComputeOrientationError(Reed * Reed2, Ree, eo);

                // Compute Jacobian Related Variables
                kdl_ft->ComputeJac(qpos, Jac);
                kdl_ft->ComputeJacDot(qpos, qvel, Jac_dot);
                kdl_ft->pinv(Jac, Jac_inv);

                Eigen::VectorXd e(6), ep(3);
                Eigen::Vector3d peed2;
                if (false)
                {
                    peed2[0] = peed[0] + At[0] * sin(omega[0] * t); //At[0];//
                    peed2[1] = peed[1] + At[1];                     //At[1]*cos(omega[1]*t);
                    peed2[2] = peed[2] + At[2];                     //At[2]*sin(omega[2]*t);

                    veed[0] = 0.0; //At[0]*omega[0]*cos(omega[0]*t);
                    veed[1] = 0.0; //-At[1]*omega[1]*sin(omega[1]*t);
                    veed[2] = 0.0; //At[2]*omega[2]*cos(omega[2]*t);
                }
                else
                {
                    peed2 = peed;
                }

                // Position and Orientation Error
                ep = peed2 - pee;
                e << ep, -eo;

                admittance_controller::publisEndEffectorError(ep, -eo, veed.head(3) - vee.head(3), veed.tail<3>() - vee.tail<3>());

                std::cout << "Error Position:\n"
                          << peed2 - pee << std::endl;
                std::cout << "Error Orientation:\n"
                          << eo << std::endl;
                std::cout << "Error Velocity:\n"
                          << veed - vee << std::endl;

                // Get Joint Limits Avoidance Dumping
                Eigen::MatrixXd Wc;
                admittance_controller::JointLimitDumpimgMat(qpos, Wc);
                std::cout << "Wc * qvel: \n"
                          << Wc * qvel << std::endl;
                // commands in Cartesian Space: Acceleration
                Eigen::VectorXd ac = Mdc.inverse() * (Kdc * e + Ddc * (veed - vee));
                // commands in Joint Space: Acceleration -> need to be integrated
                ac_ = Jac_inv * (ac - Jac_dot * qvel);
            }
        }

        // Joint Velocity Commands as Computed by Controller
        u_ = ac_;
         std::cout << "Commands\n"
                   << u_.transpose() << std::endl;
        // std::cout << "dt: " << duration.toSec() << std::endl;

        if (u_.size() != joints_.size())
        {
            ROS_ERROR("Computed Command Size is not equal to joint size!");
        }

        // Init Send Command Zeros ///(SHOULD NOT BE ZERO COMMAND, USE GRAVITY COMPENASATION INSTEAD)///
        for (unsigned int i = 0; i < joints_.size(); i++)
        {

            //joints_[i].setCommand(0.0);
            //if (i==3)
            joints_[i].setCommand(u_[i + platform_joints]);
            // Commend to send commands
            // std::cout << " Name: " << joints_[i].getName() << std::endl
            //           << " Vel: " << joints_[i].getVelocity() << " Pos: " << joints_[i].getPosition() << std::endl;
        }

        std::cout << "===========================================" << std::endl;
    }

    void admittance_controller::stopping(const ros::Time &time)
    {
        std::cout << "Stopping Dummy Controller" << std::endl;

        // Spin Once To Pub MSGS
        ros::spinOnce();

        for (unsigned int i = 0; i < joints_.size(); i++)
            joints_[i].setCommand(0.0);
    }

} // namespace force_controllers

PLUGINLIB_EXPORT_CLASS(force_controllers::admittance_controller, controller_interface::ControllerBase)