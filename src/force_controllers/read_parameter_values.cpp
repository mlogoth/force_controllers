#include <force_controllers/admittance_controller.hpp>
namespace force_controllers
{
    /*
    * Read From Parameter Server Robot Parameters
    * joint names, joint limits, robot description    
    * */

    void admittance_controller::read_xml_robot_params(const ros::NodeHandle &_node, std::string _ns)
    {
        /*
        * Get Joint Names for the controller
        */
        XmlRpc::XmlRpcValue joint_names;
        if (!node_.getParam(_ns + "/joints", joint_names))
        {
            ROS_ERROR("No 'joints' defined in controller. (namespace: %s)",
                      node_.getNamespace().c_str());
        }

        if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("'joints' is not a struct. (namespace: %s)",
                      node_.getNamespace().c_str());
        }

        std::cout << "Joint Names Parameter Structure Size: " << joint_names.size() << std::endl;

        for (int i = 0; i < joint_names.size(); i++)
        {
            XmlRpc::XmlRpcValue &name_value = joint_names[i];
            if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_ERROR("joints are not strings. (namespace: %s)",
                          node_.getNamespace().c_str());
                //return false;
            }
            //hardware_interface::JointHandle j = robot_->getHandle((std::string)name_value);
            //joints_.push_back(j);
            _joint_names.push_back((std::string)name_value);
            std::cout << _joint_names[i] << std::endl;
        }

        // Get the Robot Description Name
        if (node_.hasParam(_ns + "/robot_description"))
        {
            node_.getParam(_ns + "/robot_description", _robot_description_name);
            std::cout << "Robot Description Name: " << _robot_description_name << std::endl;
        }
        else
        {
            ROS_WARN("Parameter 'Robot Description' is not set, set the default value: /robot_description");
            _robot_description_name = "/robot_description";
        }

        /*
     * KDL Tree Parameters
    */

        if (node_.hasParam(_ns + "/kdl/root"))
        {
            node_.getParam(_ns + "/kdl/root", _robot_root_link);
            std::cout << "Robot Root Link: " << _robot_root_link << std::endl;
        }
        else
        {
            ROS_WARN("Parameter 'kdl/root' is not set, default value: world");
            _robot_root_link = "world";
        }

        if (node_.hasParam(_ns + "/kdl/tip"))
        {
            node_.getParam(_ns + "/kdl/tip", _robot_tip_link);
            std::cout << "Robot Tip Link: " << _robot_tip_link << std::endl;
        }
        else
        {
            ROS_WARN("Parameter 'kdl/tip' is not set, default value: end_effector_link");
            _robot_tip_link = "end_effector_link";
        }

        if (node_.hasParam(_ns + "/kdl/finger_1"))
        {
            node_.getParam(_ns + "/kdl/finger_1", _robot_finger1_link);
            std::cout << "Robot Tip Link: " << _robot_finger1_link << std::endl;
        }
        else
        {
            ROS_WARN("Parameter 'kdl/finger_1' is not set, default value: finger_link_1");
            _robot_finger1_link = "finger_link_1";
        }

        if (node_.hasParam(_ns + "/kdl/finger_2"))
        {
            node_.getParam(_ns + "/kdl/finger_2", _robot_finger2_link);
            std::cout << "Robot Finger 2 Link: " << _robot_finger2_link << std::endl;
        }
        else
        {
            ROS_WARN("Parameter 'kdl/finger_2' is not set, default value: finger_link_2");
            _robot_finger2_link = "finger_link_2";
        }

        /*
     * Force Torque Sensor Parameters
    */

        if (node_.hasParam(_ns + "/ft_sensor/topic_name"))
        {
            node_.getParam(_ns + "/ft_sensor/topic_name", _ft_topic_name);
            std::cout << "Robot ft_sensor/topic_name: " << _ft_topic_name << std::endl;
        }
        else
        {
            ROS_WARN("Parameter 'ft_sensor/topic_name' is not set, default value: ft_sensor");
            _ft_topic_name = "ft_sensor";
        }

        if (node_.hasParam(_ns + "/ft_sensor/attached_link"))
        {
            node_.getParam(_ns + "/ft_sensor/attached_link", _ft_link_name);
            std::cout << "Robot ft_sensor/attached_link: " << _ft_link_name << std::endl;
        }
        else
        {
            ROS_WARN("Parameter 'ft_sensor/attached_link' is not set, default value: ft_link");
            _ft_link_name = "ft_link";
        }

        // Lower
        XmlRpc::XmlRpcValue xml_lower;
        node_.getParam(_ns + "/joint_limits/lower", xml_lower);
        if (xml_lower.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Param admittance_controller/joint_limits/lower is not a list");
        }
        else
        {
            // q lim has the same size as joints
            qlim_lower.resize(_joint_names.size());

            if (_joint_names.size() != xml_lower.size())
            {
                ROS_ERROR("yaml file: Joint Names Size and joint limits size not equal");
            }

            for (unsigned int i = 0; i < _joint_names.size(); ++i)
            {
                if (xml_lower[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && xml_lower[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
                {
                    ROS_ERROR("lower[%d] is not a double", i);
                }
                else
                {
                    qlim_lower[i] = xml_lower[i];
                }
            }
        }

        // upper
        XmlRpc::XmlRpcValue xml_upper;
        node_.getParam(_ns + "/joint_limits/upper", xml_upper);
        if (xml_upper.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Param admittance_controller/joint_limits/upper is not a list");
        }
        else
        {
            // q lim has the same size as joints
            qlim_upper.resize(_joint_names.size());

            if (_joint_names.size() != xml_upper.size())
            {
                ROS_ERROR("yaml file: Joint Names Size and joint limits size not equal");
            }

            for (unsigned int i = 0; i < xml_upper.size(); ++i)
            {
                if (xml_upper[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && xml_upper[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
                {
                    ROS_ERROR("upper[%d] is not a double", i);
                }
                else
                {
                    qlim_upper[i] = xml_upper[i];
                }
            }
        }
    }

    /*
* Read From Parameter Server Controller Parameters
* Gains, Type, Space    
* */
    void admittance_controller::read_xml_controller_params(const ros::NodeHandle &_node, std::string _ns)
    {
        // Get the Control Type
        if (node_.hasParam(_ns + "/control_type"))
        {
            std::string cntrl_type;
            node_.getParam(_ns + "/control_type", cntrl_type);
            std::cout << "Control Type Name Name: " << cntrl_type << std::endl;
            // Convert to lowercase and check if it is valid
            _control_type = admittance_controller::str_tolower(cntrl_type);
            if (not(std::find(type_of_controllers.begin(), type_of_controllers.end(), _control_type) != type_of_controllers.end()))
            {
                ROS_ERROR("admittance_controller: Wrong Control Type Defined");
                _control_type = "gravity";
            }
        }
        else
        {
            ROS_WARN("Parameter 'Control Type' not set, set the default one: 'Gravity'");
            _control_type = "gravity";
        }

        // Get The Control Space - Joint Space or Cartesian
        if (node_.hasParam(_ns + "/control_space"))
        {
            std::string cntrl_space;
            node_.getParam(_ns + "/control_space", cntrl_space);
            std::cout << "Control Space Name: " << cntrl_space << std::endl;
            _control_space = admittance_controller::str_tolower(cntrl_space);
            if (not(std::find(type_of_control_space.begin(), type_of_control_space.end(), _control_space) != type_of_control_space.end()))
            {
                ROS_ERROR("admittance_controller: Wrong Control Space Defined");
                _control_space = "cartesian";
            }

            else
            {
                std::list<std::string>::iterator it = type_of_control_space.begin();
                if (_control_space == *it)
                {
                    _gains_vector_size = _joint_names.size();
                }
            }
        }
        else
        {
            ROS_WARN("Parameter 'control_space' not set, set the default one: Cartesian");
            _control_space = "cartesian";
        }

        /* Depending on the type of control space, the size of the control gains should be set
     * properly.
    */
        if (_control_space == "joint")
        {
            ROS_WARN("JOINT SPACE CONTROL");
            _gains_vector_size = _joint_names.size();
        }
        else
        {
            ROS_WARN("CARTESIAN SPACE CONTROL");
            _gains_vector_size = 6;
        }

        /*
    * Get Gains for the controller
    */
        XmlRpc::XmlRpcValue xml_gains_Md, xml_gains_Kd, xml_gains_Dd;

        // Kf 
        node_.getParam(_ns+"/gains/force_gain",xml_gains_Md);
        if (xml_gains_Md.getType() == XmlRpc::XmlRpcValue::TypeDouble || xml_gains_Md.getType() == XmlRpc::XmlRpcValue::TypeInt) 
        {
            _forceGain = xml_gains_Md;
        }

        else 
        {
            ROS_ERROR("Admittance Control Force Gain is not a double or int");
            _forceGain = 1.0;
        }


        // MD Gains
        node_.getParam(_ns + "/gains/joints/Md", xml_gains_Md);
        if (xml_gains_Md.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Param 'gains/Md' is not a list");
        }
        else
        {
            if (_joint_names.size() != xml_gains_Md.size())
            {
                ROS_ERROR("yaml file: Joint Names Size and joint Md size not equal");
            }

            Mdq.resize(_joint_names.size(), _joint_names.size());
            Mdq = Eigen::MatrixXd::Identity(_joint_names.size(), _joint_names.size());
            for (unsigned int i = 0; i < _joint_names.size(); ++i)
            {
                if (xml_gains_Md[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && xml_gains_Md[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
                {
                    ROS_ERROR("Gain Md[%d] is not a double", i);
                }
                else
                {
                    Mdq(i, i) = xml_gains_Md[i];
                }
            }
        }

        // Dd Gains
        node_.getParam(_ns + "/gains/joints/Dd", xml_gains_Dd);
        if (xml_gains_Dd.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Param 'gains/Dd' is not a list");
        }
        else
        {

            if (_joint_names.size() != xml_gains_Dd.size())
            {
                ROS_ERROR("yaml file: Joint Names Size and joint Dd size not equal");
            }

            Ddq.resize(_joint_names.size(), _joint_names.size());
            Ddq = Eigen::MatrixXd::Identity(_joint_names.size(), _joint_names.size());
            for (unsigned int i = 0; i < _joint_names.size(); ++i)
            {
                if (xml_gains_Dd[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && xml_gains_Dd[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
                {
                    ROS_ERROR("Gain Dd[%d] is not a double", i);
                }
                else
                {
                    Ddq(i, i) = xml_gains_Dd[i];
                }
            }
        }

        /*
    * Kd Gains: Get the KD matrix gains from the paramater server
    */
        node_.getParam(_ns + "/gains/joints/Kd", xml_gains_Kd);
        if (xml_gains_Kd.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Param 'gains/Κd' is not a list");
        }
        else
        {
            if (_joint_names.size() != xml_gains_Kd.size())
            {
                ROS_ERROR("yaml file: Joint Names Size and joint Kd size not equal");
            }

            Kdq.resize(_joint_names.size(), _joint_names.size());
            Kdq = Eigen::MatrixXd::Identity(_joint_names.size(), _joint_names.size());
            for (unsigned int i = 0; i < _joint_names.size(); ++i)
            {
                if (xml_gains_Kd[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && xml_gains_Kd[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
                {
                    ROS_ERROR("Gain Kd[%d] is not a double", i);
                }
                else
                {
                    Kdq(i, i) = xml_gains_Kd[i];
                }
            }
        }

        // MD Gains
        node_.getParam(_ns + "/gains/cartesian/Md", xml_gains_Md);
        if (xml_gains_Md.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Param 'gains/Md' is not a list");
        }
        else
        {
            //TODO: Check the size of the parameters lists!
            Mdc.resize(xml_gains_Md.size(), xml_gains_Md.size());
            Mdc = Eigen::MatrixXd::Identity(xml_gains_Md.size(), xml_gains_Md.size());
            for (unsigned int i = 0; i < xml_gains_Md.size(); ++i)
            {
                if (xml_gains_Md[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && xml_gains_Md[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
                {
                    ROS_ERROR("Gain Md[%d] is not a double", i);
                }
                else
                {
                    Mdc(i, i) = xml_gains_Md[i];
                }
            }
        }

        // Dd Gains
        node_.getParam(_ns + "/gains/cartesian/Dd", xml_gains_Dd);
        if (xml_gains_Dd.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Param 'gains/Dd' is not a list");
        }
        else
        {
            Ddc.resize(xml_gains_Dd.size(), xml_gains_Dd.size());
            Ddc = Eigen::MatrixXd::Identity(xml_gains_Dd.size(), xml_gains_Dd.size());
            for (unsigned int i = 0; i < xml_gains_Dd.size(); ++i)
            {
                if (xml_gains_Dd[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && xml_gains_Dd[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
                {
                    ROS_ERROR("Gain Dd[%d] is not a double", i);
                }
                else
                {
                    Ddc(i, i) = xml_gains_Dd[i];
                }
            }
        }

        /*
    * Kd Gains: Get the KD matrix gains from the paramater server
    */
        node_.getParam(_ns + "/gains/cartesian/Kd", xml_gains_Kd);
        if (xml_gains_Kd.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Param 'gains/Κd' is not a list");
        }
        else
        {
            Kdc.resize(xml_gains_Kd.size(), xml_gains_Kd.size());
            Kdc = Eigen::MatrixXd::Identity(xml_gains_Kd.size(), xml_gains_Kd.size());
            for (unsigned int i = 0; i < xml_gains_Kd.size(); ++i)
            {
                if (xml_gains_Kd[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && xml_gains_Kd[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
                {
                    ROS_ERROR("Gain Kd[%d] is not a double", i);
                }
                else
                {
                    Kdc(i, i) = xml_gains_Kd[i];
                }
            }
        }

        XmlRpc::XmlRpcValue xml_Wq;
        node_.getParam(_ns + "/gains/cartesian/Wq", xml_Wq);
        if (xml_Wq.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Param 'xml_Wq/Κd' is not a list");
        }
        else
        {
            Wq.resize(xml_Wq.size(), xml_Wq.size());
            Wq = Eigen::MatrixXd::Identity(xml_Wq.size(), xml_Wq.size());
            for (unsigned int i = 0; i < xml_gains_Kd.size(); ++i)
            {
                if (xml_Wq[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && xml_Wq[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
                {
                    ROS_ERROR("Gain Kd[%d] is not a double", i);
                }
                else
                {
                    Wq(i, i) = xml_Wq[i];
                }
            }
        }

        // Wo
        XmlRpc::XmlRpcValue xml_W0;
        node_.getParam(_ns + "/joint_limits/max_damping_gains", xml_W0);
        if (xml_W0.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Param admittance_controller/joint_limits/max_damping_gains is not a list");
        }
        else
        {
            // q lim has the same size as joints
            W0.resize(_joint_names.size());
            for (unsigned int i = 0; i < xml_W0.size(); ++i)
            {
                if (xml_W0[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && xml_W0[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
                {
                    ROS_ERROR("max_damping_gains[%d] is not a double", i);
                }
                else
                {
                    W0[i] = xml_W0[i];
                }
            }
        }

        // A
        XmlRpc::XmlRpcValue xml_At;
        node_.getParam(_ns + "/trajectory/A", xml_At);
        if (xml_At.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Param admittance_controller/trajectory/A is not a list");
        }
        else
        {
            // q lim has the same size as joints
            At.resize(xml_At.size());
            for (unsigned int i = 0; i < xml_At.size(); ++i)
            {
                if (xml_At[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && xml_At[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
                {
                    ROS_ERROR("max_damping_gains[%d] is not a double", i);
                }
                else
                {
                    At[i] = xml_At[i];
                }
            }
        }

        // omega
        XmlRpc::XmlRpcValue xml_omega;
        node_.getParam(_ns + "/trajectory/omega", xml_omega);
        if (xml_omega.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Param admittance_controller/trajectory/omega is not a list");
        }
        else
        {
            // q lim has the same size as joints
            omega.resize(xml_omega.size());
            for (unsigned int i = 0; i < xml_omega.size(); ++i)
            {
                if (xml_omega[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && xml_omega[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
                {
                    ROS_ERROR("max_damping_gains[%d] is not a double", i);
                }
                else
                {
                    omega[i] = xml_omega[i];
                }
            }
        }

        // activation percentage
        XmlRpc::XmlRpcValue xml_act;
        node_.getParam(_ns + "/joint_limits/activation_percentage", xml_act);
        if (xml_act.getType() != XmlRpc::XmlRpcValue::TypeDouble && xml_act.getType() != XmlRpc::XmlRpcValue::TypeInt)
        {
            ROS_ERROR("Param admittance_controller/joint_limits/activation_percentage is not a double");
        }
        else
        {
            activation_percentage = xml_act;
        }
    }

    std::string admittance_controller::str_tolower(std::string s)
    {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c) { return std::tolower(c); } // correct
        );
        return s;
    }

    int admittance_controller::JointLimitDumpimgMat(const Eigen::VectorXd &q, Eigen::MatrixXd &Wc)
    {

        Wc = Eigen::MatrixXd::Identity(q.size(), q.size());
        double act_bfr_low;
        double act_bfr_up;

        for (unsigned int k = 0; k < q.size(); k++)
        {
            act_bfr_low = fabs(activation_percentage * qlim_lower[k]);
            act_bfr_up = fabs(activation_percentage * qlim_upper[k]);

            if (q[k] < qlim_lower[k] || q[k] > qlim_upper[k])
                Wc(k, k) = W0[k];

            else if ((qlim_lower[k] + act_bfr_low) < q[k] && q[k] < (qlim_upper[k] - act_bfr_up))
                Wc(k, k) = 0.0;

            else if (qlim_lower[k] <= q[k] && q[k] <= (-qlim_lower[k] + act_bfr_low))
                Wc(k, k) = (W0[k] / 2) * (1 + cos(M_PI * ((q[k] - qlim_lower[k]) / act_bfr_low)));

            else
                Wc(k, k) = (W0[k] / 2) * (1 + cos(M_PI * ((qlim_upper[k] - q[k]) / act_bfr_up)));

            std::cout << "\033[1;31mbold Wc[" << k << "] =" << Wc(k, k) << "\033[0m" << std::endl;
        }
    }

    void admittance_controller::ComputeOrientationError(const Eigen::Matrix3d &Reed_, const Eigen::Matrix3d &Ree_, Eigen::Vector3d &eo_)
    {
        eo_ = 0.5 * (Ree_.row(0).cross(Reed_.row(0)) + Ree_.row(1).cross(Reed_.row(1)) + Ree_.row(2).cross(Reed_.row(2)));
    }

    void admittance_controller::publishJointError(const Eigen::VectorXd &eqpos, const Eigen::VectorXd &eqvel, const Eigen::VectorXd &qpdes)
    {
        sensor_msgs::JointState jst;
        jst.header.stamp = ros::Time::now();
        jst.name.insert(jst.name.begin(),qnam.begin(),qnam.end());
        jst.position.resize(eqpos.size());
        jst.velocity.resize(eqvel.size());
        jst.effort.resize(qpdes.size());
        for (size_t i = 0; i < eqpos.size(); i++)
        {
            jst.position[i] = eqpos[i];
            jst.velocity[i] = eqvel[i];
            jst.effort[i] = qpdes[i];
        }
        pub_eq.publish(jst);
    }

    void admittance_controller::publisEndEffectorError(const Eigen::VectorXd &ep, const Eigen::VectorXd &eo, const Eigen::VectorXd &evl, const Eigen::VectorXd &eva)
    {
        geometry_msgs::Vector3Stamped vec;
        vec.header.stamp = ros::Time::now();
        vec.vector.x = ep[0];
        vec.vector.y = ep[1];
        vec.vector.z = ep[2];
        pub_ep.publish(vec);

        vec.vector.x = eo[0];
        vec.vector.y = eo[1];
        vec.vector.z = eo[2];
        pub_eo.publish(vec);

        vec.vector.x = evl[0];
        vec.vector.y = evl[1];
        vec.vector.z = evl[2];
        pub_evl.publish(vec);

        vec.vector.x = eva[0];
        vec.vector.y = eva[1];
        vec.vector.z = eva[2];
        pub_eva.publish(vec);
    }

    void admittance_controller::publisCorrectedFT(const Eigen::VectorXd &ep, const std::string &frame_id)
    {
        geometry_msgs::WrenchStamped vec;
        vec.header.stamp = ros::Time::now();
        vec.header.frame_id = frame_id;
        vec.wrench.force.x = ep[0];
        vec.wrench.force.y = ep[1];
        vec.wrench.force.z = ep[2];
        vec.wrench.torque.x = ep[3];
        vec.wrench.torque.y = ep[4];
        vec.wrench.torque.z = ep[5];
        pub_ft.publish(vec);
    }


    void admittance_controller::publisCorrectedFTDot(const Eigen::VectorXd &ep, const std::string &frame_id)
    {
        geometry_msgs::WrenchStamped vec;
        vec.header.stamp = ros::Time::now();
        vec.header.frame_id = frame_id;
        vec.wrench.force.x = ep[0];
        vec.wrench.force.y = ep[1];
        vec.wrench.force.z = ep[2];
        vec.wrench.torque.x = ep[3];
        vec.wrench.torque.y = ep[4];
        vec.wrench.torque.z = ep[5];
        pub_ftdot.publish(vec);
    }


    void admittance_controller::getGripperCompensationWrench(Eigen::VectorXd &w)
    {
        w.resize(6);
        w << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        /* compute the wrench to compensate */
        Eigen::VectorXd wrench_to_compensate(6), gvec(3);
        gvec << 0.0, 0.0, -9.81;

        //current pose of the ft
        Eigen::Matrix3d Rftw;
        Eigen::Vector3d pftw;
        kdl_ft->ComputeFK(qpos, pftw, Rftw);

        // for each finger
        for (auto i = 0; i < fingers_link_pose.size(); i++)
        {
            auto finger_seg_pose = fingers_link_pose[i];
            auto finger_seg_mass = fingers_link_mass[i];
            Eigen::VectorXd finger_wrench(6);
            finger_wrench << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            for (auto elt = 0; elt < finger_seg_pose.size(); elt++)
            {
                Eigen::MatrixXd Asft = finger_seg_pose[elt]; // transformation matrix of each segment with non-zero mass wrt f/t link frame.
                Eigen::Matrix3d Rsft = Asft.block<3, 3>(0, 0);
                Eigen::Vector3d Psft;

                Psft << Asft(0, 3), Asft(1, 3), Asft(2, 3);
                Eigen::Vector3d gi = finger_seg_mass[elt] * Rsft * (Rftw * Rsft).transpose() * gvec;

                finger_wrench.segment(0, 3) += gi;
                finger_wrench.segment(3, 3) += Psft.cross(gi);
            }
            w += finger_wrench;
        }
    }

    void admittance_controller::printVector(Eigen::VectorXd vec, std::string name_of_vector)
    {
        std::cout << name_of_vector << ": \n[";
        for (unsigned int i = 0; i < vec.size(); i++)
            std::cout << vec[i] << " ";
        std::cout << "]" << std::endl;
    }

    void admittance_controller::getEndEffectorWrench(Eigen::VectorXd &_ft, std::string wrt_frame, bool compensate_gripper)
    {
        _ft.resize(6);
        /*  Get ft sensor measurements - express to base frame */
        Eigen::VectorXd ftee = state->ft;
        std::cout << "Force Torque Size: " << ftee << std::endl;
        // express this measurement wrt to world
        Eigen::Vector3d Pft;
        Eigen::Matrix3d Rft;
        kdl_ft->ComputeFK(qpos, Pft, Rft);

        //FOR DEBUG PURPOSES
        Eigen::VectorXd fw;
        fw.resize(6);
        fw << Rft * ftee.head(3), Rft * ftee.tail<3>();
        Eigen::Map<Eigen::RowVectorXd> v1(fw.data(), fw.size());
        std::cout << "Raw F/T Meas. wrt World: \n"
                  << v1 << std::endl;

        gripper_wrench.resize(6);
        gripper_wrench << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        admittance_controller::getGripperCompensationWrench(gripper_wrench);
        admittance_controller::publisCorrectedFT(ftee - gripper_wrench,_ft_link_name);

        if (wrt_frame.compare(_ft_link_name) == 0)
        {
            if (compensate_gripper)
                _ft = ftee - gripper_wrench;
            else
                _ft = ftee;
        }
        else if (wrt_frame.compare("world") == 0)
        {
            if (compensate_gripper)
            {
                ftee -= gripper_wrench;
                _ft << Rft * ftee.head(3), Rft * ftee.tail<3>();
            }
            else
                _ft << Rft * ftee.head(3), Rft * ftee.tail<3>();
        }
        else
        {
            ROS_WARN("I dont know the frame to express the sensor wrench!");
            _ft = ftee;
        }
    }



    

} // namespace force_controllers