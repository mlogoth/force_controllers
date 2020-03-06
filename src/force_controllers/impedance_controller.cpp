#include <force_controllers/impedance_controller.hpp>

namespace force_controllers
{

impedance_controller::impedance_controller(void)
{
    ROS_ERROR("impedance_controller initialization");
    node_ = ros::NodeHandle();

}

impedance_controller::~impedance_controller(void){};

/*
* Read From Parameter Server Robot Parameters
* joint names, joint limits, robot description    
* */

void impedance_controller::read_xml_robot_params(const ros::NodeHandle &_node, std::string _ns)
{
    /*
    * Get Joint Names for the controller
    */
    XmlRpc::XmlRpcValue joint_names;
    if (!node_.getParam(_ns+"/impedance_controller/joints", joint_names))
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
    if (node_.hasParam(_ns+"/impedance_controller/robot_description"))
    {
        node_.getParam(_ns+"/impedance_controller/robot_description", _robot_description_name);
        std::cout << "Robot Description Name: " << _robot_description_name << std::endl;
    }
    else
    {
        ROS_WARN("Parameter 'Robot Description' is not set, set the default value: /robot_description");
        _robot_description_name = "/robot_description";
    }


    // Lower
    XmlRpc::XmlRpcValue xml_lower;
    node_.getParam(_ns+"/impedance_controller/joint_limits/lower", xml_lower);
    if (xml_lower.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Param impedance_controller/joint_limits/lower is not a list");
    }
    else
    {
        // q lim has the same size as joints
        qlim_lower.resize(_joint_names.size());
        for (unsigned int i = 0; i < xml_lower.size(); ++i)
        {
            if (xml_lower[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
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
    node_.getParam(_ns+"/impedance_controller/joint_limits/upper", xml_upper);
    if (xml_upper.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Param impedance_controller/joint_limits/upper is not a list");
    }
    else
    {
        // q lim has the same size as joints
        qlim_upper.resize(_joint_names.size());
        for (unsigned int i = 0; i < xml_upper.size(); ++i)
        {
            if (xml_upper[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
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
void impedance_controller::read_xml_controller_params(const ros::NodeHandle &_node, std::string _ns)
{
    // Get the Control Type
    if (node_.hasParam(_ns+"/impedance_controller/control_type"))
    {
        std::string cntrl_type;
        node_.getParam(_ns+"/impedance_controller/control_type", cntrl_type);
        std::cout << "Control Type Name Name: " << cntrl_type << std::endl;
        // Convert to lowercase and check if it is valid
        _control_type = impedance_controller::str_tolower(cntrl_type);
        if (not(std::find(type_of_controllers.begin(), type_of_controllers.end(), _control_type) != type_of_controllers.end()))
        {
            ROS_ERROR("impedance_controller: Wrong Control Type Defined");
            _control_type = "gravity";
        }
    }
    else
    {
        ROS_WARN("Parameter 'Control Type' not set, set the default one: 'Gravity'");
        _control_type = "gravity";
    }

    // Get The Control Space - Joint Space or Cartesian
    if (node_.hasParam(_ns+"/impedance_controller/control_space"))
    {
        std::string cntrl_space;
        node_.getParam(_ns+"/impedance_controller/control_space", cntrl_space);
        std::cout << "Control Space Name: " << cntrl_space << std::endl;
        _control_space = impedance_controller::str_tolower(cntrl_space);
        if (not(std::find(type_of_control_space.begin(), type_of_control_space.end(), _control_space) != type_of_control_space.end()))
        {
            ROS_ERROR("impedance_controller: Wrong Control Space Defined");
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

    // MD Gains
    node_.getParam(_ns+"/impedance_controller/gains/joints/Md", xml_gains_Md);
    if (xml_gains_Md.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Param 'gains/Md' is not a list");
    }
    else
    {

        std::cout<< "Resize Md" <<std::endl;
        //TODO: Check the size of the parameters lists!
        Mdq.resize(xml_gains_Md.size(), xml_gains_Md.size());
        Mdq = Eigen::MatrixXd::Identity(xml_gains_Md.size(), xml_gains_Md.size());
        for (unsigned int i = 0; i < xml_gains_Md.size(); ++i)
        {
            if (xml_gains_Md[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
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
    node_.getParam(_ns+"/impedance_controller/gains/joints/Dd", xml_gains_Dd);
    if (xml_gains_Dd.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Param 'gains/Dd' is not a list");
    }
    else
    {
        Ddq.resize(xml_gains_Dd.size(), xml_gains_Dd.size());
        Ddq = Eigen::MatrixXd::Identity(xml_gains_Dd.size(), xml_gains_Dd.size());
        for (unsigned int i = 0; i < xml_gains_Dd.size(); ++i)
        {
            if (xml_gains_Dd[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
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
    node_.getParam(_ns+"/impedance_controller/gains/joints/Kd", xml_gains_Kd);
    if (xml_gains_Kd.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Param 'gains/Κd' is not a list");
    }
    else
    {
        Kdq.resize(xml_gains_Kd.size(), xml_gains_Kd.size());
        Kdq = Eigen::MatrixXd::Identity(xml_gains_Kd.size(), xml_gains_Kd.size());
        for (unsigned int i = 0; i <xml_gains_Kd.size(); ++i)
        {
            if (xml_gains_Kd[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
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
    node_.getParam(_ns+"/impedance_controller/gains/cartesian/Md", xml_gains_Md);
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
            if (xml_gains_Md[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
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
    node_.getParam(_ns+"/impedance_controller/gains/cartesian/Dd", xml_gains_Dd);
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
            if (xml_gains_Dd[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
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
    node_.getParam(_ns+"/impedance_controller/gains/cartesian/Kd", xml_gains_Kd);
    if (xml_gains_Kd.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Param 'gains/Κd' is not a list");
    }
    else
    {
        Kdc.resize(xml_gains_Kd.size(), xml_gains_Kd.size());
        Kdc = Eigen::MatrixXd::Identity(xml_gains_Kd.size(), xml_gains_Kd.size());
        for (unsigned int i = 0; i <xml_gains_Kd.size(); ++i)
        {
            if (xml_gains_Kd[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
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
    node_.getParam(_ns+"/impedance_controller/gains/cartesian/Wq", xml_Wq);
    if (xml_Wq.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Param 'xml_Wq/Κd' is not a list");
    }
    else
    {
        Wq.resize(xml_Wq.size(), xml_Wq.size());
        Wq = Eigen::MatrixXd::Identity(xml_Wq.size(), xml_Wq.size());
        for (unsigned int i = 0; i <xml_gains_Kd.size(); ++i)
        {
            if (xml_Wq[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
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
    node_.getParam(_ns+"/impedance_controller/joint_limits/max_damping_gains", xml_W0);
    if (xml_W0.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Param impedance_controller/joint_limits/max_damping_gains is not a list");
    }
    else
    {
        // q lim has the same size as joints
        W0.resize(_joint_names.size());
        for (unsigned int i = 0; i < xml_W0.size(); ++i)
        {
            if (xml_W0[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
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
    node_.getParam(_ns+"/impedance_controller/trajectory/A", xml_At);
    if (xml_At.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Param impedance_controller/trajectory/A is not a list");
    }
    else
    {
        // q lim has the same size as joints
        At.resize(xml_At.size());
        for (unsigned int i = 0; i < xml_At.size(); ++i)
        {
            if (xml_At[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
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
    node_.getParam(_ns+"/impedance_controller/trajectory/omega", xml_omega);
    if (xml_omega.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Param impedance_controller/trajectory/omega is not a list");
    }
    else
    {
        // q lim has the same size as joints
        omega.resize(xml_omega.size());
        for (unsigned int i = 0; i < xml_omega.size(); ++i)
        {
            if (xml_omega[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
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
    node_.getParam(_ns+"/impedance_controller/joint_limits/activation_percentage", xml_act);
    if (xml_act.getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
        ROS_ERROR("Param impedance_controller/joint_limits/activation_percentage is not a double");
    }
    else
    {
        activation_percentage = xml_act;
    }
}

std::string impedance_controller::str_tolower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); } // correct
    );
    return s;
}



int impedance_controller::JointLimitDumpimgMat(const Eigen::VectorXd &q, Eigen::MatrixXd &Wc)
{

    Wc = Eigen::MatrixXd::Identity(q.size(), q.size());
    double act_bfr_low;
    double act_bfr_up;

    for (unsigned int k = 0; k < 7; k++)
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

        //std::cout << "Wc[" << k << "] =" << Wc(k, k) << std::endl;
    }
}

void impedance_controller::ComputeOrientationError(const Eigen::Matrix3d &Reed_, const Eigen::Matrix3d &Ree_, Eigen::Vector3d &eo_)
{
    eo_ = 0.5 * (Ree_.row(0).cross(Reed_.row(0)) + Ree_.row(1).cross(Reed_.row(1)) + Ree_.row(2).cross(Reed_.row(2)));
    //Eigen::Matrix3d Red = Ree_.transpose()*Reed_;
    //Eigen::Vector3d eu = Red.eulerAngles(2,1,0);
    //eo_ << eu[2], eu[1], eu[0];
}

void impedance_controller::publishJointError(const Eigen::VectorXd &eqpos, const Eigen::VectorXd &eqvel, const Eigen::VectorXd& qpdes)
{
    sensor_msgs::JointState jst;
    jst.header.stamp = ros::Time::now();
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

void impedance_controller::publisEndEffectorError(const Eigen::VectorXd &ep, const Eigen::VectorXd &eo,const Eigen::VectorXd &evl, const Eigen::VectorXd &eva)
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




bool impedance_controller::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{

    
    pub_eo = n.advertise<geometry_msgs::Vector3Stamped>("/force_controller/error_orientation", 1000);
    pub_ep = n.advertise<geometry_msgs::Vector3Stamped>("/force_controller/error_position", 1000);
    pub_evl = n.advertise<geometry_msgs::Vector3Stamped>("/force_controller/error_linear_velocity", 1000);
    pub_eva = n.advertise<geometry_msgs::Vector3Stamped>("/force_controller/error_angular_velocity", 1000);
    pub_eq = n.advertise<sensor_msgs::JointState>("/force_controller/error_joints", 1000);

    // Get Parameters
    impedance_controller::read_xml_robot_params(node_,"j2n7_mod");
    
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
    kdl = new KDLWrapper(_robot_description_name, node_);
    kdl->init("world", "ee", true);
    kdl_ft = new KDLWrapper(_robot_description_name, node_);
    // init kdl with end effector and base link
    kdl_ft->init("world", "robotiq_85_base_link", true); // if s_y force torque = 'connection_link2'

    int sm = kdl->getChain().getNrOfSegments() - 1;
    int sm2 = kdl_ft->getChain().getNrOfSegments() - 1;

    std::cout << "kdl last segment: " << kdl->getChain().getSegment(sm).getName() << std::endl;
    std::cout << "kdl  ft last segment: " << kdl_ft->getChain().getSegment(sm2).getName() << std::endl;

    // Initialize Robot State
    state = new RobotState(node_, _joint_names.size(), "/j2n7_mod");


    //robot_ = robot->get<hardware_interface::EffortJointInterface>();
    for (unsigned int i = 0; i < _joint_names.size(); i++)
    {
        hardware_interface::JointHandle j = robot->getHandle(_joint_names[i]);
        joints_.push_back(j);
    }
    
    // Force Torque Sensor Handler
    //ft_ = ft_sensor->getHandle("ft_sensor");
    return true;
}




void impedance_controller::starting(const ros::Time &time)
{
    ROS_INFO("Starting Dummy - Test Controller");

    impedance_controller::read_xml_controller_params(node_,"j2n7_mod");

    ros::spinOnce();

    qdpos.resize(_joint_names.size());
    qdvel.resize(_joint_names.size());
    qdacc.resize(_joint_names.size());
    qdeff.resize(_joint_names.size());

    // Get Time Now
    begin = ros::Time::now().toSec();

    std::cout<< "Read Variables in Starting" <<std::endl;
    for (unsigned int i = 0; i < joints_.size(); i++)
    {
        qnam[i] = joints_[i].getName();
        qpos[i] = joints_[i].getPosition();
        qvel[i] = joints_[i].getVelocity();
        qeff[i] = joints_[i].getEffort();
        //qacc[i] = vel_dt[i].getDerivative(qvel[i], time.now().toSec());
    }

    //TODO: Desired Joints The First
    qdpos = qpos;
    qdvel = Eigen::VectorXd::Zero(qvel.size());
    qdacc = Eigen::VectorXd::Zero(qacc.size());

    // Desired End Effector Position
    kdl_ft->ComputeFK(qpos, peed, Reed);
    veed = Eigen::VectorXd::Zero(6);
    veeddot = Eigen::VectorXd::Zero(6);

    // ft measurement is wrt to ee
    Eigen::VectorXd ftee = state->ft;

    // Gravity Coriolis External Compensation
    Eigen::VectorXd u_;

    std::vector<Eigen::VectorXd> fwrenches;
    fwrenches.resize(kdl_ft->getChain().getNrOfSegments());

    for (unsigned int i = 0; i < kdl_ft->getChain().getNrOfSegments(); i++)
    {
        fwrenches[i] = Eigen::VectorXd::Zero(6);
    }
    fwrenches[kdl_ft->getChain().getNrOfSegments() - 1] = ftee;

    // Compute Torques
    kdl_ft->ComputeInverseDynamics(qpos, qvel, Eigen::VectorXd::Zero(qpos.size()), fwrenches, u_);

    if (u_.size() != joints_.size())
    {
        ROS_ERROR("Computed Command Size is not equal to joint size!");
    }

    // Init Send Command Zeros ///(SHOULD NOT BE ZERO COMMAND, USE GRAVITY COMPENASATION INSTEAD)///
    ROS_INFO("Send Commands");
    for (unsigned int i = 0; i < joints_.size(); i++)
        joints_[i].setCommand(u_[i]);
}

void impedance_controller::update(const ros::Time &time, const ros::Duration &duration)
{

    ros::spinOnce();

    // Time Now
    double t = ros::Time::now().toSec() - begin;

    //Control Input Vector
    Eigen::VectorXd u_, u_g;
    u_.setZero(_joint_names.size());
    u_g.setZero(_joint_names.size());

    /* Read Variables */
    for (unsigned int i = 0; i < joints_.size(); i++)
    {
        qnam[i] = joints_[i].getName();
        qpos[i] = joints_[i].getPosition();
        qvel[i] = joints_[i].getVelocity();
        qeff[i] = joints_[i].getEffort();
        qacc[i] = vel_dt[i].getDerivative(qvel[i], t);
    }

    /*  Get ft sensor measurements - express to base frame */
    // this measurement is wrt to ee
    Eigen::VectorXd ftee = state->ft;
    // express this measurement wrt to world
    Eigen::Vector3d Pft, ftfee, fttee;
    Eigen::Matrix3d Rft;
    kdl_ft->ComputeFK(qpos, Pft, Rft);
    ft << Rft * ftee.head(3), Rft * ftee.tail<3>();
    // std::cout << "Forces wrt Base: \n"
    //           << ft.head(3) << std::endl;
    // std::cout << "Torques wrt Base: \n"
    //           << ft.tail<3>() << std::endl;

    // contRotSmoothing(x, xold, cntr);

    // PRINT OUT WHAT YOU GET
    std::cout << "===========================================" << std::endl;
    /*
    std::cout << "CONTROLLER UPDATE" << std::endl;
    std::cout << "Joint Names: \n[";
    for (unsigned int i = 0; i < 7; i++)
        std::cout << qnam[i] << " ";
    std::cout << "]" << std::endl;

    std::cout << "Joint Displacements: \n[";
    for (unsigned int i = 0; i < 7; i++)
        std::cout << qpos[i] << " ";
    std::cout << "]" << std::endl;
    std::cout << "Joint Velocities: \n[";
    for (unsigned int i = 0; i < 7; i++)
        std::cout << qvel[i] << " ";
    std::cout << "]" << std::endl;

    std::cout << "Joint Accelerations: \n[";
    for (unsigned int i = 0; i < 7; i++)
        std::cout << qacc[i] << " ";
    std::cout << "]" << std::endl;

    std::cout << "Joint Efforts: \n[r<Eigen::VectorXd> &fext, Eigen::VectorXd &torques)
{
    
    assertm(fext.size()>=_nlinks,"External forces acting on the links");";
    for (unsigned int i = 0; i < 7; i++)
        std::cout << qeff[i] << " ";
    std::cout << "]" << std::endl;

    std::cout << "Force Torque Sensor:\nF:[" << ft[0] << ", " << ft[1] << ", " << ft[2] << "]\nT:[" << ft[3] << ", " << ft[4] << ", " << ft[5] << "]\n";

    */

    //CONTROLLER
    //v=ddq_des.data+MdJ.inverse()*(KdJ*(dq_des.data-dq.data)+KpJ*(q_des.data-q.data)+jac.data.transpose()*(f_w_obj+f_w_l_base));
    //v=ddq_des.data+MdJ.inverse()*(KdJ*(dq_des.data-dq.data)+KpJ*(q_des.data-q.data)+jac.data.transpose()*(f_w_obj));
    //v=ddq_des.data+MdJ.inverse()*(KdJ*(dq_des.data-dq.data)+KpJ*(q_des.data-q.data));

    //v=ddq_des.data +MdJ.inverse()*(KdJ*(dq_des.data-dq.data)+KpJ*(q_des.data-q.data));
    //v=ddq_des.data +(KdJ*(dq_des.data-dq.data)+KpJ*(q_des.data-q.data)+MdJ.inverse()*jac.data.transpose()*(ft_f));

    // if (imp)
    // {

    //v=0*ddq_des.data +(KdJ*(dq_des.data-dq.data)+KpJ*(q_des.data-q.data)+MdJ.inverse()*jac.data.transpose()*(ft_f));
    //v=ddq_des.data +(KdJ*(dq_des.data-dq.data)+KpJ*(q_des.data-q.data));
    //     v = MdJ.inverse() * (jac.data.transpose() * (ft_f - f_w_obj) - MdJ * Wc * dq.data);
    //     //v=MdJ.inverse()*(jac.data.transpose()*(ft_f-f_w_obj));
    //     //v.setZero();
    //     //v=MdJ.inverse()*jac.data.transpose()*(ft_f);

    //     //U =  -jac.data.transpose()*(ft_f+f_w_obj)+ F + G + Cr*dq.data + Mr*v;
    //     //U =  -jac.data.transpose()*ft_f + G + Cr*dq.data + Mr*v;
    //     //jac.data(1,1)=JacW8*jac.data(1,1);
    //     Uimp = -jac.data.transpose() * (ft_f) + G + F + Cr * dq.data + Mr * v;
    //     //Uimp(1) = Uimp(1) - F(1);
    //     //Uimp(3) = Uimp(3) - F(3);
    // }
    // else if (crt)
    // {
    //     vc = MdC.inverse() * (ft_f - f_w_obj);
    //     get_JacPinv();
    //     v = jacPinv * (vc - jdqd);

    //     Ucrt = -jac.data.transpose() * (ft_f) + F + G + Cr * dq.data + Mr * v;
    // }
    // else if (vlct)
    // {
    //     //	    vc=MdC.inverse()*(ft_f+f_w_obj);
    //     //        get_JacPinv();qdvel = Eigen::VectorXd::Zero(qvel.size());
    //     //        v=jacPinv*(vc-jdqd);

    //     v = MdJ.inverse() * jac.data.transpose() * (ft_f - f_w_obj);

    //     vel = vel + v * duration.toSec();
    //     Uvel = vel + 30 * (-dq.data); // + 1*(-q.data);

    //     //      std::cout.precision(20);
    //     std::cout << "Printing Time++++++++++++++++++" << std::endl;
    //     std::cout << "vel: " << vel(6) << std::endl;
    //     std::cout << "dq.data: " << dq.data(6) << std::endl;
    // }
    // //	else
    // //	{
    // //	    //q_des.data(jnt)= 50*M_PI/180.0*sin(1.0*t);
    // //	    //dq_des.data(jnt)= 50*M_PI/180.0*cos(1.0*t);
    // //	    //ddq_des.data(jnt)= -50*M_PI/180.0*sin(1.0*t);
    // //	    //v=ddq_des.data +MdJ.inverse()*(KdJ*(dq_des.data-dq.data)+KpJ*(q_des.data-q.data));
    // //	    //U =  F + G + Cr*dq.data + Mr*v;
    // //	}

    // //U =  F + G + Cr*dq.data + Mr*v;//LEIPEI  H EXWTERIKH DUNAMH
    // //U =  -jac.data.transpose()*ft_f + F + G + Cr*dq.data + Mr*v;//LEIPEI  H EXWTERIKH DUNAMH

    // Gravity Coriolis External Compensation
    std::vector<Eigen::VectorXd> fwrenches;
    fwrenches.resize(kdl_ft->getChain().getNrOfSegments());
    for (unsigned int i = 0; i < kdl_ft->getChain().getNrOfSegments(); i++)
    {
        fwrenches[i] = Eigen::VectorXd::Zero(6);
    }
    fwrenches[kdl_ft->getChain().getNrOfSegments() - 1] = ftee;

    // Compute Torques of Gravity Compensation
    kdl_ft->ComputeInverseDynamics(qpos, qvel, Eigen::VectorXd::Zero(_joint_names.size()), fwrenches, u_g);

    //////////////////////////////////////////////////////////////////////
    Eigen::VectorXd GravityVector, CoriolisVector;
    Eigen::MatrixXd Mr;
    Eigen::MatrixXd Jac(6, joints_.size()), Jacdot(6, joints_.size());

    //kdl_ft->ComputeJac(qpos,Jac);
    //kdl_ft->ComputeJacDot(qpos,qvel,Jacdot);

    //kdl_ft->ComputeGravityVector(qpos, GravityVector);
    //kdl_ft->ComputeCoriolisVector(qpos, qvel, CoriolisVector);

    //std::cout<< "M*qddot :\n"<< Mr*qacc <<std::endl;
    //std::cout<<"Norm(M*qddot): "<< (Mr*qacc).norm()<<std::endl;

    // Controller Type to Gravity Compesation
    if (_control_type == "gravity")
    {
        u_ = u_g;
    }

    else if (_control_type == "pd")
    {
        std::cout << " -----  CONTROLLER PD -------" << std::endl;

        if (_control_space == "joint")
        {
            std::cout<<"Desired Joionts"<<std::endl;
            std::cout<<qdpos<<std::endl;
            // std::cout << "Error qpos:\n"
            //           << qdpos - qpos << std::endl;
            // std::cout << "Error qvel:\n"
            //           << qdvel - qvel << std::endl;
            kdl_ft->ComputeMassMatrix(qpos, Mr);
            u_ = Mr * (Kdq * (qdpos - qpos) + Ddq * (qdvel - qvel)) + u_g;
        }

        else
        {

            Eigen::VectorXd vee = Eigen::VectorXd::Zero(6);
            Eigen::Vector3d pee, eo;
            Eigen::Matrix3d Ree;
            Eigen::MatrixXd Mx, Jac(6, qpos.size());
            Mx.resize(6, 6);

            Eigen::VectorXd Gx, Cx;
            Gx = Eigen::VectorXd::Zero(6);
            Cx = Eigen::VectorXd::Zero(6);

            // Compute Position and Velocity
            kdl_ft->ComputeFKVel(qpos, qvel, vee, pee, Ree);

            
            // Compute Error
            Eigen::Matrix3d Reed2;

            Reed2 = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(-0.0*M_PI, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(0.0*M_PI, Eigen::Vector3d::UnitZ());
            impedance_controller::ComputeOrientationError(Reed*Reed2, Ree, eo);
            
            // Euler Tranformation Matrix
            //Eigen::Matrix3d Tf; 
            //kdl_ft->eulerTf(eo,Tf);
            //Tf = Ree*Tf;
            //eo = Tf*eo;

            //Eigen::Vector3d eud = (Reed*Reed2).eulerAngles(2,1,0);
           // Eigen::Vector3d euc = Ree.eulerAngles(2,1,0);
            //Eigen::Vector3d eeu = euc - eud ;
            
            //eo << eeu[2], eeu[1], eeu[0];

            // Compute CArtesian Space Dynamics
            kdl_ft->ComputeMassMatrixCartesian(qpos, Mx);
            kdl_ft->ComputeGravityVectorCartesian(qpos, Gx);
            kdl_ft->ComputeCoriolisVectorCartesian(qpos, qvel, Cx);
            kdl_ft->ComputeJac(qpos, Jac);

            Eigen::VectorXd e(6), ep(3);
            ep = peed + At - pee;
            e << ep, -eo;

            impedance_controller::publisEndEffectorError(ep, -eo, veed.head(3)-vee.head(3),veed.tail<3>()-vee.tail<3>());

            std::cout<< "Position Desired: \n" << peed <<std::endl;
            std::cout<< "Error Combined: \n" << e<<std::endl;
            std::cout << "Error Velocity:\n"
                      << veed - vee << std::endl;

            // command to joint space
            Eigen::VectorXd ac = (Kdc * e + Ddc * (veed - vee));//Eigen::VectorXd::Zero(6);//
            u_ = Jac.transpose() * (Cx + Gx - ft + Mx * ac);
        }
    }

    // IMPEDANCE CONTROL
    else if (_control_type == "classical")
    {
         std::cout << " -----  CONTROLLER CLASSICAL IMPEDANCE -------" << std::endl;

        // JOINT SPACE
        if (_control_space == "joint")
        {
            Eigen::MatrixXd Jac(6, qpos.size());
            kdl_ft->ComputeJac(qpos, Jac);

            Eigen::VectorXd qdpos2 = Eigen::VectorXd::Zero(qdpos.size());
            for (size_t i = 0; i < qdpos.size(); i++)
            {
                qdpos2[i] = qdpos[i] + At[0]*sin(omega[0]*t);
            }
            
            
            std::cout << "Error qpos:\n"
                      << qdpos2 - qpos << std::endl;
            std::cout << "Error qvel:\n"
                      << qdvel - qvel << std::endl;

            
            kdl_ft->ComputeMassMatrix(qpos, Mr);
            impedance_controller::publishJointError(qdpos2-qpos,qdvel-qvel,qdpos2);
            Eigen::VectorXd ac = Mdq.inverse() * (Kdq * (qdpos2 - qpos) + Ddq * (qdvel - qvel) ) + Jac.transpose() * ft;
            u_ = Mr * ac + u_g;
        }

        // CARTESIAN SPACE
        else
        {

            Eigen::VectorXd vee = Eigen::VectorXd::Zero(6);
            Eigen::Vector3d pee, eo;
            Eigen::Matrix3d Ree;
            Eigen::MatrixXd Mx, Mq, Jac(6, qpos.size());
            Mx.resize(6, 6);

            Eigen::VectorXd Gx, Cx;
            Gx = Eigen::VectorXd::Zero(6);
            Cx = Eigen::VectorXd::Zero(6);

            // Compute Position and Velocity
            kdl_ft->ComputeFKVel(qpos, qvel, vee, pee, Ree);

            // Compute Error
            Eigen::Matrix3d Reed2;

            Reed2 = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(-0.2*M_PI, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(0.0*M_PI, Eigen::Vector3d::UnitZ());
            impedance_controller::ComputeOrientationError(Reed*Reed2, Ree, eo);

            // Compute CArtesian Space Dynamics
            kdl_ft->ComputeMassMatrixCartesian(qpos, Mx);
            kdl_ft->ComputeGravityVectorCartesian(qpos, Gx);
            kdl_ft->ComputeCoriolisVectorCartesian(qpos, qvel, Cx);
            kdl_ft->ComputeJac(qpos, Jac);
            kdl_ft->ComputeMassMatrix(qpos,Mq);

            Eigen::VectorXd e(6), ep(3);
            Eigen::Vector3d peed2;
            if(true){
                peed2[0] = peed[0] + At[0]*sin(omega[0]*t);//At[0];//
                peed2[1] = peed[1] + At[1];//At[1]*cos(omega[1]*t);
                peed2[2] = peed[2] + At[2];//At[2]*sin(omega[2]*t);

                veed[0] = 0.0;//At[0]*omega[0]*cos(omega[0]*t);
                veed[1] = 0.0;//-At[1]*omega[1]*sin(omega[1]*t);
                veed[2] = 0.0;//At[2]*omega[2]*cos(omega[2]*t);
            }
            else{
                peed2 = peed;
            }
            
            // Position and Orientation Error
            ep = peed2 - pee;
            e << ep, -eo;

            impedance_controller::publisEndEffectorError(ep, -eo, veed.head(3)-vee.head(3),veed.tail<3>()-vee.tail<3>());

            std::cout << "Error Position:\n"
                      << peed2 - pee << std::endl;
            std::cout << "Error Orientation:\n"
                      << eo << std::endl;
            std::cout << "Error Velocity:\n"
                      << veed - vee << std::endl;

            // Get Joint Limits Avoidance Dumping 
            Eigen::MatrixXd Wc;
            impedance_controller::JointLimitDumpimgMat(qpos,Wc);
            std::cout<<"Wc*qvel: \n"<<Wc*qvel<<std::endl;
            // command to joint space
            Eigen::VectorXd ac = Mdc.inverse() * (Kdc * e + Ddc * (veed - vee) );
            u_ = Jac.transpose() * (Cx + Gx - ft)+ Wq * Jac.transpose() * Mx * ac ;//- Mq*Wc*qvel;
        }
    }

    // Controller Type to PD

    std::cout << "Commands\n"
              << u_ << std::endl;

    if (u_.size() != joints_.size())
    {
        ROS_ERROR("Computed Command Size is not equal to joint size!");
    }

    // Init Send Command Zeros ///(SHOULD NOT BE ZERO COMMAND, USE GRAVITY COMPENASATION INSTEAD)///
    for (unsigned int i = 0; i < joints_.size(); i++)
        joints_[i].setCommand(u_[i]);

    // std::cout << "===========================================" << std::endl;
    // std::cout << "           Hz: " << (1.0 / (double)(ros::Time::now().toSec() - begin - t)) << std::endl;
    std::cout << "===========================================" << std::endl;

    // //logging
    // logging(time);
}

void impedance_controller::stopping(const ros::Time &time)
{
    std::cout << "Stopping Dummy Controller" << std::endl;

    // Spin Once To Pub MSGS
    ros::spinOnce();

    for (unsigned int i = 0; i < joints_.size(); i++)
        joints_[i].setCommand(0);
}

} // namespace variable_impedance_slippage_controller

PLUGINLIB_EXPORT_CLASS(force_controllers::impedance_controller, controller_interface::ControllerBase)