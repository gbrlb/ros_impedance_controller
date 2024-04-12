/**
 * @file controller.cpp
 * @author Gennaro Raiola, Michele Focchi
 * @date 12 June, 2018
 * @brief Ros impedance controller.
 */

#include <ros_impedance_controller/controller.h>
#include <string.h>
#include <math.h>

// #include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <chrono>

using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace ros_impedance_controller
{

    const std::string red("\033[0;31m");
    const std::string green("\033[1;32m");
    const std::string yellow("\033[1;33m");
    const std::string cyan("\033[0;36m");
    const std::string magenta("\033[0;35m");
    const std::string reset("\033[0m");

    Controller::Controller()
    {
    }

    Controller::~Controller()
    {
    }

    bool Controller::init(hardware_interface::RobotHW *robot_hw,
                          ros::NodeHandle &root_nh,
                          ros::NodeHandle &controller_nh)
    {
        // getting the names of the joints from the ROS parameter server
        std::cout << cyan << "ROS_IMPEDANCE CONTROLLER: Initialize Ros Impedance Controller" << reset << std::endl;
        root_nh_ = &root_nh;
        assert(robot_hw);

        // TODO
        //     std::string srdf, urdf;
        //     if(!controller_nh.getParam("/robot_description",urdf))
        //     {
        //         ROS_ERROR_NAMED(CLASS_NAME,"robot_description not available in the ros param server");

        //    }
        //    if(!controller_nh.getParam("/robot_semantic_description",srdf))
        //    {
        //        ROS_ERROR_NAMED(CLASS_NAME,"robot_description_semantic not available in the ros param server");

        //    }

        hardware_interface::EffortJointInterface *eff_hw = robot_hw->get<hardware_interface::EffortJointInterface>();

        if (!eff_hw)
        {
            ROS_ERROR("hardware_interface::EffortJointInterface not found");
            return false;
        }

        if (!controller_nh.getParam("joints", joint_names_))
        {
            ROS_ERROR("No joints given in the namespace: %s.", controller_nh.getNamespace().c_str());
            return false;
        }
        else
        {
            std::cout << green << "Found " << joint_names_.size() << " joints" << reset << std::endl;
        }

        // Setting up handles:
        for (int i = 0; i < joint_names_.size(); i++)
        {

            // Getting joint state handle
            try
            {
                std::cout << green << "Loading effort interface for joint " << joint_names_[i] << reset << std::endl;
                joint_states_.push_back(eff_hw->getHandle(joint_names_[i]));
            }
            catch (...)
            {
                ROS_ERROR("Error loading the effort interfaces");
                return false;
            }
        }
        assert(joint_states_.size() > 0);

        // subscriber to the ground truth
        std::string robot_name;
        ros::NodeHandle param_node;
        param_node.getParam("/robot_name", robot_name);
        param_node.getParam("/pid_discrete_implementation", discrete_implementation_);
        if (discrete_implementation_)
        {
            std::cout << green << "Discrete implementation of PID control! (low the gains)" << reset << std::endl;
        }
        else
        {
            std::cout << green << "Continuous implementation of PID control!" << reset << std::endl;
        }

        // Resize the variables
        des_joint_positions_.resize(joint_states_.size());
        des_joint_positions_.fill(0.0);
        integral_action_old_.resize(joint_states_.size());
        integral_action_old_.fill(0.0);
        des_joint_velocities_.resize(joint_states_.size());
        des_joint_velocities_.fill(0.0);
        des_joint_efforts_.resize(joint_states_.size());
        des_joint_efforts_.fill(0.0);
        des_joint_efforts_pids_.resize(joint_states_.size());
        des_joint_efforts_pids_.fill(0.0);
        des_joint_efforts_sum_.resize(joint_states_.size());
        des_joint_efforts_sum_.fill(0.0);
        measured_joint_position_.resize(joint_states_.size());
        measured_joint_position_.fill(0.0);
        measured_joint_velocity_.resize(joint_states_.size());
        measured_joint_velocity_.fill(0.0);

        joint_positions_old_.resize(joint_states_.size());
        joint_positions_old_.resize(joint_states_.size());

        integral_action_old_leg.resize(joint_states_.size());
        integral_action_old_leg.fill(0.0);

        // discrete implementation
        error_.resize(joint_states_.size());
        error_.fill(0.0);
        error1_.resize(joint_states_.size());
        error1_.fill(0.0);

        T_i.resize(joint_states_.size());
        T_i.fill(0.0);
        T_d.resize(joint_states_.size());
        T_d.fill(0.0);

        proportional_action_.resize(joint_states_.size());
        proportional_action_.fill(0.0);
        integral_action_.resize(joint_states_.size());
        integral_action_.fill(0.0);
        derivative_action_.resize(joint_states_.size());
        derivative_action_.fill(0.0);

        use_integral_action_.resize(joint_states_.size());
        use_integral_action_.fill(false);

        joint_type_.resize(joint_states_.size());
        std::fill(joint_type_.begin(), joint_type_.end(), "revolute");

        joint_p_gain_.resize(joint_states_.size());
        joint_i_gain_.resize(joint_states_.size());
        joint_d_gain_.resize(joint_states_.size());
        for (unsigned int i = 0; i < joint_states_.size(); i++)
        {
            // Getting PID gains
            if (!controller_nh.getParam("gains/" + joint_names_[i] + "/p", joint_p_gain_[i]))
            {
                ROS_ERROR("No P gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
                return false;
            }
            if (!controller_nh.getParam("gains/" + joint_names_[i] + "/i", joint_i_gain_[i]))
            {
                ROS_ERROR("No D gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
                return false;
            }
            if (!controller_nh.getParam("gains/" + joint_names_[i] + "/d", joint_d_gain_[i]))
            {
                ROS_ERROR("No I gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
                return false;
            }
            // Check if the values are positive
            if (joint_p_gain_[i] < 0.0 || joint_i_gain_[i] < 0.0 || joint_d_gain_[i] < 0.0)
            {
                ROS_ERROR("PID gains must be positive!");
                return false;
            }
            ROS_DEBUG("P value for joint %i is: %f", i, joint_p_gain_[i]);
            ROS_DEBUG("I value for joint %i is: %f", i, joint_i_gain_[i]);
            ROS_DEBUG("D value for joint %i is: %f", i, joint_d_gain_[i]);

            controller_nh.getParam("joint_type/" + joint_names_[i], joint_type_[i]);

            // get statrup go0 position from yaml (TODO srdf)
            controller_nh.getParam("home/" + joint_names_[i], des_joint_positions_[i]);

            if (joint_i_gain_[i] == 0)
            {
                use_integral_action_[i] = false;
                T_i[i] = 0;
            }
            else
            {
                use_integral_action_[i] = true;
                T_i[i] = joint_p_gain_[i] / joint_i_gain_[i];
            }

            if (joint_p_gain_[i] == 0)
            {
                T_d[i] = 0;
            }
            else
            {
                T_d[i] = joint_d_gain_[i] / joint_p_gain_[i];
            }
        }

        // Create the subscriber
        command_sub_ = root_nh.subscribe("/command", 1, &Controller::commandCallback, this, ros::TransportHints().tcpNoDelay());

        sub_contact_lf = root_nh.subscribe("/" + robot_name + "/lf_foot_bumper", 1, &Controller::receive_contact_lf, this, ros::TransportHints().tcpNoDelay());
        sub_contact_rf = root_nh.subscribe("/" + robot_name + "/rf_foot_bumper", 1, &Controller::receive_contact_rf, this, ros::TransportHints().tcpNoDelay());
        sub_contact_lh = root_nh.subscribe("/" + robot_name + "/lh_foot_bumper", 1, &Controller::receive_contact_lh, this, ros::TransportHints().tcpNoDelay());
        sub_contact_rh = root_nh.subscribe("/" + robot_name + "/rh_foot_bumper", 1, &Controller::receive_contact_rh, this, ros::TransportHints().tcpNoDelay());
        contact_state_msg.contacts.resize(4);
        
        controller_nh.getParam("/effort_limit", effort_limit);
        controller_nh.getParam("/threshold_high", threshold_high);
        controller_nh.getParam("/threshold_low", threshold_low);
        ROS_WARN("effort_limit is: %f", effort_limit);
        ROS_WARN("threshold_high is: %f", threshold_high);
        ROS_WARN("threshold_low is: %f", threshold_low);

        std::cout << cyan << "ROS_IMPEDANCE CONTROLLER: ROBOT NAME IS : " << robot_name << reset << std::endl;
        // Create the PID set service
        set_pids_srv_ = param_node.advertiseService("/set_pids", &Controller::setPidsCallback, this);

        effort_pid_pub = root_nh.advertise<EffortPid>("effort_pid", 1);

        joint_state_ts_pub = root_nh.advertise<sensor_msgs::JointState>("joint_state_ts", 1);
        joint_state_ts_msg.name = joint_names_;
        joint_state_ts_msg.position.resize(joint_names_.size());
        joint_state_ts_msg.velocity.resize(joint_names_.size());
        joint_state_ts_msg.effort.resize(joint_names_.size());

        contact_state_pub = root_nh.advertise<legged_msgs::ContactsStamped>("contact_state", 1);

        // rt publisher (uncomment if you need them)
        // pose_pub_rt_.reset(new realtime_tools::RealtimePublisher<BaseState>(param_node, "/"+robot_name + "/base_state", 1));
        // contact_state_pub_rt_.reset(new realtime_tools::RealtimePublisher<gazebo_msgs::ContactsState>(param_node, "/"+robot_name + "/contacts_state", 1));
        contact_state_pub_rt_.reset(new realtime_tools::RealtimePublisher<legged_msgs::ContactsStamped>(param_node, "/"+robot_name + "/contact_state_rt", 1));

        return true;
    }

    void Controller::starting(const ros::Time &time)
    {
        ROS_DEBUG("Starting Controller");
    }

    bool Controller::setPidsCallback(set_pids::Request &req,
                                     set_pids::Response &res)
    {
        // get params from parameter server
        root_nh_->getParam("/verbose", verbose);
        res.ack = true;

        for (unsigned int i = 0; i < req.data.size(); i++)
        {
            for (unsigned int j = 0; j < joint_names_.size(); j++)
                if (!std::strcmp(joint_names_[j].c_str(), req.data[i].joint_name.c_str()))
                {
                    if (req.data[i].p_value >= 0.0)
                    {
                        joint_p_gain_[j] = req.data[i].p_value;
                        if (verbose)
                            std::cout << "Set P gain for joint " << joint_names_[j] << " to: " << joint_p_gain_[j] << std::endl;
                    }
                    else
                    {
                        ROS_WARN("P value has to be positive");
                        res.ack = false;
                    }

                    if (req.data[i].i_value >= 0.0)
                    {
                        joint_i_gain_[j] = req.data[i].i_value;
                        if (verbose)
                            std::cout << "Set I gain for joint " << joint_names_[j] << " to: " << joint_i_gain_[j] << std::endl;
                    }
                    else
                    {
                        ROS_WARN("I value has to be positive");
                        res.ack = false;
                    }

                    if (req.data[i].d_value >= 0.0)
                    {
                        joint_d_gain_[j] = req.data[i].d_value;
                        if (verbose)
                            std::cout << "Set D gain for joint " << joint_names_[j] << " to: " << joint_d_gain_[j] << std::endl;
                    }
                    else
                    {
                        ROS_WARN("D value has to be positive");
                        res.ack = false;
                    }

                    if (joint_i_gain_[i] == 0)
                    {
                        use_integral_action_[j] = false;
                        T_i[j] = 0;
                    }
                    else
                    {
                        use_integral_action_[j] = true;
                        T_i[j] = joint_p_gain_[j] / joint_i_gain_[j];
                    }

                    if (joint_p_gain_[i] == 0)
                    {
                        T_d[i] = 0;
                    }
                    else
                    {
                        T_d[i] = joint_d_gain_[i] / joint_p_gain_[i];
                    }
                }
        }

        return true;
    }

    void Controller::commandCallback(const sensor_msgs::JointState &msg)
    {

        if (joint_states_.size() == msg.position.size() && joint_states_.size() == msg.velocity.size() && joint_states_.size() == msg.effort.size())
        {
            // des_joint_efforts_(i) = msg.data[i];
            des_joint_positions_ = Eigen::Map<const Eigen::VectorXd>(&msg.position[0], joint_states_.size());
            des_joint_velocities_ = Eigen::Map<const Eigen::VectorXd>(&msg.velocity[0], joint_states_.size());
            des_joint_efforts_ = Eigen::Map<const Eigen::VectorXd>(&msg.effort[0], joint_states_.size());
        }

        else
            ROS_WARN("Wrong dimension!");
    }

    void Controller::process_contact(const gazebo_msgs::ContactsState::ConstPtr& msg, int index)
    {
        // Check if there is at least one state
        if (!msg->states.empty()) {
            // Access the first state
            const auto& state = msg->states[0];

            // Access the total_wrench field of the first state
            const auto& total_wrench = state.total_wrench;

            // Access the force field of the total_wrench
            const auto& force = total_wrench.force;

            // Store the forces in an Eigen::Vector3d
            Eigen::Vector3d forces(force.x, force.y, force.z);

            // Print the forces in one line
            // std::cout << "Forces: " << forces.transpose() << "\n";

            // Calculate the magnitude of the force vector
            double magnitude = forces.norm();


            // Compare the magnitude with the thresholds and update the contact_state
            if (magnitude > threshold_high) {
                contact_state_msg.contacts.at(index) = true;
            } else if (magnitude < threshold_low) {
                contact_state_msg.contacts.at(index) = false;
            }
            ROS_DEBUG("threshold_high is: %f", threshold_high);
            ROS_DEBUG("threshold_low is: %f", threshold_low);


        } else {
            // ROS_WARN("Received ContactsState message with no states");
        }
    }

    void Controller::receive_contact_lf(const gazebo_msgs::ContactsState::ConstPtr& msg)
    {
        process_contact(msg, 0);
    }

    void Controller::receive_contact_rf(const gazebo_msgs::ContactsState::ConstPtr& msg)
    {
        process_contact(msg, 1);
    }
    void Controller::receive_contact_lh(const gazebo_msgs::ContactsState::ConstPtr& msg)
    {
        process_contact(msg, 2);
    }
    void Controller::receive_contact_rh(const gazebo_msgs::ContactsState::ConstPtr& msg)
    {
        process_contact(msg, 3);
    }


    void Controller::baseGroundTruthCB(const nav_msgs::OdometryConstPtr &msg)
    {

        // static tf::TransformBroadcaster br;
        // tf::Transform w_transform_b;

        // orientation of base frame
        /*
            q_base.setX(msg->pose.pose.orientation.x);
            q_base.setY(msg->pose.pose.orientation.y);
            q_base.setZ(msg->pose.pose.orientation.z);
            q_base.setW(msg->pose.pose.orientation.w);
            //position of base frame
            base_pos_w = tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
        */
        // get twist
        /*
            base_twist_w.linear.x= msg->twist.twist.linear.x;
            base_twist_w.linear.y = msg->twist.twist.linear.y;
            base_twist_w.linear.z = msg->twist.twist.linear.z;
            base_twist_w.angular.x = msg->twist.twist.angular.x;
            base_twist_w.angular.y = msg->twist.twist.angular.y;
            base_twist_w.angular.z = msg->twist.twist.angular.z;
        */

        // the vector of the base is in the world frame, so to apply to the base frame I should rotate it to the base frame before
        // tf::Vector3 world_origin_w(-msg->pose.pose.position.x,-msg->pose.pose.position.y,-msg->pose.pose.position.z);
        // tf::Vector3 world_origin_b = tf::quatRotate(q_base.inverse(), world_origin_w);

        // this is the transform from base to world to publish the world transform for rviz
        // w_transform_b.setRotation(q_base.inverse());
        // w_transform_b.setOrigin(world_origin_b);
        // br.sendTransform(tf::StampedTransform(w_transform_b, ros::Time::now(), "/base_link", "/world" ));
    }

    void Controller::update(const ros::Time &time, const ros::Duration &period)
    {
        auto start = std::chrono::high_resolution_clock::now();

        // if task_period is smaller than sim max_step_size (in world file) period it is clamped to that value!!!!!
        // std::cout<<period.toSec()<<std::endl;
        //    std::cout<<"des_joint_efforts_: " << des_joint_efforts_.transpose()<<std::endl;
        //    std::cout<<"des_joint_velocities_: " << des_joint_velocities_.transpose()<<std::endl;
        //    std::cout<<"des_joint_positions_: " << des_joint_positions_.transpose()<<std::endl;
        // Write to the hardware interface
        //(NB this is not the convention of ros but the convention that we define in ros_impedance_controller_XX.yaml!!!!
        // std::cout << "-----------------------------------" << std::endl;

        EffortPid msg;
        msg.name.resize(joint_states_.size());
        msg.effort_pid.resize(joint_states_.size());

        if (discrete_implementation_)
        {
            Ts = period.toSec();
            // check: http://www.diag.uniroma1.it/deluca/automation/Automazione_RegolazionePID.pdf
            for (unsigned int i = 0; i < joint_states_.size(); i++)
            {
                // discrete implementation
                error1_[i] = error_[i];
                error_[i] = des_joint_positions_(i) - joint_states_[i].getPosition();

                proportional_action_[i] = joint_p_gain_[i] * error_[i];
                if (use_integral_action_[i])
                {
                    integral_action_[i] = integral_action_[i] + (joint_p_gain_[i] * Ts / T_i[i]) * error_[i];
                }
                derivative_action_[i] = 1 / (1 + T_d[i] / (N * Ts)) * (T_d[i] / (N * Ts) * derivative_action_[i] + joint_p_gain_[i] * T_d[i] / Ts * (error_[i] - error1_[i]));

                des_joint_efforts_pids_(i) = proportional_action_[i] + integral_action_[i] + derivative_action_[i];

                msg.name[i] = joint_names_[i];
                msg.effort_pid[i] = des_joint_efforts_pids_(i);
                // add PID + FFWD
                joint_states_[i].setCommand(des_joint_efforts_(i) + des_joint_efforts_pids_(i));

                joint_positions_old_[i] = joint_states_[i].getPosition();
            }
        }
        else
        {
            double dt;
            dt = period.toSec();

            des_joint_efforts_pids_ = Controller::control_PD(dt);
            // std::cout << "***** des_joint_efforts_pids_: \n"<< des_joint_efforts_pids_ << std::endl;

            for (unsigned int i = 0; i < joint_states_.size(); i++)
            {

                // measured_joint_position_(i) = joint_states_[i].getPosition();
                // measured_joint_velocity_(i) = joint_states_[i].getVelocity();
                // double joint_pos_error = des_joint_positions_(i) - measured_joint_position_(i);
                // double integral_action = integral_action_old_[i] + joint_i_gain_[i] * joint_pos_error * period.toSec();

                // // std::cout << "***** joint: " << joint_names_[i] << std::endl;
                // // std::cout << "joint des:   " << des_joint_positions_(i) << std::endl;
                // // std::cout << "joint pos:   " << joint_states_[i].getPosition() << std::endl;
                // // std::cout << "wrap:        " << measured_joint_position_(i) << std::endl;

                // // std::cout << "effort pid des:  " << des_joint_efforts_pids_(i) << std::endl;
                // // std::cout << "effort meas: " << joint_states_[i].getEffort() << std::endl;

                // // compute PID
                // des_joint_efforts_pids_(i) = joint_p_gain_[i] * (des_joint_positions_(i) - measured_joint_position_(i)) +
                //                              joint_d_gain_[i] * (des_joint_velocities_(i) - measured_joint_velocity_(i)) +
                //                              integral_action;

                // integral_action_old_[i] = integral_action;

                msg.name[i] = joint_names_[i];
                msg.effort_pid[i] = des_joint_efforts_pids_(i);
                // add PID + FFWD

                des_joint_efforts_sum_(i) = des_joint_efforts_(i) + des_joint_efforts_pids_(i);

                double effort_limit = 5.0; //23.7 max
                if (std::abs(des_joint_efforts_sum_(i)) > effort_limit)
                {
                    des_joint_efforts_sum_(i) = (des_joint_efforts_sum_(i) / std::abs(des_joint_efforts_sum_(i))) * effort_limit;
                }

                // joint_states_[i].setCommand(des_joint_efforts_(i) + des_joint_efforts_pids_(i));
                joint_states_[i].setCommand(des_joint_efforts_sum_(i));
            }
        }

        effort_pid_pub.publish(msg);

        joint_state_ts_msg.header.stamp = ros::Time::now();
        joint_state_ts_pub.publish(joint_state_ts_msg);

        contact_state_msg.header.stamp = ros::Time::now();
        contact_state_pub.publish(contact_state_msg);

        if (contact_state_pub_rt_->trylock()){
            contact_state_pub_rt_->msg_ = contact_state_msg;
            contact_state_pub_rt_->unlockAndPublish();
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_ms = end - start;
        // std::cout << elapsed_ms.count() * 1000 << std::endl;
        ROS_DEBUG("dt: %f", elapsed_ms.count() * 1000);
        if (elapsed_ms.count() >= period.toSec())
        {
            ROS_ERROR("dt!!!! :%f", elapsed_ms.count() * 1000);
            std::cout << "!!!!!!" << elapsed_ms.count() * 1000 << std::endl;
        }
    }

    Eigen::MatrixXd Controller::fk_leg(Eigen::MatrixXd q)
    {

        Eigen::VectorXd L(3);
        L << 0.08, 0.213, 0.213;
        Eigen::MatrixXd p = Eigen::MatrixXd::Zero(3, 4);

        for (unsigned int i = 0; i < 4; ++i)
        {
            if (i == 0 || i == 2)
            {

                p(0, i) = L(2) * std::sin(q(1, i) + q(2, i)) + L[1] * std::sin(q(1, i));
                p(1, i) = L(1) * std::cos(q(1, i)) * std::sin(q(0, i)) - L(0) * std::cos(q(0, i)) +
                          L(2) * std::cos(q(1, i)) * std::cos(q(2, i)) * std::sin(q(0, i)) -
                          L(2) * std::sin(q(0, i)) * std::sin(q(1, i)) * std::sin(q(2, i));
                p(2, i) = L(2) * std::cos(q(0, i)) * std::sin(q(1, i)) * std::sin(q(2, i)) -
                          L(1) * std::cos(q(0, i)) * std::cos(q(1, i)) -
                          L(2) * std::cos(q(0, i)) * std::cos(q(1, i)) * std::cos(q(2, i)) -
                          L(0) * std::sin(q(0, i));
            }
            else
            {

                p(0, i) = L(2) * std::sin(q(1, i) + q(2, i)) + L(1) * std::sin(q(1, i));
                p(1, i) = L(0) * std::cos(q(0, i)) + L(1) * std::cos(q(1, i)) * std::sin(q(0, i)) +
                          L(2) * std::cos(q(1, i)) * std::cos(q(2, i)) * std::sin(q(0, i)) -
                          L(2) * std::sin(q(0, i)) * std::sin(q(1, i)) * std::sin(q(2, i));
                p(2, i) = L(0) * std::sin(q(0, i)) - L(1) * std::cos(q(0, i)) * std::cos(q(1, i)) -
                          L(2) * std::cos(q(0, i)) * std::cos(q(1, i)) * std::cos(q(2, i)) +
                          L(2) * std::cos(q(0, i)) * std::sin(q(1, i)) * std::sin(q(2, i));
            }
        }

        return p;
    }

    Eigen::MatrixXd Controller::Mq_R(Eigen::MatrixXd q)
    {
        Eigen::VectorXd L(3);
        L << 0.08, 0.213, 0.213;

        Eigen::VectorXd Lm(3);
        Lm << L(0) / 2, L(1) / 2, L(2) / 2;

        Eigen::VectorXd m(3);
        m << 0.591, 1.009, 0.28;

        Eigen::VectorXd Im1(3);
        Im1 << 0.000374268192, 0.000635923669, 0.000457647394;

        Eigen::VectorXd Im2(3);
        Im2 << 0.00602684, 0.00579594, 0.00116087;

        Eigen::VectorXd Im3(3);
        Im3 << 0.00479444, 0.0047988, 0.00016945;

        Eigen::MatrixXd D = Eigen::MatrixXd::Zero(3, 3);

        double cos_q1q2, cos_q1;
        cos_q1q2 = std::cos(q(1) + q(2));
        cos_q1 = std::cos(q(1));

        D(0, 0) = (Im2(0) + Im1(1) + Im3(0) + L(0) * L(0) * m(1) + L(0) * L(0) * m(2) +
                   Lm(0) * Lm(0) * m(0) - Im3(0) * cos_q1q2 * cos_q1q2 +
                   Im3(1) * cos_q1q2 * cos_q1q2 - Im2(0) * cos_q1 * cos_q1 +
                   Im2(1) * cos_q1 * cos_q1 + Lm(2) * Lm(2) * m(2) * cos_q1q2 * cos_q1q2 +
                   L(1) * L(1) * m(2) * cos_q1 * cos_q1 + Lm(1) * Lm(1) * m(1) * cos_q1 * cos_q1 +
                   L(1) * Lm(2) * m(2) * std::cos(q(2)) + L(1) * Lm(2) * m(2) * std::cos(2 * q(1) + q(2)));
        D(0, 1) = (-L(0) * m(2) * (Lm(2) * std::sin(q(1) + q(2)) + L(1) * std::sin(q(1))) -
                   L(0) * Lm(1) * m(1) * std::sin(q(1)));
        D(0, 2) = -L(0) * Lm(2) * m(2) * std::sin(q(1) + q(2));
        D(1, 0) = D(0, 1);
        D(1, 1) = m(2) * L(1) * L(1) + 2 * m(2) * std::cos(q(2)) * L(1) * Lm(2) + m(1) * Lm(1) * Lm(1) + m(2) * Lm(2) * Lm(2) + Im2(2) + Im3(2);
        D(1, 2) = m(2) * Lm(2) * Lm(2) + L(1) * m(2) * std::cos(q(2)) * Lm(2) + Im3(2);
        D(2, 0) = D(0, 2);
        D(2, 1) = D(1, 2);
        D(2, 2) = (m(2) * Lm(2) * Lm(2) + Im3(2));
        // std::cout << "Mq_R" << D << std::endl;

        return D;
    }

    Eigen::MatrixXd Controller::Mq_L(Eigen::MatrixXd q)
    {
        Eigen::VectorXd L(3);
        L << 0.08, 0.213, 0.213;

        Eigen::VectorXd Lm(3);
        Lm << L(0) / 2, L(1) / 2, L(2) / 2;

        Eigen::VectorXd m(3);
        m << 0.591, 1.009, 0.28;

        Eigen::VectorXd Im1(3);
        Im1 << 0.000374268192, 0.000635923669, 0.000457647394;

        Eigen::VectorXd Im2(3);
        Im2 << 0.00602684, 0.00579594, 0.00116087;

        Eigen::VectorXd Im3(3);
        Im3 << 0.00479444, 0.0047988, 0.00016945;

        Eigen::MatrixXd D = Eigen::MatrixXd::Zero(3, 3);

        double cos_q1q2, cos_q1;

        cos_q1q2 = std::cos(q(1) + q(2));
        cos_q1 = std::cos(q(1));

        D(0, 0) = (Im1(0) + Im2(0) + Im3(0) + L(0) * L(0) * m(1) + L(0) * L(0) * m(2) + Lm(0) * Lm(0) * m(0) - Im3(0) * cos_q1q2 * cos_q1q2 + Im3(1) * cos_q1q2 * cos_q1q2 - Im1(0) * cos_q1 * cos_q1 - Im2(0) * cos_q1 * cos_q1 + Im1(1) * cos_q1 * cos_q1 + Im2(1) * cos_q1 * cos_q1 + Lm(2) * Lm(2) * m(2) * cos_q1q2 * cos_q1q2 + L(1) * L(1) * m(2) * cos_q1 * cos_q1 + Lm(1) * Lm(1) * m(1) * cos_q1 * cos_q1 + L(1) * Lm(2) * m(2) * std::cos(q(2)) + L(1) * Lm(2) * m(2) * std::cos(2 * q(1) + q(2)));
        D(0, 1) = L(0) * m(2) * (Lm(2) * std::sin(q(1) + q(2)) + L(1) * std::sin(q(1))) + L(0) * Lm(1) * m(1) * std::sin(q(1));
        D(0, 2) = L(0) * Lm(2) * m(2) * std::sin(q(1) + q(2));
        D(1, 0) = D(0, 1);
        D(1, 1) = m(2) * L(1) * L(1) + 2 * m(2) * std::cos(q(2)) * L(1) * Lm(2) + m(1) * Lm(1) * Lm(1) + m(2) * Lm(2) * Lm(2) + Im2(2) + Im3(2);
        D(1, 2) = m(2) * Lm(2) * Lm(2) + L(1) * m(2) * std::cos(q(2)) * Lm(2) + Im3(2);
        D(2, 0) = D(0, 2);
        D(2, 1) = D(1, 2);
        D(2, 2) = (m(2) * Lm(2) * Lm(2) + Im3(2));
        // std::cout << "Mq_L" << D << std::endl;
        return D;
    }

    Eigen::MatrixXd Controller::J_leg_R(Eigen::MatrixXd q)
    {
        Eigen::VectorXd L(3);
        L << 0.08, 0.213, 0.213;

        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, 3);

        J(0, 0) = 0;
        J(0, 1) = L(2) * std::cos(q(1) + q(2)) + L(1) * std::cos(q(1));
        J(0, 2) = L(2) * std::cos(q(1) + q(2));

        J(1, 0) = (L(1) * std::cos(q(0)) * std::cos(q(1)) + L(2) * std::cos(q(0)) * std::cos(q(1)) * std::cos(q(2)) - L(2) * std::cos(q(0)) * std::sin(q(1)) * std::sin(q(2))) + L(0) * std::sin(q(0));
        J(1, 1) = -std::sin(q(0)) * (L(2) * std::sin(q(1) + q(2)) + L(1) * std::sin(q(1)));
        J(1, 2) = -L(2) * std::sin(q(1) + q(2)) * std::sin(q(0));

        J(2, 0) = (L(1) * std::cos(q(1)) * std::sin(q(0)) + L(2) * std::cos(q(1)) * std::cos(q(2)) * std::sin(q(0)) - L(2) * std::sin(q(0)) * std::sin(q(1)) * std::sin(q(2))) - L(0) * std::cos(q(0));

        J(2, 1) = std::cos(q(0)) * (L(2) * std::sin(q(1) + q(2)) + L(1) * std::sin(q(1)));
        J(2, 2) = L(2) * std::sin(q(1) + q(2)) * std::cos(q(0));

        return J;
    }

    Eigen::MatrixXd Controller::J_leg_L(Eigen::MatrixXd q)
    {

        Eigen::VectorXd L(3);
        L << 0.08, 0.213, 0.213;

        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, 3);

        J(0, 0) = 0;
        J(0, 1) = L(2) * std::cos(q(1) + q(2)) + L(1) * std::cos(q(1));
        J(0, 2) = L(2) * std::cos(q(1) + q(2));

        J(1, 0) = (L(1) * std::cos(q(0)) * std::cos(q(1)) + L(2) * std::cos(q(0)) * std::cos(q(1)) * std::cos(q(2)) - L(2) * std::cos(q(0)) * std::sin(q(1)) * std::sin(q(2))) - L(0) * std::sin(q(0));
        J(1, 1) = -std::sin(q(0)) * (L(2) * std::sin(q(1) + q(2)) + L(1) * std::sin(q(1)));
        J(1, 2) = -L(2) * std::sin(q(1) + q(2)) * std::sin(q(0));

        J(2, 0) = (L(1) * std::cos(q(1)) * std::sin(q(0)) + L(2) * std::cos(q(1)) * std::cos(q(2)) * std::sin(q(0)) - L(2) * std::sin(q(0)) * std::sin(q(1)) * std::sin(q(2))) + L(0) * std::cos(q(0));
        J(2, 1) = std::cos(q(0)) * (L(2) * std::sin(q(1) + q(2)) + L(1) * std::sin(q(1)));
        J(2, 2) = L(2) * std::sin(q(1) + q(2)) * std::cos(q(0));

        return J;
    }

    Eigen::MatrixXd Controller::diff_fk_leg(Eigen::MatrixXd q, Eigen::MatrixXd dq)
    {

        Eigen::MatrixXd dp = Eigen::MatrixXd::Zero(3, 4);

        for (unsigned int j = 0; j < 4; ++j)
        {
            if (j == 0 || j == 2)
            {
                Eigen::VectorXd q_ax(3);
                q_ax(0) = q(0, j);
                q_ax(1) = q(1, j);
                q_ax(2) = q(2, j);

                Eigen::VectorXd dq_ax(3);
                dq_ax(0) = dq(0, j);
                dq_ax(1) = dq(1, j);
                dq_ax(2) = dq(2, j);

                Eigen::VectorXd dp_ax(3);
                dp_ax = Controller::J_leg_R(q_ax) * dq_ax;

                dp(0, j) = dp_ax(0);
                dp(1, j) = dp_ax(1);
                dp(2, j) = dp_ax(2);
            }
            else
            {
                Eigen::VectorXd q_ax(3);
                q_ax(0) = q(0, j);
                q_ax(1) = q(1, j);
                q_ax(2) = q(2, j);

                Eigen::VectorXd dq_ax(3);
                dq_ax(0) = dq(0, j);
                dq_ax(1) = dq(1, j);
                dq_ax(2) = dq(2, j);

                Eigen::VectorXd dp_ax(3);
                dp_ax = Controller::J_leg_L(q_ax) * dq_ax;

                dp(0, j) = dp_ax(0);
                dp(1, j) = dp_ax(1);
                dp(2, j) = dp_ax(2);
            }
        }

        return dp;
    }

    Eigen::VectorXd Controller::control_PD(double period)
    {
        Eigen::MatrixXd q = Eigen::MatrixXd::Zero(3, 4);
        Eigen::MatrixXd dq = Eigen::MatrixXd::Zero(3, 4);
        Eigen::MatrixXd p = Eigen::MatrixXd::Zero(3, 4);
        Eigen::MatrixXd dp = Eigen::MatrixXd::Zero(3, 4);
        Eigen::MatrixXd des_p = Eigen::MatrixXd::Zero(3, 4);
        Eigen::MatrixXd des_dp = Eigen::MatrixXd::Zero(3, 4);
        Eigen::MatrixXd ax = Eigen::MatrixXd::Zero(3, 4);
        Eigen::VectorXd ax_12 = Eigen::VectorXd::Zero(12);
        Eigen::MatrixXd p_gain = Eigen::MatrixXd::Zero(3, 4);
        Eigen::MatrixXd d_gain = Eigen::MatrixXd::Zero(3, 4);
        Eigen::MatrixXd i_gain = Eigen::MatrixXd::Zero(3, 4);
        Eigen::MatrixXd J_FR = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd J_FL = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd J_RR = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd J_RL = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd J_FR_inv = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd J_FL_inv = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd J_RL_inv = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd J_RR_inv = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd Mq_FR = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd Mq_FL = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd Mq_RR = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd Mq_RL = Eigen::MatrixXd::Zero(3, 3);
        Eigen::VectorXd q_ax(3);
        Eigen::VectorXd dq_ax(3);
        Eigen::VectorXd dp_ax(3);
        Eigen::VectorXd p_ax(3);
        Eigen::MatrixXd integral_old = Eigen::MatrixXd::Zero(3, 4);
        Eigen::VectorXd integral_ax_old(3);
        Eigen::VectorXd integral_ax(3);

        const int gdp2L[12] = {6, 7, 8, 0, 1, 2, 9, 10, 11, 3, 4, 5};

        for (unsigned int i = 0; i < 4; i++)
        {
            q(0, i) = joint_states_[gdp2L[i * 3]].getPosition();
            q(1, i) = -1 * joint_states_[gdp2L[i * 3 + 1]].getPosition();
            q(2, i) = -1 * joint_states_[gdp2L[i * 3 + 2]].getPosition();

            dq(0, i) = joint_states_[gdp2L[i * 3]].getVelocity();
            dq(1, i) = -1 * joint_states_[gdp2L[i * 3 + 1]].getVelocity();
            dq(2, i) = -1 * joint_states_[gdp2L[i * 3 + 2]].getVelocity();

            des_p(0, i) = des_joint_positions_(gdp2L[i * 3]);
            des_p(1, i) = des_joint_positions_(gdp2L[i * 3 + 1]);
            des_p(2, i) = des_joint_positions_(gdp2L[i * 3 + 2]);

            des_dp(0, i) = des_joint_velocities_(gdp2L[i * 3]);
            des_dp(1, i) = des_joint_velocities_(gdp2L[i * 3 + 1]);
            des_dp(2, i) = des_joint_velocities_(gdp2L[i * 3 + 2]);

            p_gain(0, i) = joint_p_gain_[gdp2L[i * 3]];
            p_gain(1, i) = joint_p_gain_[gdp2L[i * 3 + 1]];
            p_gain(2, i) = joint_p_gain_[gdp2L[i * 3 + 2]];

            d_gain(0, i) = joint_d_gain_[gdp2L[i * 3]];
            d_gain(1, i) = joint_d_gain_[gdp2L[i * 3 + 1]];
            d_gain(2, i) = joint_d_gain_[gdp2L[i * 3 + 2]];

            i_gain(0, i) = joint_i_gain_[gdp2L[i * 3]];
            i_gain(1, i) = joint_i_gain_[gdp2L[i * 3 + 1]];
            i_gain(2, i) = joint_i_gain_[gdp2L[i * 3 + 2]];

            integral_old(0, i) = integral_action_old_leg[gdp2L[i * 3]];
            integral_old(1, i) = integral_action_old_leg[gdp2L[i * 3 + 1]];
            integral_old(2, i) = integral_action_old_leg[gdp2L[i * 3 + 2]];

            q_ax << q(0, i), q(1, i), q(2, i);
            dq_ax << dq(0, i), dq(1, i), dq(2, i);

            if (i == 0)
            {
                J_FR = Controller::J_leg_R(q_ax);
                J_FR_inv = J_FR.transpose();
                // Mq_FR = Controller::Mq_R(q_ax);

                dp_ax = J_FR * dq_ax;

                dp(0, i) = dp_ax(0);
                dp(1, i) = dp_ax(1);
                dp(2, i) = dp_ax(2);
            }
            else if (i == 2)
            {
                J_RR = Controller::J_leg_R(q_ax);
                J_RR_inv = J_RR.transpose();
                // Mq_RR = Controller::Mq_R(q_ax);

                dp_ax = J_RR * dq_ax;

                dp(0, i) = dp_ax(0);
                dp(1, i) = dp_ax(1);
                dp(2, i) = dp_ax(2);
            }
            else if (i == 1)
            {
                J_FL = Controller::J_leg_L(q_ax);
                J_FL_inv = J_FL.transpose();
                dp_ax = J_FL * dq_ax;
                // Mq_FL = Controller::Mq_L(q_ax);

                // std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
                // std::cout << "joint vel" << i << dq(0, i) << dq(1, i) << dq(2, i) << std::endl;
                // std::cout << "joint vel" << i << dq(0, i) << dq(1, i) << dq(2, i) << std::endl;
                // std::cout << "ee    vel 1:" << dp_ax.transpose() << std::endl;
                dp(0, i) = dp_ax(0);
                dp(1, i) = dp_ax(1);
                dp(2, i) = dp_ax(2);
            }
            else if (i == 3)
            {
                J_RL = Controller::J_leg_L(q_ax);
                J_RL_inv = J_RL.transpose();
                // Mq_RL = Controller::Mq_L(q_ax);

                dp_ax = J_RL * dq_ax;

                dp(0, i) = dp_ax(0);
                dp(1, i) = dp_ax(1);
                dp(2, i) = dp_ax(2);
            }
        }

        // std::cout << "q :\n" << q << std::endl;
        // std::cout << "dq :\n" << dq << std::endl;

        p = Controller::fk_leg(q);

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                if (std::abs(dp(i, j)) < 1e-2)
                {
                    dp(i, j) = 0;
                }
            }
        }

        // dp = Controller::diff_fk_leg(q, dq);

        for (unsigned int j = 0; j < 4; ++j)
        {
            if (j == 0 || j == 2)
            {

                q_ax << q(0, j), q(1, j), q(2, j);

                Eigen::VectorXd des_p_ax(3);
                des_p_ax << des_p(0, j), des_p(1, j), des_p(2, j);

                Eigen::VectorXd des_dp_ax(3);
                des_dp_ax << des_dp(0, j), des_dp(1, j), des_dp(2, j);

                integral_ax_old << integral_old(0, j), integral_old(1, j), integral_old(2, j);

                p_ax << p(0, j), p(1, j), p(2, j);

                dp_ax << dp(0, j), dp(1, j), dp(2, j);

                Eigen::MatrixXd p_gain_ax(3, 3);
                p_gain_ax = Eigen::MatrixXd::Zero(3, 3);
                p_gain_ax(0, 0) = p_gain(0, j);
                p_gain_ax(1, 1) = p_gain(1, j);
                p_gain_ax(2, 2) = p_gain(2, j);

                Eigen::MatrixXd d_gain_ax(3, 3);
                d_gain_ax = Eigen::MatrixXd::Zero(3, 3);
                d_gain_ax(0, 0) = d_gain(0, j);
                d_gain_ax(1, 1) = d_gain(1, j);
                d_gain_ax(2, 2) = d_gain(2, j);

                Eigen::MatrixXd i_gain_ax(3, 3);
                i_gain_ax = Eigen::MatrixXd::Zero(3, 3);
                i_gain_ax(0, 0) = i_gain(0, j);
                i_gain_ax(1, 1) = i_gain(1, j);
                i_gain_ax(2, 2) = i_gain(2, j);

                Eigen::MatrixXd J_leg_inv(3, 3);
                if (j == 0)
                    J_leg_inv = J_FR_inv;
                // J_leg_inv = Mq_FR * J_FR_inv;
                else
                {
                    J_leg_inv = J_RR_inv;
                    // J_leg_inv = Mq_RR * J_RR_inv;
                }

                Eigen::VectorXd p_e(3);
                p_e = (des_p_ax - p_ax);
                for (int i = 0; i < p_e.size(); ++i)
                {
                    if (std::abs(p_e(i)) < 1e-3)
                    {
                        p_e(i) = 0;
                    }
                }

                Eigen::VectorXd pd_e(3);
                pd_e = (des_dp_ax - dp_ax);
                for (int i = 0; i < pd_e.size(); ++i)
                {
                    if (std::abs(pd_e(i)) < 1e-2)
                    {
                        pd_e(i) = 0;
                    }
                }

                
                integral_ax = integral_ax_old + i_gain_ax * p_e * period;

                Eigen::VectorXd ax_ax(3);
                ax_ax = J_leg_inv * (p_gain_ax * p_e + d_gain_ax * pd_e + integral_ax);
                // ax_ax = J_leg_inv * (p_gain_ax * p_e + d_gain_ax * pd_e);

                ax(0, j) = ax_ax(0);
                ax(1, j) = ax_ax(1);
                ax(2, j) = ax_ax(2);

                integral_old(0, j) = integral_ax(0);
                integral_old(1, j) = integral_ax(1);
                integral_old(2, j) = integral_ax(2);
            }
            else
            {
                
                q_ax << q(0, j), q(1, j), q(2, j);

                Eigen::VectorXd des_p_ax(3);
                des_p_ax << des_p(0, j), des_p(1, j), des_p(2, j);

                Eigen::VectorXd des_dp_ax(3);
                des_dp_ax << des_dp(0, j), des_dp(1, j), des_dp(2, j);

                
                 integral_ax_old << integral_old(0, j), integral_old(1, j), integral_old(2, j);


                p_ax << p(0, j), p(1, j), p(2, j);

                dp_ax << dp(0, j), dp(1, j), dp(2, j);

                Eigen::MatrixXd p_gain_ax(3, 3);
                p_gain_ax = Eigen::MatrixXd::Zero(3, 3);
                p_gain_ax(0, 0) = p_gain(0, j);
                p_gain_ax(1, 1) = p_gain(1, j);
                p_gain_ax(2, 2) = p_gain(2, j);

                Eigen::MatrixXd d_gain_ax(3, 3);
                d_gain_ax = Eigen::MatrixXd::Zero(3, 3);
                d_gain_ax(0, 0) = d_gain(0, j);
                d_gain_ax(1, 1) = d_gain(1, j);
                d_gain_ax(2, 2) = d_gain(2, j);

                Eigen::MatrixXd i_gain_ax(3, 3);
                i_gain_ax = Eigen::MatrixXd::Zero(3, 3);
                i_gain_ax(0, 0) = i_gain(0, j);
                i_gain_ax(1, 1) = i_gain(1, j);
                i_gain_ax(2, 2) = i_gain(2, j);

                Eigen::MatrixXd J_leg_inv(3, 3);

                if (j == 1)
                    J_leg_inv = J_FL_inv;
                // J_leg_inv = Mq_FL * J_FL_inv;
                else
                {
                    J_leg_inv = J_RL_inv;
                    // J_leg_inv = Mq_RL * J_RL_inv;
                }

                Eigen::VectorXd p_e(3);
                p_e = (des_p_ax - p_ax);
                for (int i = 0; i < p_e.size(); ++i)
                {
                    if (std::abs(p_e(i)) < 1e-3)
                    {
                        p_e(i) = 0;
                    }
                }

                Eigen::VectorXd pd_e(3);
                pd_e = (des_dp_ax - dp_ax);
                for (int i = 0; i < pd_e.size(); ++i)
                {
                    if (std::abs(pd_e(i)) < 1e-2)
                    {
                        pd_e(i) = 0;
                    }
                }

                integral_ax = integral_ax_old + i_gain_ax * p_e * period;

                Eigen::VectorXd ax_ax(3);
                ax_ax = J_leg_inv * (p_gain_ax * p_e + d_gain_ax * pd_e + integral_ax);
                // ax_ax = J_leg_inv * (p_gain_ax * p_e + d_gain_ax * pd_e);

                ax(0, j) = ax_ax(0);
                ax(1, j) = ax_ax(1);
                ax(2, j) = ax_ax(2);

                integral_old(0, j) = integral_ax(0);
                integral_old(1, j) = integral_ax(1);
                integral_old(2, j) = integral_ax(2);
            }
        }

        for (unsigned int i = 0; i < 4; i++)
        {
            ax_12(gdp2L[i * 3]) = ax(0, i);
            ax_12(gdp2L[i * 3 + 1]) = -ax(1, i);
            ax_12(gdp2L[i * 3 + 2]) = -ax(2, i);

            integral_action_old_leg[gdp2L[i * 3]] = integral_old(0, i);
            integral_action_old_leg[gdp2L[i * 3 + 1]] = integral_old(1, i);
            integral_action_old_leg[gdp2L[i * 3 + 2]] = integral_old(2, i);


        }

        return ax_12;
    }

    void Controller::stopping(const ros::Time &time)
    {
        ROS_DEBUG("Stopping Controller");
    }

} // namespace
