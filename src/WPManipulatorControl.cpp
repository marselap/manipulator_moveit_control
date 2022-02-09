#include <manipulator_moveit_control/WPManipulatorControl.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/package.h>
#include <std_srvs/SetBool.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "WP_manipulator_control_node");

    ros::NodeHandle private_node_handle_("~");
    ros::NodeHandle n;
    
    int rate;
    bool simulation_flag;
    std::string robot_model_name, joint_group_name;
    std::string parameters_file;
    geometry_msgs::PoseStamped end_effector_pose;
    Eigen::Affine3d end_effector_transform;
    std_msgs::Float64MultiArray transformation_msg;

    transformation_msg.data.resize(16);

    std::string path = ros::package::getPath("manipulator_moveit_control");

    private_node_handle_.param("rate", rate, int(30));
    private_node_handle_.param("robot_name", robot_model_name, std::string("robot_description"));
    private_node_handle_.param("joint_group_name", joint_group_name, std::string("panda_arm"));
    private_node_handle_.param("parameters_file", parameters_file, std::string("/config/wp_manipulator_dh_parameters.yaml"));

    std::string end_effector_link;
    private_node_handle_.param("end_effector_link", end_effector_link, std::string("panda_link8"));
    // private_node_handle_.param("end_effector_link", end_effector_link, std::string("panda_EE"));
    // private_node_handle_.param("end_effector_link", end_effector_link, std::string("panda_K"));



    ros::Publisher manipulator_position_pub_ros_ = n.advertise<geometry_msgs::PoseStamped>("end_effector/pose", 1);
    ros::Publisher transformation_pub_ = n.advertise<std_msgs::Float64MultiArray>("transformation/world_end_effector", 1);

    // ros::Publisher position_reached_pub_ = n.advertise<std_msgs::Bool>("position_reached", 1);
    double close_enough_dist = 0.01;

    std::cout << "Waiting for position reached service" << std::endl;
    bool response = ros::service::waitForService("/position_reached", ros::Duration(5));
    std::cout << "Server up: " << response << std::endl;
    
    ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("/position_reached");
    
    std_srvs::SetBool pos_reached_srv;
    pos_reached_srv.request.data = true;

    ManipulatorControl wp_control;
    
    ros::Rate loop_rate(rate);

    wp_control.setManipulatorName(robot_model_name, joint_group_name);
    wp_control.setEndEffectorLink(end_effector_link);
    wp_control.LoadParameters(path+parameters_file);
    wp_control.init(&n);

    while(ros::ok() && !wp_control.isStarted())
    {
        ros::spinOnce();
        printf("Waiting for measurements.\n");
        ros::Duration(0.5).sleep();
    }

    bool no_ik = false;

    while(ros::ok())
    {
        ros::spinOnce();
        if (wp_control.getControlMode()==1)
        {
            if (wp_control.traj_rec_flag_) 
            {
                // std::cout << "Current control mode " << wp_control.getControlMode() << std::endl;
                if (wp_control.state_machine_state_ == "plan_grasp")
                {
                    std::cout << "New pose ref received: \n" << wp_control.getEndEffectorReferencePosition() << std::endl;
                    std::cout << "New pose ref received: \n" << wp_control.getEndEffectorPosition() << std::endl;
                }


                bool inverted = wp_control.invertQuat();
                // std::cout << (inverted ? "New ref had to be >>>inverted<<<!" : "Did not invert orientation.") << std::endl;

                no_ik = false;
                auto q = wp_control.calculateJointSetpoints(wp_control.getEndEffectorReferencePosition());

                auto q_ref = wp_control.getJointSetpoints();
                auto q_meas = wp_control.getJointMeasurements();
                double diff = 0.0;
                for (int i = 0; i < 7; i++) 
                    diff += (q_meas[i] - q[i]) * (q_meas[i] - q[i]);
                if (diff < close_enough_dist)
                {
                    no_ik = true;
                }
                else
                {
                    no_ik = false;
                }
                

                wp_control.setJointSetpoints(q);
                // wp_control.publishJointSetpoints(q);
                wp_control.traj_rec_flag_ = false;
            }
        }

        // else
        // {
        //     wp_control.publishJointSetpoints(wp_control.getJointSetpoints());
        // }

        wp_control.publishJointSetpoints(wp_control.getJointSetpoints());

        if (!wp_control.setpoint_reached_flag_)
        {
            auto q_ref = wp_control.getJointSetpoints();
            auto q_meas = wp_control.getJointMeasurements();


            if (no_ik)
            {
                wp_control.setpoint_reached_flag_ = false;
                bool resp = client.call(pos_reached_srv);
                no_ik = false;
            }
            else
            {
                double diff = 0.0;
                for (int i = 0; i < 7; i++) 
                {
                    diff += (q_meas[i] - q_ref[i]) * (q_meas[i] - q_ref[i]);
                }

                if (diff < close_enough_dist)
                {
                    if (wp_control.state_machine_state_ != "servo")
                        std::cout << "Reached setpoint." << std::endl << std::endl << std::endl;

                    wp_control.setpoint_reached_flag_ = true;
                    
                    if (wp_control.state_machine_state_ != "servo")
                        bool resp = client.call(pos_reached_srv);
                } 
            }
        }

        loop_rate.sleep();
    }

    return 0;
}