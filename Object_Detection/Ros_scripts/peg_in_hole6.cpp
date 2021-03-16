
// *** include ***
#include <cmath>
#include <functional>
#include <ros/ros.h>
// services
#include <iiwa_ros/service/control_mode.hpp>
#include <iiwa_ros/service/path_parameters.hpp>
#include <iiwa_ros/service/path_parameters_lin.hpp>
#include <iiwa_ros/service/time_to_destination.hpp>
// commands
#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/command/cartesian_pose_linear.hpp>
#include <iiwa_ros/command/joint_position.hpp>
// states
#include <iiwa_ros/state/cartesian_wrench.hpp>
#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/state/joint_velocity.hpp>
#include <iiwa_ros/state/joint_position.hpp>
// conversions functions hpp_file
#include <iiwa_ros/conversions.hpp>
// messages
#include <iiwa_msgs/DOF.h>
#include <iiwa_msgs/CartesianQuantity.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

geometry_msgs::PoseStamped desired_pose;

void waitForMotion(iiwa_ros::service::TimeToDestinationService& time_2_dist, double time_out = 2.0)
{
    double time = time_2_dist.getTimeToDestination();
    ros::Time start_wait = ros::Time::now();
    while (time < 0.0 && (ros::Time::now() - start_wait) < ros::Duration(time_out)) {
        ros::Duration(0.5).sleep();
        time = time_2_dist.getTimeToDestination();
    }
    if (time > 0.0) {
        // ROS_INFO_STREAM("Sleeping for " << time << " seconds.");
        ros::Duration(time).sleep();
    } 
}


bool atPosition(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped command)
{
	double e[] = {0.0, 0.0, 0.0};
	double max_e; 
	e[0] = fabs(current.pose.position.x-command.pose.position.x);
	e[1] = fabs(current.pose.position.y-command.pose.position.y);
	e[2] = fabs(current.pose.position.z-command.pose.position.z);
	// e[3] = fabs(current.pose.orientation.x-command.pose.orientation.x);
	// e[4] = fabs(current.pose.orientation.y-command.pose.orientation.y);
	// e[5] = fabs(current.pose.orientation.z-command.pose.orientation.z);
	// e[6] = fabs(current.pose.orientation.w-command.pose.orientation.w);
	max_e = *std::max_element(e,e+3);
	if (max_e<0.0005)
	    return true;
	else 
	    return false;
}

// bool searchForHole(iiwa_ros::command::JointPosition& jp_command1, iiwa_ros::state::JointPosition& jp_state1, geometry_msgs::PoseStamped& desired_pose)
// {
//     double j7_limit = 170.0*M_PI/180.0; // joint7 limits are +-175 degree
//     auto JP = jp_state1.getPosition();
//     if (desired_pose.pose.position.x == 0.0 && desired_pose.pose.position.y == 0.0)
//     {   
//         if (JP.position.a7 > j7_limit)
//         {
//             JP.position.a7 += 10.0*M_PI/180.0;
//         }           
//         else
//         {
//             JP.position.a7 -= 10.0*M_PI/180.0;
//         }
//     }
// }

void pose_callback(geometry_msgs::PoseStamped msg)
{
    desired_pose.pose.position.x = msg.pose.position.x;
    desired_pose.pose.position.y = msg.pose.position.y;
    desired_pose.pose.position.z = msg.pose.position.z;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_control");
    ros::NodeHandle nh;

    // ros spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // Wait a bit, so that you can be sure the subscribers are connected.
    ros::Duration(0.5).sleep();
    // *** decleare ***
    // services
    iiwa_ros::service::ControlModeService control_mode;
    iiwa_ros::service::PathParametersService j_vel;
    iiwa_ros::service::PathParametersLinService c_vel;
    iiwa_ros::service::TimeToDestinationService time_to_dist;
    // commands
    iiwa_ros::command::CartesianPose cp_command;
    iiwa_ros::command::CartesianPoseLinear cpl_command;
    iiwa_ros::command::JointPosition jp_command;
    // states
    iiwa_ros::state::CartesianWrench cw_state;
    iiwa_ros::state::CartesianPose cp_state;
    iiwa_ros::state::JointVelocity jv_state;
    iiwa_ros::state::JointPosition jp_state;
    // cartesian position msg
    geometry_msgs::PoseStamped new_pose;
    // cartesian velocity msg
    geometry_msgs::Twist cartesian_velocity;
    double vel = 0.05;
    double Jvel = 0.20;
    cartesian_velocity.linear.x = vel;
    cartesian_velocity.linear.y = vel;
    cartesian_velocity.linear.z = vel;
    cartesian_velocity.angular.x = vel;
    cartesian_velocity.angular.y = vel;
    cartesian_velocity.angular.z = vel;
    // gripper
    std_msgs::Bool gripper;
    bool open = true;
    bool close = false;
    // gripper_command: ros publisher
    ros::Publisher gripper_command = nh.advertise<std_msgs::Bool>("/iiwa/command/GripperCommand", 1000);
    ros::Subscriber sub = nh.subscribe("desired_pose", 1000, pose_callback);
    ros::spinOnce();
    // *** force control parameters ***
    // desired force
    double des_f = 6.0;
    // stiffness
    double stiff = 2.0;
    // max path deviation, max cartesian velocity, max control force:
    iiwa_msgs::CartesianQuantity max_delta = iiwa_ros::conversions::CartesianQuantityFromFloat(1000.0, 1000.0, 500.0, 2.0, 2.0, 2.0);
    iiwa_msgs::CartesianQuantity max_vel = iiwa_ros::conversions::CartesianQuantityFromFloat(10.0, 10.0, 10.0, 0.1, 0.1, 0.1);
    iiwa_msgs::CartesianQuantity max_F = iiwa_ros::conversions::CartesianQuantityFromFloat(15.0, 15.0, 20.0, 1.5, 1.5, 1.5);
    bool force_stop = false;
    // *** impedance ***
    // stiffness and damping
    iiwa_msgs::CartesianQuantity stiffness = iiwa_ros::conversions::CartesianQuantityFromFloat(5000, 5000, 1000.0, 300, 300, 300);
    iiwa_msgs::CartesianQuantity damping = iiwa_ros::conversions::CartesianQuantityFromFloat(0.7,0.7,0.7,0.7,0.7,0.7);
    double nullStiff = 0.0;
    double nullDamp = 0.7;
    // *** initialize ***
    // services
    control_mode.init("iiwa");
    j_vel.init("iiwa");
    c_vel.init("iiwa");
    time_to_dist.init("iiwa");
    // commands
    cp_command.init("iiwa");
    cpl_command.init("iiwa");
    jp_command.init("iiwa");
    // states
    cw_state.init("iiwa");
    cp_state.init("iiwa");
    jv_state.init("iiwa");
    jp_state.init("iiwa");
    double approach_vel = 0.02;
    double approach_Jvel = 0.01;
    // decleare fixed positions - flange-world coordinates sys.
    std::vector<std::vector<float>> poses = {{0.2675,-0.3143, 0.6669},         // initial position
                                             {0.095, -0.618, 0.380},         // position1 : detect peg
                                             {-0.195,-0.525,0.380}};         // position2: detect hole
    std::vector<float> orient0 =  {1.03992484845e-05, 0.999994754791, 1.49843574439e-05, -0.00323369763346};             
    std::vector<float> orient = {0.707165002823, 0.707041292473, -0.00230447391603, -0.00221763853181};
    // set the cartesian and joints velocity limit
    j_vel.setSmartServoJointSpeedLimits(Jvel, Jvel);
    c_vel.setMaxCartesianVelocity(cartesian_velocity); 
    ros::Duration(0.5).sleep();  // wait to initialize ros topics
    // ....
    auto cartesian_wrench = cw_state.getWrench();
    // PD force controller loop parameters:
    double f_ref = 8.0;
    double fz = 0.0;
    double fe = 0.0;
    double fep = 0.0;
    double dfe = 0.0;
    double dt = 0.01;
    double dz = 0;
    double kp = 0.000072;
    double kd = 0.0000014;   
    std::cout<<" ********************** "<<std::endl;
    std::cout<<" **** TASK STARTED **** "<<std::endl;
    std::cout<<" ********************** "<<std::endl;
    // open gripper
    gripper.data = open;
    gripper_command.publish(gripper);
    ros::Duration(1.0).sleep();
    // get current position
    auto cartesian_position = cp_state.getPose();
    new_pose = cartesian_position.poseStamped;
    // move to initial position
    // Position
    new_pose.pose.position.x = poses[0][0];
    new_pose.pose.position.y = poses[0][1];
    new_pose.pose.position.z = poses[0][2];
    // Orientation
    new_pose.pose.orientation.x = orient[0];
    new_pose.pose.orientation.y = orient[1];
    new_pose.pose.orientation.z = orient[2];
    new_pose.pose.orientation.w = orient[3];
    cp_command.setPose(new_pose);
    waitForMotion(time_to_dist);
    while (!atPosition(cp_state.getPose().poseStamped, new_pose));   
    std::cout<<"Reached to initial position"<<std::endl;
    // get current position
    cartesian_position = cp_state.getPose();
    // move to position1
    new_pose = cartesian_position.poseStamped;
    // Position
    new_pose.pose.position.x = poses[1][0];
    new_pose.pose.position.y = poses[1][1];
    new_pose.pose.position.z = poses[1][2];
    // Orientation
    new_pose.pose.orientation.x = orient[0];
    new_pose.pose.orientation.y = orient[1];
    new_pose.pose.orientation.z = orient[2];
    new_pose.pose.orientation.w = orient[3];
    cp_command.setPose(new_pose);
    waitForMotion(time_to_dist);
    while (!atPosition(cp_state.getPose().poseStamped, new_pose)); 
    std::cout<<"Reached to position1"<<std::endl;
    ros::Duration(1.0).sleep();
    // get desired position from detection node:
    double des_x = 0;
    double des_y = 0;
    double des_z = 0;
    double peg_in_hole = 0;
    // at this step, the program will wait till it receives meaningfull data from the HoleDetectMask node. Otherwise it will shutdown the node after 3 seonds
    double receive_flag = 1;
    ros::Time start_wait = ros::Time::now();
    while (desired_pose.pose.position.x == 0 && desired_pose.pose.position.y == 0 && desired_pose.pose.position.z == 0)
    {
        if ((ros::Time::now() - start_wait) >= ros::Duration(4.0)) // time out = 4 seconds
        {
            receive_flag = 0;
            break;
        } 
    }
    if (receive_flag)
    {
        for (int i = 0; i<10; i++)
        {
            if (desired_pose.pose.position.x != 0 && desired_pose.pose.position.y != 0)
            {
            des_x = des_x + desired_pose.pose.position.x;
            des_y = des_y + desired_pose.pose.position.y;
            }
            else
            {
                i-=1;
            }
            // des_z = des_z + desired_pose.pose.position.z;
            ros::Duration(0.05).sleep();
        }
        des_x = des_x/10;
        des_y = des_y/10;
        // des_z = des_z/10;
        std::cout<<"peg center position: ("<<std::to_string(desired_pose.pose.position.x)<<", "<<std::to_string(desired_pose.pose.position.y)<<")"<<std::endl;
        cartesian_position = cp_state.getPose();
        new_pose = cartesian_position.poseStamped;
        new_pose.pose.position.x = des_x; // Add offest x: 0.005, y: 0.003
        new_pose.pose.position.y = des_y;
        // Orientation
        new_pose.pose.orientation.x = orient[0];
        new_pose.pose.orientation.y = orient[1];
        new_pose.pose.orientation.z = orient[2];
        new_pose.pose.orientation.w = orient[3];
        cpl_command.setPose(new_pose);
        waitForMotion(time_to_dist);
        while (!atPosition(cp_state.getPose().poseStamped, new_pose)); 
        std::cout<<"Reached pick position"<<std::endl;
        // open gripper
        gripper.data = open;
        gripper_command.publish(gripper);
        ros::Duration(1.0).sleep();
        cartesian_position = cp_state.getPose();
        new_pose = cartesian_position.poseStamped;
        new_pose.pose.position.z = 0.345; // Add offest x: 0.005, y: 0.003
        // Orientation
        new_pose.pose.orientation.x = orient[0];
        new_pose.pose.orientation.y = orient[1];
        new_pose.pose.orientation.z = orient[2];
        new_pose.pose.orientation.w = orient[3];
        cpl_command.setPose(new_pose);
        waitForMotion(time_to_dist);
        while (!atPosition(cp_state.getPose().poseStamped, new_pose)); 
        // decrease the velocity limits 
        cartesian_velocity.linear.x = approach_vel;
        cartesian_velocity.linear.y = approach_vel;
        cartesian_velocity.linear.z = approach_vel;
        cartesian_velocity.angular.x = approach_vel;
        cartesian_velocity.angular.y = approach_vel;
        cartesian_velocity.angular.z = approach_vel;
        j_vel.setSmartServoJointSpeedLimits(approach_Jvel, approach_Jvel);
        c_vel.setMaxCartesianVelocity(cartesian_velocity);  
        ros::Duration(0.2).sleep();
        cartesian_position = cp_state.getPose();
        new_pose = cartesian_position.poseStamped;
        // move downwards till the robot dollides with the object
        double z0 = cartesian_position.poseStamped.pose.position.z;
        // PD force controller loop parameters:
        cartesian_wrench = cw_state.getWrench();
        f_ref = 8.0;
        fz = fabs(cw_state.getWrench().wrench.force.z);
        fe = f_ref - fz;
        fep = fe;
        dfe = fe - fep;
        dt = 0.01;
        dz = 0;
        kp = 0.000072;
        kd = 0.0000014;   
        while (true)
        {
            do{
            fz = fabs(cw_state.getWrench().wrench.force.z);  
            fe = f_ref - fz; 
            dfe = fe - fep;
            fep = fe;
            dz = kp*fe + kd*dfe/dt;
            cartesian_position = cp_state.getPose();
            new_pose = cartesian_position.poseStamped;
            new_pose.pose.position.z -= dz;
            new_pose.pose.orientation.x = orient[0];
            new_pose.pose.orientation.y = orient[1];
            new_pose.pose.orientation.z = orient[2];
            new_pose.pose.orientation.w = orient[3];
            cpl_command.setPose(new_pose);
            ros::Duration(dt).sleep();
            }while (fe > 0.25);
            if (cp_state.getPose().poseStamped.pose.position.z < 0.320) // TODO: check the value 0.320
            {
                break;
            }
            else
            {
                if (cw_state.getWrench().wrench.torque.y < 0)
                {
                    cartesian_position = cp_state.getPose();
                    new_pose = cartesian_position.poseStamped;      
                    new_pose.pose.position.z += 0.002;
                    // cpl_command.setPose(new_pose);
                    // ros::Duration(0.05).sleep();
                    // cartesian_position = cp_state.getPose();
                    // new_pose = cartesian_position.poseStamped;      
                    new_pose.pose.position.y += 0.0015;
                    cpl_command.setPose(new_pose);
                    ros::Duration(0.05).sleep();
                }
                else if (cw_state.getWrench().wrench.torque.y > 0)
                {
                    cartesian_position = cp_state.getPose();
                    new_pose = cartesian_position.poseStamped;      
                    new_pose.pose.position.z += 0.002;
                    // cpl_command.setPose(new_pose);
                    // ros::Duration(0.05).sleep();
                    // cartesian_position = cp_state.getPose();
                    // new_pose = cartesian_position.poseStamped;      
                    new_pose.pose.position.y -= 0.0015;
                    cpl_command.setPose(new_pose);
                    ros::Duration(0.05).sleep();
                }
            }
        }
        std::cout<<"Fz = "<<std::to_string(cw_state.getWrench().wrench.force.z)<<std::endl;
        cartesian_position = cp_state.getPose();
        std::cout<<"z = "<<std::to_string(cartesian_position.poseStamped.pose.position.z)<<std::endl;
        // close gripper
        gripper.data = close;
        gripper_command.publish(gripper);
        ros::Duration(1.0).sleep();
        std::cout<<"object is grasped "<<std::endl;
        // increase the velocity limits 
        cartesian_velocity.linear.x = vel;
        cartesian_velocity.linear.y = vel;
        cartesian_velocity.linear.z = vel;
        cartesian_velocity.angular.x = vel;
        cartesian_velocity.angular.y = vel;
        cartesian_velocity.angular.z = vel;
        j_vel.setSmartServoJointSpeedLimits(Jvel, Jvel);
        c_vel.setMaxCartesianVelocity(cartesian_velocity);  
        ros::Duration(0.2).sleep();
        // move back (upwards) along z
        cartesian_position = cp_state.getPose();
        new_pose = cartesian_position.poseStamped;
        new_pose.pose.position.z = z0;
        new_pose.pose.orientation.x = orient[0];
        new_pose.pose.orientation.y = orient[1];
        new_pose.pose.orientation.z = orient[2];
        new_pose.pose.orientation.w = orient[3];
        cpl_command.setPose(new_pose);
        waitForMotion(time_to_dist);
        while (!atPosition(cp_state.getPose().poseStamped, new_pose)); 
        // move to position2
        cartesian_position = cp_state.getPose();
        new_pose = cartesian_position.poseStamped;
        // Position
        new_pose.pose.position.x = poses[2][0];
        new_pose.pose.position.y = poses[2][1];
        new_pose.pose.position.z = poses[2][2];
        // Orientation
        new_pose.pose.orientation.x = orient[0];
        new_pose.pose.orientation.y = orient[1];
        new_pose.pose.orientation.z = orient[2];
        new_pose.pose.orientation.w = orient[3];
        cp_command.setPose(new_pose);
        waitForMotion(time_to_dist);
        while (!atPosition(cp_state.getPose().poseStamped, new_pose)); 
    }
    // ////////////////////////////////////////////  PLACING SECTION  //////////////////////////////////////////////////
    // get desired position from detection node:
    ros::Duration(4.0).sleep();
    double assem_time = ros::Time::now().toSec();
    double peg_x, peg_y;
    peg_x = des_x;
    peg_y = des_y;
    des_x = 0;
    des_y = 0;
    start_wait = ros::Time::now();
    do{
        if ((ros::Time::now() - start_wait) >= ros::Duration(4.0)) // time out = 4 seconds
        {
            receive_flag = 0;
            std::cout<<"Hols is not detect :("<<std::endl;
            break;
        }
        des_x = desired_pose.pose.position.x;
        des_y = desired_pose.pose.position.y;
    }while(des_x==0 && des_y==0 && fabs(des_x-peg_x)<0.20&& fabs(des_y-peg_y)<0.20 && des_x > 0.0);

    if (receive_flag)
    {    
        des_x = 0;
        des_y = 0;
        des_z = 0;
        for (int i = 0; i<20; i++)
        {
            if (desired_pose.pose.position.x != 0 && desired_pose.pose.position.y != 0)
            {
            des_x = des_x + desired_pose.pose.position.x;
            des_y = des_y + desired_pose.pose.position.y;
            }
            else
            {
                i-=1;
            }
            // des_z = des_z + desired_pose.pose.position.z;
            ros::Duration(0.05).sleep();
        }
        des_x = des_x/20;
        des_y = des_y/20;
        double hole_x, hole_y;
        hole_x = des_x;
        hole_y = des_y;
        std::cout<<"Hole position: ("<<std::to_string(des_x)<<", "<<std::to_string(des_y)<<")"<<std::endl;
        cartesian_position = cp_state.getPose();
        new_pose = cartesian_position.poseStamped;
        new_pose.pose.position.x += 9*(des_x-cartesian_position.poseStamped.pose.position.x)/10;
        new_pose.pose.position.y += 9*(des_y-cartesian_position.poseStamped.pose.position.y)/10;
        new_pose.pose.orientation.x = orient[0];
        new_pose.pose.orientation.y = orient[1];
        new_pose.pose.orientation.z = orient[2];
        new_pose.pose.orientation.w = orient[3];
        cpl_command.setPose(new_pose);
        waitForMotion(time_to_dist);
        while (!atPosition(cp_state.getPose().poseStamped, new_pose)); 
        // // // // // // Double check // // // // //
        double des_x_temp = 0, des_y_temp = 0;
        start_wait = ros::Time::now();
        do
        {
            if ((ros::Time::now() - start_wait) >= ros::Duration(2.0)) // time out = 2.0 seconds
            {
                break;
            } 
            des_x_temp = desired_pose.pose.position.x;
            des_y_temp = desired_pose.pose.position.y;
        }while(fabs(des_x_temp-cp_state.getPose().poseStamped.pose.position.x)>0.010 && fabs(des_y_temp-cp_state.getPose().poseStamped.pose.position.y)>0.01);
        
        // if (fabs(des_x_temp-des_x)<0.006 && fabs(des_y_temp-des_y)<0.006) 
        //     {
        //         std::cout<<"[Double Check] Hole position: ("<<std::to_string(des_x_temp)<<", "<<std::to_string(des_x_temp)<<")"<<std::endl;
        //         cartesian_position = cp_state.getPose();
        //         new_pose = cartesian_position.poseStamped;
        //         new_pose.pose.position.x = des_x_temp;
        //         new_pose.pose.position.y = des_y_temp;
        //         new_pose.pose.orientation.x = orient[0];
        //         new_pose.pose.orientation.y = orient[1];
        //         new_pose.pose.orientation.z = orient[2];
        //         new_pose.pose.orientation.w = orient[3];
        //         cpl_command.setPose(new_pose);
        //         waitForMotion(time_to_dist);
        //         while (!atPosition(cp_state.getPose().poseStamped, new_pose));
        //     }        
        std::cout<<"Reached over the hole"<<std::endl;
        cartesian_position = cp_state.getPose();
        new_pose = cartesian_position.poseStamped;
        new_pose.pose.position.z = 0.350;
        // Orientation
        new_pose.pose.orientation.x = orient[0];
        new_pose.pose.orientation.y = orient[1];
        new_pose.pose.orientation.z = orient[2];
        new_pose.pose.orientation.w = orient[3];
        cpl_command.setPose(new_pose);
        waitForMotion(time_to_dist);
        while (cp_state.getPose().poseStamped.pose.position.z > 0.3505); 
        // ros::Duration(1.5).sleep();
        des_x_temp = 0;
        des_y_temp = 0;
        // get desired position from detection node:
        start_wait = ros::Time::now();
        while(fabs(des_x_temp-des_x)>0.010 && fabs(des_y_temp-des_x)>0.010)
        {
            if ((ros::Time::now() - start_wait) >= ros::Duration(2.0)) // time out = 2.0 seconds
            {
                break;
            } 
            des_x_temp = desired_pose.pose.position.x;
            des_y_temp = desired_pose.pose.position.y;
        }
        // if (fabs(des_x_temp-des_x)<0.005 && fabs(des_y_temp-des_y)<0.005) 
        //     {
        //         std::cout<<"[UPDATED] Hole position: ("<<std::to_string(des_x_temp)<<", "<<std::to_string(des_x_temp)<<")"<<std::endl;
        //         cartesian_position = cp_state.getPose();
        //         new_pose = cartesian_position.poseStamped;
        //         new_pose.pose.position.x = des_x_temp;
        //         new_pose.pose.position.y = des_y_temp;
        //         new_pose.pose.orientation.x = orient[0];
        //         new_pose.pose.orientation.y = orient[1];
        //         new_pose.pose.orientation.z = orient[2];
        //         new_pose.pose.orientation.w = orient[3];
        //         cpl_command.setPose(new_pose);
        //         waitForMotion(time_to_dist);
        //         while (!atPosition(cp_state.getPose().poseStamped, new_pose));
        //     }    
        // cartesian_position = cp_state.getPose();
        // new_pose = cartesian_position.poseStamped;
        // new_pose.pose.position.z = 0.320;
        // // Orientation
        // new_pose.pose.orientation.x = orient[0];
        // new_pose.pose.orientation.y = orient[1];
        // new_pose.pose.orientation.z = orient[2];
        // new_pose.pose.orientation.w = orient[3];
        // cpl_command.setPose(new_pose);
        // waitForMotion(time_to_dist);
        // while (cp_state.getPose().poseStamped.pose.position.z > 0.3205);    
        approach_vel = 0.020;
        approach_Jvel = 0.10;
        cartesian_velocity.linear.x = approach_vel;
        cartesian_velocity.linear.y = approach_vel;
        cartesian_velocity.linear.z = approach_vel;
        cartesian_velocity.angular.x = approach_vel;
        cartesian_velocity.angular.y = approach_vel;
        cartesian_velocity.angular.z = approach_vel;
        j_vel.setSmartServoJointSpeedLimits(approach_Jvel, approach_Jvel);
        c_vel.setMaxCartesianVelocity(cartesian_velocity);  
        ros::Duration(0.2).sleep();
        cartesian_position = cp_state.getPose();
        new_pose = cartesian_position.poseStamped;
        // move downwards till the robot collides with the object
        double z_hole = 0.314; // TODO: this should be specified for safety
        // PD force controller loop parameters:
        cartesian_wrench = cw_state.getWrench();
        fz = fabs(cw_state.getWrench().wrench.force.z);
        fe = f_ref - fz;
        fep = fe;
        dfe = fe - fep;
        dz = 0;
        double m = 0;
        do{
            fz = fabs(cw_state.getWrench().wrench.force.z);  
            fe = f_ref - fz; 
            dfe = fe - fep;
            fep = fe;
            dz = kp*fe + kd*dfe/dt;
            cartesian_position = cp_state.getPose();
            new_pose = cartesian_position.poseStamped;
            new_pose.pose.position.z -= dz;
            // // This might be unnecessary //
            // des_x = desired_pose.pose.position.x;
            // des_y = desired_pose.pose.position.y;
            // if (fabs(des_x-new_pose.pose.position.x)<0.002)
            // new_pose.pose.position.x = des_x;
            // if (fabs(des_y-new_pose.pose.position.y)<0.002)
            // new_pose.pose.position.y = des_y;
            // // -------------------------- //
            new_pose.pose.orientation.x = orient[0];
            new_pose.pose.orientation.y = orient[1];
            new_pose.pose.orientation.z = orient[2];
            new_pose.pose.orientation.w = orient[3];
            cpl_command.setPose(new_pose);
            ros::Duration(dt).sleep();
        }while (fe > 0.25 && cp_state.getPose().poseStamped.pose.position.z > z_hole);
        if (cp_state.getPose().poseStamped.pose.position.z < z_hole)
        {
            peg_in_hole = 1; // flag
        }
        std::cout<<"Fz: "<<std::to_string(cw_state.getWrench().wrench.force.z)<<std::endl;
        cartesian_position = cp_state.getPose();
        std::cout<<"Z: "<<std::to_string(cartesian_position.poseStamped.pose.position.z)<<std::endl;
        if (peg_in_hole)
        {  
            // the peg is over the hole 
            std::cout<<" ********************** "<<std::endl;
            std::cout<<" **** Peg in Hole ***** "<<std::endl;
            std::cout<<" ********************** "<<std::endl;
            // // open gripper
            gripper.data = open;
            gripper_command.publish(gripper);
            ros::Duration(1.0).sleep();
        }
        //////////// SEARCHING FOR THE HOLE /////////////////////////
        else
        {
            approach_vel = 0.001;
            approach_Jvel = 0.03;
            cartesian_velocity.linear.x = approach_vel;
            cartesian_velocity.linear.y = approach_vel;
            cartesian_velocity.linear.z = approach_vel;
            cartesian_velocity.angular.x = approach_vel;
            cartesian_velocity.angular.y = approach_vel;
            cartesian_velocity.angular.z = approach_vel;
            j_vel.setSmartServoJointSpeedLimits(approach_Jvel, approach_Jvel);
            c_vel.setMaxCartesianVelocity(cartesian_velocity);  
            ros::Duration(0.2).sleep();
            std::cout<<" Reached to placing plane "<<std::endl;
            // Searching for the hole:
            double j7_limit = 170.0*M_PI/180.0; // joint7 limits are +-175 degree
            auto JP = jp_state.getPosition();
            double j7_l=1, joint_7=0.0, joint_7p=0;
            double z1 = cp_state.getPose().poseStamped.pose.position.z;
            double des_xp, des_yp, none_detected = 0;
            double x_step, y_step;
            int n = 0, EE=1;
            des_xp = des_x;
            des_yp = des_y;
            // des_x =0;
            // des_y =0;
            start_wait = ros::Time::now();

            while(!peg_in_hole && !none_detected)
            {
                EE =1;
                while (fabs(cw_state.getWrench().wrench.force.z)<4.0)
                    {
                        cartesian_position = cp_state.getPose();
                        new_pose = cartesian_position.poseStamped;
                        new_pose.pose.position.z -= 0.001;
                        cpl_command.setPose(new_pose);
                        waitForMotion(time_to_dist);
                        while (!atPosition(cp_state.getPose().poseStamped, new_pose));
                        if (cp_state.getPose().poseStamped.pose.position.z<0.313)
                        {
                            peg_in_hole = 1;
                            break;
                        }
                    }
                
                if (fabs(cw_state.getWrench().wrench.force.z)>4.0){
                    do{
                    if ((ros::Time::now() - start_wait) >= ros::Duration(1.0)) // time out = 2.0 seconds
                    {
                        EE = 0;
                        break;
                    } 
                    des_x = desired_pose.pose.position.x;
                    des_y = desired_pose.pose.position.y;
                     }while(fabs(des_x-des_xp)>0.010 && fabs(des_y-des_yp)>0.010);
                }
                else 
                {
                    EE=0;
                }
                if (fabs(des_x-cp_state.getPose().poseStamped.pose.position.x) < 0.010 && fabs(des_y-cp_state.getPose().poseStamped.pose.position.y) < 0.010)
                {
                std::cout<<"new hole coords: ("<<std::to_string(des_x)<<", "<<std::to_string(des_y)<<")"<<std::endl;
                
                // movement will be achieved in steps
                cartesian_position = cp_state.getPose();
                x_step = (des_x-cartesian_position.poseStamped.pose.position.x)/10;
                y_step = (des_y-cartesian_position.poseStamped.pose.position.y)/10;
                for (int i=0; i<10; i++)
                {
                    new_pose = cartesian_position.poseStamped;
                    new_pose.pose.position.x += x_step;
                    new_pose.pose.position.y += y_step;
                    cpl_command.setPose(new_pose);
                    waitForMotion(time_to_dist);
                    while (!atPosition(cp_state.getPose().poseStamped, new_pose));
                    while (fabs(cw_state.getWrench().wrench.force.z)<5.0)
                    {
                        cartesian_position = cp_state.getPose();
                        new_pose = cartesian_position.poseStamped;
                        new_pose.pose.position.z -= 0.0005;
                        cpl_command.setPose(new_pose);
                        waitForMotion(time_to_dist);
                        while (!atPosition(cp_state.getPose().poseStamped, new_pose));
                        if (cp_state.getPose().poseStamped.pose.position.z<0.313)
                        {
                            peg_in_hole = 1;
                            break;
                        }
                    }
                    if (peg_in_hole)
                        break;
                }
                }
                while (fabs(cw_state.getWrench().wrench.force.z)>8.0 && !peg_in_hole)
                {
                    cartesian_position = cp_state.getPose();
                    new_pose = cartesian_position.poseStamped;
                    new_pose.pose.position.z += 0.0002;
                    cpl_command.setPose(new_pose);
                    waitForMotion(time_to_dist);
                }
                des_xp = des_x;
                des_yp = des_y;
                do{
                    if ((ros::Time::now() - start_wait) >= ros::Duration(2.0)) // time out = 2.0 seconds
                    {
                        break;
                    } 
                    des_x = desired_pose.pose.position.x;
                    des_y = desired_pose.pose.position.y;
                }while(!peg_in_hole && (fabs(des_x-cp_state.getPose().poseStamped.pose.position.x)>0.015 || fabs(des_y-cp_state.getPose().poseStamped.pose.position.y)>0.015) && fabs(cw_state.getWrench().wrench.force.z)>4.0);                  
                ros::Duration(1.0).sleep();
                start_wait = ros::Time::now();
                if (fabs(cw_state.getWrench().wrench.force.z)>4.0 && !peg_in_hole)
                {
                    //|| ((fabs(des_x-des_xp)>0.01 && fabs(des_y-des_yp)>0.01))
                    //fabs(des_x-des_xp)==0.0 && fabs(des_y-des_yp)==0.0 &&
                while ( !peg_in_hole)
                {
                    if (n==0)
                    {
                        std::cout<<"Searching for the hole ..."<<std::endl;
                        n=1;
                    }
                    if (cw_state.getWrench().wrench.torque.x > 0.0)
                    j7_l = 0;
                    JP = jp_state.getPosition();
                    if ((JP.position.a7 + 20.0*M_PI/180.0) < j7_limit && j7_l == 0)
                    {
                        JP.position.a7 += 20.0*M_PI/180.0;
                    }  
                    else if ((JP.position.a7+20.0*M_PI/180.0) > j7_limit && j7_l == 0)
                    {
                        JP.position.a7=j7_limit;
                        j7_l = 1;
                    }
                    else if ((JP.position.a7-20.0*M_PI/180.0) > -j7_limit && j7_l == 1)
                    {
                        JP.position.a7 -= 20.0*M_PI/180.0;
                        
                    }
                    else if ((JP.position.a7-20.0*M_PI/180.0) < -j7_limit && j7_l == 1)
                    {
                        JP.position.a7 = -j7_limit;
                        j7_l = 0;
                    }
                    jp_command.setPosition(JP);
                    waitForMotion(time_to_dist);
                    ros::Duration(1.0).sleep();
                    if ((ros::Time::now() - start_wait) >= ros::Duration(100.0)) // time out = 15 seconds
                    {
                        none_detected = 1;
                        break;
                    } 
                    des_x = desired_pose.pose.position.x;
                    des_y = desired_pose.pose.position.y;
                    // std::cout<<"coords_1: ("<<std::to_string(des_x)<<", "<<std::to_string(des_y)<<")"<<std::endl;
                    // std::cout<<"coords_pp1: ("<<std::to_string(des_xp)<<", "<<std::to_string(des_yp)<<")"<<std::endl;
                    if (fabs(des_x-cp_state.getPose().poseStamped.pose.position.x)>0.015 && fabs(des_y-cp_state.getPose().poseStamped.pose.position.y)>0.015)
                        {
                            des_x=des_xp;
                            des_y=des_yp;
                        }
                    else if (fabs(des_x-cp_state.getPose().poseStamped.pose.position.x)<0.01 && fabs(des_y-cp_state.getPose().poseStamped.pose.position.y)<0.01)
                    {
                        break;
                    }  
                    // std::cout<<"coords: ("<<std::to_string(des_x)<<", "<<std::to_string(des_y)<<")"<<std::endl;
                    // std::cout<<"coords_pp: ("<<std::to_string(des_xp)<<", "<<std::to_string(des_yp)<<")"<<std::endl;
                    while (fabs(cw_state.getWrench().wrench.force.z)<5.0)
                    {
                        cartesian_position = cp_state.getPose();
                        new_pose = cartesian_position.poseStamped;
                        new_pose.pose.position.z -= 0.0005;
                        cpl_command.setPose(new_pose);
                        waitForMotion(time_to_dist);
                        if (cp_state.getPose().poseStamped.pose.position.z<0.313)
                        {
                            peg_in_hole = 1;
                            break;
                        }
                    }
                }
                
                std::cout<<"new detection"<<std::endl;
                des_x = 0;
                des_y = 0;
                des_z = 0;
                for (int i = 0; i<10; i++)
                {
                    if (desired_pose.pose.position.x != 0 && desired_pose.pose.position.y != 0)
                    {
                    des_x = des_x + desired_pose.pose.position.x;
                    des_y = des_y + desired_pose.pose.position.y;
                    }
                    else
                    {
                        i-=1;
                    }
                    // des_z = des_z + desired_pose.pose.position.z;
                    ros::Duration(0.05).sleep();
                }
                des_x = des_x/10;
                des_y = des_y/10;
                std::cout<<"coords: ("<<std::to_string(des_x)<<", "<<std::to_string(des_y)<<")"<<std::endl;
                }
                // movement will be achieved in steps
                cartesian_position = cp_state.getPose();
                x_step = (des_x-cartesian_position.poseStamped.pose.position.x)/10;
                y_step = (des_y-cartesian_position.poseStamped.pose.position.y)/10;
                for (int i=0; i<10; i++)
                {
                    // cartesian_position = cp_state.getPose();
                    new_pose = cartesian_position.poseStamped;
                    new_pose.pose.position.x += x_step;
                    new_pose.pose.position.y += y_step;
                    cpl_command.setPose(new_pose);
                    waitForMotion(time_to_dist);
                    while (!atPosition(cp_state.getPose().poseStamped, new_pose));
                    while (fabs(cw_state.getWrench().wrench.force.z)<5.0)
                    {
                        cartesian_position = cp_state.getPose();
                        new_pose = cartesian_position.poseStamped;
                        new_pose.pose.position.z -= 0.0005;
                        cpl_command.setPose(new_pose);
                        waitForMotion(time_to_dist);
                        while (!atPosition(cp_state.getPose().poseStamped, new_pose));
                        if (cp_state.getPose().poseStamped.pose.position.z<0.313)
                        {
                            peg_in_hole = 1;
                            break;
                        }
                    }
                    if (peg_in_hole)
                        break;
                }
                if (cp_state.getPose().poseStamped.pose.position.z<0.313)
                    break; 
                if (!peg_in_hole && (fabs(cp_state.getPose().poseStamped.pose.position.x-hole_x)>0.01 || fabs(cp_state.getPose().poseStamped.pose.position.y-hole_y)>0.01))
                {
                    cartesian_position = cp_state.getPose();
                    new_pose = cartesian_position.poseStamped;
                    new_pose.pose.position.x = hole_x;
                    new_pose.pose.position.y = hole_y;
                    cpl_command.setPose(new_pose);
                    waitForMotion(time_to_dist);
                }         
            }
            if (peg_in_hole)
            {  
                // the peg is over the hole 
                std::cout<<" ********************** "<<std::endl;
                std::cout<<" **** Peg in Hole ***** "<<std::endl;
                std::cout<<" ********************** "<<std::endl;
                // // open gripper
                gripper.data = open;
                gripper_command.publish(gripper);
                ros::Duration(1.0).sleep();
            }
            else
            {
                std::cout<<"Failed to detect the hole"<<std::endl;
                // // open gripper
                gripper.data = open;
                gripper_command.publish(gripper);
                ros::Duration(1.0).sleep();
            }
        }
        // move away
        // increase the velocity limits 
        assem_time = ros::Time::now().toSec()-assem_time;
        cartesian_velocity.linear.x = vel;
        cartesian_velocity.linear.y = vel;
        cartesian_velocity.linear.z = vel;
        cartesian_velocity.angular.x = vel;
        cartesian_velocity.angular.y = vel;
        cartesian_velocity.angular.z = vel;
        j_vel.setSmartServoJointSpeedLimits(Jvel, Jvel);
        c_vel.setMaxCartesianVelocity(cartesian_velocity); 
        ros::Duration(0.5).sleep();  // wait to initialize ros topics
        cartesian_position = cp_state.getPose();
        new_pose = cartesian_position.poseStamped;
        new_pose.pose.position.z = 0.380;
        cpl_command.setPose(new_pose);
        waitForMotion(time_to_dist);
        while (!atPosition(cp_state.getPose().poseStamped, new_pose));
    }
    
    std::cout<<"Assembly time: "<<std::to_string(assem_time)<<"sec"<<std::endl;
}