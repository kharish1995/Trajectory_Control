#include "fetch_traj/fetch_traj.h"

enum sm {
    PREPARE_START = 0,
    MOVE_BASE1,
    MOVE_ARM1,
    CLOSE_GRIPPER,
    TUCK_ARM,
    MOVE_BASE2,
    MOVE_ARM2,
    OPEN_GRIPPER
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fetch_traj_node");
    ros::NodeHandle nh;
    TrajectoryFollow dynam(nh);

    srand(1);
    KDL::JntArray result;

<<<<<<< HEAD
//    MatrixXf joint_positions(2,6);
//    //ROS_INFO_STREAM("jointstates" << joint_states[0] << std::endl);
//    joint_positions << 1.3200041507484386, 1.3999855756514323, -0.19988558358145525, 1.719970634612789, 5.510375048700666e-06, 1.6600036782876648,
//                       0, 0, 0, 0, 0, 0;
    
<<<<<<< Updated upstream
//    ROS_INFO_STREAM(joint_positions << std::endl);
//    MatrixXf outputs(6,6);
//    outputs = dynam.trajectory(joint_positions, t_max);
=======
    int num_samples;
    std::string chain_start, chain_end, urdf_param;
    double timeout;
>>>>>>> master

    //end-effector pose
    double x, y, z;


<<<<<<< HEAD
    move_base_msgs::MoveBaseGoal base_goal = dynam.baseMove();
=======
    MatrixXd outputs(6,6);

    sm state = PREPARE_START;

    switch (state)
    {
        case PREPARE_START:
        {
                outputs = dynam.trajectory(joint_positions, t_max);
                state = MOVE_BASE1;
        }
        case MOVE_BASE1:
        {
            move_base_msgs::MoveBaseGoal base_goal = dynam.baseMove();
            dynam.startMoveBase(base_goal);
            while(ros::ok() && !dynam.getBaseState().isDone())
            {
                usleep(50000);
            }
            state = MOVE_ARM1;
        }
        case MOVE_ARM1:
        {
            MatrixXd joint_positions(2,6);
            ros::spinOnce();
            std::vector<float> joint_values = dynam.getJoint_states();
            dynam.solve_ik(num_samples, chain_start, chain_end, timeout, urdf_param, result, x, y, z);
            joint_positions << joint_values.at(0), joint_values.at(1), joint_values.at(2), joint_values.at(3), joint_values.at(4), joint_values.at(5),
                               result.data;
            MatrixXd outputs(6,6);
            outputs = dynam.trajectory(joint_positions, t_max);
            control_msgs::FollowJointTrajectoryGoal traj;
            traj = dynam.armExtensionTrajectory(outputs);
            dynam.startTrajectory(traj);
            while(ros::ok() && !dynam.getArmState().isDone())
            {
                usleep(50000);
            }
            state = CLOSE_GRIPPER;
        }
        case CLOSE_GRIPPER:
        {
            control_msgs::GripperCommandGoal grip_goal;
            grip_goal = dynam.gripperTrajectory(10.0,0.05);
            dynam.startGripperAction(grip_goal);
            while(ros::ok() && !dynam.getGripperState().isDone())
            {
                usleep(50000);
            }
            state = TUCK_ARM;
        }
        case TUCK_ARM:
        {
            MatrixXd joint_positions(2,6);
            ros::spinOnce();
            std::vector<float> joint_values = dynam.getJoint_states();
            dynam.solve_ik(num_samples, chain_start, chain_end, timeout, urdf_param, result, x, y, z);
            joint_positions << joint_values.at(0), joint_values.at(1), joint_values.at(2), joint_values.at(3), joint_values.at(4), joint_values.at(5),
                               result.data;
            MatrixXd outputs(6,6);
            outputs = dynam.trajectory(joint_positions, t_max);
            control_msgs::FollowJointTrajectoryGoal traj;
            traj = dynam.armExtensionTrajectory(outputs);
            dynam.startTrajectory(traj);
            while(ros::ok() && !dynam.getArmState().isDone())
            {
                usleep(50000);
            }
            state = MOVE_BASE2;
        }
        case MOVE_BASE2:
        {
            move_base_msgs::MoveBaseGoal base_goal = dynam.baseMove();
            dynam.startMoveBase(base_goal);
            while(ros::ok() && !dynam.getBaseState().isDone())
            {
                usleep(50000);
            }
            state = MOVE_ARM2;
        }
        case MOVE_ARM2:
        {
            MatrixXd joint_positions(2,6);
            ros::spinOnce();
            std::vector<float> joint_values = dynam.getJoint_states();
            dynam.solve_ik(num_samples, chain_start, chain_end, timeout, urdf_param, result, x, y, z);
            joint_positions << joint_values.at(0), joint_values.at(1), joint_values.at(2), joint_values.at(3), joint_values.at(4), joint_values.at(5),
                               result.data;
            MatrixXd outputs(6,6);
            outputs = dynam.trajectory(joint_positions, t_max);
            control_msgs::FollowJointTrajectoryGoal traj;
            traj = dynam.armExtensionTrajectory(outputs);
            dynam.startTrajectory(traj);
            while(ros::ok() && !dynam.getArmState().isDone())
            {
                usleep(50000);
            }
            state = OPEN_GRIPPER;
        }
        case OPEN_GRIPPER:
        {
            control_msgs::GripperCommandGoal grip_goal;
            grip_goal = dynam.gripperTrajectory(10.0,0.1);
            dynam.startGripperAction(grip_goal);
            while(ros::ok() && !dynam.getGripperState().isDone())
            {
                usleep(50000);
            }
            break;
        }
    default:
        break;
    }


>>>>>>> Stashed changes
=======
    nh.param("num_samples", num_samples, 1);
    nh.param("chain_start", chain_start, std::string("torso_lift_link"));
    nh.param("chain_end", chain_end, std::string("wrist_flex_link"));

    if (chain_start=="" || chain_end=="") {
        ROS_FATAL("Missing chain info in launch file");
        exit (-1);
    }
    ros::spinOnce();

    nh.param("timeout", timeout, 0.005);
    nh.param("urdf_param", urdf_param, std::string("/robot_description"));

    nh.param("end_effector_x", x, 0.0);
    nh.param("end_effector_y", y, 0.0);
    nh.param("end_effector_z", z, 0.0);

    if (num_samples < 1)
        num_samples = 1;

    dynam.solve_ik(num_samples, chain_start, chain_end, timeout, urdf_param, result, x, y, z);

>>>>>>> master

    MatrixXd joint_positions(2,6);

    std::vector<float> joint_values = dynam.getJoint_states();

    joint_positions << joint_values.at(0), joint_values.at(1), joint_values.at(2), joint_values.at(3), joint_values.at(4), joint_values.at(5),
                       result.data;
    
    MatrixXd outputs(6,6);
    outputs = dynam.trajectory(joint_positions, t_max);

    move_base_msgs::MoveBaseGoal base_goal = dynam.baseMove();
    dynam.startMoveBase(base_goal);

    control_msgs::FollowJointTrajectoryGoal traj;
    traj = dynam.armExtensionTrajectory(outputs);
    dynam.startTrajectory(traj);

    control_msgs::GripperCommandGoal grip_goal;
    grip_goal = dynam.gripperTrajectory();
    dynam.startGripperAction(grip_goal);


    while(!dynam.getArmState().isDone() && ros::ok() && !dynam.getGripperState().isDone() && !dynam.getBaseState().isDone())
    {
        usleep(50000);
    }


    return 0;
}
