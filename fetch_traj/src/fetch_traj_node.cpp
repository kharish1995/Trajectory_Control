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

    int num_samples;
    std::string chain_start, chain_end, urdf_param;
    double timeout;

    //end-effector pose
    double x, y, z;

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
    MatrixXd outputs(6,6);

    MatrixXd joint_positions(2,6);

    std::vector<float> joint_values = dynam.getJoint_states();

    joint_positions << joint_values.at(0), joint_values.at(1), joint_values.at(2), joint_values.at(3), joint_values.at(4), joint_values.at(5),
                       result.data;

    outputs = dynam.trajectory(joint_positions, t_max);

    sm state = PREPARE_START;

    switch (state)
    {
        case PREPARE_START:
        {
                state = MOVE_BASE1;
        }
        case MOVE_BASE1:
        {
            move_base_msgs::MoveBaseGoal base_goal = dynam.baseMove(1.0, 1.2, 1.0);
            dynam.startMoveBase(base_goal);
            ROS_INFO("Move Base1");
            while(!dynam.getBaseState().isDone() && ros::ok())
            {
                usleep(50000);
            }
            state = MOVE_ARM1;
        }
        case MOVE_ARM1:
        {
            ros::spinOnce();
            joint_values = dynam.getJoint_states();
            dynam.solve_ik(num_samples, chain_start, chain_end, timeout, urdf_param, result, x, y, z);
            joint_positions << joint_values.at(0), joint_values.at(1), joint_values.at(2), joint_values.at(3), joint_values.at(4), joint_values.at(5),
                               result.data;
            outputs = dynam.trajectory(joint_positions, t_max);
            control_msgs::FollowJointTrajectoryGoal traj;
            traj = dynam.armExtensionTrajectory(outputs);
            dynam.startTrajectory(traj);
            ROS_INFO("Move Arm1");
            while(!dynam.getArmState().isDone() && ros::ok())
            {
                usleep(50000);
            }
            state = CLOSE_GRIPPER;
        }
        case CLOSE_GRIPPER:
        {
            control_msgs::GripperCommandGoal grip_goal;
            grip_goal = dynam.gripperTrajectory(10.0,0.0);
            dynam.startGripperAction(grip_goal);
            ROS_INFO("Close Gripper1");
            while(!dynam.getGripperState().isDone() && ros::ok())
            {
                usleep(50000);
            }
            state = TUCK_ARM;
        }
        case TUCK_ARM:
        {
            ros::spinOnce();
            joint_values = dynam.getJoint_states();
            dynam.solve_ik(num_samples, chain_start, chain_end, timeout, urdf_param, result, 0, 0, 0.05);
            joint_positions << joint_values.at(0), joint_values.at(1), joint_values.at(2), joint_values.at(3), joint_values.at(4), joint_values.at(5),
                               result.data;
            outputs = dynam.trajectory(joint_positions, t_max);
            control_msgs::FollowJointTrajectoryGoal traj;
            traj = dynam.armExtensionTrajectory(outputs);
            dynam.startTrajectory(traj);
            ROS_INFO("Tuck Arm");
            while(!dynam.getArmState().isDone() && ros::ok())
            {
                usleep(50000);
            }
            state = MOVE_BASE2;
        }
        case MOVE_BASE2:
        {
            move_base_msgs::MoveBaseGoal base_goal = dynam.baseMove(1.7, 1.9, 1.1);
            dynam.startMoveBase(base_goal);
            ROS_INFO("Move Base 2");
            while(!dynam.getBaseState().isDone() && ros::ok())
            {
                usleep(50000);
            }
            state = MOVE_ARM2;
        }
        case MOVE_ARM2:
        {
            ros::spinOnce();
            joint_values = dynam.getJoint_states();
            dynam.solve_ik(num_samples, chain_start, chain_end, timeout, urdf_param, result, x, y, z);
            joint_positions << joint_values.at(0), joint_values.at(1), joint_values.at(2), joint_values.at(3), joint_values.at(4), joint_values.at(5),
                               result.data;
            outputs = dynam.trajectory(joint_positions, t_max);
            control_msgs::FollowJointTrajectoryGoal traj;
            traj = dynam.armExtensionTrajectory(outputs);
            dynam.startTrajectory(traj);
            ROS_INFO("Move Arm2");
            while(!dynam.getArmState().isDone() && ros::ok())
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
            ROS_INFO("Open Gripper1");
            while(!dynam.getGripperState().isDone() && ros::ok())
            {
                usleep(50000);
            }
            break;
        }
    default:
        break;
    }

    while(!dynam.getArmState().isDone() && ros::ok() && !dynam.getGripperState().isDone() && !dynam.getBaseState().isDone())
    {
        usleep(50000);
    }


    return 0;
}
