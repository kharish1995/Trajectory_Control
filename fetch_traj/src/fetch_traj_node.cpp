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
    TrajectoryFollow dynam;

    //ros::Subscriber sub = nh.subscribe("joint_states", 1000, &TrajectoryFollow::jointsCallback, &dynam);

//    MatrixXf joint_positions(2,6);
//    //ROS_INFO_STREAM("jointstates" << joint_states[0] << std::endl);
//    joint_positions << 1.3200041507484386, 1.3999855756514323, -0.19988558358145525, 1.719970634612789, 5.510375048700666e-06, 1.6600036782876648,
//                       0, 0, 0, 0, 0, 0;
    
<<<<<<< Updated upstream
//    ROS_INFO_STREAM(joint_positions << std::endl);
//    MatrixXf outputs(6,6);
//    outputs = dynam.trajectory(joint_positions, t_max);

//    control_msgs::FollowJointTrajectoryGoal traj;
//    traj = dynam.armExtensionTrajectory(outputs);

//    dynam.startTrajectory(traj);

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

    dynam.startMoveBase(base_goal);

    while(!dynam.getState().isDone() && ros::ok())
    {
        usleep(50000);
    }

    ros::spinOnce();
    return 0;
}
