#include "fetch_traj/fetch_traj.h"


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

    if (num_samples < 1)
        num_samples = 1;

    //dynam.solve_ik(num_samples, chain_start, chain_end, timeout, urdf_param, result);


    MatrixXd joint_positions(2,6);

    std::vector<float> joint_values = dynam.getJoint_states();

    joint_positions << joint_values.at(0), joint_values.at(1), joint_values.at(2), joint_values.at(3), joint_values.at(4), joint_values.at(5),
                       0, 0, 0, 0, 0, 0;
    
    MatrixXd outputs(6,6);
    outputs = dynam.trajectory(joint_positions, t_max);

//    move_base_msgs::MoveBaseGoal base_goal = dynam.baseMove();
//    ros::Duration(30).sleep();
//    dynam.startMoveBase(base_goal);
//    ROS_INFO_STREAM("base: " << base_goal.target_pose.pose.position.x << std::endl);

    control_msgs::FollowJointTrajectoryGoal traj;
    traj = dynam.armExtensionTrajectory(outputs);
    dynam.startTrajectory(traj);

    control_msgs::GripperCommandGoal grip_goal;
    grip_goal = dynam.gripperTrajectory();
    dynam.startGripperAction(grip_goal);


    while(!dynam.getArmState().isDone() && ros::ok() && !dynam.getGripperState().isDone())
    {
        usleep(50000);
    }


    return 0;
}
