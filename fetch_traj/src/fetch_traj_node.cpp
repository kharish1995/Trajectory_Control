#include "fetch_traj/fetch_traj.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fetch_traj_node");
    ros::NodeHandle nh;
    TrajectoryFollow dynam(nh);

    //ros::Subscriber sub = nh.subscribe("joint_states", 1000, &TrajectoryFollow::jointsCallback, &dynam);

    //ROS_INFO_STREAM("jointstates" << joint_states[0] << std::endl);
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

    nh.param("timeout", timeout, 0.005);
    nh.param("urdf_param", urdf_param, std::string("/robot_description"));

    if (num_samples < 1)
        num_samples = 1;

    dynam.solve_ik(num_samples, chain_start, chain_end, timeout, urdf_param, result);



    MatrixXd joint_positions(2,6);
    joint_positions << 1.3200041507484386, 1.3999855756514323, -0.19988558358145525, 1.719970634612789, 5.510375048700666e-06, 1.6600036782876648,
                       0, 0.1485, 0, 0, 0, 0;
    
    ROS_INFO_STREAM(joint_positions << std::endl);
    MatrixXd outputs(6,6);
    outputs = dynam.trajectory(joint_positions, t_max);

    control_msgs::FollowJointTrajectoryGoal traj;
    traj = dynam.armExtensionTrajectory(outputs);

    dynam.startTrajectory(traj);
    ROS_INFO_STREAM("IK" << result.data << std::endl);


//    move_base_msgs::MoveBaseGoal base_goal = dynam.baseMove();

  //  dynam.startMoveBase(base_goal);

    while(!dynam.getState().isDone() && ros::ok())
    {
        usleep(50000);
    }

    ros::spinOnce();
    return 0;
}
