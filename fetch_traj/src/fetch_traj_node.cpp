#include "fetch_traj/fetch_traj.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fetch_traj_node");
    ros::NodeHandle nh;
    TrajectoryFollow dynam;

    //ros::Subscriber sub = nh.subscribe("joint_states", 1000, &TrajectoryFollow::jointsCallback, &dynam);

    MatrixXf joint_positions(6,2);
    //ROS_INFO_STREAM("jointstates" << joint_states[0] << std::endl);
    joint_positions << 1.3200041507484386, 1.3999855756514323, -0.19988558358145525, 1.719970634612789, 5.510375048700666e-06, 1.6600036782876648,
                       0.7, 0, 0, 0, 0, 0;

    MatrixXf outputs(6,6);
    outputs = dynam.trajectory(joint_positions, t_max);

    control_msgs::FollowJointTrajectoryGoal traj;
    traj = dynam.armExtensionTrajectory(outputs);

    dynam.startTrajectory(traj);

    while(!dynam.getState().isDone() && ros::ok())
    {
        usleep(50000);
    }

    ros::spinOnce();
    return 0;
}
