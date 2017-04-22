# pragma once

#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include "sensor_msgs/JointState.h"

#define no_of_iterations 100
#define t_max 5

std::vector<float> joint_states;

typedef actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction> armClient;
typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> baseClient;

using namespace Eigen;

class TrajectoryFollow{
private:
     ros::NodeHandle nh_;
     armClient *arm_client;
     baseClient *base_client;
     MatrixXd outputs;

public:
    TrajectoryFollow(ros::NodeHandle& nh);

    ~TrajectoryFollow();
    
    void startTrajectory(control_msgs::FollowJointTrajectoryGoal& arm_goal);
    
    actionlib::SimpleClientGoalState getState();

    void jointsCallback(const sensor_msgs::JointState& state);

    MatrixXd trajectory(const MatrixXd& joint_positions, float tf);

    control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory(const MatrixXd& input);
    
    move_base_msgs::MoveBaseGoal baseMove();

    void startMoveBase(move_base_msgs::MoveBaseGoal& base_goal);

    bool solve_ik(double num_samples, std::string chain_start, std::string chain_end, double timeout, std::string, KDL::JntArray &result);

    double fRand(double min, double max);

};
