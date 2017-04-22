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
#include "sensor_msgs/JointState.h"

#define no_of_iterations 100
#define t_max 5

std::vector<float> joint_states;

typedef actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction> armClient;
typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> baseClient;

using namespace Eigen;

class TrajectoryFollow{
private:
     armClient *arm_client;
     baseClient *base_client;
     MatrixXf outputs;

public:
    TrajectoryFollow();

    ~TrajectoryFollow();
    
    void startTrajectory(control_msgs::FollowJointTrajectoryGoal& arm_goal);
    
    actionlib::SimpleClientGoalState getState();

    void jointsCallback(const sensor_msgs::JointState& state);

    MatrixXf trajectory(const MatrixXf& joint_positions, float tf);

    control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory(const MatrixXf& input);
    
    move_base_msgs::MoveBaseGoal baseMove();

    void startMoveBase(move_base_msgs::MoveBaseGoal& base_goal);

};
