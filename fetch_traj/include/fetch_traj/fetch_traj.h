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
<<<<<<< Updated upstream
    TrajectoryFollow();
=======
     std::vector<float> joint_states;
     static bool isJointStatePopulated;

     TrajectoryFollow(ros::NodeHandle&);
>>>>>>> Stashed changes

    ~TrajectoryFollow();
    
    void startTrajectory(control_msgs::FollowJointTrajectoryGoal&);
    
    actionlib::SimpleClientGoalState getState();

    void jointsCallback(const sensor_msgs::JointState&);

<<<<<<< Updated upstream
    MatrixXf trajectory(const MatrixXf& joint_positions, float tf);

    control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory(const MatrixXf& input);
=======
    MatrixXd trajectory(const MatrixXd&, float);

    control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory(const MatrixXd&);

    control_msgs::GripperCommandGoal gripperTrajectory(double, double);
>>>>>>> Stashed changes
    
    move_base_msgs::MoveBaseGoal baseMove();

    void startMoveBase(move_base_msgs::MoveBaseGoal&);

<<<<<<< Updated upstream
=======
    void startGripperAction(control_msgs::GripperCommandGoal&);

    bool solve_ik(double, std::string, std::string, double, std::string, KDL::JntArray&, double, double, double);

    double fRand(double, double);

    std::vector<float> getJoint_states() const;

>>>>>>> Stashed changes
};
