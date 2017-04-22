#include "fetch_traj/fetch_traj.h"

bool TrajectoryFollow::isJointStatePopulated = false;



std::vector<float> TrajectoryFollow::getJoint_states() const
{
    return joint_states;
}

TrajectoryFollow::TrajectoryFollow(ros::NodeHandle& nh)
{
    ROS_INFO("Object of TrajectoryFollow created");
    jointStateSub_ = nh.subscribe("joint_states",10, &TrajectoryFollow::jointsCallback, this);
    arm_client = new armClient("arm_controller/follow_joint_trajectory", true);
    base_client = new baseClient("move_base", true);
    gripper_client = new gripperClient("gripper_controller/gripper_action", true);
    outputs = MatrixXd::Random(6,6);
    nh_ = nh;
    joint_states.resize(6);
    ros::Duration(2).sleep();
}

TrajectoryFollow::~TrajectoryFollow()
{
     delete arm_client;
     delete base_client;
     delete gripper_client;
}

void TrajectoryFollow::startTrajectory(control_msgs::FollowJointTrajectoryGoal& arm_goal)
{
    arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    arm_client->sendGoal(arm_goal);
    arm_client->waitForResult(ros::Duration(10.0));
}

void TrajectoryFollow::startMoveBase(move_base_msgs::MoveBaseGoal& base_goal)
{
    base_goal.target_pose.header.stamp = ros::Time::now() + ros::Duration(1.0);
    base_client->sendGoal(base_goal);
}

void TrajectoryFollow::startGripperAction(control_msgs::GripperCommandGoal& gripper_goal)
{
    gripper_client->sendGoal(gripper_goal);
    gripper_client->waitForResult(ros::Duration(5.0));
}

actionlib::SimpleClientGoalState TrajectoryFollow::getArmState()
{
    return arm_client->getState();
}

actionlib::SimpleClientGoalState TrajectoryFollow::getBaseState()
{
    return base_client->getState();
}

actionlib::SimpleClientGoalState TrajectoryFollow::getGripperState()
{
    return gripper_client->getState();
}

void TrajectoryFollow::jointsCallback(const sensor_msgs::JointState& state)
{
//    if (!isJointStatePopulated) {

        for(unsigned int i = 6; i < 12; i++)
        {
            joint_states[i-6] = state.position[i];
           //ROS_INFO("State position id : %d = Value : %.2f", i, joint_states[i-6]);
        }
        isJointStatePopulated = true;
//    }
}

MatrixXd TrajectoryFollow::trajectory(const MatrixXd& joint_positions, float tf)
{
    MatrixXd M(6,6);
    M << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 2, 0, 0, 0,
         1, pow(tf,1), pow(tf,2), pow(tf,3), pow(tf,4), pow(tf,5),
         0, 1, 2*tf, 3*pow(tf,2), 4*pow(tf,3), 5*pow(tf,4),
         0, 0, 2, 6*tf, 12*pow(tf,2),  20*pow(tf,3);
    //ROS_INFO_STREAM("M:" << M << std::endl);
    MatrixXd inputs(6,6);
    inputs << joint_positions(0,0), 0, 0, joint_positions(1,0), 0, 0,
              joint_positions(0,1), 0, 0, joint_positions(1,1), 0, 0,
              joint_positions(0,2), 0, 0, joint_positions(1,2), 0, 0,
              joint_positions(0,3), 0, 0, joint_positions(1,3), 0, 0,
              joint_positions(0,4), 0, 0, joint_positions(1,4), 0, 0,
              joint_positions(0,5), 0, 0, joint_positions(1,5), 0, 0;

    inputs.transposeInPlace();
    std::cout << "miNVERSE:" << M.inverse() << std::endl;
    std::cout << "inputs:" << inputs << std::endl;
    MatrixXd outputs(6,6);
    outputs = (M.inverse())*inputs;
    std::cout << "outputs:" << outputs << std::endl;
    return outputs;
}

control_msgs::GripperCommandGoal TrajectoryFollow::gripperTrajectory()
{
    control_msgs::GripperCommandGoal gripper_goal;
    gripper_goal.command.max_effort = 10;
    gripper_goal.command.position = 0;

    return gripper_goal;
}

control_msgs::FollowJointTrajectoryGoal TrajectoryFollow::armExtensionTrajectory(const MatrixXd& input)
{
    control_msgs::FollowJointTrajectoryGoal arm_goal;
    arm_goal.trajectory.joint_names.push_back("shoulder_pan_joint");
    arm_goal.trajectory.joint_names.push_back("shoulder_lift_joint");
    arm_goal.trajectory.joint_names.push_back("upperarm_roll_joint");
    arm_goal.trajectory.joint_names.push_back("elbow_flex_joint");
    arm_goal.trajectory.joint_names.push_back("forearm_roll_joint");
    arm_goal.trajectory.joint_names.push_back("wrist_flex_joint");
    arm_goal.trajectory.joint_names.push_back("wrist_roll_joint");

    arm_goal.trajectory.points.resize(no_of_iterations);
    float t = 0;

    for(size_t i = 0; i < no_of_iterations; i++)
    {
    arm_goal.trajectory.points[i].positions.resize(7);
    arm_goal.trajectory.points[i].positions[0] = input(0,0) + input(1,0)*t + input(2,0)*pow(t,2) + input(3,0)*pow(t,3) + input(4,0)*pow(t,4) + input(5,0)*pow(t,5);
    arm_goal.trajectory.points[i].positions[1] = input(0,1) + input(1,1)*t + input(2,1)*pow(t,2) + input(3,1)*pow(t,3) + input(4,1)*pow(t,4) + input(5,1)*pow(t,5);
    arm_goal.trajectory.points[i].positions[2] = input(0,2) + input(1,2)*t + input(2,2)*pow(t,2) + input(3,2)*pow(t,3) + input(4,2)*pow(t,4) + input(5,2)*pow(t,5);
    arm_goal.trajectory.points[i].positions[3] = input(0,3) + input(1,3)*t + input(2,3)*pow(t,2) + input(3,3)*pow(t,3) + input(4,3)*pow(t,4) + input(5,3)*pow(t,5);
    arm_goal.trajectory.points[i].positions[4] = input(0,4) + input(1,4)*t + input(2,4)*pow(t,2) + input(3,4)*pow(t,3) + input(4,4)*pow(t,4) + input(5,4)*pow(t,5);
    arm_goal.trajectory.points[i].positions[5] = input(0,5) + input(1,5)*t + input(2,5)*pow(t,2) + input(3,5)*pow(t,3) + input(4,5)*pow(t,4) + input(5,5)*pow(t,5);
    arm_goal.trajectory.points[i].positions[6] = 0;

    arm_goal.trajectory.points[i].velocities.resize(7);

    arm_goal.trajectory.points[i].velocities[0] = input(1,0) + 2*input(2,0)*pow(t,1) + 3*input(3,0)*pow(t,2) + 4*input(4,0)*pow(t,3) + 5*input(5,0)*pow(t,4);
    arm_goal.trajectory.points[i].velocities[1] = input(1,1) + 2*input(2,1)*pow(t,1) + 3*input(3,1)*pow(t,2) + 4*input(4,1)*pow(t,3) + 5*input(5,1)*pow(t,4);
    arm_goal.trajectory.points[i].velocities[2] = input(1,2) + 2*input(2,2)*pow(t,1) + 3*input(3,2)*pow(t,2) + 4*input(4,2)*pow(t,3) + 5*input(5,2)*pow(t,4);
    arm_goal.trajectory.points[i].velocities[3] = input(1,3) + 2*input(2,3)*pow(t,1) + 3*input(3,3)*pow(t,2) + 4*input(4,3)*pow(t,3) + 5*input(5,3)*pow(t,4);
    arm_goal.trajectory.points[i].velocities[4] = input(1,4) + 2*input(2,4)*pow(t,1) + 3*input(3,4)*pow(t,2) + 4*input(4,4)*pow(t,3) + 5*input(5,4)*pow(t,4);
    arm_goal.trajectory.points[i].velocities[5] = input(1,5) + 2*input(2,5)*pow(t,1) + 3*input(3,5)*pow(t,2) + 4*input(4,5)*pow(t,3) + 5*input(5,5)*pow(t,4);
    arm_goal.trajectory.points[i].velocities[6] = 0;

    arm_goal.trajectory.points[i].accelerations.resize(7);

    arm_goal.trajectory.points[i].accelerations[0] = 2*input(2,0) + 6*input(3,0)*pow(t,1) + 12*input(4,0)*pow(t,2) + 20*input(5,0)*pow(t,3);
    arm_goal.trajectory.points[i].accelerations[1] = 2*input(2,1) + 6*input(3,1)*pow(t,1) + 12*input(4,1)*pow(t,2) + 20*input(5,1)*pow(t,3);
    arm_goal.trajectory.points[i].accelerations[2] = 2*input(2,2) + 6*input(3,2)*pow(t,1) + 12*input(4,2)*pow(t,2) + 20*input(5,2)*pow(t,3);
    arm_goal.trajectory.points[i].accelerations[3] = 2*input(2,3) + 6*input(3,3)*pow(t,1) + 12*input(4,3)*pow(t,2) + 20*input(5,3)*pow(t,3);
    arm_goal.trajectory.points[i].accelerations[4] = 2*input(2,4) + 6*input(3,4)*pow(t,1) + 12*input(4,4)*pow(t,2) + 20*input(5,4)*pow(t,3);
    arm_goal.trajectory.points[i].accelerations[5] = 2*input(2,5) + 6*input(3,5)*pow(t,1) + 12*input(4,5)*pow(t,2) + 20*input(5,5)*pow(t,3);
    arm_goal.trajectory.points[i].accelerations[6] = 0;

    arm_goal.trajectory.points[i].time_from_start = ros::Duration(1 + 2*t);

    t = t + float(t_max)/no_of_iterations;
    }
    return arm_goal;
}

move_base_msgs::MoveBaseGoal TrajectoryFollow::baseMove()
{
    move_base_msgs::MoveBaseGoal base_goal;

    base_goal.target_pose.pose.position.x = 10.0;
    base_goal.target_pose.pose.orientation.w = 1.0;
    //base_goal.target_pose.header.frame_id = 'first';
    base_goal.target_pose.header.stamp = ros::Time::now();
    return base_goal;
}

double TrajectoryFollow::fRand(double min, double max)
{
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
}


bool TrajectoryFollow::solve_ik(double num_samples, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param, KDL::JntArray &result)
{

    double eps = 1e-5;

    // This constructor parses the URDF loaded in rosparm urdf_param into the
    // needed KDL structures.  We then pull these out to compare against the KDL
    // IK solver.
    TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

    KDL::Chain chain;
    KDL::JntArray ll, ul; //lower joint limits, upper joint limits

    bool valid = tracik_solver.getKDLChain(chain);

    if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
        return 0;
    }

    valid = tracik_solver.getKDLLimits(ll,ul);

    if (!valid) {
        ROS_ERROR("There were no valid KDL joint limits found");
        return 0;
    }

    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());

    ROS_INFO ("Using %d joints",chain.getNrOfJoints());


    // Set up KDL IK
    KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
    KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
    KDL::ChainIkSolverPos_NR_JL kdl_solver(chain,ll,ul,fk_solver, vik_solver, 1, eps); // Joint Limit Solver
    // 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK)


    // Create Nominal chain configuration midway between all joint limits
    KDL::JntArray nominal(chain.getNrOfJoints());

    for (uint j=0; j<nominal.data.size(); j++) {
        nominal(j) = (ll(j)+ul(j))/2.0;
    }

    //Create desired number of valid, random joint configurations
    std::vector<KDL::JntArray> JointList;
    KDL::JntArray q(chain.getNrOfJoints());

    for (uint i=0; i < num_samples; i++) {
        for (uint j=0; j<ll.data.size(); j++) {
            q(j)=fRand(ll(j), ul(j));
        }
        JointList.push_back(q);
    }

//    KDL::JntArray result;
    KDL::Frame end_effector_pose;
    int rc;

    end_effector_pose.M = KDL::Rotation::Quaternion(0, 0, 0, 1.0);

    end_effector_pose.p = KDL::Vector(0.1, 0.1, 0.1);



    ROS_INFO_STREAM("*** Testing TRAC-IK with "<<num_samples<<" random samples");

    for (uint i=0; i < num_samples; i++) {
        //fk_solver.JntToCart(JointList[i],end_effector_pose);
        rc=tracik_solver.CartToJnt(nominal,end_effector_pose,result);
    }

    ROS_INFO_STREAM("Result data - " << result.data);
    return 1;
}

