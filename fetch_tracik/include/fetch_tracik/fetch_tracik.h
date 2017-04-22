# pragma once

#include <iostream>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>


class Fetchik{

private:

	ros::NodeHandle nh_;
	
public:

	Fetchik(ros::NodeHandle& nh);
	
	~Fetchik();
	
	bool solve_ik(double num_samples, std::string chain_start, std::string chain_end, double timeout, std::string, KDL::JntArray &result);
        
    double fRand(double min, double max);
};

