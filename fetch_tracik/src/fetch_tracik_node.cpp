#include "fetch_tracik.cpp"

int main(int argc, char** argv)
{
    srand(1);
    ros::init(argc, argv, "fetch_tracik_node");
    ros::NodeHandle nh("~");


    Fetchik ik_solver(nh);

    KDL::JntArray result;

    bool loop = 1;
    int num_samples;
    std::string chain_start, chain_end, urdf_param;
    double timeout;

    nh.param("num_samples", num_samples, 1);
    nh.param("chain_start", chain_start, std::string("shoulder_pan_link"));
    nh.param("chain_end", chain_end, std::string("wrist_flex_link"));

    if (chain_start=="" || chain_end=="") {
        ROS_FATAL("Missing chain info in launch file");
        exit (-1);
    }

    nh.param("timeout", timeout, 0.005);
    nh.param("urdf_param", urdf_param, std::string("/robot_description"));

    if (num_samples < 1)
        num_samples = 1;

    ik_solver.solve_ik(num_samples, chain_start, chain_end, timeout, urdf_param, result);

  
    return 0;
}
