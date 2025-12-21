#include "gcopter/gcopter_planner.h"
#include "rog_map/rog_map.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "test_planner");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    rog_map::ROGMap::Ptr rog_map_ptr = std::make_shared<rog_map::ROGMap>(nh);

    rog_map::GCOPTERPlanner gcopter_planner(nh, rog_map_ptr.get());

    /* Publisher and subcriber */
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();


    ros::waitForShutdown();
    return 0;
}