#include "fast_planner/planner_manager.h"
#include "fast_planner/planning_fsm.h"
#include "rog_map/rog_map.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "test_planner");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // Initialize map
    rog_map::ROGMap::Ptr rog_map_ptr = std::make_shared<rog_map::ROGMap>(nh);

    // Initialize FSM (which initializes planner manager internally)
    fast_planner::PlanningFSM fsm;
    fsm.init(nh, rog_map_ptr);

    /* Publisher and subcriber */
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();

    ros::waitForShutdown();
    return 0;
}
