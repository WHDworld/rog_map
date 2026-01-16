cmake_minimum_required(VERSION 2.8.3)
project(rog_map)

# 使用 C++17 以支持结构化绑定等特性
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=return-type)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

add_definitions(-DORIGIN_AT_CORNER)
#add_definitions(-DORIGIN_AT_CENTER)



find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        geometry_msgs
        nav_msgs
        message_generation
        visualization_msgs
        message_filters
        sensor_msgs
        quadrotor_msgs    # GCOPTER 轨迹服务器需要
        mavros_msgs       # PX4 模式切换和解锁
        tf                # 姿态转换
        cv_bridge         # [Added]
        image_transport   # [Added]
        decomp_ros_msgs
        decomp_ros_utils
        vision_msgs
        )
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(ompl REQUIRED)
find_package(NLopt REQUIRED)  # B-spline 优化器依赖
find_package(OpenCV REQUIRED) # [Added]

#add dynamic reconfigure api
set (DYNAMIC_RECONFIGURE_PATH config/visualization.cfg)
execute_process(COMMAND chmod a+x ${DYNAMIC_RECONFIGURE_PATH}
      		  	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
      		  	RESULT_VARIABLE cmd_result
      		 	OUTPUT_VARIABLE cmd_ver)
message(STATUS "Chmod a+x the dynamic_reconfigure file")

generate_dynamic_reconfigure_options(
        ${DYNAMIC_RECONFIGURE_PATH}
)

# 添加消息文件
add_message_files(
        FILES
        Bspline.msg
        PolyTraj.msg
)

# 生成消息
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES rog_map
        CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        include/ego_planner   # EGO Planner 头文件
        ../../../devel/include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OMPL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS} # [Added]

)

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OMPL_LIBRARIES}     # GCOPTER/OMPL 依赖
        ${OpenCV_LIBRARIES}   # [Added]
        -ldw
        )

file(GLOB_RECURSE lib_srcs 
        CONFIGURE_DEPENDS 
        src/rog_map/*.cpp 
        src/star_mvf/*.cpp           # STAR-MVF 新地图模块

        include/*.h 
        include/*.hpp 
        include/*.cpp
    )

# 排除旧的地图实现（已被 star_mvf 取代）
list(FILTER lib_srcs EXCLUDE REGEX ".*/rog_map/global_map\\.cpp$")
list(FILTER lib_srcs EXCLUDE REGEX ".*map_adapter\\.cpp$")
list(FILTER lib_srcs EXCLUDE REGEX ".*/rog_map/rog_map\\.cpp$")
list(FILTER lib_srcs EXCLUDE REGEX ".*/rog_map/prob_map\\.cpp$")
list(FILTER lib_srcs EXCLUDE REGEX ".*/rog_map/sliding_map\\.cpp$")
list(FILTER lib_srcs EXCLUDE REGEX ".*/rog_map/inf_map\\.cpp$")
list(FILTER lib_srcs EXCLUDE REGEX ".*/rog_map/counter_map\\.cpp$")
list(FILTER lib_srcs EXCLUDE REGEX ".*/rog_map/esdf_map\\.cpp$")
list(FILTER lib_srcs EXCLUDE REGEX ".*/rog_map/unified_map\\.cpp$")

add_library(rog_map STATIC ${lib_srcs})
add_dependencies(rog_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(rog_map PUBLIC include)
target_link_libraries(rog_map PUBLIC ${THIRD_PARTY})

# ===========================================================
# EGO Planner (MINCO Trajectory) 规划器
# ===========================================================
# EGO Planner 库
set(EGO_PLANNER_SRCS
        src/ego_planner/rog_astar.cpp
        src/ego_planner/poly_traj_optimizer.cpp
        src/ego_planner/ego_planner_manager.cpp
        src/ego_planner/ego_replan_fsm.cpp
        src/ego_planner/yaw_optimizer.cpp
)

add_library(ego_planner_lib STATIC ${EGO_PLANNER_SRCS})
add_dependencies(ego_planner_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(ego_planner_lib PUBLIC include include/ego_planner)
target_link_libraries(ego_planner_lib PUBLIC 
        rog_map 
        ${THIRD_PARTY} 
        ${catkin_LIBRARIES}
)

# EGO Planner 规划节点
add_executable(ego_planner_node src/ego_planner/ego_planner_node.cpp)
add_dependencies(ego_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ego_planner_node 
        ego_planner_lib
        rog_map
        ${THIRD_PARTY}
        ${catkin_LIBRARIES}
)

# EGO Planner 轨迹服务器
add_executable(ego_traj_server src/ego_planner/ego_traj_server.cpp)
add_dependencies(ego_traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ego_traj_server 
        rog_map
        ${THIRD_PARTY}
        ${catkin_LIBRARIES}
)

# ===========================================================
# 探索测试节点 (Exploration FSM with Visualization)
# ===========================================================
add_executable(exploration_node src/rog_map/exploration_node.cpp)
add_dependencies(exploration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(exploration_node 
        rog_map
        ${THIRD_PARTY}
        ${catkin_LIBRARIES}
)

# ===========================================================
# 统一探索规划节点 (EGO-Planner + Exploration FSM, 共享ROG-Map)
# ===========================================================
add_executable(exploration_planner_node src/rog_map/exploration_planner_node.cpp)
add_dependencies(exploration_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(exploration_planner_node 
        ego_planner_lib
        rog_map
        ${THIRD_PARTY}
        ${catkin_LIBRARIES}
)

# Map Test Node - 单独测试地图同步
add_executable(map_test_node src/star_mvf/map_test_node.cpp)
add_dependencies(map_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_test_node 
        rog_map
        ${THIRD_PARTY}
        ${catkin_LIBRARIES}
)

# ===========================================================
# 新架构探索规划节点 (4-State FSM + Integrated Planner)
# ===========================================================
add_executable(exploration_planner_node_new 
        src/rog_map/exploration_planner_node_new.cpp
        src/rog_map/exploration_fsm_new.cpp
        src/rog_map/viewpoint_manager.cpp
        src/rog_map/history_graph.cpp
)
add_dependencies(exploration_planner_node_new ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(exploration_planner_node_new 
        ego_planner_lib
        rog_map
        ${THIRD_PARTY}
        ${catkin_LIBRARIES}
)