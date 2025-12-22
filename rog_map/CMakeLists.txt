cmake_minimum_required(VERSION 2.8.3)
project(rog_map)

set(CMAKE_CXX_STANDARD 14)
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
        quadrotor_msgs    # GCOPTER 轨迹服务器需要
        mavros_msgs       # PX4 模式切换和解锁
        tf                # 姿态转换
        )
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(ompl REQUIRED)  # GCOPTER 依赖

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

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES rog_map
        CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        include/gcopter      # GCOPTER 头文件
        ../../../devel/include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OMPL_INCLUDE_DIRS}  # OMPL 头文件
)

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OMPL_LIBRARIES}     # GCOPTER/OMPL 依赖
        -ldw
        )

file(GLOB_RECURSE lib_srcs 
        CONFIGURE_DEPENDS 
        src/rog_map/*.cpp 
        src/gcopter/*.cpp 
        include/*.h 
        include/*.hpp 
        include/*.cpp
    )

add_library(rog_map STATIC ${lib_srcs})
target_include_directories(rog_map PUBLIC include)
target_link_libraries(rog_map PUBLIC ${THIRD_PARTY})

add_executable(test_planner src/test_explore/test_planner.cpp)
target_link_libraries(test_planner 
        rog_map
        ${THIRD_PARTY}
        ${catkin_LIBRARIES}
        )

# GCOPTER 规划节点
add_executable(gcopter_node src/gcopter/gcopter_node.cpp)
add_dependencies(gcopter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gcopter_node 
        rog_map
        ${THIRD_PARTY}
        ${catkin_LIBRARIES}
        )