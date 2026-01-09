/**
 * @file ros_log_redirect.hpp
 * @brief ROS 日志重定向工具
 * 
 * 将 ROS_INFO/WARN/ERROR 输出同时写入日志文件
 */

#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include "utils/perf_logger.hpp"

namespace perf {

/**
 * @brief ROS console 日志重定向后端
 * 
 * 通过 ros::console::register_appender 注册自定义日志输出
 */
class RosLogAppender : public ros::console::LogAppender {
public:
    virtual void log(::ros::console::Level level,
                     const char* str,
                     const char* file,
                     const char* function,
                     int line) override {
        // 转换日志级别
        const char* level_str = "INFO";
        switch (level) {
            case ros::console::levels::Debug:
                level_str = "DEBUG";
                break;
            case ros::console::levels::Info:
                level_str = "INFO";
                break;
            case ros::console::levels::Warn:
                level_str = "WARN";
                break;
            case ros::console::levels::Error:
                level_str = "ERROR";
                break;
            case ros::console::levels::Fatal:
                level_str = "FATAL";
                break;
            default:
                break;
        }
        
        // 写入日志文件
        if (PerfLogger::getInstance().isInitialized()) {
            PerfLogger::getInstance().logRos(level_str, str);
        }
    }
};

/**
 * @brief 初始化 ROS 日志重定向
 * 
 * 调用此函数后，所有 ROS_INFO/WARN/ERROR 将同时输出到日志文件
 */
inline void initRosLogRedirect() {
    static RosLogAppender appender;
    ros::console::register_appender(&appender);
    ROS_INFO("[RosLogRedirect] ROS log redirect initialized");
}

} // namespace perf
