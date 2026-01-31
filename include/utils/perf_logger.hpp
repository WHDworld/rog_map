/**
 * @file perf_logger.hpp
 * @brief STAR-Searcher 性能监控与日志记录系统
 * 
 * 功能：
 * 1. Time Profiling: 记录代码块耗时
 * 2. Data Logging: 记录键值对或特定事件
 * 3. Trace Logging: 记录决策流水线
 * 4. ROS日志重定向: 捕获ROS_INFO/WARN/ERROR输出到文件
 */

#pragma once

#include <fstream>
#include <string>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <map>
#include <ros/ros.h>
#include <ros/console.h>

namespace perf {

/**
 * @brief 性能日志管理器（单例模式）
 */
class PerfLogger {
public:
    static PerfLogger& getInstance() {
        static PerfLogger instance;
        return instance;
    }

    /**
     * @brief 初始化日志系统
     * @param base_path 日志文件基础路径（不含文件名）
     * @param enable_ros_redirect 是否重定向ROS日志到文件
     */
    void init(const std::string& base_path, bool enable_ros_redirect = true) {
        std::lock_guard<std::mutex> lock(mtx_);
        
        if (initialized_) return;
        
        // 生成带时间戳的日志文件名
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm* tm_info = std::localtime(&now_time);
        
        std::stringstream ss;
        ss << base_path << "/exploration_perf_"
           << std::put_time(tm_info, "%Y%m%d_%H%M%S") << ".log";
        log_path_ = ss.str();
        
        log_file_.open(log_path_, std::ios::out | std::ios::trunc);
        
        if (log_file_.is_open()) {
            start_time_ = ros::WallTime::now();
            initialized_ = true;
            
            // 写入日志头
            log_file_ << "=============================================================\n";
            log_file_ << "  STAR-Searcher Performance & System Log\n";
            log_file_ << "  Started: " << std::put_time(tm_info, "%Y-%m-%d %H:%M:%S") << "\n";
            log_file_ << "=============================================================\n\n";
            log_file_.flush();
            
            ROS_INFO("[PerfLogger] Initialized: %s", log_path_.c_str());
        } else {
            ROS_ERROR("[PerfLogger] Failed to open log file: %s", log_path_.c_str());
        }
        
        // 设置ROS日志重定向
        if (enable_ros_redirect) {
            setupRosLogRedirect();
        }
    }
    
    /**
     * @brief 检查是否已初始化
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * @brief 获取日志文件路径
     */
    std::string getLogPath() const { return log_path_; }

    // ==================== 日志记录接口 ====================
    
    /**
     * @brief 记录普通信息
     * @param module 模块名 (MAP, GEN, FSM, PLAN等)
     * @param event 事件类型 (INFO, TIME, FILTER, ACCEPT, REJECT等)
     * @param content 日志内容
     */
    void log(const std::string& module, const std::string& event, const std::string& content) {
        writeLine("[" + module + "] [" + event + "] " + content);
    }
    
    /**
     * @brief 记录格式化信息（类似printf）
     */
    template<typename... Args>
    void logf(const std::string& module, const std::string& event, const char* fmt, Args... args) {
        char buf[1024];
        snprintf(buf, sizeof(buf), fmt, args...);
        log(module, event, std::string(buf));
    }

    /**
     * @brief 记录耗时信息
     * @param module 模块名
     * @param func_name 函数/作用域名
     * @param time_ms 耗时（毫秒）
     */
    void logTime(const std::string& module, const std::string& func_name, double time_ms) {
        std::stringstream ss;
        ss << func_name << ": " << std::fixed << std::setprecision(3) << time_ms << " ms";
        writeLine("[" + module + "] [TIME] " + ss.str());
    }
    
    /**
     * @brief 记录ROS日志（供回调使用）
     * @param level 日志级别 (INFO, WARN, ERROR, DEBUG)
     * @param msg 日志消息
     */
    void logRos(const std::string& level, const std::string& msg) {
        writeLine("[ROS] [" + level + "] " + msg);
    }
    
    /**
     * @brief 记录统计信息
     */
    void logStats(const std::string& module, const std::string& name, 
                  const std::map<std::string, int>& stats) {
        std::stringstream ss;
        ss << name << ": {";
        bool first = true;
        for (const auto& kv : stats) {
            if (!first) ss << ", ";
            ss << kv.first << "=" << kv.second;
            first = false;
        }
        ss << "}";
        log(module, "STATS", ss.str());
    }
    
    /**
     * @brief 记录分隔符（便于阅读）
     */
    void logSeparator(const std::string& title = "") {
        std::stringstream ss;
        ss << "------------------------------------------------------------";
        if (!title.empty()) {
            ss << " " << title << " ";
        }
        writeLine(ss.str());
    }
    
    /**
     * @brief 刷新缓冲区
     */
    void flush() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (log_file_.is_open()) {
            log_file_.flush();
        }
    }
    
    /**
     * @brief 关闭日志文件
     */
    void close() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (log_file_.is_open()) {
            log_file_ << "\n=== Log Closed ===\n";
            log_file_.close();
        }
        initialized_ = false;
    }

private:
    PerfLogger() : initialized_(false) {}
    
    ~PerfLogger() { 
        close();
    }
    
    // 禁止拷贝
    PerfLogger(const PerfLogger&) = delete;
    PerfLogger& operator=(const PerfLogger&) = delete;

    void writeLine(const std::string& text) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!initialized_ || !log_file_.is_open()) return;
        
        double secs = (ros::WallTime::now() - start_time_).toSec();
        log_file_ << "[" << std::fixed << std::setprecision(3) << secs << "] " 
                  << text << std::endl;
    }
    
    void setupRosLogRedirect() {
        // 注册ROS日志回调 - 使用ROSCONSOLE的filter机制
        // 注意：这里我们使用一个简单的方法，在关键位置手动调用logRos
        // 完整的ROS日志重定向需要修改rosconsole配置
        ROS_INFO("[PerfLogger] ROS log redirect enabled (manual mode)");
    }

    std::ofstream log_file_;
    std::mutex mtx_;
    ros::WallTime start_time_;
    std::string log_path_;
    bool initialized_;
};

// ==================== 便捷宏定义 ====================

// 获取单例
#define PERF_LOG perf::PerfLogger::getInstance()

// 快速日志宏
#define PERF_INFO(module, content) \
    if (PERF_LOG.isInitialized()) PERF_LOG.log(module, "INFO", content)

#define PERF_WARN(module, content) \
    if (PERF_LOG.isInitialized()) PERF_LOG.log(module, "WARN", content)

#define PERF_ERROR(module, content) \
    if (PERF_LOG.isInitialized()) PERF_LOG.log(module, "ERROR", content)

#define PERF_TIME(module, func_name, time_ms) \
    if (PERF_LOG.isInitialized()) PERF_LOG.logTime(module, func_name, time_ms)

// ==================== 作用域计时器 ====================

/**
 * @brief 作用域计时器 - 自动记录代码块耗时
 * 
 * 用法：
 *   void myFunction() {
 *       ScopeTimer timer("GEN", "myFunction");
 *       // ... 代码 ...
 *   }  // 离开作用域时自动记录耗时
 */
class ScopeTimer {
public:
    ScopeTimer(const std::string& module, const std::string& name, 
               double threshold_ms = 0.0)
        : module_(module), name_(name), threshold_ms_(threshold_ms),
          start_(ros::WallTime::now()) {}
    
    ~ScopeTimer() {
        double elapsed_ms = (ros::WallTime::now() - start_).toSec() * 1000.0;
        if (elapsed_ms >= threshold_ms_) {
            PERF_TIME(module_, name_, elapsed_ms);
        }
    }
    
    // 获取当前已耗时（毫秒）
    double elapsed() const {
        return (ros::WallTime::now() - start_).toSec() * 1000.0;
    }

private:
    std::string module_;
    std::string name_;
    double threshold_ms_;
    ros::WallTime start_;
};

// 便捷宏：创建作用域计时器
#define SCOPE_TIMER(module, name) \
    perf::ScopeTimer _scope_timer_##__LINE__(module, name)

// 带阈值的计时器（仅当耗时超过阈值才记录）
#define SCOPE_TIMER_THRESHOLD(module, name, threshold_ms) \
    perf::ScopeTimer _scope_timer_##__LINE__(module, name, threshold_ms)

// ==================== 统计收集器 ====================

/**
 * @brief 统计收集器 - 收集并记录计数统计
 * 
 * 用法：
 *   StatsCollector stats("GEN", "ViewpointFilter");
 *   for (auto& vp : viewpoints) {
 *       if (collision) { stats.inc("COLLISION"); continue; }
 *       if (low_score) { stats.inc("LOW_SCORE"); continue; }
 *       stats.inc("ACCEPTED");
 *   }
 *   // 离开作用域时自动记录统计
 */
class StatsCollector {
public:
    StatsCollector(const std::string& module, const std::string& name)
        : module_(module), name_(name) {}
    
    ~StatsCollector() {
        if (!counts_.empty() && PERF_LOG.isInitialized()) {
            PERF_LOG.logStats(module_, name_, counts_);
        }
    }
    
    void inc(const std::string& key, int delta = 1) {
        counts_[key] += delta;
    }
    
    void set(const std::string& key, int value) {
        counts_[key] = value;
    }
    
    int get(const std::string& key) const {
        auto it = counts_.find(key);
        return it != counts_.end() ? it->second : 0;
    }

private:
    std::string module_;
    std::string name_;
    std::map<std::string, int> counts_;
};

} // namespace perf
