/**
 * @file star_mvf.h
 * @brief STAR-MVF 统一头文件
 * 
 * 包含所有必要的头文件和命名空间别名，便于使用
 */

#ifndef STAR_MVF_H
#define STAR_MVF_H

// 地图模块
#include "map/map_types.h"
#include "map/local_map.h"
#include "map/global_map.h"
#include "map/map_manager.h"

// 为了向后兼容，提供 rog_map 命名空间别名
// 注意：rog_map 命名空间中的基础类型（Vec3f, Vec3i, GridType等）
// 已在 common_lib.hpp 中定义，这里只添加新的类型
namespace rog_map {
    // star_mvf 特有的类型别名（不与旧类型冲突）
    using StarMvfPosition = star_mvf::Position;
    using StarMvfQuaternion = star_mvf::Quaternion;
    
    using VoxelIndex = star_mvf::VoxelIndex;
    using GlobalVoxelIndex = star_mvf::GlobalVoxelIndex;
    using VoxelIndexHash = star_mvf::VoxelIndexHash;
    using GlobalVoxelIndexHash = star_mvf::GlobalVoxelIndexHash;
    
    using VoxelState = star_mvf::VoxelState;
    using GlobalVoxelState = star_mvf::GlobalVoxelState;
    
    using SubGridIndex = star_mvf::SubGridIndex;
    using SubGrid = star_mvf::SubGrid;
    using SubGridFrontierInfo = star_mvf::SubGridFrontierInfo;
    
    // 配置类型别名（使用新名称避免冲突）
    using StarMvfInflationConfig = star_mvf::InflationConfig;
    using StarMvfESDFConfig = star_mvf::ESDFConfig;
    using StarMvfRobotState = star_mvf::RobotState;
    using StarMvfGridType = star_mvf::GridType;
    
    // 类别名
    using GlobalMap = star_mvf::GlobalMap;
    using LocalMap = star_mvf::LocalMap;
    using MapManager = star_mvf::MapManager;
    
    // VoxelStateBits 命名空间
    namespace VoxelStateBits = star_mvf::VoxelStateBits;
}

#endif  // STAR_MVF_H

