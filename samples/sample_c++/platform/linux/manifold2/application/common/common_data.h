#include <cstdint>
#include <atomic>
#include <cmath>

inline uint8_t g_drone_id = 0;      // 无人机自身id，默认置为0
inline uint8_t g_host_drone_id = 0; // 主机无人机id

// 主机位置
inline float g_host_latitude_deg = 0.0f;   ///< 主机GPS纬度 [deg]
inline float g_host_longitude_deg = 0.0f;  ///< 主机GPS经度 [deg]
inline float g_host_altitude_fused = 0.0f; ///< 主机GPS高度 [m]
inline float g_host_height_fusion = 0.0f;  ///< 主机离地高度 [m]

// 主机确认标识
inline bool g_host_confirmed = false; // 主机确认标识，默认未确认

// 相对主机偏移量
inline float g_offset_x = 0.0f; // 相对主机偏移量X [m]
inline float g_offset_y = 0.0f; // 相对主机偏移
inline float g_offset_z = 0.0f; // 相对主机偏移量Z [m]

// 原子变量，应该进行编队和应该停止编队
inline std::atomic<bool> g_should_follow_start{false};
inline std::atomic<bool> g_should_follow_stop{true};

// =========== 1. 角色 & 状态定义 =============
enum class DroneRole {
    Unknown,
    Master,
    Slave
};

enum class DroneState {
    WaitRoleSet,        // 初始：等待主/从角色确定
    Idle,               // 主机休息
    WaitFormationStart, // 从机等待编队开始
    InFormation         // 从机编队中
};

// =========== 2. 全局 / 单例上下文 ============
inline std::atomic<DroneRole>  g_role{DroneRole::Unknown};
inline std::atomic<DroneState> g_state{DroneState::WaitRoleSet};

inline DroneRole queryCurrentRole(){
    return g_role.load();
}
inline bool isHostReady() {
    return g_host_confirmed;
}
inline uint8_t getHostDroneId() {
    return g_host_drone_id;
}
inline bool isOffsetReliable() {
    return (std::abs(g_offset_x) > 5.0f &&
            std::abs(g_offset_y) > 5.0f &&
            std::abs(g_offset_z) > 5.0f);
}
inline bool shouldFollowStart() {
    return g_should_follow_start.load();
    // 测试时直接
    // return true; // 强制开启编队
}
inline bool shouldFollowStop() {
    return g_should_follow_stop.load();
}