#pragma once

#include <string>
#include <mutex>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <dji_logger.h>
#include "common_types.h"

// 本机UDP绑定配置
struct UdpBindConfig {
    std::string ip;
    uint16_t    port;
};

// 地面站配置
struct GroundStationConfig {
    std::string ip;
    uint16_t    port;
};

// 广播配置
struct BroadcastConfig {
    std::string ip;
    uint16_t    port;
};

class UavNetConfig final {
public:
    // 单例入口（Meyers Singleton）
    static UavNetConfig& instance();

    // 从文件加载配置（成功返回 true；失败保留默认/已有值）
    bool loadFromFile(const std::string& path);

    // 重新加载（基于上一次的路径）
    bool reload();

    // 便捷获取（★原有接口保持不变）
    UdpBindConfig        localBind() const;           // {bind_ip, bind_port}
    GroundStationConfig  groundStation() const;       // {ip, port}
    std::string          bindIp() const;
    uint16_t             bindPort() const;
    std::string          groundIp() const;
    uint16_t             groundPort() const;
    uint32_t             uavId() const;

    // 新增：广播配置获取（不影响现有调用）
    BroadcastConfig      broadcast() const;
    std::string          broadcastIp() const;
    uint16_t             broadcastPort() const;

    // 导出当前配置（便于调试/观测）
    nlohmann::json toJson() const;

    // 默认值
    static constexpr const char* kDefaultBindIp   = "0.0.0.0";
    static constexpr uint16_t    kDefaultBindPort = 50000;
    static constexpr const char* kDefaultGsIp     = "192.168.0.250";
    static constexpr uint16_t    kDefaultGsPort   = 60000;
    static constexpr uint32_t    kDefaultUavId    = 1;

    // 新增：广播默认值（按你的需求设置为 192.168.0.255:50732）
    static constexpr const char* kDefaultBcastIp   = "192.168.0.255";
    static constexpr uint16_t    kDefaultBcastPort = 50732;

private:
    UavNetConfig();
    ~UavNetConfig() = default;

    UavNetConfig(const UavNetConfig&) = delete;
    UavNetConfig& operator=(const UavNetConfig&) = delete;

    // 解析并写入内部状态（要求持锁后调用）
    void applyJsonLocked(const nlohmann::json& j);

private:
    mutable std::mutex mtx_;
    std::string last_path_;

    // 本机UDP绑定
    std::string bind_ip_;
    uint16_t    bind_port_;

    // 地面站
    std::string gs_ip_;
    uint16_t    gs_port_;

    // 广播
    std::string bcast_ip_;
    uint16_t    bcast_port_;

    // 无人机编号
    uint32_t    uav_id_;
};
