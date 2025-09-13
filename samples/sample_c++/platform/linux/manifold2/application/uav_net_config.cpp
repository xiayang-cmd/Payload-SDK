#include "uav_net_config.h"
#include <fstream>
#include <limits>

UavNetConfig& UavNetConfig::instance() {
    static UavNetConfig inst;  // Meyers Singleton
    return inst;
}

UavNetConfig::UavNetConfig()
    : bind_ip_(kDefaultBindIp),
      bind_port_(kDefaultBindPort),
      gs_ip_(kDefaultGsIp),
      gs_port_(kDefaultGsPort),
      bcast_ip_(kDefaultBcastIp),
      bcast_port_(kDefaultBcastPort),
      uav_id_(kDefaultUavId) {}

bool UavNetConfig::loadFromFile(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        USER_LOG_WARN("无法打开配置文件: %s，使用默认/现有配置。", path.c_str());
        std::lock_guard<std::mutex> lk(mtx_);
        last_path_ = path; // 记录路径，方便 reload
        return false;
    }

    try {
        nlohmann::json j;
        f >> j;

        {
            std::lock_guard<std::mutex> lk(mtx_);
            last_path_ = path;

            // 每次加载先恢复默认，再按配置覆盖，确保缺失字段有回退
            bind_ip_   = kDefaultBindIp;
            bind_port_ = kDefaultBindPort;
            gs_ip_     = kDefaultGsIp;
            gs_port_   = kDefaultGsPort;
            bcast_ip_  = kDefaultBcastIp;
            bcast_port_= kDefaultBcastPort;
            uav_id_    = kDefaultUavId;

            applyJsonLocked(j);
        }

        USER_LOG_INFO("配置加载完成：local=%s:%u, ground=%s:%u, broadcast=%s:%u, uav_id=%u",
                      bindIp().c_str(), bindPort(),
                      groundIp().c_str(), groundPort(),
                      broadcastIp().c_str(), broadcastPort(),
                      static_cast<unsigned>(uavId()));
        return true;
    } catch (const nlohmann::json::exception& e) {
        USER_LOG_WARN("配置文件解析失败: %s，使用默认/现有配置。", e.what());
        std::lock_guard<std::mutex> lk(mtx_);
        last_path_ = path;
        return false;
    }
}

bool UavNetConfig::reload() {
    std::string path;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        path = last_path_;
    }
    if (path.empty()) {
        USER_LOG_WARN("尚未加载过配置文件，无法 reload。");
        return false;
    }
    return loadFromFile(path);
}

void UavNetConfig::applyJsonLocked(const nlohmann::json& j) {
    // local 节点：支持 { "local": { "bind_ip"/"ip", "bind_port"/"port" } }
    if (j.contains("local") && j["local"].is_object()) {
        const auto& jl = j["local"];
        bind_ip_ = jl.value("bind_ip", jl.value("ip", bind_ip_));
        int p = jl.value("bind_port", jl.value("port", static_cast<int>(bind_port_)));
        if (p >= 0 && p <= 65535) bind_port_ = static_cast<uint16_t>(p);
        else USER_LOG_WARN("本机UDP端口不在范围（0~65535）：%d，保持为 %u", p, bind_port_);
    } else {
        // 兜底（也支持把字段误放到根节点）
        bind_ip_ = j.value("bind_ip", j.value("ip", bind_ip_));
        int p = j.value("bind_port", j.value("port", static_cast<int>(bind_port_)));
        if (p >= 0 && p <= 65535) bind_port_ = static_cast<uint16_t>(p);
        else USER_LOG_WARN("本机UDP端口不在范围（0~65535）：%d，保持为 %u", p, bind_port_);
    }

    // ground 节点：{ "ground": { "ip", "port" } }，并兼容根级 gs_ip/gs_port/ground_ip/ground_port
    if (j.contains("ground") && j["ground"].is_object()) {
        const auto& g = j["ground"];
        gs_ip_ = g.value("ip", gs_ip_);
        int p = g.value("port", static_cast<int>(gs_port_));
        if (p >= 0 && p <= 65535) gs_port_ = static_cast<uint16_t>(p);
        else USER_LOG_WARN("地面站端口不在范围（0~65535）：%d，保持为 %u", p, gs_port_);
    }
    // 兜底键名
    gs_ip_ = j.value("gs_ip", j.value("ground_ip", gs_ip_));
    {
        int p = j.value("gs_port", j.value("ground_port", static_cast<int>(gs_port_)));
        if (p >= 0 && p <= 65535) gs_port_ = static_cast<uint16_t>(p);
    }

    // broadcast 节点：{ "broadcast": { "ip", "port" } }，并兼容根级 bcast_ip/bcast_port/broadcast_ip/broadcast_port
    if (j.contains("broadcast") && j["broadcast"].is_object()) {
        const auto& b = j["broadcast"];
        bcast_ip_ = b.value("ip", bcast_ip_);
        int p = b.value("port", static_cast<int>(bcast_port_));
        if (p >= 0 && p <= 65535) bcast_port_ = static_cast<uint16_t>(p);
        else USER_LOG_WARN("广播端口不在范围（0~65535）：%d，保持为 %u", p, bcast_port_);
    }
    // 兜底键名
    bcast_ip_ = j.value("bcast_ip", j.value("broadcast_ip", bcast_ip_));
    {
        int p = j.value("bcast_port", j.value("broadcast_port", static_cast<int>(bcast_port_)));
        if (p >= 0 && p <= 65535) bcast_port_ = static_cast<uint16_t>(p);
    }

    // uav_id（支持 uav_id / uavId / drone_id），必须为非负、<= UINT32_MAX
    auto try_set_uav_id = [this](const nlohmann::json& v, const char* key) {
        try {
            if (v.is_number_unsigned()) {
                uint64_t x = v.get<uint64_t>();
                if (x <= std::numeric_limits<uint32_t>::max()) {
                    uav_id_ = static_cast<uint32_t>(x);
                } else {
                    USER_LOG_WARN("无人机编号 %s 超出 uint32_t 上限：%llu，保持为 %u",
                                  key, static_cast<unsigned long long>(x), static_cast<unsigned>(uav_id_));
                }
            } else if (v.is_number_integer()) {
                long long x = v.get<long long>();
                if (x >= 0 && static_cast<unsigned long long>(x) <= std::numeric_limits<uint32_t>::max()) {
                    uav_id_ = static_cast<uint32_t>(x);
                } else {
                    USER_LOG_WARN("无人机编号 %s 为负或超限：%lld，保持为 %u",
                                  key, x, static_cast<unsigned>(uav_id_));
                }
            } else if (v.is_string()) {
                const std::string s = v.get<std::string>();
                size_t idx = 0;
                unsigned long x = std::stoul(s, &idx, 0);
                if (idx == s.size() && x <= std::numeric_limits<uint32_t>::max()) {
                    uav_id_ = static_cast<uint32_t>(x);
                } else {
                    USER_LOG_WARN("无人机编号 %s 无法完整解析为 uint32：%s，保持为 %u",
                                  key, s.c_str(), static_cast<unsigned>(uav_id_));
                }
            } else {
                USER_LOG_WARN("无人机编号 %s 类型不支持，保持为 %u", key, static_cast<unsigned>(uav_id_));
            }
        } catch (const std::exception& e) {
            USER_LOG_WARN("无人机编号 %s 解析异常：%s，保持为 %u", key, e.what(), static_cast<unsigned>(uav_id_));
        }
    };

    if (j.contains("uav_id"))        try_set_uav_id(j["uav_id"], "uav_id");
    else if (j.contains("uavId"))    try_set_uav_id(j["uavId"],   "uavId");
    else if (j.contains("drone_id")) try_set_uav_id(j["drone_id"],"drone_id");
}

UdpBindConfig UavNetConfig::localBind() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return UdpBindConfig{bind_ip_, bind_port_};
}

GroundStationConfig UavNetConfig::groundStation() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return GroundStationConfig{gs_ip_, gs_port_};
}

BroadcastConfig UavNetConfig::broadcast() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return BroadcastConfig{bcast_ip_, bcast_port_};
}

std::string UavNetConfig::bindIp() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return bind_ip_;
}

uint16_t UavNetConfig::bindPort() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return bind_port_;
}

std::string UavNetConfig::groundIp() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return gs_ip_;
}

uint16_t UavNetConfig::groundPort() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return gs_port_;
}

std::string UavNetConfig::broadcastIp() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return bcast_ip_;
}

uint16_t UavNetConfig::broadcastPort() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return bcast_port_;
}

uint32_t UavNetConfig::uavId() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return uav_id_;
}

nlohmann::json UavNetConfig::toJson() const {
    std::lock_guard<std::mutex> lk(mtx_);
    nlohmann::json j;
    j["local"]     = { {"bind_ip", bind_ip_}, {"bind_port", bind_port_} };
    j["ground"]    = { {"ip", gs_ip_}, {"port", gs_port_} };
    j["broadcast"] = { {"ip", bcast_ip_}, {"port", bcast_port_} };
    j["uav_id"]    = uav_id_;
    return j;
}
