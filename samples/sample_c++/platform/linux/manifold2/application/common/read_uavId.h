#include <dji_logger.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>

using json = nlohmann::json;

/**
 * @brief 从配置文件中读取第一个无人机的ID
 * @param configPath 配置文件路径（如：uavinfo.json）
 * @return std::string 无人机ID，若读取失败返回空字符串
 */
uint8_t loadUavId(const std::string &configPath) {
    std::string uavId;

    // 打开配置文件
    std::ifstream file(configPath);
    if (!file.is_open()) {
        USER_LOG_WARN("无法打开无人机配置文件: %s", configPath.c_str());
        return 0; // 返回默认ID 0
    }

    try {
        json config;
        file >> config;

        if (config.contains("uavs") && config["uavs"].is_array() && !config["uavs"].empty()) {
            uavId = config["uavs"][0].value("id", "");
            if (!uavId.empty()) {
                USER_LOG_INFO("读取到无人机ID: %s", uavId.c_str());
            } else {
                USER_LOG_WARN("配置中未找到有效的无人机ID。");
            }
        } else {
            USER_LOG_WARN("配置文件中 'uavs' 字段缺失或为空。");
        }
    } catch (const json::exception &e) {
        USER_LOG_WARN("无人机配置文件解析失败: %s", e.what());
    }

    uint8_t drone_id = 0; // 默认无人机ID
    
    if (uavId == "UAV001") {
        drone_id = 1;
    } else if (uavId == "UAV002") {
        drone_id = 2;
    } else if (uavId == "UAV003") {
        drone_id = 3;
    } else {
        drone_id = 0; // 默认无人机ID
        USER_LOG_WARN("未知的无人机ID: %s，使用默认值0", uavId.c_str());
    }

    return drone_id;
}
