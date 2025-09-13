#include "SimpleMsgHandler.h"
#include "common_data.h"
#include <cstring>  // memcpy
#include <dji_logger.h>
#include "uav_net_config.h"

using namespace SimpleProtocol;

SimpleMsgHandler::SimpleMsgHandler()
    : m_sendCallback([](const uint8_t*, std::size_t){ /* 默认空操作 */ })
{
    m_uavId = UavNetConfig::instance().uavId();
}

SimpleMsgHandler::~SimpleMsgHandler() = default;

void SimpleMsgHandler::setSendCallback(std::function<void(const uint8_t*, std::size_t)> callback) {
    m_sendCallback = std::move(callback);
}

/* ---------- 单例实例 ---------- */
SimpleMsgHandler& SimpleMsgHandler::instance()
{
    static SimpleMsgHandler s_inst;
    return s_inst;
}

/* ========== 接收入口 ========== */
void SimpleMsgHandler::handleDownlinkMessage(const uint8_t* data, std::size_t length)
{
    using namespace Downlink;

    /* 1. 协议完整性检查 */
    const CheckResult chk = checkDownlink(data, length);
    if (chk != CheckResult::Ok) {
        USER_LOG_INFO("[SimpleMsgHandler] checkDownlink failed, code=%d", static_cast<int>(chk));
        return;
    }

    // 缺少无人机id检查

    /* 2. 解析 Header 与 Payload 指针 */
    const auto* hdr = reinterpret_cast<const Header*>(data);
    const uint8_t* payload_ptr = data + sizeof(Header);

    /* 3. 分发处理 */
    switch (static_cast<Command>(hdr->cmd)) {
    case Command::SetHostMode:
        handleSetHostMode(*reinterpret_cast<const PayloadDroneId*>(payload_ptr));
        break;
    case Command::SetSlaveMode:
        handleSetSlaveMode(*reinterpret_cast<const PayloadDroneId*>(payload_ptr));
        break;
    case Command::FollowStart:
        handleFollowStart(*reinterpret_cast<const PayloadDroneId*>(payload_ptr));
        break;
    case Command::FollowStop:
        handleFollowStop(*reinterpret_cast<const PayloadDroneId*>(payload_ptr));
        break;
    case Command::SetOffset:
        handleSetOffset(*reinterpret_cast<const PayloadOffset*>(payload_ptr));
        break;
    case Command::BoardPos:
        handleBoardPos(*reinterpret_cast<const PayloadPosition*>(payload_ptr));
        break;
    default:
        USER_LOG_INFO("[SimpleMsgHandler] Unhandled cmd=0x%02X", hdr->cmd);
        break;
    }

    /* 4. 回复上位机（可选） */
    using namespace Uplink;

    uint8_t drone_id = m_uavId; 
    ReplyFrame reply_frame = makeReplyFrame(drone_id, hdr->cmd);
    m_sendCallback(reply_frame.data(), reply_frame.size());
    USER_LOG_INFO("[SimpleMsgHandler] Sent reply for cmd=0x%02X", hdr->cmd);

}

/* ========== 主动上报位置 ========== */
void SimpleMsgHandler::sendPositionReport(uint8_t droneId, const uint8_t telemetry[16])
{
    using namespace Uplink;

    PositionReportFrame frame = makePositionReportFrame(droneId, telemetry);

    /* 实际项目里可在此处附加 CRC、转义等处理 */
    m_sendCallback(frame.data(), frame.size());
}

/* ========== 具体指令处理（占位） ========== */
void SimpleMsgHandler::handleSetHostMode(const Downlink::PayloadDroneId& payload)
{
    if (payload.drone_id != m_uavId) {
        USER_LOG_WARN("[SetHostMode] Ignored for drone_id=%u (self id=%u)",
                      payload.drone_id, m_uavId);
        USER_LOG_WARN("[SetHostMode] Received command not for this drone (id=%u)", m_uavId);
        return;
    }
    g_role = DroneRole::Master; // 设置自己为主机角色
    g_host_confirmed = true; // 主机已确认
    USER_LOG_INFO("[SetHostMode] drone_id=%u", payload.drone_id);
    /* TODO: your code here */
}

void SimpleMsgHandler::handleSetSlaveMode(const Downlink::PayloadDroneId& payload)
{
    if (payload.drone_id != m_uavId) {
        USER_LOG_WARN("[SetSlaveMode] Ignored for drone_id=%u (self id=%u)",
                      payload.drone_id, m_uavId);
        USER_LOG_WARN("[SetSlaveMode] Received command not for this drone (id=%u)", m_uavId);
        return;
    }
    g_role = DroneRole::Slave; // 设置为从机角色
    USER_LOG_INFO("[SetSlaveMode] drone_id=%u", payload.drone_id);
}

void SimpleMsgHandler::handleFollowStart(const Downlink::PayloadDroneId& payload)
{
    if (payload.drone_id != m_uavId) {
        USER_LOG_WARN("[FollowStart] Ignored for drone_id=%u (self id=%u)",
                      payload.drone_id, m_uavId);
        USER_LOG_WARN("[FollowStart] Received command not for this drone (id=%u)", m_uavId);
        return;
    }
    USER_LOG_INFO("[FollowStart] drone_id=%u", payload.drone_id);
    
    if((!isHostReady())||(!isOffsetReliable()))
    {
        if(!isOffsetReliable())
        {
            USER_LOG_WARN("Offset is not reliable.");
        }
        if(!isHostReady())
        {
            USER_LOG_WARN("Host is not ready.");
        }
        USER_LOG_WARN("Cannot start formation: Host not ready or offset not reliable.");
        // 切勿在不安全的情况下开启编队
        g_should_follow_start = false;
        g_should_follow_stop = true; // 停止编队
    }else{
        g_should_follow_start = true;  // 设置编队开始标志
        g_should_follow_stop = false;  // 清除停止编队标志
    }
    
}

void SimpleMsgHandler::handleFollowStop(const Downlink::PayloadDroneId& payload)
{
    if (payload.drone_id != m_uavId) {
        USER_LOG_WARN("[FollowStop] Ignored for drone_id=%u (self id=%u)",
                      payload.drone_id, m_uavId);
        USER_LOG_WARN("[FollowStop] Received command not for this drone (id=%u)", m_uavId);
        return;
    }
    USER_LOG_INFO("[FollowStop] drone_id=%u", payload.drone_id);

    g_should_follow_start = false; // 清除编队开始标志
    g_should_follow_stop = true;  // 设置停止编队标志
}

void SimpleMsgHandler::handleSetOffset(const Downlink::PayloadOffset& payload)
{
    if (payload.drone_id != m_uavId) {
        USER_LOG_WARN("[SetOffset] Ignored for drone_id=%u (self id=%u)",
                      payload.drone_id, m_uavId);
        USER_LOG_WARN("[SetOffset] Received command not for this drone (id=%u)", m_uavId);
        return;
    }
    USER_LOG_INFO("[SetOffset] drone_id=%u, offset=(%u,%u,%u)",
           payload.drone_id,
           payload.offset_x, payload.offset_y, payload.offset_z);
    g_offset_x = static_cast<float>(payload.offset_x) / 1; // 假设单位为 m
    g_offset_y = static_cast<float>(payload.offset_y) / 1; // 假设单位为 m
    g_offset_z = static_cast<float>(payload.offset_z) / 1; // 假设单位为 m

    if(!isOffsetReliable()){
        USER_LOG_INFO("offset is not reliable.");
    }
}   

void SimpleMsgHandler::handleBoardPos(const Downlink::PayloadPosition& payload)
{
    // 打印收到的无人机位置
    USER_LOG_INFO("uav_id=%u, Lat=%.7f, Lon=%.7f, Height=%.1f m",
           payload.drone_id,
           payload.latitude_deg, payload.longitude_deg, payload.height_fusion);

    // 候选主机相关变量（静态保存）
    static uint32_t candidate_id   = 0;
    static uint8_t  candidate_cnt  = 0;

    // 如果主机未确认，则尝试通过连续收到相同ID来确认
    if (!g_host_confirmed) {
        if (payload.drone_id == candidate_id) {
            if (++candidate_cnt >= 10) {   // 连续收到10次相同ID
                g_host_drone_id  = candidate_id;
                g_host_confirmed = true;
                USER_LOG_INFO("[HOST CONFIRMED] id=%u", g_host_drone_id);
            }
        } else {
            candidate_id  = payload.drone_id;
            candidate_cnt = 1;
        }
    }else{
        // 如果收到的是本机的位置信息，则忽略
        if(payload.drone_id == m_uavId){
            USER_LOG_WARN("[BoardPos] Received position for self drone (id=%u)", m_uavId);
            return;
        }

    }

    // 主机已确认的情况下，更新位置或重新选主机
    if (g_host_confirmed) {
        if (payload.drone_id == g_host_drone_id) {
            // 更新已确认主机的位置
            g_host_latitude_deg   = payload.latitude_deg;
            g_host_longitude_deg  = payload.longitude_deg;
            g_host_altitude_fused = payload.altitude_fused;
            g_host_height_fusion  = payload.height_fusion;
        } else {
            // 收到的ID不是当前主机 → 主机丢失，切换到候选
            g_host_confirmed = false;
            candidate_id     = payload.drone_id;
            candidate_cnt    = 1;
            USER_LOG_INFO("[HOST LOST] switch to candidate id=%u, counting...", candidate_id);
            USER_LOG_WARN("[BoardPos] Received position not for host drone (host id=%u, received id=%u)", g_host_drone_id, payload.drone_id);
        }
    }
}

