#pragma once
/**
 * @file SimpleMsgHandler.h
 * @brief 消息处理类（对应 samples/.../simpleProtocol.[h|cpp]）
 *
 * 仅演示整体框架：解析 ➜ 分发 ➜ 回调发送。  
 * 具体业务处理全部使用 printf 占位。
 */

#include "simpleProtocol.h"
#include <functional>
#include <cstdint>
#include <cstddef>

class SimpleMsgHandler
{
public:
    SimpleMsgHandler();
    ~SimpleMsgHandler();
    void setSendCallback(std::function<void(const uint8_t*, std::size_t)> callback);

     /* ---------- 单例访问点 ---------- */
    static SimpleMsgHandler& instance();

    /* ========= 接收入口 ========= */
    /// 处理来自 Host 的数据包（下行）
    void handleDownlinkMessage(const uint8_t* data, std::size_t length);

    /* ========= 主动上报 ========= */
    /// 发送位置报文：droneId + 16‑byte telemetry
    void sendPositionReport(uint8_t droneId, const uint8_t telemetry[16]);

private:
    /* ---------- 单条指令处理 ---------- */
    void handleSetHostMode   (const SimpleProtocol::Downlink::PayloadDroneId& payload);
    void handleSetSlaveMode  (const SimpleProtocol::Downlink::PayloadDroneId& payload);
    void handleFollowStart   (const SimpleProtocol::Downlink::PayloadDroneId& payload);
    void handleFollowStop    (const SimpleProtocol::Downlink::PayloadDroneId& payload);
    void handleSetOffset     (const SimpleProtocol::Downlink::PayloadOffset&  payload);
    void handleBoardPos      (const SimpleProtocol::Downlink::PayloadPosition& payload);

private:
    std::function<void(const uint8_t*, std::size_t)> m_sendCallback;
};
