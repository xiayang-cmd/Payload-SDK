/**
 ********************************************************************
 * @file    main.cpp
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <liveview/test_liveview_entry.hpp>
#include <perception/test_perception_entry.hpp>
#include <perception/test_lidar_entry.hpp>
#include <perception/test_radar_entry.hpp>
// #include <flight_control/test_flight_control.h>,有个类型名称和这个库里面的重了
#include <gimbal/test_gimbal_entry.hpp>
#include "application.hpp"
#include "fc_subscription/test_fc_subscription.h"
#include <thread>
#include <gimbal_emu/test_payload_gimbal_emu.h>
#include <camera_emu/test_payload_cam_emu_media.h>
#include <camera_emu/test_payload_cam_emu_base.h>
#include <dji_logger.h>
#include "widget/test_widget.h"
#include "widget/test_widget_speaker.h"
#include <power_management/test_power_management.h>
#include "data_transmission/test_data_transmission.h"
#include <flight_controller/test_flight_controller_entry.h>
#include <positioning/test_positioning.h>
#include <hms_manager/hms_manager_entry.h>
#include "camera_manager/test_camera_manager_entry.h"
#include <widget_manager/test_widget_manager.hpp>
#include "simple_fc_subscription.h"
#include "read_uavId.h"
#include "UDPServer.h"
#include "common_data.h"
#include "SimpleMsgHandler.h"
#include "simpleProtocol.h"
#include "ManualFlight.h"
#include <atomic>
#include <future>
#include <thread>
#include <chrono>

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/

int main(int argc, char **argv)
{
    Application application(argc, argv);
    char inputChar;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;
    T_DjiTestApplyHighPowerHandler applyHighPowerHandler;

    // 1. 读取无人机ID
    g_drone_id = loadUavId("uavinfo.json");
    const std::string kDestIp   = "255.255.255.255";    // 广播地址
    const uint16_t    kDestPort = 50732;                // 地面站固定端口
    const uint16_t    kBRcvPort = 50733;                // 无人机固定端口

    // 2. 启动UDP服务
    UDPServer udpServer;

    auto serverThread = std::thread([&udpServer]() {
        udpServer.start("0.0.0.0", kBRcvPort);
    });
    serverThread.detach();

    // 3. 设置消息处理实例的发送函数e注册
    SimpleMsgHandler::instance().setSendCallback(
        [&udpServer,kDestIp](const uint8_t* buf, std::size_t len){
            udpServer.sendMessage(kDestIp, kDestPort, buf, len);
        });

    // 4. 订阅飞控数据（获取无人机位置、速度等信息）
    SimpleFcSubscription sub;
    if (sub.startService() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return -1;
    }

    // 5. 广播自身位置
    std::thread telem([&]{
        sub.run([&udpServer, kDestIp](const UAVDataInfo& t){
            printf("Lat %.7f, Lon %.7f, Alt %.1f m\n", t.latitude_deg, t.longitude_deg, t.altitude_fused);

            // 组数据帧
            const uint8_t* telemetry = reinterpret_cast<const uint8_t*>(&t);
            auto frame = SimpleProtocol::Uplink::makePositionReportFrame(g_drone_id, telemetry);

            // 如果是主机，还需要广播给其他无人机
            udpServer.sendMessage(kDestIp, kDestPort, frame.data(), frame.size());
            SimpleProtocol::Uplink::changePositionReportFrameForBoard(frame);
            auto role  = queryCurrentRole();
            if (role == DroneRole::Master) {
                udpServer.sendMessage(kDestIp, kBRcvPort, frame.data(), frame.size());
            }
            
        });
    });
    telem.detach();

    // 6. 运动控制初始化
    ManualFlight manualFlight;
    if (manualFlight.Initialize() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("ManualFlight init failed");
        return -1;
    }

    // =========== 3. 主循环状态机 =============
    static std::future<void>   g_follow_task;             // 跟随线程 future
    static std::atomic<bool>   g_follow_running{false};   // 标识线程是否在跑
    while (true) {
        switch (g_state.load()) {

        //------------------------------------
        case DroneState::WaitRoleSet:
            if (isHostReady()) {
                auto m_role  = queryCurrentRole();
                if (m_role == DroneRole::Master) {
                    g_state = DroneState::Idle;
                } else if (m_role == DroneRole::Slave) {
                    g_state = DroneState::WaitFormationStart;
                }
                // 若仍为 Unknown，保持当前状态
            }
            break;

        //------------------------------------
        case DroneState::Idle:  // 仅主机可达
            // 仅休息
            break;

        //------------------------------------
        case DroneState::WaitFormationStart:   // 从机等待开始编队
            if (isOffsetReliable()) {
                g_state = DroneState::InFormation;
            }
            break;

        //------------------------------------
        case DroneState::InFormation:          // 从机编队中
        {
            if((!isHostReady())||(!isOffsetReliable())){
                g_should_follow_start = false;
                g_should_follow_stop = true; // 停止编队
            }
            
            /* ---------- 1. 启动跟随 ---------- */
            if (!g_follow_running && shouldFollowStart())
            {
                g_follow_running = true;          // 先置位，防止多次创建
                g_follow_task = std::async(std::launch::async, [&manualFlight]{
                    /* 后台跟随循环 */
                    while (!shouldFollowStop())
                    {
                        // 位置更新
                        manualFlight.FollowHost(
                            g_host_latitude_deg,
                            g_host_longitude_deg,
                            g_host_altitude_fused,
                            g_offset_x, g_offset_y, g_offset_z);                   
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(100));        // 100 Hz 控制频率
                    }
                });
            }

            /* ---------- 2. 停止跟随 ---------- */
            if (shouldFollowStop())
            {
                // 等待后台线程自然退出
                if (g_follow_running && g_follow_task.valid())
                    g_follow_task.get();   // 线程检测到 stop 后会自己跳出

                g_follow_running = false;

                g_state = DroneState::WaitFormationStart;          // 回上一个状态
                std::this_thread::sleep_for(std::chrono::seconds(2)); // 等待 2 s
            }
            break;
        }

        //------------------------------------
        default:
            g_state = DroneState::WaitRoleSet; // 防御式回退
            break;
        }

        /* 为避免占满 CPU，可适当睡眠；长度按控制实时性调整 */
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

}

/* Private functions definition-----------------------------------------------*/

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
