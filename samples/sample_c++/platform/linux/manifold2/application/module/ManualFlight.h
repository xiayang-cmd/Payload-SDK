#ifndef MANUAL_FLIGHT_H
#define MANUAL_FLIGHT_H


#include "common_utils.h"
#include "dji_aircraft_info.h"
#include "dji_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include <cmath>
#include <dji_core.h>
#include <dji_error.h>
#include <dji_flight_controller.h>
#include <dji_typedef.h>
#include <iostream>
#include <utils/util_misc.h>
#include <widget_interaction_test/test_widget_interaction.h>
#include <future>
// #include "test_flight_control.h" // 这个封装的很好

using FlightCallback = void (*)(float pitch, float roll, float yaw);
using namespace std;

// 指令枚举
enum class MF_Command {
    NONE,                           // 无指令
    TAKE_OFF,                       // 起飞
    RETURN,                         // 返航
    RETURN_TO_SPECIFIED_AIRPORT,    // 返航到指定机场
    CANCEL_RETURN,                  // 取消返航
    LAND,                           // 降落
    CANCEL_LAND,                    // 取消降落
    FORCE_LAND,                     // 强制降落
    DIRECTION_CONTROL,              // 方向控制 无回复
    START_FLIGHT_GUIDANCE,          // 开始指点飞行
    STOP_FLIGHT_GUIDANCE,           // 停止指点飞行 无回复
    START_CIRCLE_FLIGHT,             // 开始环点飞行
    STOP_CIRCLE_FLIGHT,             // 停止环点飞行
    EMERGENCY_BRAKE,                // 紧急制动
    SWITCH_DRONE_CONTROL            // 切换无人机控制权限
};

class ManualFlight
{
public:
    MF_Command action;              // 当前执行的指令动作

    bool bool_allowJoystickControl = false;  // 允许摇杆控制，置为0会停止起飞1.2m后的升高、方向控制、指点飞行等

public:
    // 构造函数
    ManualFlight();
    // 析构函数
    ~ManualFlight();

    // 初始化手动飞行类
    T_DjiReturnCode Initialize();

public:

    // 起飞
    T_DjiReturnCode handleTakeOff(float arg);
    
    // 返航
    T_DjiReturnCode handleReturn();

    // 返航到指定机场
    T_DjiReturnCode handleReturnToSpecifiedAirport();

    // 取消返航
    T_DjiReturnCode handleCancelReturn();

    // 降落
    T_DjiReturnCode handleLand();

    // 取消降落
    T_DjiReturnCode handleCancelLand();

    // 强制降落
    T_DjiReturnCode handleForceLand();

    // 方向控制
    T_DjiReturnCode handleDirectionControl(float speed_x, float speed_y, float speed_z, float yaw, uint16_t time);

    // 开始指点飞行
    T_DjiReturnCode handleStartFlightGuidance(double lat, double lon, float height, float speed, uint8_t mode, FlightCallback callbackfun = nullptr);

    // 停止指点飞行
    T_DjiReturnCode handleStopFlightGuidance();

    // 开始环点飞行
    T_DjiReturnCode handleStartCircleFlight(double lat, double lon, float height, float speed, float radius, uint8_t mode);

    // 停止环点飞行
    T_DjiReturnCode handleStopCircleFlight();

    // 紧急制动
    T_DjiReturnCode handleEmergencyBrake(uint8_t arg);

    // 切换无人机控制权限
    T_DjiReturnCode handleSwitchDroneControl(uint8_t arg);

public:
    bool DjiTest_FlightControlMoveByPositionOffset(
        const T_DjiTestFlightControlVector3f offsetDesired, 
        float yawDesiredInDeg,
        float posThresholdInM, 
        float yawThresholdInDeg);   // 给予位置控制的相对位置和偏航角移动的飞行控制函数

    T_DjiReturnCode DjiTest_FlightControlVelocityAndYawCtrl(
        const T_DjiTestFlightControlVector3f offsetDesired, 
        float yaw, 
        uint32_t timeMs);           // 基于速度控制的速度和偏航角速度移动的飞行控制函数
    
    T_DjiReturnCode DjiTest_FlightControlMoveByGPS(
        double latDesired, 
        double lonDesired, 
        double relative_altDesired, 
        float speed,
        float posThresholdInM = 0.5); // 通过GPS位置控制实现无人机移动的飞行控制函数

    T_DjiReturnCode MoveByGpsTryCatch(
        double latDesired, 
        double lonDesired, 
        double relative_altDesired, 
        float speed,
        float posThresholdInM = 0.5); // 通过GPS位置控制实现无人机移动的飞行控制函数的安全封装函数

    T_DjiReturnCode DjiTest_FlightControlCircleFlight(
        double latInterest, 
        double lonInterest, 
        float relative_altInterest, 
        float speed, 
        float radius = 20); // 环点飞行的飞行控制函数

    T_DjiReturnCode CircleFlightTryCatch(
        double latInterest, 
        double lonInterest, 
        float relative_altInterest, 
        float speed, 
        float radius = 20); // 环点飞行的安全封装函数

    T_DjiReturnCode DjiTest_GimbalFollowTarget(double lat, double lon, float height); // 云台跟随目标函数

    bool Dji_MoveBySelfOffset(const T_DjiTestFlightControlVector3f selfOffsetDesired, float yawDesiredInDeg); // 单次移动控制
    bool FollowHost(float hostLatDeg, float hostLonDeg, float hostAltM,
                    float offsetX, float offsetY, float offsetZ);

};


#endif // MANUAL_FLIGHT_H