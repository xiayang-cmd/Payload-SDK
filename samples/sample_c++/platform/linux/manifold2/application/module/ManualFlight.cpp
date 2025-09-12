#include "ManualFlight.h"
#include "TargetArrivalMonitor.h" // for 抵达监测
#include "dji_gimbal_manager.h" // for 云台控制

/* Private types -------------------------------------------------------------*/

// 摇杆位置模式的配置：水平位置控制、垂直位置控制、偏航角控制、地面坐标系、稳定控制
T_DjiFlightControllerJoystickMode joystickMode_pos = {
    DJI_FLIGHT_CONTROLLER_HORIZONTAL_POSITION_CONTROL_MODE,
    DJI_FLIGHT_CONTROLLER_VERTICAL_POSITION_CONTROL_MODE,
    DJI_FLIGHT_CONTROLLER_YAW_ANGLE_CONTROL_MODE,
    DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
    DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
};

// 摇杆速度模式的配置：水平速度控制，垂直速度控制，偏航角速度控制，地面坐标系，稳定控制
T_DjiFlightControllerJoystickMode joystickMode_vel = {
    DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
    DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
    DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
    DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
    DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
};

// /* Private functions declaration  ---------------------------------------------*/
// /* 从示例中移植  ---------------------------------------------*/
// // 相对位置和偏航角移动的飞行控制函数
static bool DjiTest_FlightControlMoveByPositionOffset(const T_DjiTestFlightControlVector3f offsetDesired, float yawDesiredInDeg, float posThresholdInM, float yawThresholdInDeg);

/* Exported functions definition ---------------------------------------------*/
// 构造函数
ManualFlight::ManualFlight()
{
    action = MF_Command::NONE;
    bool_allowJoystickControl = true;
}

// 析构函数
ManualFlight::~ManualFlight()
{   
    T_DjiReturnCode returnCode;
    // 反初始化飞行控制器模块
    returnCode = DjiFlightController_DeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit flight controller module failed, error code:0x%08llX",
                       returnCode);
    }
}

// 初始化无人机手动飞行类
T_DjiReturnCode ManualFlight::Initialize()
{
    // 获取OSAL处理器
    T_DjiReturnCode returnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();
    if (!s_osalHandler) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    
    // 获取飞机基本信息
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;
    returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get aircraft base info error");
    }

    // 获取当前GPS
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};       // GPS位置
    returnCode = DjiDemo_GetCurrentGpsPosition(&gpsPosition);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic gps error, error code: 0x%08X", returnCode);
        return returnCode;
    }

    // 初始化飞行控制模块
    T_DjiFlightControllerRidInfo ridInfo = {0};
    ridInfo.longitude = gpsPosition.x*1.0/10000000;
    ridInfo.latitude = gpsPosition.y*1.0/10000000;
    ridInfo.altitude = gpsPosition.z/100;
    returnCode = DjiFlightController_Init(ridInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("初始化无人机控制模块失败, error code:0x%08llX", returnCode);
        return returnCode;
    }

    // 获取API控制权限
    USER_LOG_INFO("获取API控制权限");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("获取操纵杆控制权限失败, error code: 0x%08X", returnCode);
        return returnCode;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

// 起飞
T_DjiReturnCode ManualFlight::handleTakeOff(float arg)
{
    T_DjiReturnCode returnCode;

    // 获取OSAL处理器
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();
    if (!s_osalHandler) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    // 检查飞机是否在地面
    if(DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR)
    {
        // 飞机已经在空中
        USER_LOG_WARN("飞机已经在空中");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    g_cur_taskID++; // 每次起飞视为一个新的任务

    // 写入起飞点经纬度全局变量
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    returnCode =  DjiDemo_GetCurrentGpsPosition(&gpsPosition);
    g_startPoint_latitude = gpsPosition.y * 1.0 / 10000000;
    g_startPoint_longitude = gpsPosition.x * 1.0 / 10000000;

    //! Start takeoff
    returnCode = DjiFlightController_StartTakeoff();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to take off failed, error code: 0x%08X", returnCode);
        return returnCode;
    }
    
    //睡眠0.1秒
    s_osalHandler->TaskSleepMs(100);
    USER_LOG_INFO("起飞成功");

    // 飞到指定高度(实际上起飞API只起到启动电机的作用，飞到指定高度的则是调用moveByPositionOffset实现，从原理来看最终抵达的高度可能不准确)
    float height = arg;
    T_DjiTestFlightControlVector3f position = {0, 0, height};
    bool_allowJoystickControl = true; // 允许摇杆控制
    try{
        if (!DjiTest_FlightControlMoveByPositionOffset(position, 0, 0.3, 1)) {
            USER_LOG_ERROR("起飞到指定高度失败");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }catch(std::exception &e){
        USER_LOG_ERROR("Move to specified height failed, error code: 0x%08X", returnCode);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

// 返航
T_DjiReturnCode ManualFlight::handleReturn()
{
    T_DjiReturnCode returnCode;
    bool_allowJoystickControl = false; // 打断摇杆控制

    // 获取OSAL处理器
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();
    if (!s_osalHandler) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    //! Start go home
    returnCode = DjiFlightController_StartGoHome();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to go home failed, error code: 0x%08X", returnCode);
        return returnCode;
    }

    //睡眠0.1秒
    s_osalHandler->TaskSleepMs(100);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

// 取消返航
T_DjiReturnCode ManualFlight::handleCancelReturn()
{
    T_DjiReturnCode returnCode;

    // 获取OSAL处理器
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();
    if (!s_osalHandler) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    //! Cancel go home
    returnCode = DjiFlightController_CancelGoHome();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to cancel go home failed, error code: 0x%08X", returnCode);
        return returnCode;
    }

    //睡眠0.1秒
    s_osalHandler->TaskSleepMs(100);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

// 返航到指定机场
T_DjiReturnCode ManualFlight::handleReturnToSpecifiedAirport(){
    USER_LOG_WARN("Return to specified airport is not supported yet.");
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

// 降落
T_DjiReturnCode ManualFlight::handleLand()
{
    T_DjiReturnCode returnCode;
    bool_allowJoystickControl = false; // 打断摇杆控制

    // 获取OSAL处理器
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();
    if (!s_osalHandler) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    //! Start landing
    returnCode = DjiFlightController_StartLanding();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to land failed, error code: 0x%08X", returnCode);
        return returnCode;
    }

    //睡眠0.1秒
    s_osalHandler->TaskSleepMs(100);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

// 取消降落
T_DjiReturnCode ManualFlight::handleCancelLand()
{
    T_DjiReturnCode returnCode;

    // 获取OSAL处理器
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();
    if (!s_osalHandler) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    //! Cancel landing
    returnCode = DjiFlightController_CancelLanding();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to cancel landing failed, error code: 0x%08X", returnCode);
        return returnCode;
    }

    //睡眠0.1秒
    s_osalHandler->TaskSleepMs(100);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

// 强制降落
T_DjiReturnCode ManualFlight::handleForceLand()
{
    T_DjiReturnCode returnCode;
    bool_allowJoystickControl = false; // 打断摇杆控制

    // 获取OSAL处理器
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();
    if (!s_osalHandler) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    //! Force landing
    returnCode = DjiFlightController_StartForceLanding();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to force landing failed, error code: 0x%08X", returnCode);
        return returnCode;
    }

    //睡眠0.1秒
    s_osalHandler->TaskSleepMs(100);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

// 方向控制
T_DjiReturnCode ManualFlight::handleDirectionControl(float speed_x, float speed_y, float speed_z, float yaw_rate, uint16_t time)
{
    T_DjiReturnCode returnCode;
    bool_allowJoystickControl = true;
    // 调用DjiTest_FlightControlVelocityAndYawCtrl
    try{
        returnCode = DjiTest_FlightControlVelocityAndYawCtrl({speed_x, speed_y, speed_z}, yaw_rate, time);
    }catch(std::exception &e){
        USER_LOG_ERROR("Direction control failed, error code: 0x%08X", returnCode);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    
    return returnCode;
}

// 开始指点飞行:0-节能模式【飞机直线飞到目标点 ，需飞机在空中】 ，1-安全模式【飞机飞行至目标点高度再平飞到目标点 ，飞机可以 在地面或空中】 默认为节能模式
T_DjiReturnCode ManualFlight::handleStartFlightGuidance(double lat, double lon, float height, float speed, uint8_t mode, FlightCallback callbackfun)
{
    bool_allowJoystickControl = true;
    T_DjiReturnCode returnCode;
    T_DjiFcSubscriptionPositionFused originGPSPosition;

    // 先异步调用DjiTest_GimbalFollowTarget函数
    std::atomic<bool> keepGimbal{true};
    // 启动云台跟随线程
    auto gimbalFuture = std::async(std::launch::async, [&, lat, lon, height]() {
        try {
            while (keepGimbal.load(std::memory_order_relaxed)) {
                DjiTest_GimbalFollowTarget(lat, lon, height);   // 内部已含 sleep
            }
        } catch (const std::exception& e) {
            USER_LOG_ERROR("Gimbal Follow thread exception: %s", e.what());
        }
    });

    switch (mode) {
        case 0:
            // 节能模式,直接飞到目标点
            g_cur_pointFlightState.pointMode = 0;
            returnCode = MoveByGpsTryCatch(lat, lon, height, speed); 
            break;
        case 1:
            // 安全模式
            g_cur_pointFlightState.pointMode = 1;
            // 先尝试起飞，起飞失败则执行下一步
            returnCode = handleTakeOff(height);
            if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                // 起飞成功，直接飞到目标点(由于已升高，这里是平飞)
                returnCode = MoveByGpsTryCatch(lat, lon, height, speed);
            }else
            {
                // 起飞失败视作无人机已经在空中
                USER_LOG_ERROR("Take off failed, Maybe the aircraft is already in the air, error code: 0x%08X", returnCode);
                // 先升高到目标高度
                originGPSPosition = DjiTest_FlightControlGetValueOfPositionFused(); // 获取飞机的GPS位置
                returnCode = MoveByGpsTryCatch(originGPSPosition.latitude* RAD_TO_DEG, originGPSPosition.longitude* RAD_TO_DEG, height, 4);//升高用另一个函数(不控制云台)，然后速度为4，不要用spped（设了speed向上也只能到4m/s），没法计算预计执行时间。
                
                if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    // 成功升高后，开始指点飞行
                    returnCode = MoveByGpsTryCatch(lat, lon, height, speed);
                } else {
                    USER_LOG_ERROR("Ascend to target altitude failed, error code: 0x%08X", returnCode);
                }
            }
            break;
        default:
            USER_LOG_ERROR("Invalid mode!");
            returnCode = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
            break;
    }
    /* ---- 结束云台线程 ---- */
    keepGimbal.store(false, std::memory_order_relaxed);
    gimbalFuture.get();               // 等待线程安全退出
    USER_LOG_INFO("Gimbal thread finished.");

    g_cur_pointFlightState.isPointControl = false; // 结束指点飞行,更新全局标志
    return returnCode;
}

// 停止指点飞行
T_DjiReturnCode ManualFlight::handleStopFlightGuidance()
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler(); // 获取OSAL处理器

    bool_allowJoystickControl = false;

    // 睡眠0.1秒
    osalHandler->TaskSleepMs(100);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

// 开始环点飞行
T_DjiReturnCode ManualFlight::handleStartCircleFlight(double lat, double lon, float height, float speed, float radius, uint8_t mode)
{
    bool_allowJoystickControl = true;
    T_DjiReturnCode returnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    T_DjiFcSubscriptionPositionFused originGPSPosition;

    // 标记进入环点飞行
    g_cur_circleFlightState.isCircleControl = true;

    // 先异步调用DjiTest_GimbalFollowTarget函数
    std::atomic<bool> keepGimbal{true};
    // 启动云台跟随线程
    auto gimbalFuture = std::async(std::launch::async, [&, lat, lon, height]() {
        try {
            while (keepGimbal.load(std::memory_order_relaxed)) {
                DjiTest_GimbalFollowTarget(lat, lon, height);   // 内部已含 sleep
            }
        } catch (const std::exception& e) {
            USER_LOG_ERROR("Gimbal Follow thread exception: %s", e.what());
        }
    });

    switch (mode) {
        case 0:
            // 节能模式：直接进行环点飞行
            g_cur_circleFlightState.circleMode = 0;
            USER_LOG_INFO("Start Circle Flight in mode 0 (节能模式).");
            returnCode = CircleFlightTryCatch(lat, lon, height, speed, radius);
            break;

        case 1:
            // 安全模式：先升到目标高度，再开始环点飞行
            g_cur_circleFlightState.circleMode = 1;
            USER_LOG_INFO("Start Circle Flight in mode 1 (安全模式).");

            // 获取当前GPS位置
            originGPSPosition = DjiTest_FlightControlGetValueOfPositionFused();
            // 先升高到目标高度，速度为4，可自行设定
            returnCode = MoveByGpsTryCatch(
                originGPSPosition.latitude * RAD_TO_DEG, 
                originGPSPosition.longitude * RAD_TO_DEG, 
                height, 
                4
            );

            if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                // 成功升高后，开始环点飞行
                returnCode = CircleFlightTryCatch(lat, lon, height, speed, radius);
            } else {
                USER_LOG_ERROR("Ascend to target altitude failed, error code: 0x%08X", returnCode);
            }
            break;

        default:
            USER_LOG_ERROR("Invalid circle flight mode: %d", mode);
            returnCode = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
            break;
    }

    /* ---- 结束云台线程 ---- */
    keepGimbal.store(false, std::memory_order_relaxed);
    gimbalFuture.get();               // 等待线程安全退出
    USER_LOG_INFO("Gimbal thread finished.");

    g_cur_circleFlightState.isCircleControl = false; // 结束环点飞行,更新全局标志
    return returnCode;
}

// 停止环点飞行
T_DjiReturnCode ManualFlight::handleStopCircleFlight()
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler(); // 获取OSAL处理器

    bool_allowJoystickControl = false;

    // 睡眠0.1秒
    osalHandler->TaskSleepMs(100);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

// 紧急制动:0x00-解除紧急制动 0x01-紧急制动
T_DjiReturnCode ManualFlight::handleEmergencyBrake(uint8_t arg)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler(); // 获取OSAL处理器
    T_DjiReturnCode returnCode;

    // 获取API控制权限
    USER_LOG_INFO("获取无人机控制权限.");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("获取操纵杆控制权限失败, error code: 0x%08X", returnCode);
        return returnCode;
    }

    switch (arg) {
        case 0:
            // 解除紧急制动
            g_flag_isEmerge = 0;
            bool_allowJoystickControl = true; // 允许摇杆控制
            USER_LOG_INFO("Cancel emergency brake action");
            returnCode = DjiFlightController_CancelEmergencyBrakeAction();
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("Cancel emergency brake action failed, error code: 0x%08X", returnCode);
                return returnCode;
            }
            osalHandler->TaskSleepMs(1000);
            break;
        case 1:
            // 紧急制动
            g_flag_isEmerge = 1;
            bool_allowJoystickControl = false; // 打断摇杆控制
            USER_LOG_INFO("Emergency brake action");
            returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
                return returnCode;
            }
            osalHandler->TaskSleepMs(1000);
            break;
        default:
            USER_LOG_ERROR("Invalid arg!");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/*
* @brief 切换无人机控制权限
* @param arg 0-获取控制权限 1-释放控制权限
*/
T_DjiReturnCode ManualFlight::handleSwitchDroneControl(uint8_t arg)
{
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();

    switch (arg) {
        case 0:
            // 获取API控制权限
            USER_LOG_INFO("获取操纵杆控制权限");
            returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("获取操纵杆控制权限失败, error code: 0x%08X", returnCode);
                return returnCode;
            }
            osdk_is_get_CtrlAuthority = true;
            break;
        case 1:
            // 释放API控制权限
            USER_LOG_INFO("释放操纵杆控制权限");
            returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("释放操纵杆控制权限失败, error code: 0x%08X", returnCode);
                return returnCode;
            }
            osdk_is_get_CtrlAuthority = false;
            break;
        default:
            USER_LOG_ERROR("Invalid arg!");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* 以下函数的实现均是从从示例中移植的  ---------------------------------------------*/

int DjiTest_FlightControlSignOfData(dji_f32_t data)
{
    return data < 0 ? -1 : 1;
}
void DjiTest_FlightControlHorizCommandLimit(dji_f32_t speedFactor, dji_f32_t *commandX, dji_f32_t *commandY)
{
    if (fabs(*commandX) > speedFactor)
        *commandX = speedFactor * DjiTest_FlightControlSignOfData(*commandX);
    if (fabs(*commandY) > speedFactor)
        *commandY = speedFactor * DjiTest_FlightControlSignOfData(*commandY);
}

/**
 * @brief 通过位置偏移和偏航角移动的飞行控制函数
 * @param offsetDesired 期望偏移（相对于home点的）
 * @param yawDesiredInDeg 期望偏航角
 * @param posThresholdInM 位置阈值
 */
bool ManualFlight::DjiTest_FlightControlMoveByPositionOffset(
    const T_DjiTestFlightControlVector3f offsetDesired,
    float yawDesiredInDeg,
    float posThresholdInM,
    float yawThresholdInDeg)
{
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler(); // 获取OSAL处理器
    
    // 0. 设置参数
    const int timeoutInMilSec  = 80000;                  // 该函数整体的超时时间
    const int controlFreqInHz  = 50;                     // 控制频率
    const int cycleTimeInMs    = 1000 / controlFreqInHz; // 控制周期
    const int speedFactor      = 2;                      // 速度因子

    // 1. 获取飞机初始的地理位置（相对于homepoint的高度）
    DJI_CustomGeoPosition originCustomGeoPosition = DjiTest_FlightControlGetValueOfCustomGeoPosition();
    if(originCustomGeoPosition.relative_alt == -1) {
        USER_LOG_ERROR("Relative height is invalid!");
        return false;
    }

    // 2. 设置摇杆控制模式为位置控制模式
    T_DjiFlightControllerJoystickMode joystickMode = joystickMode_pos;
    DjiFlightController_SetJoystickMode(joystickMode);

    // 3. 创建并配置 TargetArrivalMonitor 用于到达检测
    TargetArrivalMonitor arrivalMonitor(cycleTimeInMs);
    arrivalMonitor.SetPosThreshold(posThresholdInM);
    arrivalMonitor.SetYawThreshold(yawThresholdInDeg);
    // 设置在范围内和范围外的计时时间限制（ 10* 和 100* 周期）
    arrivalMonitor.SetTimeRequirements(10 * cycleTimeInMs, 100 * cycleTimeInMs);

    // 4. 控制循环
    int elapsedTimeInMs        = 0; // 该函数的已执行时间,用于控制循环的超时检测
    while (elapsedTimeInMs < timeoutInMilSec) {
        // 4.1 检查是否需要停止飞行（外部中断标志）
        if (!bool_allowJoystickControl) {
            USER_LOG_INFO("Flight interrupted by stop flag");
            return false;
        }

        // 4.2 获取当前位置、姿态
        DJI_CustomGeoPosition currentCustomGeoPosition = DjiTest_FlightControlGetValueOfCustomGeoPosition();
        if (currentCustomGeoPosition.relative_alt == -1) {
            USER_LOG_ERROR("Relative height is invalid!");
            return false;
        }
        T_DjiFcSubscriptionQuaternion currentQuaternion = DjiTest_FlightControlGetValueOfQuaternion();
        float yawInRad = DjiTest_FlightControlQuaternionToEulerAngle(currentQuaternion).z;

        // 4.3 获取飞机当前点相对于原点的向量
        T_DjiTestFlightControlVector3f localOffset = 
        DjiTest_FlightControlLocalOffsetFromCustomGeoPosition(
            currentCustomGeoPosition,            
            originCustomGeoPosition);

        // 4.4 计算剩余偏移向量 & 偏航误差
        T_DjiTestFlightControlVector3f offsetRemaining = DjiTest_FlightControlVector3FSub(offsetDesired, localOffset);

        float posOffsetInM = DjiTest_FlightControlVectorNorm(offsetRemaining);
        float yawErrorDeg  = yawInRad / s_degToRad - yawDesiredInDeg;

        // 4.5 使用到达检测器进行更新
        arrivalMonitor.Update(posOffsetInM, yawErrorDeg);
        // 若满足“到达”条件，则退出循环
        if (arrivalMonitor.checkArrival()) {
            break;
        }

        // 4.6 生成并执行操纵杆命令
        // 先限制水平方向命令的最大值
        T_DjiTestFlightControlVector3f positionCommand = offsetRemaining;
        DjiTest_FlightControlHorizCommandLimit(speedFactor, &positionCommand.x, &positionCommand.y);

        // 构造并发送 Joystick
        T_DjiFlightControllerJoystickCommand joystickCommand = {
            positionCommand.x,
            positionCommand.y,
            offsetDesired.z + originCustomGeoPosition.relative_alt, // 这里实际是有些问题的。
            yawDesiredInDeg
        };
        DjiFlightController_ExecuteJoystickAction(joystickCommand);

        // 4.7 等待一个控制周期
        s_osalHandler->TaskSleepMs(cycleTimeInMs);
        elapsedTimeInMs += cycleTimeInMs;
    }

    // 若超时
    if (elapsedTimeInMs >= timeoutInMilSec) {
        USER_LOG_ERROR("Task timeout!");
    }

    // 5. 最终的稳定保持操作：保持当前位置，什么都不做，确保 0 速度以维持稳定
    int brakeTimeAccum          = 0;
    const int brakeTimeMs       = 100 * cycleTimeInMs;    // 稳定保持时间
    while (brakeTimeAccum < brakeTimeMs) {
        if (!bool_allowJoystickControl) {
            USER_LOG_INFO("Flight interrupted by stop flag");
            return false;
        }
        s_osalHandler->TaskSleepMs(cycleTimeInMs);
        brakeTimeAccum += cycleTimeInMs;
    }

    USER_LOG_INFO("基于位置的控制结束");

    return true;
}


/*
* @brief 速度和偏航控制
* @param offsetDesired 期望偏移
* @param yaw_rate 偏航角速度
* @param timeMs 时间                        
*/
T_DjiReturnCode ManualFlight::DjiTest_FlightControlVelocityAndYawCtrl(const T_DjiTestFlightControlVector3f offsetDesired, float yaw_rate, uint32_t timeMs)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler(); // 获取OSAL处理器
    uint32_t originTime = 0;                        // 起始时间
    uint32_t currentTime = 0;                       // 当前时间
    uint32_t elapsedTimeInMs = 0;                   // 经过时间
    osalHandler->GetTimeMs(&originTime);            // 获取当前时间
    osalHandler->GetTimeMs(&currentTime);           // 获取当前时间
    elapsedTimeInMs = currentTime - originTime;     // 计算经过时间

    T_DjiFlightControllerJoystickMode joystickMode = joystickMode_vel; // 速度控制模式

    DjiFlightController_SetJoystickMode(joystickMode); // 设置操纵杆模式
    T_DjiFlightControllerJoystickCommand joystickCommand = {offsetDesired.x, offsetDesired.y, offsetDesired.z, yaw_rate}; // 执行操纵杆命令

    while (elapsedTimeInMs <= timeMs) {
        DjiFlightController_ExecuteJoystickAction(joystickCommand);
        osalHandler->TaskSleepMs(2);
        osalHandler->GetTimeMs(&currentTime);
        elapsedTimeInMs = currentTime - originTime;
        if (!bool_allowJoystickControl) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief  带 try-catch 的 GPS 飞控安全封装
 * @param  latDesired          目标纬度（度）
 * @param  lonDesired          目标经度（度）
 * @param  relative_altDesired 目标相对高度（米）
 * @param  speed               飞行速度（米/秒）
 * @param  posThresholdInM     到达判定阈值（米）
 * @return DJI SDK 统一返回码
 */
T_DjiReturnCode ManualFlight::MoveByGpsTryCatch(double latDesired,
                                                double lonDesired,
                                                double relative_altDesired,
                                                float  speed,
                                                float  posThresholdInM)
{
    try
    {
        // 调用原始实现
        return DjiTest_FlightControlMoveByGPS(latDesired,
                                              lonDesired,
                                              relative_altDesired,
                                              speed,
                                              posThresholdInM);
    }
    catch (const std::exception& e)
    {
        std::cerr << "[MoveByGpsTryCatch] std::exception: "
                  << e.what() << std::endl;
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;   // 根据项目实际错误码替换
    }
    catch (...)
    {
        std::cerr << "[MoveByGpsTryCatch] 未知异常！" << std::endl;
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;         // 兜底错误码
    }
}

/**
 * @brief 通过GPS坐标移动的飞行控制函数
 * @param latDesired 目标纬度，单位：度
 * @param lonDesired 目标经度，单位：度
 * @param relative_altDesired 目标高度，单位：米
 * @param speed 飞行速度，单位：米/秒
 * @param posThresholdInM 到达阈值
 */
T_DjiReturnCode ManualFlight::DjiTest_FlightControlMoveByGPS(double latDesired, 
                                                             double lonDesired, 
                                                             double relative_altDesired,
                                                             float speed, 
                                                             float posThresholdInM)
{
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler(); // 获取OSAL处理器
    
    // 0. 设置参数
    int timeoutInMilSec        = 2000000;                // 该函数整体的超时时间
    const int controlFreqInHz  = 50;                     // 控制频率
    const int cycleTimeInMs    = 1000 / controlFreqInHz; // 控制周期
    const int speedFactor      = 2;                      // 速度因子

    if (posThresholdInM <= 0) {
        posThresholdInM = 0.8f;     // 如果位置阈值不合法，设置默认值
    }

    // 1. 构建目标位置结构体（注意经纬度顺序）
    double latDesiredRad = latDesired * s_degToRad;
    double lonDesiredRad = lonDesired * s_degToRad;
    DJI_CustomGeoPosition targeCustomGeoPosition = {
        (dji_f64_t)lonDesiredRad, 
        (dji_f64_t)latDesiredRad, 
        (dji_f32_t)relative_altDesired, 
    };

    // 2. 初始距离检查，防止目标点过远
    // 2.1 获取飞机初始的地理位置（相对于homepoint的高度）
    DJI_CustomGeoPosition originCustomGeoPosition = DjiTest_FlightControlGetValueOfCustomGeoPosition();
    if(originCustomGeoPosition.relative_alt == -1) {
        USER_LOG_ERROR("Relative height is invalid!");
        return false;
    }
    // 2.2 计算目标点和起始点之间的偏移向量
    T_DjiTestFlightControlVector3f offsetToTarget =
    DjiTest_FlightControlLocalOffsetFromCustomGeoPosition(
        targeCustomGeoPosition,
        originCustomGeoPosition);

    // 2.3 距离检查
    float distanceToTarget = DjiTest_FlightControlVectorNorm(offsetToTarget);
    if (distanceToTarget > 20000) {
        USER_LOG_ERROR("The distance between the target point and the origin point is too large (>20km)!");
        return DJI_ERROR_SYSTEM_MODULE_CODE_OUT_OF_RANGE;
    }

    // 3. 自适应更新超时时间
    int expectedTimeInMs = (distanceToTarget / speed) * 1000;
    timeoutInMilSec = min(expectedTimeInMs * 3, timeoutInMilSec); // 取最小值，避免过长的超时
        
    // 4. 设置摇杆控制模式为速度控制模式
    T_DjiFlightControllerJoystickMode joystickMode = joystickMode_vel;
    DjiFlightController_SetJoystickMode(joystickMode);

    // 5. 创建并配置 TargetArrivalMonitor 用于到达检测
    TargetArrivalMonitor arrivalMonitor(cycleTimeInMs);
    arrivalMonitor.SetPosThreshold(posThresholdInM);
    arrivalMonitor.SetTimeRequirements(10 * cycleTimeInMs, 100 * cycleTimeInMs);

    USER_LOG_INFO("Starting flight to target. Distance: %.2f m, Expected time: %.1f s", 
        distanceToTarget, expectedTimeInMs / 1000.0f);

    // 6. 控制循环
    int elapsedTimeInMs        = 0; // 该函数的已执行时间,用于控制循环的超时检测
    while (elapsedTimeInMs < timeoutInMilSec) {
        // 6.1 检查是否需要停止飞行（外部中断标志）
        if (!bool_allowJoystickControl) {
            USER_LOG_INFO("Flight interrupted by stop flag");
            return false;
        }

        // 6.2 获取当前位置
        DJI_CustomGeoPosition currentCustomGeoPosition = DjiTest_FlightControlGetValueOfCustomGeoPosition();
        if (currentCustomGeoPosition.relative_alt == -1) {
            USER_LOG_ERROR("Relative height is invalid!");
            return false;
        }

        // 6.3 计算当前位置到目标位置的偏移向量
        T_DjiTestFlightControlVector3f offsetRemaining = 
        DjiTest_FlightControlLocalOffsetFromCustomGeoPosition(
            targeCustomGeoPosition,
            currentCustomGeoPosition);
        
        // 6.4 计算剩余距离
        float distanceRemaining = DjiTest_FlightControlVectorNorm(offsetRemaining);
        g_cur_pointFlightState.distanceRemaining = distanceRemaining;
        g_cur_pointFlightState.timeRemaining = (distanceRemaining / speed);

        // 6.5 使用到达检测器进行更新
        arrivalMonitor.Update(distanceRemaining);
        // 若满足“到达”条件，则退出循环
        if (arrivalMonitor.checkArrival()) {
            break;
        }

        // 6.6 计算速度向量
        float adaptiveSpeed = speed;
        if (distanceRemaining < speed * 2.5) {
            // 当距离小于2倍速度时，线性降低速度
            adaptiveSpeed = distanceRemaining/2.5;
        }
        
        // 计算三个方向的速度分量
        T_DjiTestFlightControlVector3f speedVector = {
            adaptiveSpeed * offsetRemaining.x / distanceRemaining,
            adaptiveSpeed * offsetRemaining.y / distanceRemaining,
            adaptiveSpeed * offsetRemaining.z / distanceRemaining
        };

        // 6.7 执行操纵杆命令
        T_DjiFlightControllerJoystickCommand joystickCommand = {
            speedVector.x, 
            speedVector.y, 
            speedVector.z, 
            0  // 偏航角为0，保持当前偏航角
        };
        DjiFlightController_ExecuteJoystickAction(joystickCommand);

        // 6.8 等待一个控制周期
        s_osalHandler->TaskSleepMs(cycleTimeInMs);
        elapsedTimeInMs += cycleTimeInMs;

        //  6.9 每5秒记录一次日志，报告当前状态
        if (elapsedTimeInMs % 5000 == 0) {
            USER_LOG_INFO("In flight: Distance remaining: %.2f m, Time elapsed: %.1f s", 
                          distanceRemaining, elapsedTimeInMs / 1000.0f);
        }

        // 6.10 更新全局标志
        g_cur_pointFlightState.isPointControl = true;

    }

    // 7. 检查是否超时
    if (elapsedTimeInMs >= timeoutInMilSec) {
        USER_LOG_ERROR("Task timeout!");
        return DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT;
    }

    // 8. 最终的稳定保持操作：保持当前位置，什么都不做，确保 0 速度以维持稳定
    int brakeTimeAccum          = 0;
    const int brakeTimeMs       = 100 * cycleTimeInMs;    // 稳定保持时间
    while (brakeTimeAccum < brakeTimeMs) {
        if (!bool_allowJoystickControl) {
            USER_LOG_INFO("Flight interrupted by stop flag");
            return false;
        }
        s_osalHandler->TaskSleepMs(cycleTimeInMs);
        brakeTimeAccum += cycleTimeInMs;
    }

    USER_LOG_INFO("基于速度的GPS移动控制结束");

    return true;
}

/**
 * @brief 通过 try-catch 封装的环点飞行控制函数
 * @param latInterest 纬度兴趣点，单位：度
 * @param lonInterest 经度兴趣点，单位：度
 * @param relative_altInterest 相对高度兴趣点，单位：米
 * @param speed 飞行速度，单位：米/秒
 * @param radius 环点半径，单位：米
 * @return DJI SDK 统一返回码
 */
T_DjiReturnCode ManualFlight::CircleFlightTryCatch(double latInterest, 
                                                        double lonInterest, 
                                                        float relative_altInterest, 
                                                        float speed, 
                                                        float radius)
{
    try
    {
        // 调用原始实现
        return DjiTest_FlightControlCircleFlight(latInterest, 
                                                 lonInterest, 
                                                 relative_altInterest, 
                                                 speed, 
                                                 radius);
    }
    catch (const std::exception& e)
    {
        std::cerr << "[CircleFlightTryCatch] std::exception: "
                  << e.what() << std::endl;
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;   // 根据项目实际错误码替换
    }
    catch (...)
    {
        std::cerr << "[CircleFlightTryCatch] 未知异常！" << std::endl;
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;         // 兜底错误码
    }
}

static const double EARTH_RADIUS_M = 6378137.0; // 地球平均半径(米)，用于简化计算

// 计算环绕起点的偏移经纬度
static void CalcOffsetByRadius(
    double latCenterDeg,
    double lonCenterDeg,
    double radiusMeters,
    double &outLatDeg,
    double &outLonDeg
) {
    // 1度纬度约等于 111319 米
    // 1度经度约等于 111319 * cos(lat) 米（在维度lat处）
    double latCenterRad = latCenterDeg * (M_PI / 180.0);
    double dLat = (radiusMeters / 111319.0); 
    double dLon = (radiusMeters / (111319.0 * cos(latCenterRad)));

    // 这里假设“正东方向”作为起始点
    outLatDeg = latCenterDeg;       // 纬度不变
    outLonDeg = lonCenterDeg + dLon; // 经度 + dLon
}

/**
 * @brief 环点飞行控制函数
 * @param latInterest 纬度兴趣点，单位：度
 * @param lonInterest 经度兴趣点，单位：度
 * @param relative_altInterest 相对高度兴趣点，单位：米
 * @param speed 飞行速度，单位：米/秒
 * @param radius 环点半径，单位：米
 * @return DJI SDK 统一返回码
 */
T_DjiReturnCode ManualFlight::DjiTest_FlightControlCircleFlight(double latInterest, 
                                                                double lonInterest, 
                                                                float relative_altInterest, 
                                                                float speed, 
                                                                float radius
)
{
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();
    if (!s_osalHandler) {
        USER_LOG_ERROR("OsalHandler is null!");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    // 0. 参数检查
    if (radius <= 0 || speed <= 0) {
        USER_LOG_ERROR("Invalid radius or speed!");
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    // 1. 先飞到圆周起始点（以圆心正东方向为例）
    double startLatDeg, startLonDeg;
    CalcOffsetByRadius(latInterest, lonInterest, radius, startLatDeg, startLonDeg);

    // 调用MoveByGPS移动到起始点(高度为relative_altInterest，速度用传参，以便更精确)
    USER_LOG_INFO("Moving to circle start position...");
    T_DjiReturnCode returnCode = DjiTest_FlightControlMoveByGPS(
        startLatDeg,
        startLonDeg,
        relative_altInterest,
        speed,   // 移动到起始点时的速度
        1.0f    // 位置阈值，1米左右
    );
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Failed to move to circle start position! code=0x%08X", returnCode);
        return returnCode;
    }
    USER_LOG_INFO("Arrived at circle start position, ready to circle.");

    // 2. 进行环绕飞行
    // 2.1 设置速度控制模式
    T_DjiFlightControllerJoystickMode joystickMode = joystickMode_vel;
    DjiFlightController_SetJoystickMode(joystickMode);

    // 2.2 计算一圈理论时间
    double circleCircumference = 2.0 * M_PI * radius;    // 圆周长度
    double circleTimeSec = circleCircumference / speed;  // 单圈理论时间(s)
    USER_LOG_INFO("Circle flight begin. One circle time ~ %.1f s", circleTimeSec);

    // 2.3 定义控制循环相关参数
    const int controlFreqInHz  = 50;  
    const int cycleTimeInMs    = 1000 / controlFreqInHz;
    float k_r = 0.3f;    // 径向修正增益(可根据测试调整)
    float k_alt = 0.5f;  // 高度修正增益(可根据测试调整)

    // 2.4 将圆心做成 DJI_CustomGeoPosition 以方便计算局部偏移
    DJI_CustomGeoPosition centerPos;
    centerPos.longitude = lonInterest * (M_PI/180.0);   // 注意：传入单位是弧度
    centerPos.latitude  = latInterest * (M_PI/180.0);
    centerPos.relative_alt = relative_altInterest;

    // 2.5 进入循环，时间控制：期望做一圈(或多圈)
    //     这里仅做一圈示例，若需多圈可放大倍数，修改：最多环绕100圈
    int totalTimeMs = static_cast<int>((100 * circleTimeSec + 3) * 1000); // +3秒冗余
    int elapsedTimeMs = 0;

    while (elapsedTimeMs < totalTimeMs) {
        // a) 检查外部停止标志
        if (!bool_allowJoystickControl) {
            USER_LOG_WARN("Circle flight is interrupted by external stop.");
            break;
        }

        // b) 获取当前飞机位置
        DJI_CustomGeoPosition currentPos = DjiTest_FlightControlGetValueOfCustomGeoPosition();
        if (currentPos.relative_alt == -1) {
            USER_LOG_ERROR("Relative height is invalid, break circle flight.");
            break;
        }

        // c) 计算相对于圆心的局部偏移 (x,y,z)
        T_DjiTestFlightControlVector3f offset = 
            DjiTest_FlightControlLocalOffsetFromCustomGeoPosition(centerPos, currentPos);
        float x = offset.x;
        float y = offset.y;
        float z = offset.z; // 相对高度误差 centerPos 这边假设0的话 z 就是 current 相对 alt ?

        // d) 计算当前半径 r = sqrt(x^2 + y^2)，以及径向方向、切向方向
        float r = sqrtf(x*x + y*y);
        // 若 r == 0，说明刚好在圆心上，需要特殊处理，这里做个保护
        if (r < 0.001f) {
            USER_LOG_WARN("Drone is extremely close to center, skip small r.");
            s_osalHandler->TaskSleepMs(cycleTimeInMs);
            elapsedTimeMs += cycleTimeInMs;
            continue;
        }
        float dx = x / r;  // 径向单位向量
        float dy = y / r;

        // 逆时针的切向单位向量 ( -dy, dx )；若想顺时针就反过来 ( dy, -dx )
        float tx = -dy;
        float ty = dx;

        // e) 计算径向误差并生成径向修正速度
        float radialErr = (r - radius);
        float radialV   = k_r * radialErr; // 径向修正速度大小(可能为正或负)

        // f) 合成水平速度：切向 + 径向修正
        //   切向速度大小恒为 speed
        float vx = speed * tx + radialV * dx;
        float vy = speed * ty + radialV * dy;

        // ==== 这里加一个水平速度的限幅 ====
        {
            // 希望水平速度总和不超过 20 m/s
            float maxHorizontalSpeed = 20.0f;
            float horizontalSpeed = sqrtf(vx * vx + vy * vy);
            if (horizontalSpeed > maxHorizontalSpeed) {
                float scale = maxHorizontalSpeed / horizontalSpeed;
                vx *= scale;
                vy *= scale;
            }
        }

        // g) 高度保持：简单P控制
        float altErr   = relative_altInterest - currentPos.relative_alt;
        float vz       = k_alt * altErr; 

        // ==== 对垂直速度限幅 ====
        // 比如不希望垂直速度超过 5 m/s
        {
            float maxVerticalSpeed = 5.0f;
            if (vz >  maxVerticalSpeed)  vz =  maxVerticalSpeed;
            if (vz < -maxVerticalSpeed)  vz = -maxVerticalSpeed;
        }

        // h) 发送速度控制指令
        T_DjiFlightControllerJoystickCommand joystickCmd = {
            vx,    // ENU x
            vy,    // ENU y
            vz,    // 速度向上为正
            0      // 偏航不做控制(或者您也可以根据需求设置某个偏航)
        };
        DjiFlightController_ExecuteJoystickAction(joystickCmd);

        // i) 休眠一个控制周期
        s_osalHandler->TaskSleepMs(cycleTimeInMs);
        elapsedTimeMs += cycleTimeInMs;

        // j) 可适时打印调试信息
        if (elapsedTimeMs % 2000 == 0) {
            USER_LOG_INFO("Circling... r=%.1f (desired=%.1f), alt=%.1f (desired=%.1f), t=%.1f s",
                          r, (double)radius,
                          currentPos.relative_alt, (double)relative_altInterest,
                          elapsedTimeMs/1000.0f);
        }

        // k) 更新全局状态
        g_cur_circleFlightState.isCircleControl = true;
    }

    // 3. 收尾：可让速度归零，使飞机悬停
    //    简单做一个“刹车”过程
    USER_LOG_INFO("Stop circle motion, letting drone brake...");
    int brakeTimeMs = 2000; // 2s
    int brakeElapsed = 0;
    while (brakeElapsed < brakeTimeMs) {
        if (!bool_allowJoystickControl) {
            USER_LOG_INFO("Circle flight is interrupted during brake.");
            break;
        }
        T_DjiFlightControllerJoystickCommand joystickCmd = {0, 0, 0, 0};
        DjiFlightController_ExecuteJoystickAction(joystickCmd);

        s_osalHandler->TaskSleepMs(cycleTimeInMs);
        brakeElapsed += cycleTimeInMs;
    }

    USER_LOG_INFO("Circle flight finished.");
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


T_DjiReturnCode ManualFlight::DjiTest_GimbalFollowTarget(double lat, double lon, float height)
{
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler(); // 获取OSAL处理器

     // 1. 构建目标位置结构体（注意经纬度顺序）
     /* 这里高度设为与homepoint一致，假设这就是地面 */
     double latDesiredRad = lat * s_degToRad;
     double lonDesiredRad = lon * s_degToRad;
     DJI_CustomGeoPosition targeCustomGeoPosition = {
         (dji_f64_t)lonDesiredRad, 
         (dji_f64_t)latDesiredRad, 
         (dji_f32_t)0.0f,
     };

     // 2. 获取飞机当前的地理位置（相对于homepoint的高度）
    DJI_CustomGeoPosition currentCustomGeoPosition = DjiTest_FlightControlGetValueOfCustomGeoPosition();
    if(currentCustomGeoPosition.relative_alt == -1) {
        USER_LOG_ERROR("Relative height is invalid!");
        return false;
    }

    // 2.1 保证目标至少位于无人机下方3m
    if (currentCustomGeoPosition.relative_alt - targeCustomGeoPosition.relative_alt < 3) {
        USER_LOG_WARN("假设的目标高度过低，无法跟随");
        targeCustomGeoPosition.relative_alt = currentCustomGeoPosition.relative_alt - 3;
        return false;
    }

    // 3. 计算当前位置到目标位置的偏移向量
    T_DjiTestFlightControlVector3f offsetRemaining =
    DjiTest_FlightControlLocalOffsetFromCustomGeoPosition(
        targeCustomGeoPosition,
        currentCustomGeoPosition);

    // 4. 计算云台需要的俯仰角和偏航角
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;

    // 4.1 俯仰角计算：atan2(z, sqrt(x*x + y*y))，将弧度转为度
    pitch = atan2f(offsetRemaining.z, sqrtf(offsetRemaining.x * offsetRemaining.x + 
        offsetRemaining.y * offsetRemaining.y)) * (180.0f / M_PI);

    // 4.2 偏航角计算：atan2(y, x)，将弧度转为度
    yaw = atan2f(offsetRemaining.y, offsetRemaining.x) * (180.0f / M_PI);

    // 5. 限制角度范围
    // 限制：Pitch  Yaw  Roll：(-120, 30)  (-310, 310)  (-45, 45)
    if (pitch < -120) {
        pitch = -120;
    } else if (pitch > 30) {
        pitch = 30;
    }
    
    if (yaw < -310) {
        yaw = -310;
    } else if (yaw > 310) {
        yaw = 310;
    }
    
    if (roll < -45) {
        roll = -45;
    } else if (roll > 45) {
        roll = 45;
    }

    // 6. 获取当前云台位置
    // 获取云台角度
    T_DjiFcSubscriptionGimbalAngles gimbalAngles = {0};             // 云台角度
    T_DjiDataTimestamp timestamp = {0};                             // 时间戳
    returnCode = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
            (uint8_t *) &gimbalAngles,
            sizeof(T_DjiFcSubscriptionGimbalAngles),
            &timestamp);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
        USER_LOG_WARN("TOPIC_GIMBAL_ANGLES: failed to get value.");
        return returnCode;
    }

    // x. 定义一个简单的CLAMP宏
    #define CLAMP(val, minVal, maxVal) \
    ((val) < (minVal) ? (minVal) : ((val) > (maxVal) ? (maxVal) : (val)))

    // x1. 获取当前云台角度(示例省略获取过程，假设已放在 gimbalAngles.x, y, z)
    float currentPitch = gimbalAngles.x;
    float currentRoll  = gimbalAngles.y;
    float currentYaw   = gimbalAngles.z;

    // x2. 计算各轴差值
    float diffPitch = fabsf(pitch - currentPitch);
    float diffRoll  = fabsf(roll  - currentRoll);
    float diffYaw   = fabsf(yaw   - currentYaw);

    // x3. 各轴最大理论范围（已限制角度，因此不会超）
    float maxPitchRange = 150.0f; // (-120 ~ 30)
    float maxRollRange  = 90.0f;  // (-45 ~ 45)
    float maxYawRange   = 620.0f; // (-310 ~ 310)

    // x4. 映射到 [0.2, 1.0]
    float pitchTime = CLAMP(0.2f + 0.8f * diffPitch / maxPitchRange, 0.2f, 1.0f);
    float rollTime  = CLAMP(0.2f + 0.8f * diffRoll  / maxRollRange,  0.2f, 1.0f);
    float yawTime   = CLAMP(0.2f + 0.8f * diffYaw   / maxYawRange,   0.2f, 1.0f);

    // x5. 取三轴中最大值作为整体旋转耗时
    float rotateTime = fmaxf(pitchTime, fmaxf(rollTime, yawTime));

    // 7. 调用云台控制API
    T_DjiGimbalManagerRotation rotation;
    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE; // 绝对角度模式
    rotation.pitch = pitch;
    rotation.roll = roll;
    rotation.yaw = yaw;
    rotation.time = rotateTime; // 旋转时间
    E_DjiMountPosition mountPosition = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
    returnCode = DjiGimbalManager_Rotate(mountPosition, rotation);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Target gimbal pry = (%.1f, %.1f, %.1f) failed, error code: 0x%08X",
                       rotation.pitch, rotation.roll, rotation.yaw,
                       returnCode);
    }

    s_osalHandler->TaskSleepMs(rotateTime * 1000); // 等待旋转完成

    return returnCode;

}


bool ManualFlight::Dji_MoveBySelfOffset(
    const T_DjiTestFlightControlVector3f selfOffsetDesired,
    float yawDesiredInDeg)
{
    // 设置摇杆模式为位置控制
    DjiFlightController_SetJoystickMode(joystickMode_pos);

    // 构造并发送摇杆指令
    T_DjiFlightControllerJoystickCommand cmd = {
        selfOffsetDesired.x,  // X 方向偏移（Body 轴）
        selfOffsetDesired.y,  // Y 方向偏移（Body 轴）
        selfOffsetDesired.z,  // 高度（相对 Home 点）
        yawDesiredInDeg       // 偏航角（单位：度）
    };

    T_DjiReturnCode ret = DjiFlightController_ExecuteJoystickAction(cmd);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Joystick action failed, err = 0x%08X", ret);
        return false;
    }

    return true;
}

bool ManualFlight::FollowHost(float hostLatDeg, float hostLonDeg, float hostAltM,
                              float offsetDirX, float offsetDirY, float offsetDirZ)
{
    // 1. 获取飞机当前位置
    DJI_CustomGeoPosition currentCustomGeoPosition = DjiTest_FlightControlGetValueOfCustomGeoPosition();
    if (currentCustomGeoPosition.relative_alt == -1) {
        USER_LOG_ERROR("Relative height is invalid!");
        return false;
    }

    // 2. 构建主机位置
    double hostLatRad = hostLatDeg * s_degToRad;
    double hostLonRad = hostLonDeg * s_degToRad;
    DJI_CustomGeoPosition hostCustomGeoPosition = {
        (dji_f64_t)hostLonRad,
        (dji_f64_t)hostLatRad,
        (dji_f32_t)hostAltM // 相对高度
    };

    // 3. 计算自身相对于主机的偏移量
    T_DjiTestFlightControlVector3f offsetFromHost = 
        DjiTest_FlightControlLocalOffsetFromCustomGeoPosition(
            currentCustomGeoPosition,hostCustomGeoPosition);

    // 4. 期望偏移量-当前偏移量 = 控制偏移量
    T_DjiTestFlightControlVector3f desiredOffset = {offsetDirX, offsetDirY, offsetDirZ};
    T_DjiTestFlightControlVector3f offsetRemaining = 
        DjiTest_FlightControlVector3FSub(desiredOffset, offsetFromHost);

    offsetRemaining.z = hostAltM + offsetDirZ; // 高度单独计算

    // 根据控制偏移量生成摇杆指令
    Dji_MoveBySelfOffset(offsetRemaining, 0.0f); // 偏航角设为0

}