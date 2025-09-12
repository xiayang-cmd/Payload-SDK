#include "common_utils.h"

/* 私有类型和变量定义 -------------------------------------------------------*/

// 结构体用于映射飞控显示模式的枚举值与对应的字符串描述
typedef struct {
    E_DjiFcSubscriptionDisplayMode displayMode;
    char *displayModeStr;
} T_DjiTestFlightControlDisplayModeStr;

// 飞控显示模式与对应字符串描述的映射表
static const T_DjiTestFlightControlDisplayModeStr s_flightControlDisplayModeStr[] = {
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_MANUAL_CTRL, .displayModeStr = "manual control mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE, .displayModeStr = "attitude mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS, .displayModeStr = "p_gps mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_HOTPOINT_MODE, .displayModeStr = "hotpoint mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF, .displayModeStr = "assisted takeoff mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF, .displayModeStr = "auto takeoff mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING, .displayModeStr = "auto landing mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME, .displayModeStr = "go home mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_SDK_CTRL, .displayModeStr = "SDK control mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING, .displayModeStr = "force landing mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_SEARCH_MODE, .displayModeStr = "search mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START, .displayModeStr = "engine start mode"},

    // 未知模式
    {.displayMode = (E_DjiFcSubscriptionDisplayMode)0x42, .displayModeStr = "unknown mode"}
};


/* 全局函数实现 -------------------------------------------------------------*/
/**
 * @brief 检查指定的文件是否存在
 * @param name 文件名（包含路径）
 * @return bool 文件存在返回true，否则返回false
 */
bool IsFileExist(string& name) {
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

/**
 * @brief 四元数转欧拉角
 * @param quaternion 四元数
 * @param pitch 欧拉角pitch
 * @param roll 欧拉角roll
 * @param yaw 欧拉角yaw
 */
void QuaternionToEuler(const T_DjiFcSubscriptionQuaternion* quaternion, dji_f64_t* pitch, dji_f64_t* roll, dji_f64_t* yaw) {
    // 四元数到欧拉角的转换公式
    *pitch = (dji_f64_t) asinf(-2 * quaternion->q1 * quaternion->q3 + 2 * quaternion->q0 * quaternion->q2) * 57.3;
    *roll = (dji_f64_t) atan2f(2 * quaternion->q2 * quaternion->q3 + 2 * quaternion->q0 * quaternion->q1,
                               -2 * quaternion->q1 * quaternion->q1 - 2 * quaternion->q2 * quaternion->q2 + 1) * 57.3;
    *yaw = (dji_f64_t) atan2f(2 * quaternion->q1 * quaternion->q2 + 2 * quaternion->q0 * quaternion->q3,
                              -2 * quaternion->q2 * quaternion->q2 - 2 * quaternion->q3 * quaternion->q3 + 1) * 57.3;
}

/**
 * @brief 检查飞机是否在空中
 * @return 飞机在空中返回true，否则返回false
 */
bool checkUavIsAir(){
    if(DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR){
        return true;
    }else{
        return false;
    }
}

// 获取当前飞行状态
/**
 * @brief 获取飞行状态
 * @return 飞行状态
 */
T_DjiFcSubscriptionFlightStatus DjiTest_FlightControlGetValueOfFlightStatus(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionFlightStatus flightStatus;
    T_DjiDataTimestamp flightStatusTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                      (uint8_t *) &flightStatus,
                                                      sizeof(T_DjiFcSubscriptionFlightStatus),
                                                      &flightStatusTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic flight status error, error code: 0x%08X", djiStat);
        flightStatus = 0;
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", flightStatusTimestamp.millisecond,
                       flightStatusTimestamp.microsecond);
        USER_LOG_DEBUG("Flight status: %d.", flightStatus);
    }

    return flightStatus;
}

/**
 * @brief 获取当前GPS位置信息
 * @param gpsPosition 指向存储GPS位置信息的结构体的指针
 * @return T_DjiReturnCode 返回操作状态码
 */
T_DjiReturnCode DjiDemo_GetCurrentGpsPosition(T_DjiFcSubscriptionGpsPosition *gpsPosition)
{
    T_DjiReturnCode djiStat;
    T_DjiDataTimestamp timestamp = {0};

    if (gpsPosition == NULL) {
        USER_LOG_ERROR("GPS position pointer is NULL");
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                    (uint8_t *)gpsPosition,
                                                    sizeof(T_DjiFcSubscriptionGpsPosition),
                                                    &timestamp);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic gps error, error code: 0x%08X", djiStat);
        return djiStat;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief 获取飞行器的GPS位置和海拔高度(融合数据)
 * @return T_DjiFcSubscriptionPositionFused GPS位置和海拔高度
 */
T_DjiFcSubscriptionPositionFused DjiTest_FlightControlGetValueOfPositionFused(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionPositionFused positionFused = {0};
    T_DjiDataTimestamp positionFusedTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                                      (uint8_t *) &positionFused,
                                                      sizeof(T_DjiFcSubscriptionPositionFused),
                                                      &positionFusedTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic position fused error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", positionFusedTimestamp.millisecond,
                       positionFusedTimestamp.microsecond);
        USER_LOG_DEBUG("PositionFused: %f, %f,%f,%d.", positionFused.latitude, positionFused.longitude,
                       positionFused.altitude, positionFused.visibleSatelliteNumber);
    }

    return positionFused;
}

/**
 * @brief 获取飞行器的相对home点的高度
 * @return dji_f32_t 相对高度
 */
dji_f32_t DjiTest_FlightControlGetValueOfRelativeHeight(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionAltitudeFused altitudeFused = 0;
    T_DjiFcSubscriptionAltitudeOfHomePoint homePointAltitude = 0;
    dji_f32_t relativeHeight = 0;
    T_DjiDataTimestamp relativeHeightTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                                      (uint8_t *) &homePointAltitude,
                                                      sizeof(T_DjiFcSubscriptionAltitudeOfHomePoint),
                                                      &relativeHeightTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic altitude of home point error, error code: 0x%08X", djiStat);
        return -1;
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", relativeHeightTimestamp.millisecond,
                       relativeHeightTimestamp.microsecond);
    }

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                                      (uint8_t *) &altitudeFused,
                                                      sizeof(T_DjiFcSubscriptionAltitudeFused),
                                                      &relativeHeightTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic altitude fused error, error code: 0x%08X", djiStat);
        return -1;
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", relativeHeightTimestamp.millisecond,
                       relativeHeightTimestamp.microsecond);
    }

    relativeHeight = altitudeFused - homePointAltitude;

    return relativeHeight;
}

/**
 * @brief 获取飞行器的自定义地理位置
 * @return DJI_CustomGeoPosition 自定义地理位置
 */
DJI_CustomGeoPosition DjiTest_FlightControlGetValueOfCustomGeoPosition(void)
{
    T_DjiReturnCode djiStat;
    DJI_CustomGeoPosition customGeoPosition = {0};

    T_DjiFcSubscriptionPositionFused positionFused = DjiTest_FlightControlGetValueOfPositionFused();
    dji_f32_t relativeHeight = DjiTest_FlightControlGetValueOfRelativeHeight();

    customGeoPosition.latitude = positionFused.latitude;
    customGeoPosition.longitude = positionFused.longitude;
    customGeoPosition.relative_alt = relativeHeight;
    
    return customGeoPosition;
}

/**
 * @brief 获取飞行器的四元数
 * @return T_DjiFcSubscriptionQuaternion 四元数
 */
T_DjiFcSubscriptionQuaternion DjiTest_FlightControlGetValueOfQuaternion(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionQuaternion quaternion = {0};
    T_DjiDataTimestamp quaternionTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                      (uint8_t *) &quaternion,
                                                      sizeof(T_DjiFcSubscriptionQuaternion),
                                                      &quaternionTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic quaternion error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", quaternionTimestamp.millisecond,
                       quaternionTimestamp.microsecond);
        USER_LOG_DEBUG("Quaternion: %f %f %f %f.", quaternion.q0, quaternion.q1, quaternion.q2, quaternion.q3);
    }

    return quaternion;
}

/**
 * @brief 四元数转欧拉角
 */
T_DjiTestFlightControlVector3f DjiTest_FlightControlQuaternionToEulerAngle(const T_DjiFcSubscriptionQuaternion quat)
{
    T_DjiTestFlightControlVector3f eulerAngle;
    double q2sqr = quat.q2 * quat.q2;
    double t0 = -2.0 * (q2sqr + quat.q3 * quat.q3) + 1.0;
    double t1 = (dji_f64_t) 2.0 * (quat.q1 * quat.q2 + quat.q0 * quat.q3);
    double t2 = -2.0 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);
    double t3 = (dji_f64_t) 2.0 * (quat.q2 * quat.q3 + quat.q0 * quat.q1);
    double t4 = -2.0 * (quat.q1 * quat.q1 + q2sqr) + 1.0;
    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;
    eulerAngle.x = asin(t2);
    eulerAngle.y = atan2(t3, t4);
    eulerAngle.z = atan2(t1, t0);
    return eulerAngle;
}

/**
 * @brief 计算向量偏移
 * @param vectorA 向量A
 * @param vectorB 向量B
 * @return T_DjiTestFlightControlVector3f 向量偏移
 */
T_DjiTestFlightControlVector3f DjiTest_FlightControlVector3FSub(const T_DjiTestFlightControlVector3f vectorA,
    const T_DjiTestFlightControlVector3f vectorB)
{
T_DjiTestFlightControlVector3f result;
result.x = vectorA.x - vectorB.x;
result.y = vectorA.y - vectorB.y;
result.z = vectorA.z - vectorB.z;
return result;
}

/**
 * @brief 计算向量的模
 * @param v 向量
 * @return dji_f32_t 向量的模
 */
dji_f32_t DjiTest_FlightControlVectorNorm(T_DjiTestFlightControlVector3f v)
{
    return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

/**
 * @brief 计算两个经纬度点之间的距离向量
 * @param target 目标点(经纬度，单位: 弧度)
 * @param origin 原点（经纬度，单位: 弧度）
 * @param targetHeight 目标点高度
 * @param originHeight 原点高度
 * @return T_DjiTestFlightControlVector3f 距离向量
 */
T_DjiTestFlightControlVector3f DjiTest_FlightControlLocalOffsetFromGpsAndFusedHeightOffset(const T_DjiFcSubscriptionPositionFused target,
                                                            const T_DjiFcSubscriptionPositionFused origin,
                                                            const dji_f32_t targetHeight,
                                                            const dji_f32_t originHeight)
{
    T_DjiTestFlightControlVector3f deltaNed;
    double deltaLon = target.longitude - origin.longitude;
    double deltaLat = target.latitude - origin.latitude;
    deltaNed.x = deltaLat * s_earthCenter;
    deltaNed.y = deltaLon * s_earthCenter * cos(target.latitude);
    deltaNed.z = targetHeight - originHeight;

    return deltaNed;
}

/**
 * @brief 计算两个自定义格式的经纬度点之间的距离向量
 * @param target 目标点（自定义地理位置）
 * @param origin 原点（自定义地理位置）
 * @return T_DjiTestFlightControlVector3f 距离向量：原点指向目标点的向量
 */
T_DjiTestFlightControlVector3f DjiTest_FlightControlLocalOffsetFromCustomGeoPosition(
    const DJI_CustomGeoPosition target,
    const DJI_CustomGeoPosition origin)
{
    T_DjiTestFlightControlVector3f deltaNed;
    double deltaLon = target.longitude - origin.longitude;
    double deltaLat = target.latitude - origin.latitude;
    deltaNed.x = deltaLat * s_earthCenter;
    deltaNed.y = deltaLon * s_earthCenter * cos(target.latitude);
    deltaNed.z = target.relative_alt - origin.relative_alt;

    return deltaNed;
}

/**
 * @brief 获取当前时间字符串，格式类似 "2025-02-25 12:00:00" (C++11 兼容)
 * @return std::string
 */
std::string getCurrentTimeString()
{
    // 1. 获取当前系统时钟对应的 time_point
    auto now = std::chrono::system_clock::now();

    // 2. 转换为 time_t
    auto time_t_now = std::chrono::system_clock::to_time_t(now);

    // 3. 使用平台相关的 localtime
    std::tm tm_buf;
#if defined(_WIN32) || defined(_WIN64)
    localtime_s(&tm_buf, &time_t_now);  // Windows
#else
    localtime_r(&time_t_now, &tm_buf);  // Linux / Unix
#endif

    // 4. 格式化输出到缓冲区
    char buffer[64] = {0};
    // 格式: "YYYY-MM-DD HH:MM:SS"
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm_buf);

    return std::string(buffer);
}

/**
 * @brief 获取当前时间戳 (单位: 毫秒, C++11 兼容)
 * @return uint64_t - 毫秒级时间戳（自 1970-01-01 00:00:00 UTC 开始）
 */
uint64_t getCurrentTimestampMillis()
{
    // 1. 获取当前系统时钟对应的 time_point
    auto now = std::chrono::system_clock::now();

    // 2. 转换为自 1970-01-01 00:00:00 UTC 开始的毫秒数
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()
              ).count();

    // 3. 返回 uint64_t
    return static_cast<uint64_t>(ms);
}

/**
 * @brief 获取当前时间戳 (单位: 秒, C++11 兼容)
 * @return uint32_t - 秒级时间戳（自 1970-01-01 00:00:00 UTC 开始）
 */
uint32_t getCurrentTimestampSeconds()
{
    // 1. 获取当前系统时钟对应的 time_point
    auto now = std::chrono::system_clock::now();

    // 2. 转换为自 1970-01-01 00:00:00 UTC 开始的秒数
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(
                       now.time_since_epoch()
                   ).count();

    // 3. 返回 uint32_t
    return static_cast<uint32_t>(seconds);
}

/**
 * @brief 填充心跳响应数据
 * @param responseHeartbeat 响应数据数组（长度需为 13）
 */
void fillHeartbeatResponse(uint8_t responseHeartbeat[13]) {
    // 获取当前时间戳（从1970年1月1日0点开始的秒数）
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    uint64_t timestamp = std::chrono::duration_cast<std::chrono::seconds>(duration).count();

    // 将时间戳转换为大端字节序并填充到 responseHeartbeat 数组中
    for (int i = 0; i < 8; i++) {
        responseHeartbeat[5 + i] = (timestamp >> ((7 - i) * 8)) & 0xFF;
    }
}

/**
 * @brief 大端转换函数：从大端字节数组转换为 float
 * @param data 大端字节数组
 * @return float 转换后的浮点数
 */
float bigEndianToFloat(const uint8_t* data) {
    uint32_t temp = (static_cast<uint32_t>(data[0]) << 24) |
                    (static_cast<uint32_t>(data[1]) << 16) |
                    (static_cast<uint32_t>(data[2]) << 8)  |
                    (static_cast<uint32_t>(data[3]));
    float result;
    std::memcpy(&result, &temp, sizeof(float)); // 将 uint32_t 数据复制为 float
    return result;
}

/**
 * @brief 大端转换函数：从大端字节数组转换为 double
 * @param data 大端字节数组
 * @return double 转换后的双精度浮点数
 */
double bigEndianToDouble(const uint8_t* data) {
    uint64_t temp = (static_cast<uint64_t>(data[0]) << 56) |
                    (static_cast<uint64_t>(data[1]) << 48) |
                    (static_cast<uint64_t>(data[2]) << 40) |
                    (static_cast<uint64_t>(data[3]) << 32) |
                    (static_cast<uint64_t>(data[4]) << 24) |
                    (static_cast<uint64_t>(data[5]) << 16) |
                    (static_cast<uint64_t>(data[6]) << 8)  |
                    (static_cast<uint64_t>(data[7]));
    double result;
    std::memcpy(&result, &temp, sizeof(double)); // 将 uint64_t 数据复制为 double
    return result;
}

/**
 * @brief 将 float 转换为大端字节数组
 * @param value 要转换的 float 值
 * @param data 输出字节数组（长度需为 4）
 */
void floatToBigEndian(float value, uint8_t* data) {
    // 1. 先将 float 复制到 uint32_t
    uint32_t temp;
    std::memcpy(&temp, &value, sizeof(float));

    // 2. 按照大端顺序写入输出数组
    data[0] = static_cast<uint8_t>((temp >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((temp >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((temp >>  8) & 0xFF);
    data[3] = static_cast<uint8_t>((temp      ) & 0xFF);
}

/**
 * @brief 将 double 转换为大端字节数组
 * @param value 要转换的 double 值
 * @param data 输出字节数组（长度需为 8）
 */
void doubleToBigEndian(double value, uint8_t* data) {
    // 1. 先将 double 复制到 uint64_t
    uint64_t temp;
    std::memcpy(&temp, &value, sizeof(double));

    // 2. 按照大端顺序写入输出数组
    data[0] = static_cast<uint8_t>((temp >> 56) & 0xFF);
    data[1] = static_cast<uint8_t>((temp >> 48) & 0xFF);
    data[2] = static_cast<uint8_t>((temp >> 40) & 0xFF);
    data[3] = static_cast<uint8_t>((temp >> 32) & 0xFF);
    data[4] = static_cast<uint8_t>((temp >> 24) & 0xFF);
    data[5] = static_cast<uint8_t>((temp >> 16) & 0xFF);
    data[6] = static_cast<uint8_t>((temp >>  8) & 0xFF);
    data[7] = static_cast<uint8_t>((temp      ) & 0xFF);
}

/**
 * @brief 将 uint64_t 转换为大端字节数组
 * @param value 要转换的 64 位无符号整数
 * @param data 输出字节数组（长度需为 8）
 */
void uint64ToBigEndian(uint64_t value, uint8_t* data) {
    data[0] = static_cast<uint8_t>((value >> 56) & 0xFF);
    data[1] = static_cast<uint8_t>((value >> 48) & 0xFF);
    data[2] = static_cast<uint8_t>((value >> 40) & 0xFF);
    data[3] = static_cast<uint8_t>((value >> 32) & 0xFF);
    data[4] = static_cast<uint8_t>((value >> 24) & 0xFF);
    data[5] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data[6] = static_cast<uint8_t>((value >>  8) & 0xFF);
    data[7] = static_cast<uint8_t>((value      ) & 0xFF);
}

/**
 * @brief 将 uint32_t 转换为大端字节数组
 * @param value 要转换的 32 位无符号整数
 * @param data 输出字节数组（长度需为 4）
 */
void uint32ToBigEndian(uint32_t value, uint8_t* data) {
    data[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((value >>  8) & 0xFF);
    data[3] = static_cast<uint8_t>((value      ) & 0xFF);
}

/**
 * @brief 将 int32_t 转换为大端字节数组
 * @param value 要转换的 32 位有符号整数
 * @param data 输出字节数组（长度需为 4）
 */
void int32ToBigEndian(int32_t value, uint8_t* data) {
    data[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((value >>  8) & 0xFF);
    data[3] = static_cast<uint8_t>((value      ) & 0xFF);
}

/**
 * @brief 统一回复帧的格式
 * @param action_id 动作编号
 * @param error_code 错误码
 * @return std::vector<uint8_t> 回复帧
 * @note 各数据段的长度均为固定值
 * @note 帧头2字节，数据长度2字节，指令号1字节，加密标志1字节，动作编号1字节，执行结果1字节，错误码4字节，云盒编号15字节
 */
std::vector<uint8_t> reply_frame(uint8_t action_id, T_DjiReturnCode error_code)
{
    std::vector<uint8_t> frame;
    frame.clear();
    
    // 构建回复帧
    frame.push_back(0x6A);
    frame.push_back(0x77);      // 0.帧头
    frame.push_back(0x00);
    frame.push_back(0x08);      // 1.数据长度
    frame.push_back(0xD1);      // 2.指令号
    frame.push_back(0x00);      // 3.加密标志
    frame.push_back(action_id); // 4.动作编号
    
    // 5.执行结果：成功为1，失败为0
    if (error_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        frame.push_back(0x00);  // 执行失败
    } else {
        frame.push_back(0x01);  // 执行成功
    }
    
    // 6.错误码：32位，分4个字节存储
    // frame.push_back((error_code >> 32) & 0xFF); // 将64位错误码转换为自定义32位，保留第32-40的8位和低24位,暂时不改
    frame.push_back((error_code >> 24) & 0xFF);
    frame.push_back((error_code >> 16) & 0xFF);
    frame.push_back((error_code >> 8) & 0xFF);
    frame.push_back(error_code & 0xFF);
    
    // 7.云盒编号：15字节
    // 插入云盒编号:DBM250974065008(15字节)
    const char str3[15] = {'D', 'B', 'M', '2', '5', '0', '9', '7', '4', '0', '6', '5', '0', '0', '8'};
    frame.insert(frame.end(), str3, str3+15);
    
    return frame;
}


/**
 * @brief 构建图片URL通知帧
 * @param photo_url   [in] 图片原图地址
 * @param longitude   [in] 经度
 * @param latitude    [in] 纬度
 * @param timestamp   [in] 时间戳(单位:毫秒 或者秒都可以，根据实际需求确定)
 * @param originalSize[in] 原图大小
 * @param encrypt     [in] true: 加密, false: 不加密
 * @return std::vector<uint8_t> 
 */
std::vector<uint8_t> reply_photourl_frame(
    const std::string &photo_url,
    double longitude,
    double latitude,
    uint64_t timestamp,
    uint64_t originalSize,
    bool encrypt = false
)
{
    // 大端转换缓存
    uint8_t buf[8];

    // 固定的云盒SN号(15字节) - 根据需要可提为参数
    const char cloudBoxSn[15] = {
        'D','B','M','2','5','0','9','7','4','0','6','5','0','0','8'
    };

    // 1. 指令编号、加密标志
    const uint8_t CMD_ID       = 0x09;
    uint8_t encryptFlag        = encrypt ? 0x01 : 0x00;

    // 2. 准备要写入的帧数据(不含帧头与长度，稍后再插入)
    std::vector<uint8_t> payload;
    // 指令编号(1B)
    payload.push_back(CMD_ID);
    // 加密标志(1B)
    payload.push_back(encryptFlag);

    // 经度(8B)
    doubleToBigEndian(longitude, buf);
    payload.insert(payload.end(), buf, buf + 8);

    // 纬度(8B)
    doubleToBigEndian(latitude, buf);
    payload.insert(payload.end(), buf, buf + 8);

    // 时间戳(8B)
    uint64ToBigEndian(timestamp, buf);
    payload.insert(payload.end(), buf, buf + 8);

    // 原图大小(8B)
    uint64ToBigEndian(originalSize, buf);
    payload.insert(payload.end(), buf, buf + 8);

    // 云盒SN号(15B)
    payload.insert(payload.end(), cloudBoxSn, cloudBoxSn + 15);

    // 原图地址(NB)
    payload.insert(payload.end(), photo_url.begin(), photo_url.end());

    // 3. 构建最终帧
    //    帧头(2B)  + 数据长度(2B) + payload
    std::vector<uint8_t> frame;
    frame.reserve(2 + 2 + payload.size());

    // (a) 帧头(2B)
    frame.push_back(0x6A);
    frame.push_back(0x77);

    // (b) 数据长度(2B) - 先占位，后面回填
    frame.push_back(0x00);
    frame.push_back(0x00);

    // (c) 将 payload 插入
    frame.insert(frame.end(), payload.begin(), payload.end());

    // 4. 回填数据长度 
    //    dataLength = payload.size() (从 指令编号到最后)
    //    不包含帧头(2B)与数据长度(2B)本身
    uint16_t dataLength = static_cast<uint16_t>(payload.size());
    frame[2] = (dataLength >> 8) & 0xFF;
    frame[3] = dataLength & 0xFF;

    return frame;
}

/**
 * @brief 构建激光测距回复帧
 * @param longitude [in] 经度
 * @param latitude  [in] 纬度
 * @param height    [in] 高度
 * @param distance  [in] 距离
 * @return std::vector<uint8_t>
 */
std::vector<uint8_t> reply_laser_frame(double longitude, double latitude, float height, float distance, T_DjiReturnCode returnCode)
{
    // 大端转换缓存
    uint8_t buf[8];

    // 固定的云盒SN号(15字节) - 根据需要可提为参数
    const char cloudBoxSn[15] = {
        'D','B','M','2','5','0','9','7','4','0','6','5','0','0','8'
    };

    // 1. 指令编号、加密标志
    const uint8_t CMD_ID       = 0xD1;
    const uint8_t encryptFlag  = 0x00;

    // 2. 准备要写入的帧数据(不含帧头与长度，稍后再插入)
    std::vector<uint8_t> payload;
    // 指令编号(1B)
    payload.push_back(CMD_ID);
    // 加密标志(1B)
    payload.push_back(encryptFlag);
    // 动作编号(1B)
    payload.push_back(0x1A);  // 0x1A: 激光测距
    // 执行结果(1B)
    payload.push_back(returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ? 0x01 : 0x00);    // 0x01: 成功, 0x00: 失败

    // 经度(8B)
    doubleToBigEndian(longitude, buf);
    payload.insert(payload.end(), buf, buf + 8);

    // 纬度(8B)
    doubleToBigEndian(latitude, buf);
    payload.insert(payload.end(), buf, buf + 8);

    // 高度(8B)
    floatToBigEndian(height, buf);
    payload.insert(payload.end(), buf, buf + 4);

    // 距离(8B)
    floatToBigEndian(distance, buf);
    payload.insert(payload.end(), buf, buf + 4);

    // 云盒SN号(15B)
    payload.insert(payload.end(), cloudBoxSn, cloudBoxSn + 15);

    // 3. 构建最终帧
    //    帧头(2B)  + 数据长度(2B) + payload
    std::vector<uint8_t> frame;
    frame.reserve(2 + 2 + payload.size());

    // (a) 帧头(2B)
    frame.push_back(0x6A);
    frame.push_back(0x77);

    // (b) 数据长度(2B) - 先占位，后面回填
    frame.push_back(0x00);
    frame.push_back(0x00);

    // (c) 将 payload 插入
    frame.insert(frame.end(), payload.begin(), payload.end());

    // 4. 回填数据长度
    //    dataLength = payload.size() (从 指令编号到最后)
    //    不包含帧头(2B)与数据长度(2B)本身
    // 回复帧额外不包括云台编号
    uint16_t dataLength = static_cast<uint16_t>(payload.size()) - 15; 
    frame[2] = (dataLength >> 8) & 0xFF;
    frame[3] = dataLength & 0xFF;

    return frame;

}

/**
 * @brief 序列化后的数据打包成统一回复帧
 * @param serialized_data 序列化后的字符串数据
 * @param cmd_id          指令编号
 * @return 打包后的数据帧(二进制形式)
 */
std::vector<uint8_t> reply_realtime_frame(std::string serialized_data, uint8_t cmd_id)
{
    std::vector<uint8_t> frame_data;
    // 帧头
    frame_data.push_back(0x6A);
    frame_data.push_back(0x77);

    // 数据长度（大端序），多加一个字节用于后面放编号0xA8
    uint16_t len = serialized_data.size() + 1;
    frame_data.push_back(len >> 8);
    frame_data.push_back(len & 0xFF);

    // 编号(0xA9:遥测，0xA8:状态)
    frame_data.push_back(cmd_id);

    // 插入实际的序列化数据
    frame_data.insert(frame_data.end(), serialized_data.begin(), serialized_data.end());

    return frame_data;
}


/**
 * @brief 回复伪心跳帧的格式(包含云盒编号)
 * @return 打包好的伪心跳帧数据
 */
std::vector<uint8_t> reply_heartbeat_frame()
{
    // 创建数据帧缓存
    std::vector<uint8_t> frame_sn;

    // 1.帧头
    frame_sn.push_back(0x6A);
    frame_sn.push_back(0x77);

    // 2.数据长度（大端序）
    uint16_t len_sn = 8;  // 后面会插入动作编号、执行结果、错误码、以及云盒编号
    frame_sn.push_back(len_sn >> 8);
    frame_sn.push_back(len_sn & 0xFF);

    // 3.指令编号
    frame_sn.push_back(0xD0);

    // 4.加密标志
    frame_sn.push_back(0x00);

    // 5.动作编号
    frame_sn.push_back(0x12);

    // 6.执行成功标记(0x01表示成功)
    frame_sn.push_back(0x01);

    // 7.错误码：32位，分4个字节存储
    T_DjiReturnCode error_code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    frame_sn.push_back((error_code >> 24) & 0xFF);
    frame_sn.push_back((error_code >> 16) & 0xFF);
    frame_sn.push_back((error_code >> 8) & 0xFF);
    frame_sn.push_back(error_code & 0xFF);

    // 8.插入云盒编号:DBM250974065008(15字节)
    const char str3[15] = {
        'D', 'B', 'M', '2', '5', '0', '9', '7', '4', '0', '6', '5', '0', '0', '8'
    };
    frame_sn.insert(frame_sn.end(), str3, str3 + 15);

    return frame_sn;
}


/**
 * @brief 自定义任务开始、结束回复帧的格式
 * @param toggle 开始/结束标志
 * @return std::vector<uint8_t> 回复帧
 */
std::vector<uint8_t> reply_custom_task_frame(bool toggle)
{
    // 函数：统一回复帧的格式0x11开始、0x29结束
    // std::vector<uint8_t> reply_frame(uint8_t action_id, T_DjiReturnCode error_code);

    std::vector<uint8_t> frame;
    switch (toggle) {
        case true:
            frame = reply_frame(0x11, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
        case false:
            frame = reply_frame(0x29, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
        default:
            frame = reply_frame(0x00, DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER);
    }
    frame[4] = 0xE1;  // 指令编号

    return frame;
}

/**
 * @brief 查找指定的显示模式字符串
 * @param displayMode 显示模式
 * @return uint8_t 显示模式字符串的索引
 * @note 未找到时返回最后一个索引，会打印“”unknown mode“”
 */
uint8_t DjiTest_FlightControlGetDisplayModeIndex(E_DjiFcSubscriptionDisplayMode displayMode)
{
    uint8_t i;

    for (i = 0; i < sizeof(s_flightControlDisplayModeStr) / sizeof(T_DjiTestFlightControlDisplayModeStr); i++) {
        if (s_flightControlDisplayModeStr[i].displayMode == displayMode) {
            std::cout << "\033[1;34m[cur mode:] " << s_flightControlDisplayModeStr[i].displayModeStr << "\033[0m" << std::endl;
            return i;
        }
    }
    
    std::cout << "\033[1;34m[cur mode:] unknown mode\033[0m" << std::endl;
    return i;
}


// 函数：航线结束通知帧（成功）的格式
std::vector<uint8_t> reply_route_end_frame()
{
    std::vector<uint8_t> frame;
    uint8_t action_id = 0x2E;  // 航线结束通知
    T_DjiReturnCode error_code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    frame = reply_frame(action_id, error_code);
    
    return frame;
}

/**
 * @brief 回复“准备完成”帧的格式
 * @param battery 电池电量
 * @param longitude 经度
 * @param latitude 纬度
 * @param altitude 海拔高度
 * @return std::vector<uint8_t> 回复帧
 */
std::vector<uint8_t> reply_uav_ready_frame(uint8_t battery, double longitude, double latitude, int32_t altitude)
{
    // 1. 固定的云盒 SN 号(根据实际需求可改为参数)
    const char cloudBoxSn[15] = {
        'D','B','M','2','5','0','9','7','4','0','6','5','0','0','8'
    };

    // 2. 先构造 Payload(除帧头和数据长度外的所有部分)
    std::vector<uint8_t> payload;
    payload.reserve(1 + 1 + 1 + 1 + 8 + 8 + 4 + 15);

    // 2.1 指令编号(1B) 0xD1
    payload.push_back(0xD1);

    // 2.2 加密标志(1B) - 0x00未加密, 0x01加密
    payload.push_back(0x00);

    // 2.3 动作编号(1B) - “准备完成”为 0x3D
    payload.push_back(0x3D);

    // 2.4 电池电量(1B)
    payload.push_back(battery);

    // 2.5 经度(8B, double 大端)
    uint8_t buf[8];
    doubleToBigEndian(longitude, buf);
    payload.insert(payload.end(), buf, buf + 8);

    // 2.6 纬度(8B, double 大端)
    doubleToBigEndian(latitude, buf);
    payload.insert(payload.end(), buf, buf + 8);

    // 2.7 海拔高度(4B, int 大端)
    int32ToBigEndian(altitude, buf);
    payload.insert(payload.end(), buf, buf + 4);

    // 2.8 云盒SN号(15B)
    payload.insert(payload.end(), cloudBoxSn, cloudBoxSn + 15);

    // 3. 构建最终帧：帧头(2B) + 数据长度(2B) + payload
    std::vector<uint8_t> frame;
    frame.reserve(2 + 2 + payload.size());

    // 3.1 帧头(2B)：固定 0x6A 0x77
    frame.push_back(0x6A);
    frame.push_back(0x77);

    // 3.2 数据长度(2B) - 先占位，后面再回填
    frame.push_back(0x00);
    frame.push_back(0x00);

    // 3.3 插入 payload
    frame.insert(frame.end(), payload.begin(), payload.end());

    // 4. 回填“数据长度”
    //    这里的数据长度定义为：从“指令编号”开始到最后(不含帧头和长度本身)
    //    即 dataLength = payload.size()
    uint16_t dataLength = static_cast<uint16_t>(payload.size()) - 15; // 减去云盒编号15字节
    frame[2] = static_cast<uint8_t>((dataLength >> 8) & 0xFF);
    frame[3] = static_cast<uint8_t>( dataLength       & 0xFF);

    return frame;
}
