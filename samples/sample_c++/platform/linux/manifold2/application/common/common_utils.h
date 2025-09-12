#include "common_types.h"
#include <vector>


/* Namespace --------------------------------------------------------------*/
using namespace std;

/* -----------------------------------------------------------------------
 * 通用小工具
 * -----------------------------------------------------------------------*/

// 函数：检查指定的文件是否存在
bool IsFileExist(string &name);

// 函数：获取当前时间字符串
std::string getCurrentTimeString();

// 函数：获取时间戳（单位：毫秒）
uint64_t getCurrentTimestampMillis();

// 函数：获取时间戳（单位：秒）
uint32_t getCurrentTimestampSeconds();

// 函数：填充时间戳
void fillHeartbeatResponse(uint8_t responseHeartbeat[13]);

// 函数：统一回复帧的格式r0
std::vector<uint8_t> reply_frame(uint8_t action_id, T_DjiReturnCode error_code);

// 函数：实时数据回复帧的格式r1
std::vector<uint8_t> reply_realtime_frame(std::string serialized_data, uint8_t cmd_id);

// 函数：回复伪心跳帧的格式(包含云盒编号)r2
std::vector<uint8_t> reply_heartbeat_frame();

// 函数：自定义任务开始、结束回复帧的格式r3
std::vector<uint8_t> reply_custom_task_frame(bool toggle);

// 函数：图片URL通知帧的格式r4
std::vector<uint8_t> reply_photourl_frame(const std::string &photo_url, double longitude, double latitude, uint64_t timestamp, uint64_t originalSize, bool encrypt);

// 函数：实时激光测距回复帧的格式r5
std::vector<uint8_t> reply_laser_frame(double longitude, double latitude, float height, float distance, T_DjiReturnCode returnCode);

// 函数：航线结束通知帧的格式r6
std::vector<uint8_t> reply_route_end_frame();

// 函数：无人机准备好通知帧的格式r7
std::vector<uint8_t> reply_uav_ready_frame(uint8_t battery,double longitude, double latitude, int32_t altitude);

// 大端转换函数：从大端字节数组转换为 float
float bigEndianToFloat(const uint8_t* data);

// 大端转换函数：从大端字节数组转换为 double
double bigEndianToDouble(const uint8_t* data);

// 将 float 转换为大端字节数组
void floatToBigEndian(float value, uint8_t* data);

// 将 double 转换为大端字节数组
void doubleToBigEndian(double value, uint8_t* data);

// 将 uint32_t 转换为大端字节数组
void uint32ToBigEndian(uint32_t value, uint8_t* data);

// 将 uint64_t 转换为大端字节数组
void uint64ToBigEndian(uint64_t value, uint8_t* data);

// 将 int32_t 转换为大端字节数组
void int32ToBigEndian(int32_t value, uint8_t* data);

// 函数：打印飞行模式
uint8_t DjiTest_FlightControlGetDisplayModeIndex(E_DjiFcSubscriptionDisplayMode displayMode);


/* -----------------------------------------------------------------------
 * 无人机相关函数（位置、姿态等）
 * -----------------------------------------------------------------------*/

// 函数：四元数转欧拉角
void QuaternionToEuler(const T_DjiFcSubscriptionQuaternion *quaternion,
                       dji_f64_t *pitch, dji_f64_t *roll, dji_f64_t *yaw);

// 函数：检查飞机是否在空中
bool checkUavIsAir();

// 函数：获取无人机状态
T_DjiFcSubscriptionFlightStatus DjiTest_FlightControlGetValueOfFlightStatus(void);

// 函数：获取无人机的GPS位置和海拔高度
T_DjiReturnCode DjiDemo_GetCurrentGpsPosition(T_DjiFcSubscriptionGpsPosition *gpsPosition);

// 函数：获取飞行器的GPS位置和海拔高度(融合数据)
T_DjiFcSubscriptionPositionFused DjiTest_FlightControlGetValueOfPositionFused(void);

// 函数：获取飞行器的相对home点的高度
dji_f32_t DjiTest_FlightControlGetValueOfRelativeHeight(void);

// 函数：获取飞行器的自定义地理位置
DJI_CustomGeoPosition DjiTest_FlightControlGetValueOfCustomGeoPosition(void);

// 函数：获取飞行器的四元数
T_DjiFcSubscriptionQuaternion DjiTest_FlightControlGetValueOfQuaternion(void);

// 函数：通过四元数获取欧拉角
T_DjiTestFlightControlVector3f DjiTest_FlightControlQuaternionToEulerAngle(const T_DjiFcSubscriptionQuaternion quat);

// 函数：计算向量偏移
T_DjiTestFlightControlVector3f DjiTest_FlightControlVector3FSub(const T_DjiTestFlightControlVector3f vectorA,
                                                                const T_DjiTestFlightControlVector3f vectorB);

// 函数：计算向量的模
dji_f32_t DjiTest_FlightControlVectorNorm(T_DjiTestFlightControlVector3f v);

// 函数：计算两个经纬度点之间的距离向量
T_DjiTestFlightControlVector3f DjiTest_FlightControlLocalOffsetFromGpsAndFusedHeightOffset(
    const T_DjiFcSubscriptionPositionFused target,
    const T_DjiFcSubscriptionPositionFused origin,
    const dji_f32_t targetHeight,
    const dji_f32_t originHeight);

// 函数：计算两个自定义格式的经纬度点之间的距离向量
T_DjiTestFlightControlVector3f DjiTest_FlightControlLocalOffsetFromCustomGeoPosition(
    const DJI_CustomGeoPosition target,
    const DJI_CustomGeoPosition origin);
