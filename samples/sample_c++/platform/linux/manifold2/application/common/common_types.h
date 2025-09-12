#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

/* Includes ----------------------------------------------------------------*/
#include "dji_fc_subscription.h"
#include "dji_typedef.h"
#include "dji_logger.h"
#include "dji_waypoint_v2.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <functional>
#include <vector>
#include <atomic>
#include <cstdint>
#include <iostream>

/* Namespace --------------------------------------------------------------*/
using namespace std;

/* Constants --------------------------------------------------------------*/
#define DEG_TO_RAD 0.017453293                      // 角度转弧度系数
#define RAD_TO_DEG 57.295779513                     // 弧度转角度系数
#define LOCATION_SCALING_FACTOR 111318.84502145034  // 位置缩放因子

extern const double s_earthCenter;  // 地球半径
extern const double s_degToRad;     // 角度转弧度系数

/* Type Definitions -------------------------------------------------------*/
// 上位机服务器配置结构体
struct ServerConfig {
    std::string ip;
    uint16_t port;
};

/**
 * @brief 自定义位置结构体
 */
typedef struct {
    dji_f64_t longitude; /*!< Longitude, unit: rad. */
    dji_f64_t latitude; /*!< Latitude, unit: rad. */
    dji_f32_t relative_alt; /*!< Altitude, reference homePoint, unit: m. */
} DJI_CustomGeoPosition;

// DJI向量结构体
typedef struct {
    dji_f32_t x;
    dji_f32_t y;
    dji_f32_t z;
} T_DjiTestFlightControlVector3f; // pack(1)

// 相机状态结构体
typedef struct {
    uint32_t mode;                  // 相机模式：1=拍照模式，2=录像模式，0=未知
    uint32_t isRecording;           // 是否录制中：1=录制中，0=未录制
    uint32_t recordDuration;        // 录制时长(单位:秒)
    uint32_t source;                // 相机视源：0=可见，1=变焦，2=红外
    uint32_t camera;                // 相机: 0=FPV，1=相机(1)，2=相机(2)
    float zoomfactor;               // 相机当前变焦倍数
    uint32_t width;                 // 水平像素(例如：1920)
    uint32_t height;                // 垂直像素(例如：1080)
    uint32_t frameRate;             // 帧率(例如：30)
    uint32_t bitstream;             // 码率(单位：0.001)(例如：6兆设置值为6000)
    uint32_t pointThermometrying;   // 单点测温状态：0=非单点测温中，1=单点测温中
    uint32_t areaThermometrying;    // 区域测温状态：0=非区域测温中，1=区域测温中
    uint32_t laserRanging;          // 激光测距状态：0=非激光测距中，1=激光测距中
} MyCameraState;

// 自定义的航线状态结构体
struct MyFlightMissionStatus {
    int flight_id;                  // 任务ID
    uint8_t flight_status;          // 任务状态（参见：E_DJIWaypointV2MissionState）
    double overall_progress;        // 任务进度（百分比）
    int cur_waypoint_index;         // 当前航点索引
    double remaining_distance;      // 距离下一个航点的距离（米）
    double leg_progress;            // 当前航段进度（百分比）
    bool flag_route_end;            // 航线是否结束
    std::vector<T_DjiWaypointV2> waypointList; // 航点列表
};

// 打点飞行状态结构体
struct M_PointFlightState {
    double distanceRemaining;       // 剩余距离
    double timeRemaining;           // 剩余时间
    uint8_t pointMode;              // 打点飞行模式：0-节能模式，1-安全模式
    bool isPointControl;            // 是否打点飞行控制中：0=否，1=是
};

// 环点飞行状态结构体
struct M_CircleFlightState {
    double distanceRemaining;        // 剩余距离
    double timeRemaining;            // 剩余时间
    uint8_t circleMode = 0;          // 环点飞行模式
    bool isCircleControl = false;    // 是否处于环点飞行状态
};


// 任务状态结构体
struct M_MissionState {
    uint32_t isPause;              // 航线是否暂停中：0=未暂停，1=暂停中
    uint32_t targetWaypointIndex;  // 目标航点下标
    uint32_t pushVideo;            // 云盒是否发送视频数据：0=不发送，1=发送
    uint32_t boxModel;             // 是否开通图片自动回传功能：0=未开通，1=已开通
    uint32_t mapPlay;              // 自动拍照间隔：0未开始，3~255间隔时间
    uint32_t loseAction;           // 网络失联后动作：0=返回HOME点，1=继续航线
    uint32_t isPointControl;       // 是否打点飞行控制中：0=否，1=是
    uint32_t isPushVideoing;       // 视频是否推流中：0=不是，1=是
};


/* Global Variables -------------------------------------------------------*/
/* 0.相机相关 */
extern MyCameraState g_cameraState;                     // 相机状态[用于相机参数获取任务]
extern std::atomic<int> g_set_cameraSource;             // 控制当前推流的相机视源，0: 可见光, 1: 变焦, 2: 红外[用于推流任务，从相机模块获取]
extern std::atomic<int> g_set_cameraType;               // 控制当前推流的相机类型，1: PAYLOAD摄像头, 0: FPV摄像头[用于推流任务，从相机模块获取]
extern std::atomic<bool> g_flag_isStreaming;            // 控制是否推流
extern std::atomic<bool> g_state_isStreaming;           // 当前是否在推流
extern std::atomic<bool> g_flag_exitThread;             // 控制线程退出
extern std::atomic<int> g_flag_isLaserRanging;          // 当前是否激光测距
extern std::atomic<int> g_flag_photoBackIsOn;           // 是否开启自动回传拍照功能状态
extern uint32_t g_mapPlay;                                   // 自动拍照间隔：0未开始，3~255间隔时间

/* 1.航线与任务相关 */
extern T_DjiWaypointV2MissionStatePush g_cur_waypointV2MissionState;    // 航点任务状态
extern MyFlightMissionStatus g_cur_flightMissionStatus;                 // 自定义的航线任务状态
extern uint8_t g_cur_taskID;                                            // 任务ID

/* 2.飞行控制相关 */
extern double g_startPoint_latitude;                    // 起飞点纬度[用于遥测回传任务]
extern double g_startPoint_longitude;                   // 起飞点经度
extern M_PointFlightState g_cur_pointFlightState;       // 打点飞行状态
extern M_CircleFlightState g_cur_circleFlightState;     // 环点飞行状态
extern uint8_t g_flag_isEmerge;                         // 控制是否紧急制动
extern bool osdk_is_get_CtrlAuthority;                  // osdk是否获取了控制权限
extern uint8_t g_cur_batteryCapacity;                   // 电池电量
extern double g_cur_latitude;                           // 当前纬度
extern double g_cur_longitude;                          // 当前经度
extern int g_cur_height;                                // 当前高度
extern uint32_t g_flyStartTime;                         // 当前架次开始时间(单位:秒)
extern uint32_t g_flyTimes;                             // 当前架次飞行时间(单位:秒)
extern float g_flyDistance;                             // 当前架次飞行距离(单位:米)
extern uint8_t g_cur_sortie_ID;                         // 当前架次ID
extern bool g_flag_sortie_running;                      // 当前架次是否正在飞行
extern bool g_signal_sortie_start;                      // 当前架次开始飞行信号
extern bool g_signal_sortie_end;                        // 当前架次结束飞行信号
extern float g_ultrasonicMax;                           // 当前架次最大离地高度
extern float g_ultrasonicMin;                           // 当前架次最小离地高度
extern float g_cur_homeRange;                           // 当前位置离HOME点的距离

/* 3.无人机设置相关 */
extern string g_uavType;                                // 无人机型号
extern bool g_uav_fcdata_ready;                         // 飞控数据是否准备好
extern bool g_module_Init_success;                      // 模块初始化是否成功
extern bool g_uav_reafy;                                // 无人机是否准备好
extern const char my_box_id[15];                        // 云盒SN号
extern bool g_enable_telemetry;                         // 遥测回传功能是否开启
extern bool g_debug_mode;                               // 调试模式是否开启

/* 4.云台相关 */
// (暂无)


extern ServerConfig g_EdgeServer; // 上位机服务器配置

/* Function Type Definitions -------------------------------------------------------*/
// 定义向上位机发送数据的回调函数类型
typedef std::function<void(const uint8_t*, size_t)> SendDataCallback;


#endif // COMMON_TYPES_H
