#include "common_types.h"
#include <limits>

/* 常量定义 --------------------------------------------------------------- */
const double s_earthCenter = 6378137.0;               // 地球半径
const double s_degToRad    = 0.01745329252;           // 角度到弧度的转换因子

/* 相机相关变量 ----------------------------------------------------------- */
MyCameraState g_cameraState;                          // 相机状态
std::atomic<int> g_set_cameraSource{0};                  // 控制当前推流的相机视源，0: 可见光, 1: 变焦, 2: 红外[用于推流任务，从相机模块获取]
std::atomic<int> g_set_cameraType{1};                 // 推流的相机类型，1: PAYLOAD摄像头, 0: FPV摄像头
#ifdef DEBUG_FLAG_ENABLE_LIVE
std::atomic<bool> g_flag_isStreaming{true};           // 是否开启推流
#else
std::atomic<bool> g_flag_isStreaming{false};           // 是否开启推流
#endif
std::atomic<bool> g_state_isStreaming{false};         // 当前是否在推流
std::atomic<bool> g_flag_exitThread{false};           // 线程退出标志
std::atomic<int> g_flag_isLaserRanging{0};              // 当前是否激光测距
std::atomic<int> g_flag_photoBackIsOn{1};           // 是否开启自动回传拍照功能状态
uint32_t g_mapPlay = 0;                                   // 自动拍照间隔：0未开始，3~255间隔时间


/* 航线与任务相关变量 ----------------------------------------------------- */
T_DjiWaypointV2MissionStatePush g_cur_waypointV2MissionState = {
    0,
    DJI_WAYPOINT_V2_MISSION_STATE_GROUND_STATION_NOT_START,
    0
};                                                   // 航点任务状态

MyFlightMissionStatus g_cur_flightMissionStatus = {0, 0, 0, 0, 0, 0, false, {}};
uint8_t g_cur_taskID = 0;                             // 当前任务ID

/* 飞行控制相关变量 ------------------------------------------------------- */
double g_startPoint_latitude  = 0;                    // 起飞点纬度
double g_startPoint_longitude = 0;                    // 起飞点经度
M_PointFlightState g_cur_pointFlightState = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),0,0};           // 打点飞行状态
M_CircleFlightState g_cur_circleFlightState = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0, false};   // 环点飞行状态
uint8_t g_flag_isEmerge = 0;                          // 是否紧急制动
bool osdk_is_get_CtrlAuthority = true;                // 是否获取OSDK控制权限，默认为true
uint8_t g_cur_batteryCapacity = 0;                    // 电池电量
double g_cur_latitude = 0;                            // 当前纬度
double g_cur_longitude = 0;                           // 当前经度
int g_cur_height = 0;                                 // 当前高度
uint32_t g_flyStartTime = 0;                         // 当前架次开始时间(单位:秒)
uint32_t g_flyTimes = 0;                              // 当前架次飞行时间(单位:秒)
float g_flyDistance = 0;                              // 当前架次飞行距离(单位:米)
uint8_t g_cur_sortie_ID = 9999;                       // 当前架次ID
bool g_flag_sortie_running = false;                   // 当前架次是否正在飞行
bool g_signal_sortie_start = false;                   // 当前架次开始飞行信号
bool g_signal_sortie_end = false;                     // 当前架次结束飞行信号
float g_ultrasonicMax = 0;                           // 当前架次最大离地高度
float g_ultrasonicMin = 0;                           // 当前架次最小离地高度
float g_cur_homeRange = 0;                           // 当前位置离HOME点的距离

/* 无人机设置相关变量 ----------------------------------------------------- */
string g_uavType = "M350 RTK";                        // 无人机型号
bool g_uav_fcdata_ready = false;                      // 飞控数据是否准备好
bool g_module_Init_success = false;                   // 模块初始化是否成功
bool g_uav_reafy = false;                             // 无人机是否准备好
bool g_enable_telemetry = false;                      // 是否开启遥测回传
bool g_debug_mode = false;                            // 调试模式是否开启
const char my_box_id[15] = {'D', 'B', 'M', '2', '5', '0', '9', '7', '4', '0', '6', '5', '0', '0', '8'};


ServerConfig g_EdgeServer = {"192.168.66.97", 12345}; // 上位机服务器配置
