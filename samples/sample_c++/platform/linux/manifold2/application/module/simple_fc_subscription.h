#ifndef SIMPLE_FC_SUBSCRIPTION_H
#define SIMPLE_FC_SUBSCRIPTION_H

#include <cstdint>
#include <functional>
#include "dji_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"

// ========== User‑defined telemetry packet ==========
#pragma pack(push, 1)
struct UAVDataInfo
{
    double  latitude_deg;   ///< fused GPS latitude  [deg]
    double  longitude_deg;  ///< fused GPS longitude [deg]
    float   altitude_fused; ///< fused altitude (AMSL)            [m]
    float   height_fusion;  ///< height above ground              [m]
    float   pitch_deg;      ///< aircraft pitch                   [deg]
    float   roll_deg;       ///< aircraft roll                    [deg]
    float   yaw_deg;        ///< aircraft yaw                     [deg]
    float   vel_x;          ///< ground‑referenced velocity X     [m/s]
    float   vel_y;          ///< ground‑referenced velocity Y     [m/s]
    float   vel_z;          ///< ground‑referenced velocity Z     [m/s]
    uint64_t timestamp_ms;  ///< host‑side time stamp             [ms]
};
#pragma pack(pop)

// optional – user can register a callback that receives freshly‑filled data
using TelemetryCallback = std::function<void(const UAVDataInfo&)>;

// ===================================================

class SimpleFcSubscription
{
public:
    SimpleFcSubscription();
    ~SimpleFcSubscription();

    /** Initialise PSDK subscription module and bind topics */
    T_DjiReturnCode startService();

    /** Main loop – call in its own RTOS / pthread task, or spawn std::thread */
    void run(TelemetryCallback cb = nullptr,
             float user_loop_hz = 10.0f /* how fast you want publish() to run */);

    /** Last data sample (thread‑safe read if you guard elsewhere) */
    const UAVDataInfo& latest() const { return data_; }

private:
    /* helpers -------------------------------------------------------------- */
    static void quaternionToEuler(const T_DjiFcSubscriptionQuaternion& q,
                                  float& pitch_deg, float& roll_deg, float& yaw_deg);

    /* member data ---------------------------------------------------------- */
    UAVDataInfo         data_{};
    TelemetryCallback   userCb_{};
    float               loopHz_{10.0f};
};

#endif // SIMPLE_FC_SUBSCRIPTION_H
