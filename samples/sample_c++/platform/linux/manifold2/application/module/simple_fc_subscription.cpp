#include "simple_fc_subscription.h"
#include <chrono>
#include <cmath>
#include <iostream>

/* ------------ compile‑time settings ----------------- */
#define DEG_PER_RAD  (180.0 / M_PI)

/* ------------ ctor / dtor --------------------------- */
SimpleFcSubscription::SimpleFcSubscription()
{
    // nothing to do – all zero‑initialised
}

SimpleFcSubscription::~SimpleFcSubscription()
{
    DjiFcSubscription_DeInit();
}

// 检查订阅是否成功
static bool fcSub_isPrint = true;                              // 是否打印数据
static int  count_fcSub_print = 0;                          // 打印计数
T_DjiReturnCode checkSubscription(T_DjiReturnCode djiStat){
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("订阅主题失败。%d",count_fcSub_print++);
    } else {
        if (fcSub_isPrint) USER_LOG_DEBUG("订阅主题成功。");
    }
    return djiStat;
}

/* ------------ public API ---------------------------- */
T_DjiReturnCode SimpleFcSubscription::startService()
{
    T_DjiReturnCode rc = DjiFcSubscription_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("PSDK FcSubscription init failed.");
        return rc;
    }

    // 休息2s
    auto* osal = DjiPlatform_GetOsalHandler();
    osal->TaskSleepMs(2000);

    // ---------- bind the five topics we need ----------
    std::cout << "订阅位置信息..." << std::endl;
    rc = DjiFcSubscription_SubscribeTopic(
             DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, NULL);
    checkSubscription(rc);
    osal->TaskSleepMs(1000);

    std::cout << "订阅高度信息..." << std::endl;
    rc = DjiFcSubscription_SubscribeTopic(
             DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, NULL);
    checkSubscription(rc);
    osal->TaskSleepMs(1000);

    std::cout << "订阅融合高度信息..." << std::endl;
    rc = DjiFcSubscription_SubscribeTopic(
             DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,   DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, NULL);
    checkSubscription(rc);
    osal->TaskSleepMs(1000);

    // std::cout << "订阅四元数信息..." << std::endl;
    // rc = DjiFcSubscription_SubscribeTopic(
    //          DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,      DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL);
    // checkSubscription(rc);
    // osal->TaskSleepMs(1000);

    std::cout << "订阅速度信息..." << std::endl;
    rc = DjiFcSubscription_SubscribeTopic(
             DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,        DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, NULL);
    checkSubscription(rc);
    osal->TaskSleepMs(1000);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void SimpleFcSubscription::run(TelemetryCallback cb, float user_loop_hz)
{
    userCb_ = cb;
    loopHz_ = user_loop_hz <= 0.0f ? 10.0f : user_loop_hz;

    auto* osal = DjiPlatform_GetOsalHandler();
    const uint32_t sleep_ms = static_cast<uint32_t>(1000.0f / loopHz_);

    // PSDK containers
    T_DjiDataTimestamp ts{};
    T_DjiFcSubscriptionPositionFused   posFused{};
    T_DjiFcSubscriptionAltitudeFused   altFused{};
    T_DjiFcSubscriptionHeightFusion    hFusion{};
    T_DjiFcSubscriptionQuaternion      quat{};
    T_DjiFcSubscriptionVelocity        vel{};

    while (true)
    {
        // ---------- pull latest values (non‑blocking) ----------
        DjiFcSubscription_GetLatestValueOfTopic(
            DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
            reinterpret_cast<uint8_t*>(&posFused), sizeof(posFused), &ts);

        DjiFcSubscription_GetLatestValueOfTopic(
            DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
            reinterpret_cast<uint8_t*>(&altFused), sizeof(altFused), &ts);

        DjiFcSubscription_GetLatestValueOfTopic(
            DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
            reinterpret_cast<uint8_t*>(&hFusion),  sizeof(hFusion),  &ts);

        DjiFcSubscription_GetLatestValueOfTopic(
            DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
            reinterpret_cast<uint8_t*>(&quat), sizeof(quat), &ts);

        DjiFcSubscription_GetLatestValueOfTopic(
            DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
            reinterpret_cast<uint8_t*>(&vel), sizeof(vel), &ts);

        // ---------- convert to user struct ----------
        data_.latitude_deg   = posFused.latitude  * DEG_PER_RAD;
        data_.longitude_deg  = posFused.longitude * DEG_PER_RAD;
        data_.altitude_fused = altFused;
        data_.height_fusion  = hFusion;

        quaternionToEuler(quat, data_.pitch_deg, data_.roll_deg, data_.yaw_deg);

        data_.vel_x = vel.data.x;
        data_.vel_y = vel.data.y;
        data_.vel_z = vel.data.z;

        data_.timestamp_ms = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());

        // ---------- user callback ----------
        if (userCb_) userCb_(data_);

        osal->TaskSleepMs(sleep_ms);
    }
}

/* ------------ helpers ------------------------------- */
void SimpleFcSubscription::quaternionToEuler(const T_DjiFcSubscriptionQuaternion& q,
                                             float& pitch_deg, float& roll_deg, float& yaw_deg)
{
    // standard aerospace conv. (YXZ local‑level)
    const double q0 = q.q0, q1 = q.q1, q2 = q.q2, q3 = q.q3;

    double pitch =  std::asin(2.0 * (q0*q2 - q3*q1));
    double roll  =  std::atan2(2.0 * (q0*q1 + q2*q3),
                               1.0 - 2.0 * (q1*q1 + q2*q2));
    double yaw   =  std::atan2(2.0 * (q0*q3 + q1*q2),
                               1.0 - 2.0 * (q2*q2 + q3*q3));

    pitch_deg = static_cast<float>(pitch * DEG_PER_RAD);
    roll_deg  = static_cast<float>(roll  * DEG_PER_RAD);
    yaw_deg   = static_cast<float>(yaw   * DEG_PER_RAD);
}
