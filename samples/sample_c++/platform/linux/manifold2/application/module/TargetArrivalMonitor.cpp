#include "TargetArrivalMonitor.h"
#include <cmath>  // for std::fabs

// 构造函数：初始化默认参数
TargetArrivalMonitor::TargetArrivalMonitor(int cycleTimeInMs)
    : cycleTimeMs_(cycleTimeInMs)
{
    // 给一些默认值，后续可通过 SetXX 进行修改
    posThresholdM_     = 0.5f;
    yawThresholdDeg_   = 5.0f;

    outOfBoundsLimitMs_ = 10 * cycleTimeMs_;
    inBoundsReqMs_      = 100 * cycleTimeMs_;

    inBoundsTimeMs_     = 0;
    outOfBoundsTimeMs_  = 0;
}

/**
 * @brief 带航向偏差的 Update：认为用户想“位置 + 航向”一起监控
 */
void TargetArrivalMonitor::Update(float posOffsetInM, float yawOffsetInDeg)
{
    // 是否满足同时在位置和航向阈值内
    bool inRange = (posOffsetInM < posThresholdM_)
                && (std::fabs(yawOffsetInDeg) < yawThresholdDeg_);

    if (inRange) {
        // 每个控制周期都处于范围内，则累计 inBoundsTimeMs_
        inBoundsTimeMs_ += cycleTimeMs_;
    } else {
        // 若之前已进入过范围，现在离开了范围，则开始累计 outOfBoundsTimeMs_
        if (inBoundsTimeMs_ > 0) {
            outOfBoundsTimeMs_ += cycleTimeMs_;
        }
    }

    // 如果离开范围的时间过久，则认为不稳定，重置计时
    if (outOfBoundsTimeMs_ > outOfBoundsLimitMs_) {
        inBoundsTimeMs_    = 0;
        outOfBoundsTimeMs_ = 0;
    }
}

/**
 * @brief 仅位置偏差的 Update：认为用户只想监控位置到达
 */
void TargetArrivalMonitor::Update(float posOffsetInM)
{
    // 是否在位置阈值内
    bool inRange = (posOffsetInM < posThresholdM_);

    if (inRange) {
        inBoundsTimeMs_ += cycleTimeMs_;
    } else {
        if (inBoundsTimeMs_ > 0) {
            outOfBoundsTimeMs_ += cycleTimeMs_;
        }
    }

    if (outOfBoundsTimeMs_ > outOfBoundsLimitMs_) {
        inBoundsTimeMs_    = 0;
        outOfBoundsTimeMs_ = 0;
    }
}

/**
 * @brief 重置计时器
 */
void TargetArrivalMonitor::Reset()
{
    inBoundsTimeMs_    = 0;
    outOfBoundsTimeMs_ = 0;
}

/**
 * @brief 检查是否满足“仅位置”的到达条件
 */
bool TargetArrivalMonitor::checkArrival() const
{
    // 只要满足累计时间就返回true
    return (inBoundsTimeMs_ >= inBoundsReqMs_);
}

// --------------------- 参数设置接口 ---------------------
void TargetArrivalMonitor::SetPosThreshold(float posThresholdInM)
{
    posThresholdM_ = posThresholdInM;
}

void TargetArrivalMonitor::SetYawThreshold(float yawThresholdInDeg)
{
    yawThresholdDeg_ = yawThresholdInDeg;
}

void TargetArrivalMonitor::SetTimeRequirements(int outOfBoundsLimitMs,
                                               int inBoundsReqMs)
{
    outOfBoundsLimitMs_ = outOfBoundsLimitMs;
    inBoundsReqMs_      = inBoundsReqMs;
}
