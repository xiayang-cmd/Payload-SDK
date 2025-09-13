#pragma once

class TargetArrivalMonitor
{
public:
    explicit TargetArrivalMonitor(int cycleTimeInMs);

    void Update(float posOffsetInMs, float yawOffsetInDeg);
    void Update(float posOffsetInMs);
    void Reset();

    bool checkArrival() const;
    bool checkArrival_withYaw() const;

    void SetPosThreshold(float posThresholdInM);
    void SetYawThreshold(float yawThresholdInDeg);
    void SetTimeRequirements(int outOfBoundsLimitMs, int inBoundsReqMs);

private:

    float posThresholdM_;           // 位置阈值（单位：米）
    float yawThresholdDeg_;         // 偏航角阈值（单位：度）

    int outOfBoundsLimitMs_;        // 允许超出范围的最大累计时间（ms）
    int inBoundsReqMs_;             // 在范围内所需的累计时间（ms）

    int inBoundsTimeMs_;            // 已在范围内累计的时间（ms）
    int outOfBoundsTimeMs_;         // 已超出范围累计的时间（ms）

    int cycleTimeMs_;               // 控制循环周期（ms）
};
