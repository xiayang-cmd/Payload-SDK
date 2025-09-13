#pragma once
//
//  SimpleProtocol  –  one header for both host & device
//
#include <array>
#include <cstddef>
#include <cstdint>

namespace SimpleProtocol {

/* =========================================================
 *  Downlink  :  Host ➜ Device
 * =========================================================*/
namespace Downlink {

#pragma pack(push,1)
struct Header {
    uint16_t sync   {0x7479};      // 'y','t'
    uint16_t length {0};           // payload size
    uint8_t  cmd    {0};
};

enum class Command : uint8_t {
    SetHostMode  = 0x03,
    SetSlaveMode = 0x04,
    FollowStart  = 0xC1,
    FollowStop   = 0xC2,
    SetOffset    = 0xC3,
    BoardPos     = 0x0A,
};

struct PayloadDroneId { uint8_t drone_id; };

struct PayloadOffset {
    uint8_t drone_id;
    uint8_t offset_x;
    uint8_t offset_y;
    uint8_t offset_z;
};

struct PayloadPosition {
    uint8_t  drone_id;
    double   latitude_deg;
    double   longitude_deg;
    float    altitude_fused;
    float    height_fusion;
    float    pitch_deg;
    float    roll_deg;
    float    yaw_deg;
    float    vel_x, vel_y, vel_z;
    uint64_t timestamp_ms;
};
#pragma pack(pop)
static_assert(sizeof(Header) == 5, "Downlink Header size");

template<std::size_t N> using FrameBytes = std::array<std::uint8_t, N>;

enum class CheckResult {
    Ok, IncompleteHeader, BadSync,
    IncompleteFrame, UnknownCommand, PayloadSizeMismatch
};

/* ---------- host‑side helpers ---------- */
FrameBytes<6>  makeSimpleCmdFrame(Command cmd, uint8_t drone_id);          // 1‑byte payload
FrameBytes<9>  makeOffsetFrame  (const PayloadOffset& p);                  // 4‑byte payload
FrameBytes<62> makeBoardPosFrame(const PayloadPosition& p);                // 57‑byte payload

/* ---------- device‑side validator ---------- */
CheckResult checkDownlink(const uint8_t* buf, std::size_t length);

} // namespace Downlink



/* =========================================================
 *  Uplink  :  Device ➜ Host
 * =========================================================*/
namespace Uplink {

using Byte = std::uint8_t;

#pragma pack(push,1)
struct Header {
    uint16_t sync   {0x7767};      // 'w','g'
    uint16_t length {0};
    uint8_t  cmd    {0};
};

enum class Command : uint8_t {
    Reply          = 0x01,   // payload: drone_id
    PositionReport = 0x0A    // payload: drone_id + 56 telemetry bytes
};

struct PayloadDroneId  { Byte drone_id; };
struct PayloadTelemetry{ Byte data[56]; };
#pragma pack(pop)
static_assert(sizeof(Header) == 5, "Uplink Header size");

template<std::size_t N> using FrameBytes = std::array<Byte, N>;

/* ----- frame templates (compile‑time constants) ----- */
using PositionReportFrame = FrameBytes<62>;
using ReplyFrame          = FrameBytes<6>;

inline constexpr PositionReportFrame kPositionReportTemplate{
    0x67,0x77,         // sync
    0x39,0x00,         // len = 57
    0x0A,              // cmd
    0x00,              // drone_id (filled later)
    /* 56 zero bytes telemetry     */
    0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0
};

inline constexpr ReplyFrame kReplyFrameTemplate{
    0x67,0x77,         // sync
    0x01,0x00,         // len = 1
    0x01,              // cmd (filled later)
    0x01               // drone_id (filled later)
};

/* ---------- device‑side helpers ---------- */
PositionReportFrame makePositionReportFrame(Byte id, const Byte* telemetry56);
void                changePositionReportFrameForBoard(PositionReportFrame& f);  // optional tweak
ReplyFrame          makeReplyFrame(Byte id, uint8_t cmd);

/* ---------- host‑side validator ---------- */
enum class CheckResult {
    Ok, IncompleteHeader, BadSync,
    IncompleteFrame, UnknownCommand, PayloadSizeMismatch
};

CheckResult checkUplink(const uint8_t* buf, std::size_t length);

} // namespace Uplink
} // namespace SimpleProtocol
