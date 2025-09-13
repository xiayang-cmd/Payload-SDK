#include "simpleProtocol.h"
#include <algorithm>
#include <cstring>

namespace SimpleProtocol {

/* =========================================================
 *  Downlink  –  impl.
 * =========================================================*/
namespace Downlink {

FrameBytes<6> makeSimpleCmdFrame(Command cmd, uint8_t id)
{
    FrameBytes<6> f{};
    f[0] = 0x79; f[1] = 0x74;
    f[2] = 0x01; f[3] = 0x00;              // len = 1
    f[4] = static_cast<uint8_t>(cmd);
    f[5] = id;
    return f;
}

FrameBytes<9> makeOffsetFrame(const PayloadOffset& p)
{
    FrameBytes<9> f{};
    f[0] = 0x79; f[1] = 0x74;
    f[2] = 0x04; f[3] = 0x00;              // len = 4
    f[4] = static_cast<uint8_t>(Command::SetOffset);
    f[5] = p.drone_id;
    f[6] = p.offset_x; f[7] = p.offset_y; f[8] = p.offset_z;
    return f;
}

FrameBytes<62> makeBoardPosFrame(const PayloadPosition& p)
{
    FrameBytes<62> f{};
    f[0] = 0x79; f[1] = 0x74;
    uint16_t len = static_cast<uint16_t>(sizeof(PayloadPosition));
    f[2] = static_cast<uint8_t>(len & 0xFF);
    f[3] = static_cast<uint8_t>(len >> 8);
    f[4] = static_cast<uint8_t>(Command::BoardPos);
    std::memcpy(f.data() + 5, &p, sizeof(PayloadPosition));
    return f;
}

CheckResult checkDownlink(const uint8_t* buf, std::size_t len)
{
    if (len < sizeof(Header)) return CheckResult::IncompleteHeader;
    const auto* h = reinterpret_cast<const Header*>(buf);
    if (h->sync != 0x7479)    return CheckResult::BadSync;
    std::size_t total = sizeof(Header) + h->length;
    if (len < total)          return CheckResult::IncompleteFrame;

    std::size_t expected = 0;
    switch (static_cast<Command>(h->cmd))
    {
        case Command::SetHostMode:
        case Command::SetSlaveMode:
        case Command::FollowStart:
        case Command::FollowStop: expected = sizeof(PayloadDroneId); break;
        case Command::SetOffset:  expected = sizeof(PayloadOffset);  break;
        case Command::BoardPos:   expected = sizeof(PayloadPosition);break;
        default: return CheckResult::UnknownCommand;
    }
    return (h->length == expected) ? CheckResult::Ok
                                   : CheckResult::PayloadSizeMismatch;
}

} // namespace Downlink



/* =========================================================
 *  Uplink  –  impl.
 * =========================================================*/
namespace Uplink {

PositionReportFrame makePositionReportFrame(Byte id, const Byte* telemetry56)
{
    PositionReportFrame f = kPositionReportTemplate;
    f[5] = id;
    std::copy(telemetry56, telemetry56 + 56, f.begin() + 6);
    return f;
}

void changePositionReportFrameForBoard(PositionReportFrame& f)
{
    // 头两个字节改成79 74
    f[0] = 0x79;
    f[1] = 0x74;
}

ReplyFrame makeReplyFrame(Byte id, uint8_t cmd)
{
    ReplyFrame f = kReplyFrameTemplate;
    f[4] = cmd;
    f[5] = id;
    return f;
}

CheckResult checkUplink(const uint8_t* buf, std::size_t len)
{
    if (len < sizeof(Header)) return CheckResult::IncompleteHeader;
    const auto* h = reinterpret_cast<const Header*>(buf);
    if (h->sync != 0x7767)    return CheckResult::BadSync;
    std::size_t total = sizeof(Header) + h->length;
    if (len < total)          return CheckResult::IncompleteFrame;

    std::size_t expected = 0;
    switch (static_cast<Command>(h->cmd))
    {
        case Command::Reply:          expected = sizeof(PayloadDroneId); break;
        case Command::PositionReport: expected = 57;                     break; // 1+56
        default: return CheckResult::UnknownCommand;
    }
    return (h->length == expected) ? CheckResult::Ok
                                   : CheckResult::PayloadSizeMismatch;
}

} // namespace Uplink
} // namespace SimpleProtocol
