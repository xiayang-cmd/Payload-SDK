/*
 * File: UDPServer.cpp
 * Desc: Linux 平台 UDP 服务器实现（已移除 Windows 相关代码与依赖）
 *
 * 功能概述：
 *  - 创建非阻塞 UDP socket 并绑定本地端口；
 *  - 接收线程：循环调用 onDataReceived() 读取并交由 SimpleMsgHandler 处理；
 *  - 定时器线程：每 1 秒 tick（可扩展为周期性业务逻辑）；
 *  - sendMessage()：向指定 ip:port 发送二进制数据。
 *
 * 线程与资源：
 *  - start() 创建接收与定时线程；stop() 负责安全退出与资源回收；
 *  - running 使用 std::atomic<bool> 控制，避免数据竞争；
 *  - socket 采用非阻塞模式（fcntl(O_NONBLOCK)）。
 *
 * I/O 行为：
 *  - 非阻塞 recvfrom()，主循环中以 10ms 休眠降低 CPU 占用；
 *  - 绑定端口后打印本地端口信息（IP 固定为 0.0.0.0）。
 *
 * 兼容性：
 *  - 仅适用于 Linux/POSIX 环境。
 *
 * 使用示例（伪代码）：
 *   UDPServer s;
 *   s.start("127.0.0.1", 9000);
 *   s.sendMessage("127.0.0.1", 9000, buf, len);
 *   s.stop();
 */

#include <iostream>
#include <cstring>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#include "UDPServer.h"
#include "SimpleMsgHandler.h"

UDPServer::UDPServer()
    : udpSocket(INVALID_SOCKET_VALUE), serverPort(0), running(false)
{
    initializeNetwork();
}

UDPServer::~UDPServer()
{
    stop();
    cleanupNetwork();
}

bool UDPServer::initializeNetwork()
{
    // Linux/Posix 下无需特殊初始化
    return true;
}

void UDPServer::cleanupNetwork()
{
    // Linux/Posix 下无需特殊清理
}

void UDPServer::start(const std::string& address, uint16_t port)
{
    serverAddress = address;
    serverPort = port;

    // 创建 UDP socket
    udpSocket = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket == INVALID_SOCKET_VALUE) {
        std::cerr << "Failed to create UDP socket!" << std::endl;
        return;
    }

    // 非阻塞
    int flags = fcntl(udpSocket, F_GETFL, 0);
    fcntl(udpSocket, F_SETFL, flags | O_NONBLOCK);

    // 复用/广播
    int yes = 1;
    ::setsockopt(udpSocket, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
#ifdef SO_REUSEPORT
    ::setsockopt(udpSocket, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(yes)); // 可选：多进程/多实例一起监听
#endif
    ::setsockopt(udpSocket, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));  // ★ 开启广播权限

    // 绑定：使用传入的 address/port（port==0 自动分配；address 为空或非法则 ANY）
    struct sockaddr_in localAddr{};
    localAddr.sin_family = AF_INET;
    localAddr.sin_port   = htons(port);

    if (!address.empty() &&
        ::inet_pton(AF_INET, address.c_str(), &localAddr.sin_addr) == 1) {
        // 绑定到指定本地地址（用于多网卡时固定出接口）
    } else {
        localAddr.sin_addr.s_addr = INADDR_ANY;
    }

    if (::bind(udpSocket, reinterpret_cast<struct sockaddr*>(&localAddr),
               sizeof(localAddr)) < 0) {
        std::cerr << "Failed to bind UDP socket!" << std::endl;
        socket_close(udpSocket);
        udpSocket = INVALID_SOCKET_VALUE;
        return;
    }

    // 打印实际绑定
    socklen_t addrLen = sizeof(localAddr);
    ::getsockname(udpSocket, reinterpret_cast<struct sockaddr*>(&localAddr), &addrLen);
    char ipbuf[INET_ADDRSTRLEN] = {0};
    ::inet_ntop(AF_INET, &localAddr.sin_addr, ipbuf, sizeof(ipbuf));
    std::cout << "Local IP address: " << ipbuf << std::endl;
    std::cout << "Local port: " << ntohs(localAddr.sin_port) << std::endl;

    // 线程
    running = true;
    receiveThread = std::make_unique<std::thread>(&UDPServer::receiveDataLoop, this);
    timerThread   = std::make_unique<std::thread>(&UDPServer::messageTimerLoop, this);
}



void UDPServer::stop()
{
    running = false;

    if (receiveThread && receiveThread->joinable()) {
        receiveThread->join();
    }

    if (timerThread && timerThread->joinable()) {
        timerThread->join();
    }

    if (udpSocket != INVALID_SOCKET_VALUE) {
        socket_close(udpSocket);
        udpSocket = INVALID_SOCKET_VALUE;
    }
}

void UDPServer::receiveDataLoop()
{
    while (running) {
        onDataReceived();
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 避免 CPU 占用过高
    }
}

void UDPServer::onDataReceived()
{
    if (udpSocket == INVALID_SOCKET_VALUE) return;

    char buffer[1024];
    struct sockaddr_in clientAddr{};
    socklen_t clientAddrLen = sizeof(clientAddr);

    ssize_t receivedBytes = ::recvfrom(udpSocket,
                                       buffer,
                                       sizeof(buffer) - 1,
                                       0,
                                       reinterpret_cast<struct sockaddr*>(&clientAddr),
                                       &clientAddrLen);

    if (receivedBytes > 0) {
        SimpleMsgHandler::instance().handleDownlinkMessage(
            reinterpret_cast<const uint8_t*>(buffer),
            static_cast<std::size_t>(receivedBytes));
    }
}

bool UDPServer::sendMessage(const std::string& ip,
                            uint16_t           port,
                            const uint8_t*     buf,
                            std::size_t        len)
{
    if (udpSocket < 0 || buf == nullptr || len == 0)
        return false;

    sockaddr_in destAddr{};
    destAddr.sin_family = AF_INET;
    destAddr.sin_port   = htons(port);
    if (::inet_pton(AF_INET, ip.c_str(), &destAddr.sin_addr) != 1)
        return false;

    ssize_t sent = ::sendto(udpSocket,
                            reinterpret_cast<const char*>(buf),
                            static_cast<ssize_t>(len),
                            0,
                            reinterpret_cast<sockaddr*>(&destAddr),
                            sizeof(destAddr));

    return sent == static_cast<ssize_t>(len);
}

void UDPServer::messageTimerLoop()
{
    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(1)); // 1 秒间隔
    }
}
