#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <functional>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <memory>

// 仅 Linux/Posix 所需的头文件
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

// Linux/Posix 下的 socket 类型与工具宏
typedef int socket_t;                    // 套接字类型（Linux）
#define INVALID_SOCKET_VALUE -1          // 无效套接字值（Linux）
#define socket_close ::close             // 关闭套接字（Linux）

class UDPServer
{
public:
    explicit UDPServer();                                                    // 构造：初始化（Linux 下为空操作）
    ~UDPServer();                                                            // 析构：确保停止并清理资源

    void start(const std::string& address, uint16_t port);                   // 启动：绑定端口并启动接收/定时线程
    void stop();                                                             // 停止：停止线程并关闭套接字

    bool sendMessage(const std::string& ip,                                   // 发送：向指定 ip:port 发送二进制数据
                     uint16_t           port,
                     const uint8_t*     buf,
                     std::size_t        len);

private:
    void receiveDataLoop();                                                  // 接收线程主循环
    void onDataReceived();                                                   // 单次非阻塞接收与处理
    void messageTimerLoop();                                                 // 定时器线程循环（1 秒 tick）

private:
    socket_t udpSocket;                                                      // UDP 套接字
    struct sockaddr_in serverAddr;                                           // 预留：服务器地址结构
    std::string serverAddress;                                               // 目标服务端 IP（用于外发）
    uint16_t serverPort;                                                     // 目标服务端端口（用于外发）

    std::atomic<bool> running;                                               // 运行标志
    std::unique_ptr<std::thread> receiveThread;                              // 接收线程
    std::unique_ptr<std::thread> timerThread;                                // 定时器线程

    std::mutex callbackMutex;                                                // 回调互斥量（预留）

    bool initializeNetwork();                                                // 初始化网络（Linux 下为空操作）
    void cleanupNetwork();                                                   // 清理网络（Linux 下为空操作）
};
