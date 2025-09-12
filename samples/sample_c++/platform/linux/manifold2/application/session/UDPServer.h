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

// 网络相关头文件
#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
#endif

// 定义socket类型
#ifdef _WIN32
    typedef SOCKET socket_t;
    #define INVALID_SOCKET_VALUE INVALID_SOCKET
    #define socket_close closesocket
#else
    typedef int socket_t;
    #define INVALID_SOCKET_VALUE -1
    #define socket_close close
#endif

class UDPServer
{
public:
    
    explicit UDPServer();
    ~UDPServer();

    void start(const std::string& address, uint16_t port);
    void stop();

    bool sendMessage(const std::string& ip,
                     uint16_t           port,
                     const uint8_t*     buf,
                     std::size_t        len);

private:
    void receiveDataLoop();
    void onDataReceived();
    void sendHelloMessage();
    void sendPeriodicMessages();
    void messageTimerLoop();

private:
    socket_t udpSocket;
    struct sockaddr_in serverAddr;
    std::string serverAddress;
    uint16_t serverPort;
    
    std::atomic<bool> running;
    std::unique_ptr<std::thread> receiveThread;
    std::unique_ptr<std::thread> timerThread;
    
    std::mutex callbackMutex;
    
    // 网络初始化和清理
    bool initializeNetwork();
    void cleanupNetwork();
};
