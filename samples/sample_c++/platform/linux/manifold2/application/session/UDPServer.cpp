#include <iostream>
#include <cstring>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#endif

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
#ifdef _WIN32
    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (result != 0) {
        std::cerr << "WSAStartup failed: " << result << std::endl;
        return false;
    }
#endif
    return true;
}

void UDPServer::cleanupNetwork()
{
#ifdef _WIN32
    WSACleanup();
#endif
}

void UDPServer::start(const std::string& address, uint16_t port)
{
    serverAddress = address;
    serverPort = port;

    // 创建UDP socket
    udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket == INVALID_SOCKET_VALUE) {
        std::cerr << "Failed to create UDP socket!" << std::endl; 
        return;
    }

    // 设置socket为非阻塞模式
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(udpSocket, FIONBIO, &mode);
#else
    int flags = fcntl(udpSocket, F_GETFL, 0);
    fcntl(udpSocket, F_SETFL, flags | O_NONBLOCK);
#endif

    // 设置地址重用
    int reuse = 1;
    setsockopt(udpSocket, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));

    // 绑定本地端口
    struct sockaddr_in localAddr{};
    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = INADDR_ANY;
    localAddr.sin_port = htons(43434);  // 自动选择可用端口

    if (bind(udpSocket, (struct sockaddr*)&localAddr, sizeof(localAddr)) < 0) {
        std::cerr << "Failed to bind UDP socket!" << std::endl;
        socket_close(udpSocket);
        udpSocket = INVALID_SOCKET_VALUE;
        return;
    }

    // 获取实际绑定的地址信息
    socklen_t addrLen = sizeof(localAddr);
    getsockname(udpSocket, (struct sockaddr*)&localAddr, &addrLen);
    
    std::cout << "Local IP address: 0.0.0.0" << std::endl;
    std::cout << "Local port: " << ntohs(localAddr.sin_port) << std::endl;

    // 设置运行状态
    running = true;

    // 启动接收数据线程
    receiveThread = std::make_unique<std::thread>(&UDPServer::receiveDataLoop, this);

    // 启动定时器线程，每隔1秒发送一次消息
    timerThread = std::make_unique<std::thread>(&UDPServer::messageTimerLoop, this);
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
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 避免CPU占用过高
    }
}

void UDPServer::onDataReceived()
{
    if (udpSocket == INVALID_SOCKET_VALUE) return;

    char buffer[1024];
    struct sockaddr_in clientAddr{};
    socklen_t clientAddrLen = sizeof(clientAddr);

    ssize_t receivedBytes = recvfrom(udpSocket, buffer, sizeof(buffer) - 1, 0,
                                   (struct sockaddr*)&clientAddr, &clientAddrLen);

    if (receivedBytes > 0) {
        SimpleMsgHandler::instance().handleDownlinkMessage(reinterpret_cast<const uint8_t*>(buffer), receivedBytes);
    }
}

void UDPServer::sendHelloMessage()
{
    // MessageProcessor::test_pos();
    // std::string helloMessage = "hello";
    // struct sockaddr_in destAddr{};
    // destAddr.sin_family = AF_INET;
    // inet_pton(AF_INET, serverAddress.c_str(), &destAddr.sin_addr);
    // destAddr.sin_port = htons(serverPort);
    // 
    // sendto(udpSocket, helloMessage.c_str(), helloMessage.length(), 0,
    //        (struct sockaddr*)&destAddr, sizeof(destAddr));
    // std::cout << "Sent hello message to " << serverAddress << ":" << serverPort << std::endl;
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
    if (inet_pton(AF_INET, ip.c_str(), &destAddr.sin_addr) != 1)
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
        std::this_thread::sleep_for(std::chrono::seconds(1)); // 1秒间隔
    }
}

