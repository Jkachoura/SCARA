// Camera.h
#ifndef CAMERA_H
#define CAMERA_H

#include <WinSock2.h>
#include <iostream>
#include <cstring>

#pragma comment(lib, "ws2_32.lib")

class Camera {
public:
    Camera(const char* targetIp, int targetPort);
    ~Camera();

    void capture();

private:
    const char* target_ip;
    int target_port;
    SOCKET client_socket;
};

#endif // CAMERA_H
