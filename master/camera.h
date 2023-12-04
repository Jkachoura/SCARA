// Camera.h
#ifndef CAMERA_H
#define CAMERA_H

#include <WinSock2.h>
#include <iostream>
#include <cstring>
#include <vector>
#include <sstream>
#include <stdexcept>

#pragma comment(lib, "ws2_32.lib")

class Camera {
    public:
        Camera(const char* targetIp, int targetPort);
        ~Camera();

        void capture();
        std::vector<double> receiveMessage();
        std::vector<double> splitAndConvertToDoubles(const std::string& message);

    private:
        const char* target_ip;
        int target_port;
        SOCKET client_socket;
};

#endif // CAMERA_H
