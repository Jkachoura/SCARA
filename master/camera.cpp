// Camera.cpp
#include "Camera.h"

Camera::Camera(const char* targetIp, int targetPort) : target_ip(targetIp), target_port(targetPort) {
    // Initialize Winsock
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "Failed to initialize Winsock" << std::endl;
        // Handle error, maybe throw an exception or exit the program
    }

    // Create a socket object
    client_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket == INVALID_SOCKET) {
        std::cerr << "Error creating socket: " << WSAGetLastError() << std::endl;
        // Handle error, maybe throw an exception or exit the program
    }

    // Set up the server address structure
    sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(target_port);
    server_address.sin_addr.s_addr = inet_addr(target_ip);

    // Connect to the server
    if (connect(client_socket, (struct sockaddr*)&server_address, sizeof(server_address)) == SOCKET_ERROR) {
        std::cerr << "Error connecting to the server: " << WSAGetLastError() << std::endl;
        // Handle error, maybe throw an exception or exit the program
    }
}

Camera::~Camera() {
    // Close the socket
    closesocket(client_socket);

    // Cleanup Winsock
    WSACleanup();
}

void Camera::capture() {
    // Your TRG message (replace this with your actual message)
    const char* trg_message = "TRG";

    // Send the message
    send(client_socket, trg_message, strlen(trg_message), 0);
}
