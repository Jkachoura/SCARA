// Camera.cpp
#include "Camera.h"

/**
 * Constructor that initializes Winsock and creates a socket.
 * 
 * @param targetIp The IP address of the camera
 * @param targetPort The port of the camera
*/
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

/**
 * Closes the socket and cleans up Winsock.
*/
Camera::~Camera() {
    // Close the socket
    closesocket(client_socket);

    // Cleanup Winsock
    WSACleanup();
}

/**
 * Splits the message by semicolons and converts each token to a double.
 * 
 * @param message The message to split and convert
 * 
 * @return A vector of doubles
*/
std::vector<double> Camera::splitAndConvertToDoubles(const std::string& message) {
    std::vector<double> values(3, 0.0);  // Initialize a vector of size 3 with zeros
    std::istringstream iss(message);
    std::string token;

    for (int i = 0; i < 3; ++i) {
        if (!std::getline(iss, token, ';')) {
            std::cerr << "Error: Not enough values in the message" << std::endl;
            break;
        }
        try {
            double value = std::stod(token) / 1000.0; // Convert to double and divide by 1000
            values[i] = value;
        } catch (const std::invalid_argument& e) {
            std::cerr << "Invalid argument: " << e.what() << std::endl;
        } catch (const std::out_of_range& e) {
            std::cerr << "Out of range: " << e.what() << std::endl;
        }
    }

    return values;
}


/**
 * Sends a TRG message to the camera.
*/
void Camera::capture() {
    // Your TRG message (replace this with your actual message)
    const char* trg_message = "TRG";

    // Send the message
    send(client_socket, trg_message, strlen(trg_message) + 1, 0);

    std::cout << "Capture done" << std::endl;
}

/**
 * Receives a message from the camera and splits it into a vector of doubles.
 * 
 * @return A vector of doubles
*/
std::vector<double> Camera::receiveMessage() {
    //print 
    std::cout << "Receiving message" << std::endl;
    char buffer[1024];
    int bytesReceived = recv(client_socket, buffer, sizeof(buffer), 0);
    //print
    std::cout << "Received " << bytesReceived << " bytes" << std::endl;
    if (bytesReceived <= 0) {
        std::cerr << "Error receiving message or connection closed" << std::endl;
        exit(EXIT_FAILURE);
    }

    return splitAndConvertToDoubles(std::string(buffer, bytesReceived));
}
