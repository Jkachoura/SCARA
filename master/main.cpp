#include "master.h" 
#include "slave.h"
#include "scara.h"
#include "camera.h"

JointAngles calculateJointAngles2(double x, double y, bool elbowLeft);
int main(int argc, char* argv[]){
    int numSlaves = 4;
    
    char ifaceName[] = "\\Device\\NPF_{DEA85026-34BA-4C8B-9840-A3CE7793A348}";
    Master ecMaster(ifaceName, 8000);
    Camera client("192.168.0.101", 2006); // send message to camera
    Camera client2("192.168.0.101", 2005); //receive message from camera
    if (ecMaster.connected()){
       std::vector <Slave> ecSlaves;
        
        for(int i = 1; i <= numSlaves; i++) {
            ecSlaves.push_back(Slave(ecMaster, i));
        }

        SCARA scaraRobot(250, 280, ecSlaves, 3);
        
        // camera.recieve_message();
      
        client.capture();
        std::vector<double> coordinates = client2.receiveMessage();
        if (coordinates.empty()) {
            std::cerr << "Error receiving message or connection closed" << std::endl;
        }

        // Access the individual coordinates
        double x = coordinates[0];
        double y = coordinates[1];
        double angle = coordinates[2];

        // Use the coordinates as needed
        std::cout << "Received coordinates: X=" << x << ", Y=" << y << ", Angle=" << angle << std::endl;
    

        

        return EXIT_SUCCESS;
    }
    else{
        return EXIT_FAILURE;
    }
}

// #include <iostream>
// #include <WS2tcpip.h>

// #pragma comment(lib, "ws2_32.lib")

// int main() {
//     // Initialize Winsock
//     WSADATA wsData;
//     WORD ver = MAKEWORD(2, 2);

//     if (WSAStartup(ver, &wsData) != 0) {
//         std::cerr << "Error initializing Winsock" << std::endl;
//         return -1;
//     }

//     // Create a socket
//     SOCKET clientSocket = socket(AF_INET, SOCK_STREAM, 0);
//     if (clientSocket == INVALID_SOCKET) {
//         std::cerr << "Error creating socket" << std::endl;
//         WSACleanup();
//         return -1;
//     }

//     // Set up the server address
//     sockaddr_in serverAddr;
//     serverAddr.sin_family = AF_INET;
//     serverAddr.sin_port = htons(2005); // Change this to the desired port
//     inet_pton(AF_INET, "192.168.0.101", &(serverAddr.sin_addr));

//     // Connect to the server
//     if (connect(clientSocket, reinterpret_cast<sockaddr*>(&serverAddr), sizeof(serverAddr)) == SOCKET_ERROR) {
//         std::cerr << "Error connecting to server" << std::endl;
//         closesocket(clientSocket);
//         WSACleanup();
//         return -1;
//     }

//     std::cout << "Connected to server" << std::endl;

//     // Receive and print messages in a loop
//     char buffer[1024];
//     while (true) {
//         int bytesReceived = recv(clientSocket, buffer, sizeof(buffer), 0);
//         if (bytesReceived <= 0) {
//             std::cerr << "Connection closed or error" << std::endl;
//             break;
//         }

//         std::cout << "Received message: " << std::string(buffer, bytesReceived) << std::endl;
//     }

//     // Cleanup
//     closesocket(clientSocket);
//     WSACleanup();

//     return 0;
// }