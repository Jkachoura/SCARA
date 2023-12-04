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