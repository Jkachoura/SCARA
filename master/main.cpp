#include "master.h" 
#include "slave.h"
#include "scara.h"
#include "camera.h"

int main(int argc, char* argv[]){
    int numSlaves = 4;    
    char ifaceName[] = "\\Device\\NPF_{DEA85026-34BA-4C8B-9840-A3CE7793A348}";
    Master ecMaster(ifaceName, 8000);
    Camera client("192.168.0.101", 2006); // send message to camera
    Camera client2("192.168.0.101", 2005); // receive message from camera
    if (ecMaster.connected()){
       std::vector <Slave> ecSlaves;
        
        for(int i = 1; i <= numSlaves; i++) {
            ecSlaves.push_back(Slave(ecMaster, i));
        }

        SCARA scaraRobot(250, 280, ecSlaves, 3);

        while(true){
            client.capture();

            // Save the data from the camera in a vector
            std::vector<double> coordinates = client2.receiveMessage();

            double x = coordinates[0];
            double y = coordinates[1];
            double angle = coordinates[2];

            // Pick up the battery from the coordinates given by the camera
            scaraRobot.pickUp(x, y, angle, false);

            // Deliver the battery to destination
            scaraRobot.drop(false);
        }
            
    
        return EXIT_SUCCESS;
    }
    else{
        return EXIT_FAILURE;
    }
}