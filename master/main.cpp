#include "master.h"
#include "slave.h"
#include "scara.h"
#include <iostream>
#include <vector>

const int numSlaves = 3;

int main(int argc, char* argv[]) {
    char ifaceName[] = "\\Device\\NPF_{DEA85026-34BA-4C8B-9840-A3CE7793A348}";
    Master ecMaster(ifaceName, 8000);

    if (ecMaster.connected()) {
        std::vector <Slave> ecSlaves;
        
        for(int i = 1; i <= numSlaves; i++) {
            ecSlaves.push_back(Slave(ecMaster, i));
        }

        SCARA scaraRobot(250, 280, ecSlaves);

        scaraRobot.initSlaves();

        std::vector<int> velocities = {40000, 50000, 400};

        std::vector<int> StartPositions = {0, 0, 0};
        scaraRobot.moveToPosT(velocities, StartPositions, true);
        
        while(true){
            std::vector<std::pair<double, double>> targetPositions = {
                {244.16, -187.19}, {333.86, -186.59}, {424.23, -186.09},
                {244.58, -47.11}, {334.07, -45.80}, {423.70, -45.51},
                {244.73, 92.74}, {333.82, 93.47}, {423.77, 94.23}
            };

            std::vector<std::vector<int>> endPositions;

            // Calculate and store end positions for each target position
            for (const auto& target : targetPositions) {
                JointAngles angles = scaraRobot.calculateJointAngles(target.first, target.second, false);
                endPositions.push_back({ (int)angles.j1 * 1000, (int)angles.j2 * 1000, 275 * 1000 });
            }

            // Move to each target position
            for (const auto& endPos : endPositions) {
                scaraRobot.moveToPosT(velocities, endPos, false);
            }

            // Move back to the starting position
            scaraRobot.moveToPosT(velocities, StartPositions, true);
            Sleep(1000);
        }

        return EXIT_SUCCESS;
    }
    else {
        return EXIT_FAILURE;
    }
}

