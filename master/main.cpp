#include "master.h"
#include "slave.h"
#include "scara.h"
#include <iostream>

//-90000 is -90 graden
int main(int argc, char* argv[]) {
    char ifaceName[] = "\\Device\\NPF_{DEA85026-34BA-4C8B-9840-A3CE7793A348}";
    Master ecMaster(ifaceName, 8000);

    SCARA scaraRobot(250, 280);

    JointAngles angles = scaraRobot.calculateJointAngles(423.70, -45.51, false);

    if (ecMaster.connected()) {
        Slave ecSlave(ecMaster, 1);
        Slave ecSlave2(ecMaster, 2);
        Slave ecSlave3(ecMaster, 3);

        ecSlave.acknowledge_faults();
        ecSlave.enable_powerstage();
        ecSlave2.acknowledge_faults();
        ecSlave2.enable_powerstage();
        ecSlave3.acknowledge_faults();
        ecSlave3.enable_powerstage();
        
        ecSlave3.position_task(0, 400, true);
        ecSlave2.position_task(0 * 1000, 40000, true, false);
        ecSlave.position_task(0 * 1000, 40000, true, false);

        Sleep(1000);
   
        ecSlave2.position_task(angles.j2 * 1000, 40000, true, false);
        ecSlave.position_task(angles.j1 * 1000, 40000, true, false);
        ecSlave3.position_task(275 * 1000, 400, true);

        Sleep(1000);

        ecSlave3.position_task(0, 400, true);
        ecSlave2.position_task(0 * 1000, 40000, true, false);
        ecSlave.position_task(0 * 1000, 40000, true, false);

        Sleep(1000);

        return EXIT_SUCCESS;
    }
    else {
        return EXIT_FAILURE;
    }
}

