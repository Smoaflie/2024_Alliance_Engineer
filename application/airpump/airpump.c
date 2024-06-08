#include "airpump_interface.h"

void AIRPUMPInit()
{
    AirpumpInit_Motor();
    AirpumpInit_Communication();
    AirpumpInit_IO();
    AirpumpInit_Param();
}

void AIRPUMPTask()
{
    AirpumpSubMessage();
    AirpumpParamPretreatment();
    AirpumpContro_Valve();
    AirpumpContro_Pump();
    AirpumpContro_Sucker();
    AirpumpEmergencyHandler();
    AirpumpPubMessage();

    AirpumpDebugInterface();
}
