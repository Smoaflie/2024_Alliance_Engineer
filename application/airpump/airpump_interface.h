#ifndef AIRPUMP_INTERFACE_H
#define AIRPUMP_INTERFACE_H

void AirpumpInit_Motor();
void AirpumpInit_Communication();
void AirpumpInit_IO();
void AirpumpInit_Param();


void AirpumpSubMessage();
void AirpumpParamPretreatment();
void AirpumpContro_Valve();
void AirpumpContro_Pump();
void AirpumpContro_Sucker();
void AirpumpEmergencyHandler();
void AirpumpPubMessage();

void AirpumpDebugInterface();

#endif // AIRPUMP_INTERFACE_H