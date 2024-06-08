#ifndef ARM_INTERFACE_H
#define ARM_INTERFACE_H

void ArmInit_Encoder();
void ArmInit_Motor();
void ArmInit_Communication();
void ArmInit_IO();

void ArmSubMessage();
void ArmParamPretreatment();
void ArmControInterface();
void ArmCommunicateHOST();
void ArmPubMessage();
void ArmDebugInterface();
#endif // ARM_INTERFACE_H