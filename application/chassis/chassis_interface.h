#ifndef CHASSIS_INTERFACE_H
#define CHASSIS_INTERFACE_H

void ChassisInit_Motor();
void ChassisInit_Communication();
void ChassisInit_IO();

void RobotTumbleDetect();
void ChassisSubMessage();
void ChassisModeSelect();
void SpecialFuncApply();
void MecanumCalculate();
void SetChassisRef();
void ChassisDebugInterface();

#endif // CHASSIS_INTERFACE_H