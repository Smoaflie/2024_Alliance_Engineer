#ifndef GIMBAL_INTERFACE_H
#define GIMBAL_INTERFACE_H

void GimbalInit_Motor();
void GimbalInit_Communication();
void GimbalInit_IO();
void GimbalInit_Param();


void GimbalSubMessage();
void GimbalParamPretreatment();
void GimbalContro();
void GimbalPubMessage();

void GimbalDebugInterface();

#endif // GIMBAL_INTERFACE_H