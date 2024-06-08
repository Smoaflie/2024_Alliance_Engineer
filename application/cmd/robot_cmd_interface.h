#ifndef CMD_INTERFACE_H

void RobotCMDInit_VisionLine();
void RobotCMDInit_RC();
void RobotCMDInit_Param();
void RobotCMDInit_IO();
void RobotCMDInit_Communication();

void RobotActive();
void RobotCMDSubMessage();
void RobotCMDParamPretreatment();
void RobotCMDGenerateCommand();
void RobotCMDPubMessage();
void RobotCMDDebugInterface();

#define CMD_INTERFACE_H
#endif // CMD_INTERFACE_H