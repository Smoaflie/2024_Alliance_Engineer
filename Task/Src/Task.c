#include "Task.h"
#include "CanSendTask.h"
#include "LedTask.h"
#include "KeyTask.h"

void InitTask()
{
    CanTimeInitInstance();
}

void Task()
{
    Key_Task();
    LedTask();
}
