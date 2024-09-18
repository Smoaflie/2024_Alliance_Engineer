#pragma once

#include "main.h"

// void LedControlDelay(char LedLable, GPIO_PinState State, uint32_t DelayTime);

void LedControl(char LedLable, GPIO_PinState State);

void LedToggle(char LedLable);

// void LedToggleDelay(char LedLable, uint32_t DelayTime);