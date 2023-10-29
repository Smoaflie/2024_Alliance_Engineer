/**
 * @Author       : HDC h2019dc@outlook.com
 * @Date         : 2023-09-08
 * @LastEditors  : HDC h2019dc@outlook.com
 * @LastEditTime : 2023-10-28
 * @FilePath     : \2024_Control_New_Framework_Base-dev-all\modules\alarm\buzzer.h
 * @Description  :
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */

#ifndef BUZZER_H
#define BUZZER_H
#include "bsp_pwm.h"

// 音符模式
typedef enum {
    NORMAL,  // 正常
    LEGATO,  // 连音
    STACCATO // 断音
} NoteMode_e;
typedef struct
{
    PWMInstance *buzzer_pwm;
    char *sound;
    char *_next_tune;
    uint8_t _note_mode;    // 音符模式
    unsigned _note_length; // 音符长度 2分，4分，8分，16分，32分，64分
    unsigned _octave;      // 八度 0-6
    unsigned _tempo;       // 节拍 32-255
    uint8_t _repeat;       // 是否重复
    unsigned note;         // 音符 1-84
    uint8_t busy;          // 是否忙
} BuzzerInstance;

extern char StartUP_sound[]; // 除了extern想不出smarter的方法了，各位大佬有什么好的方法可以提出来
extern char No_RC_sound[];

void BuzzerRegister(void);

void BuzzerPlay(char *sound);

#endif // !BUZZER_H
