/**
 * @Author       : HDC h2019dc@outlook.com
 * @Date         : 2023-09-08
 * @LastEditors  : HDC h2019dc@outlook.com
 * @LastEditTime : 2023-10-31
 * @FilePath     : \2024_Control_New_Framework_Base-dev-all\modules\buzzer\buzzer.h
 * @Description  :
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */

#ifndef BUZZER_H
#define BUZZER_H
#include "bsp_pwm.h"


#define Do_freq Note_Freq[49]
#define Re_freq Note_Freq[51]
#define Mi_freq Note_Freq[53]
#define Fa_freq Note_Freq[54]
#define So_freq Note_Freq[56]
#define La_freq Note_Freq[58]
#define Si_freq Note_Freq[60]

// 音符模式
typedef enum {
    NORMAL,  // 正常
    LEGATO,  // 连音
    STACCATO // 断音
} NoteMode_e;
typedef struct
{
    PWMInstance *buzzer_pwm;
    const char *sound;
    const char *_next_tune;
    uint8_t _note_mode;    // 音符模式
    unsigned _note_length_single; // 单音符长度 1分，2分，4分，8分，16分，32分，64分
    unsigned _note_length; // 音符长度 1分，2分，4分，8分，16分，32分，64分
    unsigned dots;         // 附点数
    unsigned _octave;      // 八度 0-8
    unsigned _tempo;       // 节拍 32-255
    uint8_t _repeat;       // 是否重复
    unsigned note;         // 音符 1-84
    uint8_t busy;          // 是否忙
    unsigned _slur;        // 是否连奏
} BuzzerInstance;

extern const uint16_t Note_Freq[];

extern const char StartUP_sound[]; // 除了extern想不出smarter的方法了，各位大佬有什么好的方法可以提出来,
extern const char No_RC_sound[];
extern const char RoboMaster_You[];
extern const char RoboMaster_Prepare[];
extern const char Test[];
extern const char DIDIDA[];
extern const char GuYongZhe[];
extern const char YongZheDouELong[];
extern const char DuoLaAMeng[];

void BuzzerRegister(void);

void BuzzerPlay(const char *sound);

void BuzzerStop();

void buzzer_silence(void);

void buzzer_one_note(uint16_t Note, float delay);

void BuzzerTask(void *argument);

#endif // !BUZZER_H
