/**
 * @Author       : HDC h2019dc@outlook.com
 * @Date         : 2023-09-08
 * @LastEditors  : HDC h2019dc@outlook.com
 * @LastEditTime : 2023-10-31
 * @FilePath     : \2024_Control_New_Framework_Base-dev-all\modules\buzzer\buzzer.c
 * @Description  : 蜂鸣器模块，单实例模块，不返回实例指针
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */
#include "bsp_pwm.h"
#include "buzzer.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"
#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include "user_lib.h"
#include "robot_def.h"

extern osThreadId_t BuzzerHandle; // 由于线程挂起后无法自己恢复，所以需要使用线程句柄
static BuzzerInstance *buzzer = NULL;
// 音符偏移量
static const uint8_t _note_tab[] = {9, 11, 0, 2, 4, 5, 7};
// 音符频率
const uint16_t Note_Freq[] = {0,
                              16, 17, 18, 19, 21, 22, 23, 25, 26, 28, 29, 31,
                              33, 35, 37, 39, 41, 44, 46, 49, 52, 55, 58, 61,
                              65, 69, 73, 78, 82, 87, 92, 98, 104, 110, 117, 123,
                              131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247,
                              262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494,
                              523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988,
                              1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976,
                              2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951,
                              4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902};
// 音乐字符串 @todo（存入flash节省空间）
char StartUP_sound[]      = "T240L4 O6cde L2g";
char No_RC_sound[]        = "T200L8 O5ecececec";
char RoboMaster_You[]     = "T75 L4O5ef g.L8e gL16eL8g.L4b O6c. O5L8cdeL4g L8a...L16gL8aL16gL8g.g L2d L4efL2g L8gL16eL8g.O6L4dc. O5L8cde O6L4c O5L8a...L16g L8aL16gL8a. O6L4cd. P8O5L8deL16d L1c";
char RoboMaster_Prepare[] = "T140L8 O4aaO5cc O4g#g#aa O4aaO5cc O4g#g#aa O4aaO5cc O4g#g#aa O4aaO5cc O4g#g#aa O4aaO5dd O4g#g#aa O4aaO5dd O4g#g#aa O4g#g#aa bbO5cc ddee O4bbO5aa O4aaO5cc O4g#g#aa O4aaO5cc O4g#g#aa O4aaO5cc O4g#g#aa O4aaO5cc O4g#g#aa O4aaO5dd O4g#g#aa O4aaO5dd O4g#g#aa O4g#g#aa bbO5cc ddee O4bbO5aa L2O5eL1O6c L2O5eL1O5a L2O5cL1O5f L2O4bL1O5e L2O5eL1O6c L2O5eL1O5a L2O5cL1O5f L2O4bL1O5e";
char Test[]               = "T240L8 O5aO6dc O5aO6dc O5aO6dc L16dcdcdcdc";
/**
 * @brief :  蜂鸣器注册
 * @return  void
 */
void BuzzerRegister(void)
{
    BuzzerInstance *buzzer_ins      = (BuzzerInstance *)zmalloc(sizeof(BuzzerInstance));
    PWM_Init_Config_s buzzer_config = {
        .htim      = &htim4,
        .channel   = TIM_CHANNEL_3,
        .dutyratio = 0,
        .period    = 0.001f,
        .callback  = NULL,
        .id        = NULL,
    };
    buzzer_ins->buzzer_pwm   = PWMRegister(&buzzer_config);
    buzzer_ins->sound        = NULL;
    buzzer_ins->_repeat      = false;
    buzzer_ins->_note_mode   = NORMAL;
    buzzer_ins->_octave      = 4;
    buzzer_ins->_tempo       = 120;
    buzzer_ins->_note_length = 4;
    buzzer_ins->_next_tune   = NULL;
    buzzer_ins->note         = 0;
    buzzer_ins->busy         = 0;
    buzzer                   = buzzer_ins;
}
/**
 * @brief :  蜂鸣器播放
 * @param *sound 音乐字符串
 * @return  void
 */
void BuzzerPlay(char *sound)
{
    // 如果蜂鸣器未注册，则注册
    if (buzzer == NULL) {
        BuzzerRegister();
    }
    if (buzzer->busy) return;
    buzzer->sound      = sound;
    buzzer->_next_tune = sound;
    buzzer->busy       = 1;
    osThreadResume(BuzzerHandle); // 恢复线程
}
/**
 * @brief :  音符播放
 * @return   void
 */
inline static void NotePlay()
{
    if (buzzer->note == 0) { // 休止符
        PWMSetDutyRatio(buzzer->buzzer_pwm, 0);
    } else {
        float freq = Note_Freq[buzzer->note];
        PWMSetPeriod(buzzer->buzzer_pwm, 1.0f / freq);
#if (BUZZER_SILENCE == 1)
        PWMSetDutyRatio(buzzer->buzzer_pwm, 0);
#else
        PWMSetDutyRatio(buzzer->buzzer_pwm, 0.5f); // 音量
#endif
    }
    float note_delay = 1000 * (4.0f / (float)buzzer->_note_length) * 60 / (float)buzzer->_tempo;
    note_delay += note_delay / 2.0f * (float)buzzer->dots;
    osDelay((uint32_t)note_delay);
    PWMSetDutyRatio(buzzer->buzzer_pwm, 0);
    osDelay(10);
}
/**
 * @brief :  下一个字符
 * @return  int
 */
inline static int next_char()
{
    char *next_tune = buzzer->_next_tune;
    while (*next_tune == ' ') {
        next_tune++;
    }
    buzzer->_next_tune = next_tune;

    return toupper(*buzzer->_next_tune);
}
/**
 * @brief :  下一个数字
 * @return  unsigned
 */
inline static unsigned next_number()
{
    unsigned number = 0;
    int next_character;
    buzzer->_next_tune++;
    for (;;) {
        next_character = next_char();

        if (!isdigit(next_character)) {
            return number;
        }

        buzzer->_next_tune++;
        number = (number * 10) + (next_character - '0');
    }
}
/**
 * @brief :  下一个点
 * @return  unsigned
 */
inline static unsigned next_dots()
{
    unsigned dots = 0;
    while (next_char() == '.') {
        buzzer->_next_tune++;
        dots++;
    }
    return dots;
}
/**
 * @brief :  音符处理
 * @return  void
 */
inline static void Note_handle()
{
    int c = next_char();
    if (c >= 'A' && c <= 'G') { buzzer->note = _note_tab[c - 'A'] + (buzzer->_octave * 12) + 1; }
    buzzer->_next_tune++;
    c = next_char();
    switch (c) {
        case '#': // Up a semitone.
        case '+':
            if (buzzer->note < 84) {
                buzzer->note++;
            }
            buzzer->_next_tune++;
            break;
        case '-': // Down a semitone.
            if (buzzer->note > 1) {
                buzzer->note--;
            }
            buzzer->_next_tune++;
            break;
        default:
            // 0 / No next char here is OK.
            break;
    }
    buzzer->dots = next_dots();
#if 0
    // Shorthand length notation.
    unsigned note_length = next_number();

    if (note_length == 0) {
        note_length = buzzer->_note_length;
    }
#endif
}
/**
 * @brief :  音符模式处理
 * @return  void
 */
inline static void Mode_handle()
{
    int c = next_char();
    if (c == 0) {
        return;
    }
    buzzer->_next_tune++;

    switch (c) {
        case 'N':
            buzzer->_note_mode = NORMAL;
            break;

        case 'L':
            buzzer->_note_mode = LEGATO;
            break;

        case 'S':
            buzzer->_note_mode = STACCATO;
            break;

        case 'F':
            buzzer->_repeat = false;
            break;

        case 'B':
            buzzer->_repeat = true;
            break;

        default:
            return;
    }
}
/**
 * @brief :  音乐字符串处理
 * @return  void
 */
inline static void string_handle()
{
    while (1) {
        if (buzzer->_next_tune == NULL)
            return;
        int c = next_char();
        switch (c) {
            case 'L': // Select note length.
                buzzer->_note_length = next_number();
                if (buzzer->_note_length < 1)
                    return;
                break;
            case 'O': // Select octave.
                buzzer->_octave = next_number();
                if (buzzer->_octave > 8) {
                    buzzer->_octave = 8;
                }
                break;
            case '<': // Decrease octave.
                if (buzzer->_octave > 0) {
                    buzzer->_octave--;
                }
                break;
            case '>': // Increase octave.
                if (buzzer->_octave < 8) {
                    buzzer->_octave++;
                }
                break;
            case 'M': // Select inter-note gap.
                Mode_handle();
                break;
            case 'P': // Pause for a note length.
                buzzer->_note_length = next_number();
                buzzer->dots         = next_dots();
                buzzer->note         = 0;
                NotePlay();
                if (buzzer->_note_length < 1)
                    return;
                break;
            case 'T': { // Change tempo.
                unsigned nt = next_number();
                if ((nt >= 32) && (nt <= 255)) {
                    buzzer->_tempo = nt;
                } else {
                    return;
                }
                break;
            }
#if 0
            case 'N': // Play an arbitrary note.
                note = next_number();

                if (note > 84) {
                    return tune_error();
                }

                if (note == 0) {
                    // This is a rest - pause for the current note length.
                    silence = rest_duration(_note_length, next_dots());
                    return Tunes::Status::Continue;
                }

                break;
#endif
            case 'A' ... 'G': // Play a note in the current octave.
                Note_handle();
                NotePlay();
                break;
            default:
                return;
        }
    }
}

/**
 * @brief :  蜂鸣器静音
 * @return  void
 */
void buzzer_silence(void)
{
    PWMSetDutyRatio(buzzer->buzzer_pwm, 0);
}
/**
 * @brief :  蜂鸣器播放单个音符，阻塞模式
 * @param Note 音符
 * @param delay 延时
 * @return  void
 */
void buzzer_one_note(uint16_t Note, float delay)
{
    // 如果蜂鸣器未注册，则注册
    if (buzzer == NULL) {
        BuzzerRegister();
    }
    PWMSetPeriod(buzzer->buzzer_pwm, 1.0f / (float)Note);
#if (BUZZER_SILENCE == 1)
    PWMSetDutyRatio(buzzer->buzzer_pwm, 0);
#else
    PWMSetDutyRatio(buzzer->buzzer_pwm, 0.5f); // 音量
#endif
    DWT_Delay(delay);
    PWMSetDutyRatio(buzzer->buzzer_pwm, 0);
}
/**
 * @brief :  蜂鸣器任务，播放完毕后自动挂起线程，循环播放除外
 * @return  void
 */
__attribute__((noreturn)) void BuzzerTask(void *argument)
{
    UNUSED(argument);
    osThreadSuspend(NULL); // 挂起线程
    for (;;) {
        do {
            string_handle();
            buzzer_silence();
        } while (buzzer->_repeat);
        buzzer->busy = 0;
        osThreadSuspend(NULL); // 挂起线程
    }
}
