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
static float buzzer_one_note_time;
static uint8_t buzzer_one_note_flag = 0;
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
const char StartUP_sound[]      = "T240L4 O6cde L2g";
const char No_RC_sound[]        = "T200L8 O5ecececec";
const char RoboMaster_You[]     = "T75 L4O5ef g.L8e gL16eL8g.L4b O6c. O5L8cdeL4g L8a...L16gL8aL16gL8g.g L2d L4efL2g L8gL16eL8g.O6L4dc. O5L8cde O6L4c O5L8a...L16g L8aL16gL8a. O6L4cd. P8O5L8deL16d L1c";
const char RoboMaster_Prepare[] = "T140L8 O4aaO5cc O4g#g#aa O4aaO5cc O4g#g#aa O4aaO5cc O4g#g#aa O4aaO5cc O4g#g#aa O4aaO5dd O4g#g#aa O4aaO5dd O4g#g#aa O4g#g#aa bbO5cc ddee O4bbO5aa O4aaO5cc O4g#g#aa O4aaO5cc O4g#g#aa O4aaO5cc O4g#g#aa O4aaO5cc O4g#g#aa O4aaO5dd O4g#g#aa O4aaO5dd O4g#g#aa O4g#g#aa bbO5cc ddee O4bbO5aa L2O5eL1O6c L2O5eL1O5a L2O5cL1O5f L2O4bL1O5e L2O5eL1O6c L2O5eL1O5a L2O5cL1O5f L2O4bL1O5e";
const char Test[]               = "T240L8 O5aO6dc O5aO6dc O5aO6dc L16dcdcdcdc";
const char DIDIDA[]             = "t130L8 g.&a+64r64<f+16.&f+64r64>f+16.&f+64r64f+16.&f+64r64>c+16.&c+64r64<f+16.&f+64r64f+16.&f+64r64b16.&b64r64a+16.&a+64r64<g+16.&g+64r64>g+16.&g+64r64g+16.&g+64r64b16.&b64r64b16.&b64r64a+16.&a+64r64b16.&b64r64g+16.&g+64r64<f+16.&f+64r64>f+16.&f+64r64f+16.&f+64r64>d+16.&d+64r64<f+16.&f+64r64c+16.&c+64r64g+16.&g+64r64a+16.&a+64r64<e16.&e64r64>d+16.&d+64r64d+16.&d+64r64b16.";
const char GuYongZhe[]          = "t65L4 <g+16>f+16g+8&g+32.r64g+16g+16f+16g+16f+16a+8&a+32.r64a+16a+16g+16a+8&a+32.r64g+8d+2&d+32r16.d+16f+16d+16c+8&c+32.r64d+16c+8&c+32.r64d+16c+8&c+32.r64d+16f+16d+16f+16d+16c+8&c+32.";
const char YongZheDouELong[]    = "t65L4 @4v10o5l8d4a4g2.fed4c<b->c<a>e4d1.a4>c4<b2.gfe4fga1&a1d4a4g2.fed4c<b->c<a>e4d1.";
const char DuoLaAMeng[]         = "t180l8 o6dc+<bab>c+dc+<bab>c+dc+<bab>c+dc+<ba4.>edc+c-c+dedc+c-c+dedc+c-c+dc+dd+e4.c-c+d<gababn73f+gagabef+gf+gadef+>ed+dc+c<ba+ag+gf+fe1&e2<a4>dd4f+b4f+a4.a4ba4f+g4f+e4.c-4ee4g>c+4c+<b4ag4gg4f+c-4c+4.de4.<ab>dc+del4<a.a>d8df+8bf+8a.ab8af+8gf+8e.c-e8eg8>c+.<ba8gg8f+e8c+.e.d<a8>f+e8dl8ef+gab4.b4agabl4a.ef+8g+e8a2.<b.a.ef+8g+e8a.a>e8ea8ag8b.a.g2.e.n73b8ab8ag2al8bf+2&f+ed1&d2>d4dc+4c+<b4ba4ab4b>c+4c+d4.<dc+de4<bb4.>e4<bb4.b4>c+d4c-e2.>d4dc+4c+<b4ba>def+4f+f+ef+g2&g<d<b4a+b4>dc+4dl4e.d2.d2.<a>d8df+8bf+8a.ab8af+8gf+8e.c-e8eg8>c+c+8<ba8gg8gf+8c-c+.d8e.l8<ab>dc+del4<a.a>d8df+8bf+8a.ab8af+8gf+8e.c-e8eg8>c+.<ba8gg8f+e8c+.e.d<a8>f+e8dl8ef+gab4.b4agabl4a.ef+8g+e8a2.<b.a.ef+8g+e8a.a>e8ea8ag8b.a.g2.e.n73b8ab8ag2al8bf+2&f+ed2.&d<ab>c+def+ff+d4<a>f+ff+d4<a>gf+ge4.<a4aa4.>gf+ge4<a>gf+ge4<a>f+gg+a4.a4aa4.ba+ba+ba+b4>c+d4<bag+ag+ag+a4>c+d4<a>c+cc+cc+cc+4dedc+d1&d2<<a4>dd4f+b4f+a4.a4ba4f+g4f+e4.c-4ee4g>c+4c+<b4ag4gg4f+c-4c+4.de4.<ab>dc+del4<a.a>d8df+8bf+8a.ab8af+8gf+8e.c-e8eg8>c+c+8<ba8gg8f+e8c+.e.d<a8>f+e8dl8ef+gab4.b4agabl4a.ef+8g+e8a2.<b.a.ef+8g+e8a.a>e8ea8ag8b.a.g2.e.n73b8ab8ag2ab8f+2&f+8e8d2.<de8f+a8>b.a.g2.e.n73b8ab8ag2al8bf+2&f+ed4.>d<af+d4de4f+gf+gf+4gf+4e4.ff+ff+f4f+l4ed2<b.b.>c+d8e.<b.b.>c+d8ed2.&d8d1&d1&d8&d64";
/**
 * @brief :  蜂鸣器注册
 * @return  void
 */
void BuzzerRegister(void)
{
    BuzzerInstance *buzzer_ins      = (BuzzerInstance *)zmalloc(sizeof(BuzzerInstance));
    PWM_Init_Config_s buzzer_config = {
        .htim      = &htim12,
        .channel   = TIM_CHANNEL_2,
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
void BuzzerPlay(const char *sound)
{
    // 如果蜂鸣器未注册，则注册
    if (buzzer == NULL) {
        BuzzerRegister();
    }
    // if (buzzer->busy) return;
    buzzer->sound      = sound;
    buzzer->_next_tune = sound;
    buzzer->busy       = 1;
    osThreadResume(BuzzerHandle); // 恢复线程
}
/**
 * @brief :  蜂鸣器停止唱歌
 * @param none
 * @return  void
 */
void BuzzerStop()
{
    // 如果蜂鸣器未注册，则注册
    if (buzzer == NULL) {
        BuzzerRegister();
    }
    buzzer->sound      = NULL;
    buzzer->_next_tune = NULL;
    buzzer->busy       = 0;
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
    float _note_length,note_delay;
    if(buzzer->_note_length_single != 0){
        _note_length = buzzer->_note_length_single;
        buzzer->_note_length_single = 0;
    }else{
        _note_length = buzzer->_note_length;
    }
    note_delay = 1000 * (4.0f / _note_length) * 60 / (float)buzzer->_tempo;
    note_delay += note_delay / 2.0f * (float)buzzer->dots;
    osDelay((uint32_t)note_delay);
    if(buzzer->_slur){
        buzzer->_slur = 0;
    }else{
        PWMSetDutyRatio(buzzer->buzzer_pwm, 0);
        // osDelay(10);
    }
}
/**
 * @brief :  下一个字符
 * @return  int
 */
inline static int next_char()
{
    const char *next_tune = buzzer->_next_tune;
    while (*next_tune == ' ' || *next_tune == '|') {
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
    uint8_t addNoteParam = 1;
    while(addNoteParam){
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
            case '0' ... '9':
                buzzer->_note_length_single = next_number();
                break;
            default:
                addNoteParam = 0;
                // 0 / No next char here is OK.
                break;
        }
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
                buzzer->_next_tune++;
                if (buzzer->_octave > 0) {
                    buzzer->_octave--;
                }
                break;
            case '>': // Increase octave.
                buzzer->_next_tune++;
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
            case 'N': //不知道有什么用，可能是某种自定义标记，但扒下来的哆啦A梦谱有这东西
            case 'R':   //休止符
                buzzer->note = 0;
                buzzer->_note_length_single = next_number();
                break;
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

                if(*buzzer->_next_tune == '&'){
                    buzzer->_next_tune++;
                    buzzer->_slur = 1;
                }
                    
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
    BuzzerStop();
    PWMSetPeriod(buzzer->buzzer_pwm, 1.0f / (float)Note);
#if (BUZZER_SILENCE == 1)
    PWMSetDutyRatio(buzzer->buzzer_pwm, 0);
#else
    PWMSetDutyRatio(buzzer->buzzer_pwm, 0.9f); // 音量
#endif
    buzzer_one_note_flag = 1;
    buzzer_one_note_time = delay;
    osThreadResume(BuzzerHandle); // 恢复线程
}
/**
 * @brief :  蜂鸣器任务，播放完毕后自动挂起线程，循环播放除外
 * @return  void
 */
__attribute__((noreturn)) void BuzzerTask(void *argument)
{
    UNUSED(argument);
    for (;;) {
        if(buzzer == NULL){
            osThreadSuspend(BuzzerHandle); // 挂起线程
            continue;
        }
        if(buzzer_one_note_flag)
        {
            DWT_Delay(buzzer_one_note_time);
            buzzer_silence();
            buzzer_one_note_flag = 0;
        }
        else
        {
            do {
                string_handle();
                buzzer_silence();
            } while (buzzer->_repeat);
            buzzer->busy = 0;   
        }
            
        osThreadSuspend(BuzzerHandle); // 挂起线程
    }
}
