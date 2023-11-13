# buzzer

用于各种提示音

## 使用范例

### 播放字符串

```c
BuzzerPlay(StartUP_sound);
```
由于buzzer为单实例，故不保存实例，如果未初始化实例，程序会自动初始化。

其中`StartUP_sound`是一个数组，用于存放提示音的频率和持续时间。

### 播放单个音符（阻塞模式）

```c
buzzer_one_note(Do_freq , 0.5);//音符，延时
```

其中音符有以下选择

```c
#define Do_freq Note_Freq[61]
#define Re_freq Note_Freq[63]
#define Mi_freq Note_Freq[65]
#define Fa_freq Note_Freq[66]
#define So_freq Note_Freq[68]
#define La_freq Note_Freq[70]
#define Si_freq Note_Freq[72]
```

对应第四八度

## 可选曲目

==`StartUP_sound`==
==`No_RC_sound`==
==`RoboMaster_You`==
==`RoboMaster_Prepare`==
==`Test`==

## 编曲指南

> 功能：演奏由给定的字符串所表示的音乐 
> 格式：PLAY [命令字符串] 
> 说明：[命令字符串]是一种音乐语言单个字符命令组成，用以表示音符的音高、音长、速度及演奏方式等。 
> 乐谱中的音符CDEFGAB(当1=C时相当于简符1234567)在BASIC中仍然是这7个字母，“#”(或“+”)号与“-”号分别表示乐谱中的升半音(#)和降半音(b)，字符“.”代表延长前面音符的一半时长(用来表示附点音符)，其他字符命今如下： 
> 1、O命令 
> 功能：用来设置要演奏的音符所在的八度，即音阶。 
> 格式：On 
> 说明：命令中n的取值为0-6。比如．使计算机在演奏完音阶3的CD之后，再演奏一个高八度的C，即音阶4中的c调，可以用PLAY “O3CDO4C”来实现。普通中音是指音阶3中的音符。 
> 2、L命令 
> 功能；用来设置要演奏的每个音符的长度(亦称音符的时值)。 
> 格式：Ln 
> 说明：n的取值1-64，L1表示以全音符，L2表示半音符，L4表示四分音符，L8表示八分音符…… 
> 一个音符的时值也可以直接写在该音符后面，而不用L命令。如L8A与A8等效，C4E4D4A4等效于L4CEDA，而L4CDEFC2AB2则表示音符后面不带数字的按L设定的四分音符演奏，后面带数字的G2和B2按数字指定的音值演奏，显然用L命令更简洁方便。 
> 3、P命令 
> 功能：表示休止符的长度 
> 格式：Pn 
> 说明：n的取值1-64，P1表示全休止符．P2表示半休止符，P4表示四分休止符…… 
> 4、T命令 
> 功能；用来设置演奏的节拍速度。 
> 格式：Tn 
> 说明：n为每分钟演奏四分音符的数量，取值32-255，缺省是T120 
> 注意：除了T命令外还可以用以下三个命令来控制演奏的速度： 
> (1)MN(音乐标准)：它使每个音符以L设定的音值的7/8来演奏，使得两两音符之间有短暂的停顿。 
> (2)ML(音乐连奏)；使演奏时两两音符间不停顿，每个音符将演奏足L规定的音长。 
> (3)Ms(音乐断音)：使每个音符更短一些，以L给出的音值的3/4来演奏，以此来延长音符间的停顿。 
> 另外．QB演奏音乐时有两种方式供选择：前台音乐和背景音乐。由MF和MB来确定。当“命令字符串”中出现MF时，表示在前台插放音乐．即QB要等到执行完PLAY语句后才能继续往下执行程序，在播放音乐时不能做其它的事情。而出现MB时，则表示在后台播放音乐，QB可以将最多32个音符或休止符保存在缓冲区，然后继续执行下面程序，而无需等待把这些音符全部演奏完。这样，MB允许QB在演奏的同时做一些其它的事情，如运算、绘图等等。 
>
> 
>
> 为清晰起见，程序中各小节的音乐编码之间以空格分开，这样编码与乐谱之间的对应关系一目了然。
>
> 

```c
/*** From Wikibooks:
 *
 * PLAY "[string expression]"
 *
 * Used to play notes and a score ... The tones are indicated by letters A through G.
 * Accidentals are indicated with a "+" or "#" (for sharp) or "-" (for flat)
 * immediately after the note letter. See this example:
 *
 *   PLAY "C C# C C#"
 *
 * Whitespaces are ignored inside the string expression. There are also codes that
 * set the duration, octave and tempo. They are all case-insensitive. PLAY executes
 * the commands or notes the order in which they appear in the string. Any indicators
 * that change the properties are effective for the notes following that indicator.
 *
 * Ln     Sets the duration (length) of the notes. The variable n does not indicate an actual duration
 *        amount but rather a note type; L1 - whole note, L2 - half note, L4 - quarter note, etc.
 *        (L8, L16, L32, L64, ...). By default, n = 4.
 *        For triplets and quintets, use L3, L6, L12, ... and L5, L10, L20, ... series respectively.
 *        The shorthand notation of length is also provided for a note. For example, "L4 CDE L8 FG L4 AB"
 *        can be shortened to "L4 CDE F8G8 AB". F and G play as eighth notes while others play as quarter notes.
 * On     Sets the current octave. Valid values for n are 0 through 6. An octave begins with C and ends with B.
 *        Remember that C- is equivalent to B.
 * < >    Changes the current octave respectively down or up one level.
 * Nn     Plays a specified note in the seven-octave range. Valid values are from 0 to 84. (0 is a pause.)
 *        Cannot use with sharp and flat. Cannot use with the shorthand notation neither.
 * MN     Stand for Music Normal. Note duration is 7/8ths of the length indicated by Ln. It is the default mode.
 * ML     Stand for Music Legato. Note duration is full length of that indicated by Ln.
 * MS     Stand for Music Staccato. Note duration is 3/4ths of the length indicated by Ln.
 * Pn     Causes a silence (pause) for the length of note indicated (same as Ln).
 * Tn     Sets the number of "L4"s per minute (tempo). Valid values are from 32 to 255. The default value is T120.
 * .      When placed after a note, it causes the duration of the note to be 3/2 of the set duration.
 *        This is how to get "dotted" notes. "L4 C#." would play C sharp as a dotted quarter note.
 *        It can be used for a pause as well.
 *
 * Extensions/variations:
 *
 * MB MF  The MF command causes the tune to play once and then stop. The MB command causes the
 *        tune to repeat when it ends.
 *
 ***/
```

