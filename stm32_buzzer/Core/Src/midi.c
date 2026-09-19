/**
  ******************************************************************************
  * @file    midi.c
  * @brief   极简 MIDI 风格容器读取与播放器实现 (头块校验 + 逐事件读取).
  ******************************************************************************
  */
#include "midi.h"
#include "buzzer.h"
#include <stddef.h>

typedef struct {
  const Midi_Song *song;
  uint32_t         index;      /* 当前事件索引 (读指针) */
  uint32_t         next_tick;  /* 下一事件的触发时刻 */
  int8_t           transpose;  /* 升/降半音数 */
  uint16_t         rate;       /* 速率百分比 */
  uint8_t          releasing;  /* 1 = 当前音已奏完, 处于事件间静音 */
  uint8_t          playing;
} Midi_Player;

static Midi_Player s_player = {0};

/* 十二平均律半音倍率 (×1000): ratio[n] = 2^(n/12) * 1000 */
static const uint16_t s_semitone_x1000[12] = {
  1000U, 1059U, 1122U, 1189U, 1260U, 1335U,
  1414U, 1498U, 1587U, 1682U, 1782U, 1888U
};

/* 升/降调: 把频率按半音数缩放 (先拆成八度 + 0..11 半音, 避免移位越界) */
static uint32_t Midi_TransposeFreq(uint32_t freq_hz, int8_t semitones)
{
  int16_t  s;
  int16_t  oct;
  uint32_t out;

  if (freq_hz == 0U)
  {
    return 0U;                       /* 休止符保持 */
  }
  if (semitones > 24)
  {
    semitones = 24;                  /* 限制在 ±2 个八度内 */
  }
  else if (semitones < -24)
  {
    semitones = -24;
  }

  s = semitones;
  oct = s / 12;
  s = s % 12;
  if (s < 0)
  {
    s += 12;
    oct -= 1;
  }

  out = (freq_hz * s_semitone_x1000[s]) / 1000U;
  out = (oct >= 0) ? (out << oct) : (out >> (-oct));
  return out;
}

/* 变速: rate=100 原速, 越大越快 (时长按 100/rate 缩放) */
static uint32_t Midi_ScaleTime(uint32_t ms, uint16_t rate)
{
  return (rate != 0U) ? (ms * 100U / rate) : ms;
}

/* 校验头块 (类似 MIDI 检查 "MThd") */
static uint8_t Midi_ReadHeader(const Midi_Song *song)
{
  if (song == NULL || song->notes == NULL)
  {
    return 0U;
  }
  if (song->header.magic != MIDI_MAGIC || song->header.version != MIDI_VERSION)
  {
    return 0U;
  }
  if (song->header.note_count == 0U || song->header.bpm == 0U ||
      song->header.ticks_per_beat == 0U)
  {
    return 0U;
  }
  return 1U;
}

/* tick -> ms: 一拍 = 60000/bpm, 一 tick = 一拍 / ticks_per_beat */
static uint32_t Midi_NoteDuration(const Midi_Header *header, const Midi_Note *note)
{
  return (60000UL / header->bpm) * note->time / header->ticks_per_beat;
}

void Midi_PlayEx(const Midi_Song *song, const Midi_Config *config)
{
  if (!Midi_ReadHeader(song))
  {
    return;
  }
  s_player.song = song;
  s_player.index = 0U;
  s_player.next_tick = HAL_GetTick();
  s_player.transpose = (config != NULL) ? config->transpose : 0;
  s_player.rate = (config != NULL && config->rate != 0U) ? config->rate : 100U;
  s_player.releasing = 0U;
  s_player.playing = 1U;
}

void Midi_Play(const Midi_Song *song)
{
  Midi_PlayEx(song, NULL);
}

void Midi_SetTranspose(int8_t semitones)
{
  s_player.transpose = semitones;
}

void Midi_SetRate(uint16_t percent)
{
  if (percent != 0U)
  {
    s_player.rate = percent;
  }
}

void Midi_Task(void)
{
  const Midi_Note *note;

  if (!s_player.playing)
  {
    return;
  }
  if ((int32_t)(HAL_GetTick() - s_player.next_tick) < 0)
  {
    return;
  }

  if (s_player.releasing)
  {
    Buzzer_Stop();                       /* 事件间短暂静音, 避免同音粘连 */
    s_player.releasing = 0U;
    s_player.next_tick += Midi_ScaleTime(s_player.song->header.gap_ms, s_player.rate);
    s_player.index++;
    if (s_player.index >= s_player.song->header.note_count)
    {
      if (s_player.song->header.loop)
      {
        s_player.index = 0U;             /* 循环播放 */
      }
      else
      {
        s_player.playing = 0U;           /* 播放一次后停止 */
      }
    }
    return;
  }

  note = &s_player.song->notes[s_player.index];
  Buzzer_SetTone(Midi_TransposeFreq(note->freq_hz, s_player.transpose));  /* 50% 方波 */
  s_player.next_tick += Midi_ScaleTime(Midi_NoteDuration(&s_player.song->header, note),
                                       s_player.rate);
  s_player.releasing = 1U;
}

void Midi_Stop(void)
{
  s_player.playing = 0U;
  Buzzer_Stop();
}

uint8_t Midi_IsPlaying(void)
{
  return s_player.playing;
}

const char *Midi_Title(void)
{
  return (s_player.song != NULL) ? s_player.song->header.title : "";
}
