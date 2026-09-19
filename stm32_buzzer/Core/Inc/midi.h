/**
  ******************************************************************************
  * @file    midi.h
  * @brief   极简 MIDI 风格乐曲容器与播放器.
  *
  *            MIDI 文件:  [MThd 头块: 格式/轨道数/division] + [MTrk 轨道: 事件序列]
  *            本格式:     [Midi_Header 头块]               + [Midi_Note 事件表]
  *
  *          - Midi_Header.bpm / ticks_per_beat 对应 MIDI 的 tempo 与 division;
  *          - Midi_Note 是 (freq, time) 键值对, time 单位为 tick;
  *          - 播放器把 tick 按时值换算成 ms, 逐条读取播放, 末尾循环.
  *
  *          歌曲数据 (songs.c) 与播放逻辑 (本模块) 分离.
  ******************************************************************************
  */
#ifndef __MIDI_H
#define __MIDI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---- 音名 -> 频率 (Hz, 十二平均律, A4 = 440) ---- */
#define N_G3 196U
#define N_A3 220U
#define N_B3 247U
#define N_C4 262U
#define N_D4 294U
#define N_E4 330U
#define N_F4 349U
#define N_Fs4 370U
#define N_G4 392U
#define N_A4 440U
#define N_B4 494U
#define N_C5 523U
#define N_D5 587U
#define N_E5 659U
#define N_F5 698U
#define N_G5 784U
#define N_A5 880U

/* 头块标识 / 版本 (类似 MIDI 的 "MThd") */
#define MIDI_MAGIC   0x4D494449UL   /* "MIDI" */
#define MIDI_VERSION 1U

/* 音符事件: (freq, time) 键值对 */
typedef struct {
  uint16_t freq_hz;   /* 频率 Hz, 0 = 休止符 */
  uint16_t time;      /* 时值, 单位 tick */
} Midi_Note;

/* 头块: 元数据 */
typedef struct {
  uint32_t magic;           /* 固定为 MIDI_MAGIC, 用于识别/校验 */
  uint16_t version;         /* 格式版本 */
  uint16_t bpm;             /* 速度 (拍/分钟), 对应 MIDI tempo */
  uint16_t ticks_per_beat;  /* 每拍 tick 数, 对应 MIDI division */
  uint16_t note_count;      /* 事件数量 */
  uint16_t gap_ms;          /* 事件间静音时长 (ms) */
  uint8_t  loop;            /* 1 = 循环播放, 0 = 播放一次后停止 */
  char     title[24];       /* 曲名 */
  char     artist[24];      /* 作者 */
} Midi_Header;

/* 一首歌 = 头块 + 指向事件表的指针 */
typedef struct {
  Midi_Header      header;
  const Midi_Note *notes;
} Midi_Song;

/* 便捷构造宏: time 单位为 tick */
#define NOTE(freq, time) { (freq), (time) }
#define REST(time)       { 0U,     (time) }

/* 播放配置: 升调 / 变速 (作用于播放, 不改动歌曲数据) */
typedef struct {
  int8_t   transpose;   /* 升/降半音数: +2=升全音, +12=升八度, 负数降调, 0=原调 */
  uint16_t rate;        /* 播放速率百分比: 100=原速, 200=快一倍, 50=慢一倍 */
} Midi_Config;

/* ---- 播放器 (读取 / 解码) ---- */
void        Midi_Play(const Midi_Song *song);                            /* 用默认配置播放 */
void        Midi_PlayEx(const Midi_Song *song, const Midi_Config *config); /* config 可为 NULL */
void        Midi_Task(void);                   /* 非阻塞推进, 主循环调用 */
void        Midi_Stop(void);
uint8_t     Midi_IsPlaying(void);
const char *Midi_Title(void);                  /* 当前曲名, 无则返回 "" */

/* 运行时调整 (播放中亦可调用) */
void        Midi_SetTranspose(int8_t semitones);   /* 升/降调, 单位半音 */
void        Midi_SetRate(uint16_t percent);        /* 变速, 100 = 原速 */

#ifdef __cplusplus
}
#endif

#endif /* __MIDI_H */
