/**
  ******************************************************************************
  * @file    songs.h
  * @brief   内置歌曲数据声明 (数据与播放逻辑分离).
  ******************************************************************************
  */
#ifndef __SONGS_H
#define __SONGS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "midi.h"

extern const Midi_Song SONG_XiaoXingXing;   /* 小星星 */
extern const Midi_Song SONG_MoLihua;        /* 茉莉花 (前四句) */
extern const Midi_Song SONG_LanHuaCao;      /* 兰花草 (前八句) */

#ifdef __cplusplus
}
#endif

#endif /* __SONGS_H */
