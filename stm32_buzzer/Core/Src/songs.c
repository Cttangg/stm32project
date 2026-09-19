/**
  ******************************************************************************
  * @file    songs.c
  * @brief   内置歌曲数据 (MIDI 风格: 头块 + (freq,time) 事件表).
  *
  *          加新歌: 定义事件数组 + Midi_Song 常量, 在 songs.h 里 extern 即可.
  *          ticks_per_beat = 8 => 四分音符 = 8, 八分音符 = 4, 二分音符 = 16.
  ******************************************************************************
  */
#include "songs.h"

/* ===================== 《小星星》 ===================== */
static const Midi_Note xingxing_notes[] = {
  /* 1 1 5 5 6 6 5- */
  NOTE(N_C4, 8), NOTE(N_C4, 8), NOTE(N_G4, 8), NOTE(N_G4, 8),
  NOTE(N_A4, 8), NOTE(N_A4, 8), NOTE(N_G4, 16),
  /* 4 4 3 3 2 2 1- */
  NOTE(N_F4, 8), NOTE(N_F4, 8), NOTE(N_E4, 8), NOTE(N_E4, 8),
  NOTE(N_D4, 8), NOTE(N_D4, 8), NOTE(N_C4, 16),
  /* 5 5 4 4 3 3 2- */
  NOTE(N_G4, 8), NOTE(N_G4, 8), NOTE(N_F4, 8), NOTE(N_F4, 8),
  NOTE(N_E4, 8), NOTE(N_E4, 8), NOTE(N_D4, 16),
  /* 5 5 4 4 3 3 2- */
  NOTE(N_G4, 8), NOTE(N_G4, 8), NOTE(N_F4, 8), NOTE(N_F4, 8),
  NOTE(N_E4, 8), NOTE(N_E4, 8), NOTE(N_D4, 16),
  /* 1 1 5 5 6 6 5- */
  NOTE(N_C4, 8), NOTE(N_C4, 8), NOTE(N_G4, 8), NOTE(N_G4, 8),
  NOTE(N_A4, 8), NOTE(N_A4, 8), NOTE(N_G4, 16),
  /* 4 4 3 3 2 2 1- */
  NOTE(N_F4, 8), NOTE(N_F4, 8), NOTE(N_E4, 8), NOTE(N_E4, 8),
  NOTE(N_D4, 8), NOTE(N_D4, 8), NOTE(N_C4, 16),
};

const Midi_Song SONG_XiaoXingXing = {
  .header = {
    .magic          = MIDI_MAGIC,
    .version        = MIDI_VERSION,
    .bpm            = 150U,
    .ticks_per_beat = 8U,
    .note_count     = sizeof(xingxing_notes) / sizeof(xingxing_notes[0]),
    .gap_ms         = 30U,
    .loop           = 0U,
    .title          = "Xiao Xing Xing",
    .artist         = "Trad.",
  },
  .notes = xingxing_notes,
};

/* ===================== 《茉莉花》(江苏民歌) 前四句 =====================
 * 1=C 4/4, 简谱: 6 6 1 2 4 4 2 | 1 1 2 1 -  (反复)
 *              1 1 1 6 1 | 2 2 1 -
 *              6 5 6 1 6 5 | 4 4 5 4 -
 * (1=C, 上加点为高八度: 1=C5, 2=D5, 4=F4, 5=G4, 6=A4)
 */
static const Midi_Note molihua_notes[] = {
  /* 好一朵美丽的茉莉花 */
  NOTE(N_A4, 8), NOTE(N_A4, 4), NOTE(N_C5, 4), NOTE(N_D5, 4),
  NOTE(N_F4, 4), NOTE(N_F4, 4), NOTE(N_D5, 4),
  NOTE(N_C5, 8), NOTE(N_C5, 4), NOTE(N_D5, 4), NOTE(N_C5, 16),
  /* 好一朵美丽的茉莉花 (反复) */
  NOTE(N_A4, 8), NOTE(N_A4, 4), NOTE(N_C5, 4), NOTE(N_D5, 4),
  NOTE(N_F4, 4), NOTE(N_F4, 4), NOTE(N_D5, 4),
  NOTE(N_C5, 8), NOTE(N_C5, 4), NOTE(N_D5, 4), NOTE(N_C5, 16),
  /* 芬芳美丽满枝桠 */
  NOTE(N_C5, 8), NOTE(N_C5, 8), NOTE(N_C5, 8), NOTE(N_A4, 4), NOTE(N_C5, 4),
  NOTE(N_D5, 8), NOTE(N_D5, 8), NOTE(N_C5, 16),
  /* 又香又白人人夸 */
  NOTE(N_A4, 8), NOTE(N_G4, 4), NOTE(N_A4, 4), NOTE(N_C5, 8), NOTE(N_A4, 4), NOTE(N_G4, 4),
  NOTE(N_F4, 8), NOTE(N_F4, 4), NOTE(N_G4, 4), NOTE(N_F4, 16),
};

const Midi_Song SONG_MoLihua = {
  .header = {
    .magic          = MIDI_MAGIC,
    .version        = MIDI_VERSION,
    .bpm            = 80U,
    .ticks_per_beat = 8U,
    .note_count     = sizeof(molihua_notes) / sizeof(molihua_notes[0]),
    .gap_ms         = 20U,
    .loop           = 0U,
    .title          = "Mo Li Hua",
    .artist         = "Jiangsu Folk",
  },
  .notes = molihua_notes,
};

/* ===================== 《兰花草》前八句 (第一段) =====================
 * 1=bB 2/4 (此处按 1=C 记谱, 相对音高不变)
 *   6 3 3 3 | 3. 2 | 1. 2 1 7 | 6 - | 6 6 6 6 | 6. 5 | 3 5 5 #4 | 3 - | 3 6 6 5 |
 *   3. 2 | 1. 2 1 7 | 6 3 | 3 1 1 7 | 6. 3 | 2. 1 7 5 | 6 -
 * 下加点为低八度: 6̣=A3, 7̣=B3, 5̣=G3; #4=F#4
 */
static const Midi_Note lanhuacao_notes[] = {
  /* 我从山中来 */
  NOTE(N_A3, 4), NOTE(N_E4, 4), NOTE(N_E4, 4), NOTE(N_E4, 4),
  NOTE(N_E4, 12), NOTE(N_D4, 4),
  /* 带着兰花草 */
  NOTE(N_C4, 4), NOTE(N_D4, 4), NOTE(N_C4, 4), NOTE(N_B3, 4),
  NOTE(N_A3, 16),
  /* 种在小园中 */
  NOTE(N_A4, 4), NOTE(N_A4, 4), NOTE(N_A4, 4), NOTE(N_A4, 4),
  NOTE(N_A4, 12), NOTE(N_G4, 4),
  /* 希望花开早 */
  NOTE(N_E4, 4), NOTE(N_G4, 4), NOTE(N_G4, 4), NOTE(N_Fs4, 4),
  NOTE(N_E4, 16),
  /* 一日看三回 */
  NOTE(N_E4, 4), NOTE(N_A4, 4), NOTE(N_A4, 4), NOTE(N_G4, 4),
  NOTE(N_E4, 12), NOTE(N_D4, 4),
  /* 看得花时过 */
  NOTE(N_C4, 4), NOTE(N_D4, 4), NOTE(N_C4, 4), NOTE(N_B3, 4),
  NOTE(N_A3, 8), NOTE(N_E4, 8),
  /* 兰花却依然 */
  NOTE(N_E4, 4), NOTE(N_C4, 4), NOTE(N_C4, 4), NOTE(N_B3, 4),
  NOTE(N_A3, 12), NOTE(N_E4, 4),
  /* 苞也无一个 */
  NOTE(N_D4, 4), NOTE(N_C4, 4), NOTE(N_B3, 4), NOTE(N_G3, 4),
  NOTE(N_A3, 16),
};

const Midi_Song SONG_LanHuaCao = {
  .header = {
    .magic          = MIDI_MAGIC,
    .version        = MIDI_VERSION,
    .bpm            = 100U,
    .ticks_per_beat = 8U,
    .note_count     = sizeof(lanhuacao_notes) / sizeof(lanhuacao_notes[0]),
    .gap_ms         = 20U,
    .loop           = 0U,
    .title          = "Lan Hua Cao",
    .artist         = "Hu Shi / Folk",
  },
  .notes = lanhuacao_notes,
};
