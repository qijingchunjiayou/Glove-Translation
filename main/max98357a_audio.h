#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 MAX98357A 的 I2S 输出；仅由左手固件调用。 */
esp_err_t max98357a_audio_init(void);

/* 播放约 0.5 秒、低音量的 1 kHz 启动测试音。 */
esp_err_t max98357a_play_test_tone(void);

#ifdef __cplusplus
}
#endif
