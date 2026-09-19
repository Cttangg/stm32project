/**
 ******************************************************************************
 * @file    app.c
 * @brief   应用层 (L4) 实现 — 状态机 / 标定 / 开环运动 / 调试打印
 *
 * 轴定义 (与硬件一致):
 *   X 轴 = 电机2 = Pan  = TIM13 + I2C2 编码器 (水平)
 *   Y 轴 = 电机1 = Tilt = TIM14 + I2C1 编码器 (竖直)
 *
 * 模式:
 *   IDLE   空闲; 按键触发
 *   CALIB  标定: 电机解锁, 手动摆位, 每 1s 采样当前角度覆写到标定数组
 *   RESET  开环复位到原点 (使用 CALIB 标定坐标, 角度闭环)
 *   BORDER 开环沿屏幕边线顺时针一周 (使用 CALIB 标定坐标, 角度闭环)
 *
 * 标定点顺序 (CALIB 单击依次 0→1→2→3→4, 点 4 停留后自动结束):
 *   0 中心(原点) | 1 左上 | 2 右上 | 3 右下 | 4 左下
 *   屏幕坐标 (原点=屏中心, x 右, y 上, 单位 cm):
 *   0(0,0) 1(-25,25) 2(25,25) 3(25,-25) 4(-25,-25)
 *
 * 标定算法: 由 5 点求仿射运动学 [pan;tilt] = k·[x;y] + b
 *   b  = 中心点角度;  k00=(右均值-左均值)/50; k01=(上均值-下均值)/50
 *   (角度先相对中心解卷绕, 避免 0/360 接缝)
 ******************************************************************************
 */
#include "app.h"
#include "main.h"
#include "mt6701.h"
#include "tmc2209.h"
#include "motor_axis.h"
#include "motion.h"
#include "motion_arbiter.h"
#include "uart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ========================================================================= */
/*  配置                                                                       */
/* ========================================================================= */
#define APP_CTRL_MS           5u        /* 控制周期 5ms */
#define APP_PRINT_MS          200u      /* 角度实时打印周期 5Hz */
#define APP_CALIB_PERIOD_MS   1000u     /* 标定自动采样周期 1s */
#define APP_CALIB_LAST_MS     3000u     /* 标定点4停留窗口, 超时自动结束回 IDLE (可单击提前结束) */
#define APP_KEY_DEBOUNCE_MS   20u       /* 按键消抖 */
#define APP_KEY_STARTUP_MS    300u      /* 上电按键屏蔽窗口 (防悬空/上电抖动误触发) */

/* 电机1 MS2 实际绑定引脚:
   CubeMX 中 PD11 = MS2_1_BK, PD12 = MS2_1.
   按实际接线使用 PD11; PD12 已改为输入且与 PD11 联通 (不再驱动).
   改回 PD12 只需换这里, CubeMX 重新生成不会覆盖. */
#define APP_MS2_1_Port   MS2_1_BK_GPIO_Port
#define APP_MS2_1_Pin    MS2_1_BK_Pin

/* ── 调试命令目标电机 (当前 = 电机2 / X 轴 / Pan) ──
   en1/ms1/ms2/step/dir/pins/mode 全部操作此电机.
   切换到电机1: 把下面 5 组引脚宏换成 *_1 版本, APP_DBG_MOTOR 换成 &s_motor_tilt,
   APP_DBG_NAME 换成 "M1(Y/Tilt)" 即可 (s_motor_* 在下方定义, 宏在使用处展开). */
#define APP_DBG_EN_Port    EN_2_GPIO_Port
#define APP_DBG_EN_Pin     EN_2_Pin
#define APP_DBG_MS1_Port   MS1_2_GPIO_Port
#define APP_DBG_MS1_Pin    MS1_2_Pin
#define APP_DBG_MS2_Port   MS2_2_GPIO_Port
#define APP_DBG_MS2_Pin    MS2_2_Pin
#define APP_DBG_STEP_Port  STEP_2_GPIO_Port
#define APP_DBG_STEP_Pin   STEP_2_Pin
#define APP_DBG_DIR_Port   DIR_2_GPIO_Port
#define APP_DBG_DIR_Pin    DIR_2_Pin
#define APP_DBG_MOTOR      (&s_motor_pan)
#define APP_DBG_NAME       "M2(X/Pan)"

/* 按键有效电平: GPIO_PIN_RESET = 低电平按下 (上拉+按键接地); 高有效改 GPIO_PIN_SET */
#define APP_KEY_PRESSED_LEVEL GPIO_PIN_RESET

/* 应用层是否重新配置按键 GPIO 上下拉.
   现在 CubeMX (.ioc) 已把 6 个按键配为 GPIO_PULLUP, 由 MX_GPIO_Init 处理, 故默认关闭.
   若某块板子在 CubeMX 里改回 NOPULL 且无外部上拉, 置 1 由本驱动补内部上拉. */
#define APP_KEY_CFG_PULL      0

#define APP_MOVE_SPEED        20.0f     /* (XY 路径用) 速度 cm/s */
#define APP_BORDER_HALF       25.0f     /* 屏幕边线半边长 cm (0.5m 正方形, 仅用于标定拟合) */
#define APP_ANG_RATE          20.0f     /* 角度空间设定点速率 deg/s (RESET/BORDER) */

/* 空闲保持策略由 motion.h 的 MOTION_HOLD_AT_IDLE 控制:
   1 = TMC 通电静态锁位 (默认, 光斑不跑); 0 = 空闲释放力矩 (完全静音). */

/* ========================================================================= */
/*  HAL 句柄 (定义于 main.c)                                                   */
/* ========================================================================= */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* 读引脚实测电平 (调试命令/标定日志用; 定义在调试命令区) */
static uint8_t pin_lvl(GPIO_TypeDef *port, uint16_t pin);

/* ========================================================================= */
/*  模式                                                                       */
/* ========================================================================= */
typedef enum {
    MODE_IDLE = 0,
    MODE_CALIB,
    MODE_RESET,
    MODE_BORDER,
} AppMode;

static const char *const CAL_PT_NAME[5] = {
    "CENTER", "TOP-LEFT", "TOP-RIGHT", "BOTTOM-RIGHT", "BOTTOM-LEFT"
};

/* ========================================================================= */
/*  驱动实例                                                                   */
/* ========================================================================= */
static MT6701_HandleTypeDef  s_enc_tilt;
static MT6701_HandleTypeDef  s_enc_pan;
static TMC2209_HandleTypeDef s_motor_tilt;
static TMC2209_HandleTypeDef s_motor_pan;

/* 全局: stm32f4xx_it.c 中断分发需要 (符号名固定为 stepper_tilt/stepper_pan) */
MotorStepper stepper_tilt;
MotorStepper stepper_pan;

static MotorAxis s_tilt_axis;    /* Y 轴 (电机1) */
static MotorAxis s_pan_axis;     /* X 轴 (电机2) */
static Motion    s_motion;
static MotionArbiter s_arb;
static UART_Device   s_uart_dbg;
static UART_Device   s_uart_cam;

/* ========================================================================= */
/*  标定 / 状态                                                                */
/* ========================================================================= */
static AppMode s_mode = MODE_IDLE;

static float   s_cal_pan[5];     /* 标定: 各点 X(pan) 角度 */
static float   s_cal_tilt[5];    /* 标定: 各点 Y(tilt) 角度 */
static uint8_t s_cal_valid = 0;  /* 是否完成过标定 */
static uint8_t s_calib_idx = 0;
static uint8_t s_calib_hold = 0; /* 点4停留计数 (每 1s 一次, 满窗口自动结束) */

static uint32_t s_last_ctrl;
static uint32_t s_print_tick;
static uint32_t s_calib_tick;
static uint32_t s_boot_tick;

static uint8_t s_enc_pan_ok;
static uint8_t s_enc_tilt_ok;

/* ========================================================================= */
/*  按键                                                                       */
/* ========================================================================= */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       stable;     /* 消抖后电平 (1 = 按下) */
    uint8_t       last_raw;
    uint32_t      t_change;
} AppKey;

static AppKey s_key_calib, s_key_reset, s_key_border, s_key_pause, s_key_track, s_key_a4;

static void app_key_bind(AppKey *k, GPIO_TypeDef *port, uint16_t pin) {
#if APP_KEY_CFG_PULL
    GPIO_InitTypeDef g = {0};

    /* 兜底: CubeMX 若为 NOPULL 输入, 这里补内部上拉 (按键接地) 防悬空误读 */
    g.Pin   = pin;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &g);
#endif

    k->port = port; k->pin = pin;
    k->stable = 0;
    k->last_raw = (HAL_GPIO_ReadPin(port, pin) == APP_KEY_PRESSED_LEVEL) ? 1U : 0U;
    k->t_change = HAL_GetTick();
}

/* 返回 1 = 本周期检测到按下沿 (已消抖) */
static uint8_t app_key_edge(AppKey *k, uint32_t now) {
    uint8_t raw = (HAL_GPIO_ReadPin(k->port, k->pin) == APP_KEY_PRESSED_LEVEL) ? 1U : 0U;
    if (raw != k->last_raw) { k->last_raw = raw; k->t_change = now; }
    if ((uint32_t)(now - k->t_change) >= APP_KEY_DEBOUNCE_MS && raw != k->stable) {
        k->stable = raw;
        if (raw) return 1U;
    }
    return 0U;
}

/* ========================================================================= */
/*  串口打印                                                                   */
/* ========================================================================= */
static int iabs(int v) { return (v < 0) ? -v : v; }

/* 角度 → 百分度整数 (四舍五入) */
static int to_c100(float deg) {
    float v = deg * 100.0f;
    return (int)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

static void app_logf(const char *fmt, ...) {
    char buf[128];
    uint16_t w;
    int n;
    va_list ap;
    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
    UART_Send(&s_uart_dbg, (uint8_t *)buf, (uint16_t)n, &w);
}

static void app_print(uint32_t now) {
    char buf[128];
    char extra[32];
    const char *st;
    const char *pause;
    uint16_t w;
    int xi, yi, n;

    if ((uint32_t)(now - s_print_tick) < APP_PRINT_MS) return;
    s_print_tick = now;

    xi = to_c100(MT6701_ReadDegrees(&s_enc_pan));    /* X 轴 (电机2/Pan) */
    yi = to_c100(MT6701_ReadDegrees(&s_enc_tilt));   /* Y 轴 (电机1/Tilt) */

    extra[0] = '\0';
    switch (s_mode) {
    case MODE_CALIB:
        st = "CALIB";
        snprintf(extra, sizeof(extra), " %u/4 %s", s_calib_idx, CAL_PT_NAME[s_calib_idx]);
        break;
    case MODE_RESET:  st = "RESET";  break;
    case MODE_BORDER: st = "BORDER"; break;
    case MODE_IDLE:
    default:          st = "IDLE";   break;
    }
    pause = MotionArbiter_IsPaused(&s_arb) ? " PAUSE" : "";

    n = snprintf(buf, sizeof(buf), "[%s%s]%s X=%d.%02d Y=%d.%02d\r\n",
                 st, extra, pause, xi / 100, iabs(xi % 100), yi / 100, iabs(yi % 100));
    if (n > 0) UART_Send(&s_uart_dbg, (uint8_t *)buf, (uint16_t)n, &w);
}

/* ========================================================================= */
/*  标定                                                                       */
/* ========================================================================= */
static void calib_sample(void) {
    s_cal_pan[s_calib_idx]  = MT6701_ReadDegrees(&s_enc_pan);
    s_cal_tilt[s_calib_idx] = MT6701_ReadDegrees(&s_enc_tilt);
}

/* 相对参考角解卷绕到同圈 (避免 0/360 接缝) */
static float unwrap_rel(float a, float ref) {
    float d = a - ref;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return ref + d;
}

/* 5 点求仿射运动学 k/b */
static void calib_compute(void) {
    float pc = s_cal_pan[0], tc = s_cal_tilt[0];
    float p1 = unwrap_rel(s_cal_pan[1], pc), p2 = unwrap_rel(s_cal_pan[2], pc);
    float p3 = unwrap_rel(s_cal_pan[3], pc), p4 = unwrap_rel(s_cal_pan[4], pc);
    float t1 = unwrap_rel(s_cal_tilt[1], tc), t2 = unwrap_rel(s_cal_tilt[2], tc);
    float t3 = unwrap_rel(s_cal_tilt[3], tc), t4 = unwrap_rel(s_cal_tilt[4], tc);

    float pr = (p2 + p3) * 0.5f, pl = (p1 + p4) * 0.5f;   /* 右 / 左 均值 */
    float pt = (p1 + p2) * 0.5f, pb = (p3 + p4) * 0.5f;   /* 上 / 下 均值 */
    float tr = (t2 + t3) * 0.5f, tl = (t1 + t4) * 0.5f;
    float tt = (t1 + t2) * 0.5f, tb = (t3 + t4) * 0.5f;

    float denom = 2.0f * APP_BORDER_HALF;

    s_motion.calib.k[0][0] = (pr - pl) / denom;   /* d(pan)/dx  */
    s_motion.calib.k[0][1] = (pt - pb) / denom;   /* d(pan)/dy  */
    s_motion.calib.k[1][0] = (tr - tl) / denom;   /* d(tilt)/dx */
    s_motion.calib.k[1][1] = (tt - tb) / denom;   /* d(tilt)/dy */
    s_motion.calib.b[0] = pc;
    s_motion.calib.b[1] = tc;
}

static void calib_begin(void) {
    MotionArbiter_CancelAll(&s_arb);
    MotionArbiter_Pause(&s_arb, 0);
    Motion_Release(&s_motion);            /* 完全释放两轴 (EN 拉高), 手动摆位 */

    s_calib_idx  = 0;
    s_calib_hold = 0;
    s_calib_tick = HAL_GetTick();
    calib_sample();
    s_mode = MODE_CALIB;
    app_logf(">> CALIB start: pt 0/4 CENTER (motors unlocked)\r\n");
    app_logf("   EN1(PD8)=%u EN2(PE15)=%u (1=released); press CALIB for next pt\r\n",
             pin_lvl(EN_1_GPIO_Port, EN_1_Pin), pin_lvl(EN_2_GPIO_Port, EN_2_Pin));
}

/* 结束标定: 计算仿射 + 重新锁定 + 回 IDLE */
static void calib_finish(void) {
    calib_sample();                       /* 最后一点再采一次, 取最终位置 */

    calib_compute();
    s_cal_valid = 1;
    s_mode = MODE_IDLE;
    Motion_Enable(&s_motion, 1);          /* 重新锁定 */
    /* 打印 k (deg/cm ×10000 定点) 与 b (角度 ×100) */
    app_logf(">> CALIB done. k(x1e4)=%d,%d,%d,%d b(x100)=%d,%d\r\n",
             (int)(s_motion.calib.k[0][0] * 10000.0f), (int)(s_motion.calib.k[0][1] * 10000.0f),
             (int)(s_motion.calib.k[1][0] * 10000.0f), (int)(s_motion.calib.k[1][1] * 10000.0f),
             to_c100(s_motion.calib.b[0]), to_c100(s_motion.calib.b[1]));
}

static void calib_next(void) {
    if (s_calib_idx < 4) {
        calib_sample();                   /* 按下瞬间再采一次, 取最终位置 */
        s_calib_idx++;
        s_calib_hold = 0;
        s_calib_tick = HAL_GetTick();
        app_logf(">> CALIB pt %u/4 %s\r\n", (unsigned)s_calib_idx, CAL_PT_NAME[s_calib_idx]);
    } else {
        /* 点 4 已记录: 单击提前结束 (否则停留 APP_CALIB_LAST_MS 后自动结束) */
        calib_finish();
    }
}

/* ========================================================================= */
/*  开环运动 (基于标定坐标, 角度闭环)                                          */
/* ========================================================================= */
/* 角度空间: 目标 = CALIB 第0点(中心)预设轴角 → 误差 → 角度 PID 闭环 */
static void start_reset(void) {
    float pan  = s_cal_pan[0];
    float tilt = s_cal_tilt[0];
    int bp = to_c100(pan), bt = to_c100(tilt);

    MotionArbiter_CancelAll(&s_arb);
    MotionArbiter_Pause(&s_arb, 0);
    MotionArbiter_GotoAngle(&s_arb, MOTION_SRC_RESET, pan, tilt, APP_ANG_RATE, 1);
    s_mode = MODE_RESET;
    app_logf(">> RESET to origin%s tgt pan=%d.%02d tilt=%d.%02d\r\n",
             s_cal_valid ? "" : " (WARN: uncalibrated)",
             bp / 100, iabs(bp % 100), bt / 100, iabs(bt % 100));
}

/* 角度空间路径顶点 (派发前需保持有效 → 文件静态) */
static Motion_PointAngle s_ang_border[4];

/* 角度空间: 顶点 = CALIB 第1..4点(左上→右上→右下→左下), 顺时针一周 */
static void start_border(void) {
    uint8_t i;
    for (i = 0; i < 4; i++) {
        s_ang_border[i].pan  = s_cal_pan[i + 1];
        s_ang_border[i].tilt = s_cal_tilt[i + 1];
    }
    MotionArbiter_CancelAll(&s_arb);
    MotionArbiter_Pause(&s_arb, 0);
    MotionArbiter_FollowAnglePath(&s_arb, MOTION_SRC_BORDER,
                                  s_ang_border, 4, 1, 1, APP_ANG_RATE, 1);
    s_mode = MODE_BORDER;
    app_logf(">> BORDER clockwise%s\r\n", s_cal_valid ? "" : " (WARN: uncalibrated)");
}

/* ========================================================================= */
/*  按键处理                                                                   */
/* ========================================================================= */
static void app_keys(uint32_t now) {
    /* 上电屏蔽窗口: 期间仅消抖跟踪, 不响应任何按下沿 (确保上电停在 IDLE) */
    if ((uint32_t)(now - s_boot_tick) < APP_KEY_STARTUP_MS) {
        (void)app_key_edge(&s_key_calib,  now);
        (void)app_key_edge(&s_key_reset,  now);
        (void)app_key_edge(&s_key_border, now);
        (void)app_key_edge(&s_key_pause,  now);
        (void)app_key_edge(&s_key_track,  now);
        (void)app_key_edge(&s_key_a4,     now);
        return;
    }

    if (app_key_edge(&s_key_calib, now)) {
        if (s_mode == MODE_CALIB) calib_next();
        else                      calib_begin();
        return;
    }
    if (s_mode == MODE_CALIB) return;     /* 标定中忽略其它按键 */

    if (app_key_edge(&s_key_pause, now)) {
        MotionArbiter_TogglePause(&s_arb);
        app_logf(">> PAUSE %s\r\n", MotionArbiter_IsPaused(&s_arb) ? "on" : "off");
    }
    if (app_key_edge(&s_key_reset, now))  start_reset();
    if (app_key_edge(&s_key_border, now)) start_border();
    /* s_key_track / s_key_a4: 预留 (追踪属绿系统; A4 需视觉角点) */
}

/* ========================================================================= */
/*  周期运行                                                                   */
/* ========================================================================= */
static void app_run(float dt, uint32_t now) {
    if (s_mode == MODE_CALIB) {
        /* 硬保证: 标定期间两轴必须释放 (无条件拉高 EN, 含调试命令绕过的情况) */
        MotorAxis_Enable(&s_pan_axis, 0);
        MotorAxis_Enable(&s_tilt_axis, 0);

        if ((uint32_t)(now - s_calib_tick) >= APP_CALIB_PERIOD_MS) {
            s_calib_tick = now;
            calib_sample();
            app_logf(">> CALIB rec pt%u X=%d.%02d Y=%d.%02d\r\n",
                     (unsigned)s_calib_idx,
                     to_c100(s_cal_pan[s_calib_idx]) / 100, iabs(to_c100(s_cal_pan[s_calib_idx]) % 100),
                     to_c100(s_cal_tilt[s_calib_idx]) / 100, iabs(to_c100(s_cal_tilt[s_calib_idx]) % 100));
            /* 点 4 停留窗口 (APP_CALIB_LAST_MS) 满 → 自动回 IDLE */
            if (s_calib_idx >= 4 &&
                ++s_calib_hold >= (APP_CALIB_LAST_MS / APP_CALIB_PERIOD_MS)) {
                calib_finish();
                return;
            }
        }
        return;
    }

    MotionArbiter_Task(&s_arb, dt);

    if ((s_mode == MODE_RESET || s_mode == MODE_BORDER) && MotionArbiter_IsIdle(&s_arb)) {
        app_logf(">> %s done\r\n", (s_mode == MODE_RESET) ? "RESET" : "BORDER");
        s_mode = MODE_IDLE;
    }
}

/* ========================================================================= */
/*  调试命令 (USART1)                                                          */
/*                                                                            */
/*  命令 (行尾 \r 或 \n 结束, 大小写不敏感), 目标电机见 APP_DBG_* 绑定块:       */
/*    en1 0|1        目标电机 EN   (0=使能 低, 1=禁止 高)                      */
/*    ms1 0|1        目标电机 MS1                                              */
/*    ms2 0|1        目标电机 MS2                                              */
/*    step 0|1       目标电机 STEP 静态电平 (手动点动测试, 非脉冲)              */
/*    dir 0|1        目标电机 DIR                                              */
/*    pins           打印目标电机全部引脚实测电平                              */
/*    status         打印模式/两轴 EN 电平/PID 状态/步率/步数 (排障)            */
/*    mode [n]       显示/设置细分 (0=1/8 1=1/16 2=1/32 3=1/64)                */
/*    stop           立即制动 (等价暂停键)                                     */
/*    release        释放两轴力矩                                              */
/*    help           帮助                                                     */
/* ========================================================================= */
static char s_cmd_buf[48];
static uint8_t s_cmd_len;

static uint8_t arg_bit(const char *s) { return (s && s[0] == '1') ? 1U : 0U; }

static uint8_t pin_lvl(GPIO_TypeDef *port, uint16_t pin) {
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static void cmd_pins_report(void) {
    app_logf("%s pins: EN=%u MS1=%u MS2=%u STEP=%u DIR=%u\r\n", APP_DBG_NAME,
             pin_lvl(APP_DBG_EN_Port,   APP_DBG_EN_Pin),
             pin_lvl(APP_DBG_MS1_Port,  APP_DBG_MS1_Pin),
             pin_lvl(APP_DBG_MS2_Port,  APP_DBG_MS2_Pin),
             pin_lvl(APP_DBG_STEP_Port, APP_DBG_STEP_Pin),
             pin_lvl(APP_DBG_DIR_Port,  APP_DBG_DIR_Pin));
}

static void cmd_help(void) {
    app_logf("cmds: en1 <0|1> ms1 <0|1> ms2 <0|1> step <0|1> dir <0|1> |"
             " pins | status | mode [n] | stop | release | help\r\n");
    app_logf("  en1: 0=enable(L) 1=disable(H); mode: 0=1/8 1=1/16 2=1/32 3=1/64\r\n");
    app_logf("  target: %s\r\n", APP_DBG_NAME);
}

/* 全量状态: 模式 / 运动层 / 仲裁 / 两轴 EN + PID + 角度/目标/误差 + 步率步数 */
static void cmd_status(void) {
    const char *mode = (s_mode == MODE_CALIB)  ? "CALIB"  :
                       (s_mode == MODE_RESET)  ? "RESET"  :
                       (s_mode == MODE_BORDER) ? "BORDER" : "IDLE";
    int x1 = to_c100(s_tilt_axis.current_deg), t1 = to_c100(s_tilt_axis.target_deg);
    int e1 = to_c100(MotorAxis_GetErrorDeg(&s_tilt_axis));
    int x2 = to_c100(s_pan_axis.current_deg),  t2 = to_c100(s_pan_axis.target_deg);
    int e2 = to_c100(MotorAxis_GetErrorDeg(&s_pan_axis));

    app_logf("mode=%s motion{en=%u state=%u axes=%u} arb{win=%d paused=%u}\r\n",
             mode, (unsigned)s_motion.enabled, (unsigned)s_motion.state,
             (unsigned)s_motion.axes_active, (int)MotionArbiter_GetWinner(&s_arb),
             (unsigned)MotionArbiter_IsPaused(&s_arb));
    app_logf("M1 Y/Tilt: EN=%u axis_en=%u pid_en=%u pid_st=%u rate=%d cnt=%lu\r\n",
             pin_lvl(EN_1_GPIO_Port, EN_1_Pin),
             (unsigned)s_tilt_axis.enabled, (unsigned)s_tilt_axis.pid.enabled,
             (unsigned)s_tilt_axis.pid.state, (int)s_tilt_axis.pid.vel_ref,
             (unsigned long)stepper_tilt.step_count);
    app_logf("   deg=%d.%02d tgt=%d.%02d err=%d.%02d\r\n",
             x1 / 100, iabs(x1 % 100), t1 / 100, iabs(t1 % 100), e1 / 100, iabs(e1 % 100));
    app_logf("M2 X/Pan : EN=%u axis_en=%u pid_en=%u pid_st=%u rate=%d cnt=%lu\r\n",
             pin_lvl(EN_2_GPIO_Port, EN_2_Pin),
             (unsigned)s_pan_axis.enabled, (unsigned)s_pan_axis.pid.enabled,
             (unsigned)s_pan_axis.pid.state, (int)s_pan_axis.pid.vel_ref,
             (unsigned long)stepper_pan.step_count);
    app_logf("   deg=%d.%02d tgt=%d.%02d err=%d.%02d\r\n",
             x2 / 100, iabs(x2 % 100), t2 / 100, iabs(t2 % 100), e2 / 100, iabs(e2 % 100));
}

static void app_cmd_exec(char *line) {
    char *arg;
    char *sp;

    /* 转小写 */
    for (sp = line; *sp; sp++)
        if (*sp >= 'A' && *sp <= 'Z') *sp = (char)(*sp - 'A' + 'a');

    arg = strchr(line, ' ');
    if (arg) { *arg++ = '\0'; while (*arg == ' ') arg++; }

    if (!strcmp(line, "help")) {
        cmd_help();
    } else if (!strcmp(line, "en1")) {
        if (arg) {
            HAL_GPIO_WritePin(APP_DBG_EN_Port, APP_DBG_EN_Pin, arg_bit(arg) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            app_logf(">> %s EN=%u (0=enable 1=disable)\r\n", APP_DBG_NAME, arg_bit(arg));
        } else {
            app_logf("%s EN=%u\r\n", APP_DBG_NAME, pin_lvl(APP_DBG_EN_Port, APP_DBG_EN_Pin));
        }
    } else if (!strcmp(line, "ms1")) {
        if (arg) {
            HAL_GPIO_WritePin(APP_DBG_MS1_Port, APP_DBG_MS1_Pin, arg_bit(arg) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            app_logf(">> %s MS1=%u\r\n", APP_DBG_NAME, arg_bit(arg));
        } else {
            app_logf("%s MS1=%u\r\n", APP_DBG_NAME, pin_lvl(APP_DBG_MS1_Port, APP_DBG_MS1_Pin));
        }
    } else if (!strcmp(line, "ms2")) {
        if (arg) {
            HAL_GPIO_WritePin(APP_DBG_MS2_Port, APP_DBG_MS2_Pin, arg_bit(arg) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            app_logf(">> %s MS2=%u\r\n", APP_DBG_NAME, arg_bit(arg));
        } else {
            app_logf("%s MS2=%u\r\n", APP_DBG_NAME, pin_lvl(APP_DBG_MS2_Port, APP_DBG_MS2_Pin));
        }
    } else if (!strcmp(line, "step")) {
        if (arg) {
            HAL_GPIO_WritePin(APP_DBG_STEP_Port, APP_DBG_STEP_Pin, arg_bit(arg) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            app_logf(">> %s STEP=%u\r\n", APP_DBG_NAME, arg_bit(arg));
        } else {
            app_logf("%s STEP=%u\r\n", APP_DBG_NAME, pin_lvl(APP_DBG_STEP_Port, APP_DBG_STEP_Pin));
        }
    } else if (!strcmp(line, "dir")) {
        if (arg) {
            HAL_GPIO_WritePin(APP_DBG_DIR_Port, APP_DBG_DIR_Pin, arg_bit(arg) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            app_logf(">> %s DIR=%u\r\n", APP_DBG_NAME, arg_bit(arg));
        } else {
            app_logf("%s DIR=%u\r\n", APP_DBG_NAME, pin_lvl(APP_DBG_DIR_Port, APP_DBG_DIR_Pin));
        }
    } else if (!strcmp(line, "pins")) {
        cmd_pins_report();
    } else if (!strcmp(line, "status")) {
        cmd_status();
    } else if (!strcmp(line, "mode")) {
        if (arg) {
            uint8_t n = (uint8_t)(arg[0] - '0');
            if (n <= 3) {
                TMC2209_SetMicrostep(APP_DBG_MOTOR, (TMC2209_Microstep)n);
                app_logf(">> microstep set, ");
            } else {
                app_logf(">> bad mode (0..3), ");
            }
        }
        app_logf("%s MS1=%u MS2=%u\r\n", APP_DBG_NAME,
                 pin_lvl(APP_DBG_MS1_Port, APP_DBG_MS1_Pin),
                 pin_lvl(APP_DBG_MS2_Port, APP_DBG_MS2_Pin));
    } else if (!strcmp(line, "stop")) {
        MotionArbiter_Pause(&s_arb, 1);
        app_logf(">> STOP (paused)\r\n");
    } else if (!strcmp(line, "release")) {
        Motion_Release(&s_motion);
        app_logf(">> axes released\r\n");
    } else if (line[0]) {
        app_logf("?? unknown: %s (try 'help')\r\n", line);
    }
}

/* 从调试口环形缓冲取字节组行 (在 UART_Task 之前调用, 避免被帧解析吃掉) */
static void app_cmd_poll(void) {
    uint8_t b;
    while (UART_Read(&s_uart_dbg, &b, 1) == 1) {
        if (b == '\r' || b == '\n') {
            if (s_cmd_len > 0) {
                s_cmd_buf[s_cmd_len] = '\0';
                app_cmd_exec(s_cmd_buf);
                s_cmd_len = 0;
            }
        } else if (b == 8 || b == 127) {          /* 退格 */
            if (s_cmd_len > 0) s_cmd_len--;
        } else if (s_cmd_len < sizeof(s_cmd_buf) - 1) {
            s_cmd_buf[s_cmd_len++] = (char)b;
        }
    }
}

/* ========================================================================= */
/*  初始化 / 主任务                                                            */
/* ========================================================================= */
void APP_Init(void) {
    /* ── 编码器 (Y/I2C1, X/I2C2) ── */
    s_enc_tilt_ok = (MT6701_Init(&s_enc_tilt, &hi2c1) == 0) ? 1U : 0U;
    s_enc_pan_ok  = (MT6701_Init(&s_enc_pan,  &hi2c2) == 0) ? 1U : 0U;

    /* ── TMC2209 (电机1=Y/Tilt, 电机2=X/Pan) ── */
    TMC2209_Init(&s_motor_tilt, &(TMC2209_PinConfig){
        .en   = {EN_1_GPIO_Port,   EN_1_Pin},
        .step = {STEP_1_GPIO_Port, STEP_1_Pin},
        .dir  = {DIR_1_GPIO_Port,  DIR_1_Pin},
        .ms1  = {MS1_1_GPIO_Port,  MS1_1_Pin},
        .ms2  = {APP_MS2_1_Port,   APP_MS2_1_Pin},
    });
    TMC2209_Init(&s_motor_pan, &(TMC2209_PinConfig){
        .en   = {EN_2_GPIO_Port,   EN_2_Pin},
        .step = {STEP_2_GPIO_Port, STEP_2_Pin},
        .dir  = {DIR_2_GPIO_Port,  DIR_2_Pin},
        .ms1  = {MS1_2_GPIO_Port,  MS1_2_Pin},
        .ms2  = {MS2_2_GPIO_Port,  MS2_2_Pin},
    });

    /* ── STEP 脉冲引擎: Y→TIM14, X→TIM13 ── */
    MotorStepper_Init(&stepper_tilt, &s_motor_tilt, &htim14);
    MotorStepper_Init(&stepper_pan,  &s_motor_pan,  &htim13);

    /* ── 轴控层 ── */
    MotorAxis_Init(&s_tilt_axis, &s_enc_tilt, &stepper_tilt, &s_motor_tilt);
    MotorAxis_Init(&s_pan_axis,  &s_enc_pan,  &stepper_pan,  &s_motor_pan);

    /* ── 运动层 / 仲裁层 ── */
    Motion_Init(&s_motion, &s_pan_axis, &s_tilt_axis);
    Motion_DefaultCalib(&s_motion.calib, 100.0f);   /* 云台距屏 ~100cm 默认标定 */
    MotionArbiter_Init(&s_arb, &s_motion);

    /* ── 串口 ── */
    UART_Init(&s_uart_dbg, &huart1);
    UART_Init(&s_uart_cam, &huart2);
    UART_Open(&s_uart_dbg);
    UART_Open(&s_uart_cam);

    /* ── 按键 ── */
    app_key_bind(&s_key_calib,  KEY_CALIB_GPIO_Port,  KEY_CALIB_Pin);
    app_key_bind(&s_key_reset,  KEY_RESET_GPIO_Port,  KEY_RESET_Pin);
    app_key_bind(&s_key_border, KEY_BORDER_GPIO_Port, KEY_BORDER_Pin);
    app_key_bind(&s_key_pause,  KEY_PAUSE_GPIO_Port,  KEY_PAUSE_Pin);
    app_key_bind(&s_key_track,  KEY_TRACK_GPIO_Port,  KEY_TRACK_Pin);
    app_key_bind(&s_key_a4,     KEY_A4_GPIO_Port,     KEY_A4_Pin);

    /* ── 上电锁定当前位置 (target = 当前角度, 不会产生运动) ── */
    Motion_Enable(&s_motion, 1);

    /* ── 标定数组初值 (未标定) ── */
    {
        uint8_t i;
        for (i = 0; i < 5; i++) { s_cal_pan[i] = 0.0f; s_cal_tilt[i] = 0.0f; }
    }

    s_last_ctrl = s_print_tick = s_calib_tick = s_boot_tick = HAL_GetTick();
    s_mode = MODE_IDLE;

    /* ── Banner ── */
    app_logf("\r\n==== STM32_MotorKing_orz (2023E red gimbal) ====\r\n");
    app_logf("X axis = Motor2 Pan  (TIM13/I2C2) %s\r\n", s_enc_pan_ok  ? "OK" : "NO ENC!");
    app_logf("Y axis = Motor1 Tilt (TIM14/I2C1) %s\r\n", s_enc_tilt_ok ? "OK" : "NO ENC!");
    app_logf("KEY_CALIB=PE0 (calib 0..4) RESET=PB5 BORDER=PB9 PAUSE=PB3\r\n");
    app_logf("cmd target = %s (see 'help')\r\n", APP_DBG_NAME);
    app_logf(">> IDLE (boot, keys muted %ums)\r\n", (unsigned)APP_KEY_STARTUP_MS);
}

void APP_Task(void) {
    uint32_t now = HAL_GetTick();

    app_cmd_poll();              /* 先取调试命令行 (在 UART_Task 消费前) */
    UART_Task(&s_uart_dbg);
    UART_Task(&s_uart_cam);

    if ((uint32_t)(now - s_last_ctrl) >= APP_CTRL_MS) {
        float dt = (float)(now - s_last_ctrl) * 0.001f;
        s_last_ctrl = now;
        if (dt > 0.05f) dt = 0.05f;      /* 调试暂停等导致的长间隔限幅 */
        app_keys(now);
        app_run(dt, now);
    }

    app_print(now);
}
