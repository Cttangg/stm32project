/**
 ******************************************************************************
 * @file    app.c
 * @brief   Application 层实现 (双轴 XY 角度闭环 + 串口调试)
 *
 * 反馈链: MT6701 (多圈角度 deg) → AxisController → PID → MotorStepper 速度
 * 机械:   丝杆导程 2mm/r (见 motion_config.h); 位置单位 cm (对外)
 *
 * 串口命令 (USART1, 115200-8N1, 以回车/换行结束):
 *   pos <x> <y>   移动到 (x,y) 位置, 单位 cm (负值非法)
 *   cw <n>        顺时针转 n 圈 (X 轴)
 *   ccw <n>       逆时针转 n 圈 (X 轴)
 *   home          自动校零 (当前: 以上电位置为 0)
 *   stop          停止两轴
 *   clear         清两轴故障
 *   ?             打印两轴 圈数/角度/位置/状态
 *
 * 方向约定: 以编码器角度增大为"逆时针(ccw)"。
 ******************************************************************************
 */
#include "app.h"
#include "motion_config.h"
#include <string.h>
#include <stdio.h>

#define APP_LINE_MAX 64

static UART_Device   *s_dbg;
static AxisController s_axis_x;
static AxisController s_axis_y;
static MT6701_HandleTypeDef *s_enc_x;
static MT6701_HandleTypeDef *s_enc_y;

static char     s_line[APP_LINE_MAX];
static uint8_t  s_line_len;
static AxisState_t s_last_state_x;
static AxisState_t s_last_state_y;
static uint32_t s_last_progress_tick;
static uint32_t s_stream_ms;      /* 高频遥测周期 ms (0=关) */
static uint32_t s_stream_last;

/* ========================================================================= */
/*  反馈                                                                      */
/* ========================================================================= */
static float app_read_enc(MT6701_HandleTypeDef *e)
{
    int32_t turns = 0;
    float   deg;
    if (!e || !e->inited) return 0.0f;
    deg = MT6701_ReadMultiTurn(e, &turns);
    return AXIS_FEEDBACK_SIGN * ((float)turns * 360.0f + deg);
}

static float enc_read_deg(void *ctx)
{
    return app_read_enc((MT6701_HandleTypeDef *)ctx);
}

/* ========================================================================= */
/*  打印                                                                      */
/* ========================================================================= */
static void app_send(const char *s)
{
    if (!s_dbg) return;
    UART_Send(s_dbg, (const uint8_t *)s, (uint16_t)strlen(s), NULL);
}

/* 单轴状态行: 累计圈数 + 圈内角度 + 位置/目标 (cm) */
static void app_axis_line(const char *name, AxisController *ax)
{
    AxisStatus_t st = AxisController_GetStatus(ax);
    int32_t turns;
    float   angle;
    char    buf[160];

    turns = (int32_t)(st.current_deg / 360.0f);
    angle = st.current_deg - (float)turns * 360.0f;
    if (angle < 0.0f) { angle += 360.0f; turns -= 1; }

    snprintf(buf, sizeof(buf),
             "[%s] turns=%ld angle=%.2fdeg | pos=%.3fcm tgt=%.3fcm state=%d fault=%u err=%d\r\n",
             name, (long)turns, angle,
             st.current_mm / 10.0f, st.target_mm / 10.0f,
             (int)st.state, (unsigned)st.fault, (int)st.last_error);
    app_send(buf);
}

static void app_print_status(void)
{
    app_axis_line("X", &s_axis_x);
    app_axis_line("Y", &s_axis_y);
}

/* 高频遥测: CSV 一行 (期望 vs 实测速度对比)
 * t_ms, X:state,pos,tgt,err,p,i,d,vref,vd_act(d/s),vs_act(steps/s),dir,
 *        Y:state,pos,tgt,err,vref,vd_act(d/s),vs_act(steps/s),dir
 * vd_act = filtered_vel (编码器实测, deg/s); vs_act = vd_act * STEPS_PER_REV/360 */
static void app_stream_sample(void)
{
    AxisStatus_t sx = AxisController_GetStatus(&s_axis_x);
    AxisStatus_t sy = AxisController_GetStatus(&s_axis_y);
    float spd = AXIS_STEPS_PER_REV / 360.0f;   /* steps/deg */
    float xv = s_axis_x.pid.filtered_vel;
    float yv = s_axis_y.pid.filtered_vel;
    char b[240];
    snprintf(b, sizeof(b),
             "%lu,%d,%.4f,%.4f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,"
             "%d,%.4f,%.4f,%.3f,%.1f,%.1f,%.1f,%d\r\n",
             (unsigned long)HAL_GetTick(),
             (int)sx.state, sx.current_mm, sx.target_mm, MotorPID_GetError(&s_axis_x.pid),
             s_axis_x.pid.p_term, s_axis_x.pid.i_term, s_axis_x.pid.d_term,
             s_axis_x.pid.vel_ref, xv, xv * spd, (int)s_axis_x.stepper->direction,
             (int)sy.state, sy.current_mm, sy.target_mm, MotorPID_GetError(&s_axis_y.pid),
             s_axis_y.pid.vel_ref, yv, yv * spd, (int)s_axis_y.stepper->direction);
    app_send(b);
}

static void app_help(void)
{
    app_send("cmds: pos <x> <y>(cm) | cw/ccw <n> (X) | cw2/ccw2 <n> (Y) | home | stop | clear |"
             " raw | enc | freq | pid <kp> <ki> <kd> | dalpha <0..1> | stream <ms> |"
             " vmax <n> | ms <8|16|32|64> | ms1 <8|16|32|64> | mspin <1|2> <0|1> |"
             " ol | cl | jog <steps> [rate] | ?\r\n");
}

/* ========================================================================= */
/*  命令解析                                                                  */
/* ========================================================================= */
static void app_process_line(char *line)
{
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return;

    /* pos <x_cm> <y_cm> */
    if (strncmp(line, "pos", 3) == 0 &&
        (line[3] == ' ' || line[3] == '\t')) {
        float x, y;
        if (sscanf(line + 3, "%f %f", &x, &y) == 2) {
            AxisResult_t r;
            if (x < 0.0f || y < 0.0f) { app_send("error: x/y must be >= 0 (cm)\r\n"); return; }
            r = App_MovePosCM(x, y);
            if (r != AXIS_RESULT_OK) {
                char b[48]; snprintf(b, sizeof(b), "pos failed: result=%d\r\n", (int)r); app_send(b);
            } else {
                app_print_status();
            }
        } else {
            app_send("usage: pos <x_cm> <y_cm>   (>=0)\r\n");
        }
        return;
    }

    if (strncmp(line, "cw", 2) == 0 &&
        (line[2] == '\0' || line[2] == ' ' || line[2] == '\t')) {
        float n;
        if (sscanf(line + 2, "%f", &n) == 1 && n > 0.0f) {
            AxisResult_t r = App_MoveTurns(-n);
            if (r != AXIS_RESULT_OK) { char b[48]; snprintf(b, sizeof(b), "cw failed: result=%d\r\n", (int)r); app_send(b); }
            else app_print_status();
        } else app_send("usage: cw <revolutions>\r\n");
        return;
    }

    if (strncmp(line, "ccw", 3) == 0 &&
        (line[3] == '\0' || line[3] == ' ' || line[3] == '\t')) {
        float n;
        if (sscanf(line + 3, "%f", &n) == 1 && n > 0.0f) {
            AxisResult_t r = App_MoveTurns(n);
            if (r != AXIS_RESULT_OK) { char b[48]; snprintf(b, sizeof(b), "ccw failed: result=%d\r\n", (int)r); app_send(b); }
            else app_print_status();
        } else app_send("usage: ccw <revolutions>\r\n");
        return;
    }

    if (strncmp(line, "cw2", 3) == 0 &&
        (line[3] == '\0' || line[3] == ' ' || line[3] == '\t')) {
        float n;
        if (sscanf(line + 3, "%f", &n) == 1 && n > 0.0f) {
            AxisResult_t r = App_MoveTurnsY(-n);
            if (r != AXIS_RESULT_OK) { char b[48]; snprintf(b, sizeof(b), "cw2 failed: result=%d\r\n", (int)r); app_send(b); }
            else app_print_status();
        } else app_send("usage: cw2 <revolutions>\r\n");
        return;
    }

    if (strncmp(line, "ccw2", 4) == 0 &&
        (line[4] == '\0' || line[4] == ' ' || line[4] == '\t')) {
        float n;
        if (sscanf(line + 4, "%f", &n) == 1 && n > 0.0f) {
            AxisResult_t r = App_MoveTurnsY(n);
            if (r != AXIS_RESULT_OK) { char b[48]; snprintf(b, sizeof(b), "ccw2 failed: result=%d\r\n", (int)r); app_send(b); }
            else app_print_status();
        } else app_send("usage: ccw2 <revolutions>\r\n");
        return;
    }

    if (strncmp(line, "home", 4) == 0) {
        App_Home();
        app_send("home: zero set to current position\r\n");
        app_print_status();
        return;
    }

    if (strncmp(line, "stop", 4) == 0) {
        App_Stop();
        app_print_status();
        return;
    }

    if (strncmp(line, "clear", 5) == 0) {
        AxisController_ClearFault(&s_axis_x);
        AxisController_ClearFault(&s_axis_y);
        app_print_status();
        return;
    }

    if (strncmp(line, "enc", 3) == 0) {
        char b[96];
        snprintf(b, sizeof(b), "[ENC] X=%.2f deg  Y=%.2f deg\r\n",
                 app_read_enc(s_enc_x), app_read_enc(s_enc_y));
        app_send(b);
        return;
    }

    if (strncmp(line, "raw", 3) == 0) {
        char b[192];
        snprintf(b, sizeof(b),
                 "[RAW X] enc=%.2f steps=%lu vel_ref=%.1f | [RAW Y] enc=%.2f steps=%lu vel_ref=%.1f\r\n",
                 app_read_enc(s_enc_x), (unsigned long)MotorStepper_GetExecutedSteps(s_axis_x.stepper),
                 s_axis_x.pid.vel_ref,
                 app_read_enc(s_enc_y), (unsigned long)MotorStepper_GetExecutedSteps(s_axis_y.stepper),
                 s_axis_y.pid.vel_ref);
        app_send(b);
        return;
    }

    if (strncmp(line, "freq", 4) == 0) {
        MotorStepper *st = s_axis_x.stepper;
        uint32_t psc = st->htim->Instance->PSC;
        uint32_t arr = st->htim->Instance->ARR;
        char     b[160];
        if (st->step_rate <= 0.0f) {
            snprintf(b, sizeof(b), "[FREQ X] stopped | STEP=0 Hz | PSC=%lu ARR=%lu\r\n",
                     (unsigned long)psc, (unsigned long)arr);
        } else {
            float upd_hz = 84000000.0f / ((float)(psc + 1) * (float)(arr + 1));
            snprintf(b, sizeof(b),
                     "[FREQ X] step_rate=%.1f steps/s | timer_upd=%.1f Hz | STEP=%.1f Hz | PSC=%lu ARR=%lu\r\n",
                     st->step_rate, upd_hz, upd_hz * 0.5f, (unsigned long)psc, (unsigned long)arr);
        }
        app_send(b);
        return;
    }

    if (strncmp(line, "pid", 3) == 0) {
        float kp, ki, kd;
        if (sscanf(line + 3, "%f %f %f", &kp, &ki, &kd) == 3) {
            s_axis_x.pid.kp = kp; s_axis_x.pid.ki = ki; s_axis_x.pid.kd = kd;
            s_axis_y.pid.kp = kp; s_axis_y.pid.ki = ki; s_axis_y.pid.kd = kd;
            { char b[96]; snprintf(b, sizeof(b), "pid: kp=%.1f ki=%.1f kd=%.1f (X,Y)\r\n", kp, ki, kd); app_send(b); }
        } else app_send("usage: pid <kp> <ki> <kd>\r\n");
        return;
    }

    if (strncmp(line, "dalpha", 6) == 0) {
        float a;
        if (sscanf(line + 6, "%f", &a) == 1 && a >= 0.0f && a < 1.0f) {
            s_axis_x.pid.d_alpha = a; s_axis_y.pid.d_alpha = a;
            { char b[64]; snprintf(b, sizeof(b), "d_alpha=%.2f (X,Y)\r\n", a); app_send(b); }
        } else app_send("usage: dalpha <0..1>\r\n");
        return;
    }

    if (strncmp(line, "stream", 6) == 0) {
        int n = 0;
        if (sscanf(line + 6, "%d", &n) == 1 && n >= 0) {
            s_stream_ms = (uint32_t)n;
            s_stream_last = HAL_GetTick();
            if (n == 0) {
                app_send("stream off\r\n");
            } else {
                char b[48]; snprintf(b, sizeof(b), "stream on: %d ms\r\n", n); app_send(b);
                app_send("t_ms,Xst,Xpos,Xtgt,Xerr,Xp,Xi,Xd,Xvref,Xvact_dps,Xvact_sps,Xdir,"
                         "Yst,Ypos,Ytgt,Yerr,Yvref,Yvact_dps,Yvact_sps,Ydir\r\n");
            }
        } else app_send("usage: stream <ms>  (0=off)\r\n");
        return;
    }

    if (strncmp(line, "vmax", 4) == 0) {
        float v;
        if (sscanf(line + 4, "%f", &v) == 1 && v > 0.0f && v <= 50000.0f) {
            s_axis_x.pid.max_speed = v; s_axis_x.pid.max_accel = v * 4.0f;
            s_axis_y.pid.max_speed = v; s_axis_y.pid.max_accel = v * 4.0f;
            { char b[64]; snprintf(b, sizeof(b), "vmax=%.0f steps/s (X,Y)\r\n", v); app_send(b); }
        } else app_send("usage: vmax <steps_per_sec 1..50000>\r\n");
        return;
    }

    /* ms1 <n>: 仅电机1(X)细分; 打印 MS1/MS2 回读 */
    if (strncmp(line, "ms1", 3) == 0 &&
        (line[3] == '\0' || line[3] == ' ' || line[3] == '\t')) {
        int n = 0; TMC2209_Microstep ms;
        if (sscanf(line + 3, "%d", &n) != 1) { app_send("usage: ms1 <8|16|32|64>\r\n"); return; }
        switch (n) {
            case 8:  ms = TMC2209_MICROSTEP_8;  break;
            case 16: ms = TMC2209_MICROSTEP_16; break;
            case 32: ms = TMC2209_MICROSTEP_32; break;
            case 64: ms = TMC2209_MICROSTEP_64; break;
            default: app_send("usage: ms1 <8|16|32|64>\r\n"); return;
        }
        MotorStepper_SetMicrostep(s_axis_x.stepper, ms);
        { char b[96]; snprintf(b, sizeof(b), "X microstep=1/%d  MS1(PD10)=%d MS2(PD12)=%d\r\n", n,
            HAL_GPIO_ReadPin(MS1_1_GPIO_Port, MS1_1_Pin) ? 1 : 0,
            HAL_GPIO_ReadPin(MS2_1_GPIO_Port, MS2_1_Pin) ? 1 : 0); app_send(b); }
        return;
    }

    /* mspin <1|2> <0|1>: 直接驱动电机1 的 MS1/MS2 引脚并回读 (排查硬件) */
    if (strncmp(line, "mspin", 5) == 0) {
        int which = 0, val = 0;
        if (sscanf(line + 5, "%d %d", &which, &val) == 2 &&
            (which == 1 || which == 2) && (val == 0 || val == 1)) {
            TMC2209_PinDef *p = (which == 1)
                ? &s_axis_x.stepper->motor->pins.ms1
                : &s_axis_x.stepper->motor->pins.ms2;
            char b[112];
            HAL_GPIO_WritePin(p->port, p->pin, val ? GPIO_PIN_SET : GPIO_PIN_RESET);
            snprintf(b, sizeof(b), "MS%d write %d -> readback=%d  (port=%p pin=0x%04X)\r\n",
                     which, val, HAL_GPIO_ReadPin(p->port, p->pin) ? 1 : 0,
                     (void *)p->port, (unsigned)p->pin);
            app_send(b);
        } else {
            app_send("usage: mspin <1|2> <0|1>\r\n");
        }
        return;
    }

    if (strncmp(line, "ms", 2) == 0 &&
        (line[2] == '\0' || line[2] == ' ' || line[2] == '\t')) {
        int n = 0; TMC2209_Microstep ms;
        if (sscanf(line + 2, "%d", &n) != 1) { app_send("usage: ms <8|16|32|64>\r\n"); return; }
        switch (n) {
            case 8:  ms = TMC2209_MICROSTEP_8;  break;
            case 16: ms = TMC2209_MICROSTEP_16; break;
            case 32: ms = TMC2209_MICROSTEP_32; break;
            case 64: ms = TMC2209_MICROSTEP_64; break;
            default: app_send("usage: ms <8|16|32|64>\r\n"); return;
        }
        if (s_axis_x.closed_loop) AxisController_SetFeedback(&s_axis_x, NULL, NULL);
        if (s_axis_y.closed_loop) AxisController_SetFeedback(&s_axis_y, NULL, NULL);
        MotorStepper_SetMicrostep(s_axis_x.stepper, ms);
        MotorStepper_SetMicrostep(s_axis_y.stepper, ms);
        app_send("microstep set (X,Y)\r\n");
        return;
    }

    if (strncmp(line, "ol", 2) == 0) {
        AxisController_Stop(&s_axis_x); AxisController_Stop(&s_axis_y);
        AxisController_SetFeedback(&s_axis_x, NULL, NULL);
        AxisController_SetFeedback(&s_axis_y, NULL, NULL);
        app_send("mode: open-loop (X,Y)\r\n");
        return;
    }

    if (strncmp(line, "cl", 2) == 0) {
        if (s_enc_x && s_enc_x->inited) AxisController_SetFeedback(&s_axis_x, enc_read_deg, s_enc_x);
        if (s_enc_y && s_enc_y->inited) AxisController_SetFeedback(&s_axis_y, enc_read_deg, s_enc_y);
        s_axis_x.timeout_ms = AXIS_MOVE_TIMEOUT_MS;
        s_axis_y.timeout_ms = AXIS_MOVE_TIMEOUT_MS;
        app_send("mode: closed-loop (X,Y)\r\n");
        return;
    }

    if (strncmp(line, "jog", 3) == 0) {
        float n = 0.0f, rate = 500.0f;
        int   got = sscanf(line + 3, "%f %f", &n, &rate);
        if (got >= 1 && n != 0.0f) {
            AxisResult_t res; float dur_ms;
            if (got < 2) rate = 500.0f;
            if (rate < 10.0f) rate = 10.0f;
            if (rate > (float)AXIS_MAX_VELOCITY) rate = (float)AXIS_MAX_VELOCITY;
            if (s_axis_x.closed_loop) AxisController_SetFeedback(&s_axis_x, NULL, NULL);
            dur_ms = ((n < 0.0f) ? -n : n) / rate * 1000.0f;
            s_axis_x.timeout_ms = (uint32_t)(dur_ms * 2.0f) + 2000U;
            res = AxisController_MoveSteps(&s_axis_x, (int32_t)n, (uint32_t)rate);
            if (res != AXIS_RESULT_OK) { char b[64]; snprintf(b, sizeof(b), "jog failed: result=%d\r\n", (int)res); app_send(b); }
            else { char b[80]; snprintf(b, sizeof(b), "jog: %ld steps @ %lu steps/s (X, open-loop)\r\n", (long)(int32_t)n, (unsigned long)(uint32_t)rate); app_send(b); }
        } else app_send("usage: jog <steps> [rate_steps_per_sec]\r\n");
        return;
    }

    if (line[0] == '?') {
        app_print_status();
        return;
    }

    app_help();
}

/* ========================================================================= */
/*  初始化 / 调度                                                             */
/* ========================================================================= */
void App_Init(MotorStepper *x_stepper, MT6701_HandleTypeDef *x_encoder,
              MotorStepper *y_stepper, MT6701_HandleTypeDef *y_encoder,
              UART_Device *debug_uart)
{
    s_dbg   = debug_uart;
    s_enc_x = x_encoder;
    s_enc_y = y_encoder;
    s_line_len = 0;

    /* X 轴 (电机1, TIM14 + I2C1) */
    AxisController_Init(&s_axis_x, "X", x_stepper);
    MotorStepper_SetMicrostep(s_axis_x.stepper, TMC2209_MICROSTEP_16);
    if (x_encoder && x_encoder->inited) AxisController_SetFeedback(&s_axis_x, enc_read_deg, x_encoder);
    AxisController_Enable(&s_axis_x);

    /* Y 轴 (电机2, TIM13 + I2C2) */
    AxisController_Init(&s_axis_y, "Y", y_stepper);
    MotorStepper_SetMicrostep(s_axis_y.stepper, TMC2209_MICROSTEP_16);
    if (y_encoder && y_encoder->inited) AxisController_SetFeedback(&s_axis_y, enc_read_deg, y_encoder);
    AxisController_Enable(&s_axis_y);

    /* 自动零点校准 (预留): 当前无回零/限位硬件 → 以上电位置为 0 点 */
    AxisController_Home(&s_axis_x);
    AxisController_Home(&s_axis_y);

    s_last_state_x = s_axis_x.state;
    s_last_state_y = s_axis_y.state;
    s_last_progress_tick = HAL_GetTick();

    app_send("[APP] XY axis ready (closed-loop, lead=2mm/r); no auto motion.\r\n");
    app_help();
    app_print_status();
}

void App_Update(void)
{
    uint8_t c;

    AxisController_Update(&s_axis_x);
    AxisController_Update(&s_axis_y);

    while (UART_Read(s_dbg, &c, 1) == 1) {
        if (c == '\r' || c == '\n') {
            if (s_line_len > 0) { s_line[s_line_len] = '\0'; app_process_line(s_line); s_line_len = 0; }
        } else if (c == '\b' || c == 127) {
            if (s_line_len > 0) s_line_len--;
        } else if (s_line_len < APP_LINE_MAX - 1) {
            s_line[s_line_len++] = (char)c;
        }
    }

    if (s_axis_x.state != s_last_state_x || s_axis_y.state != s_last_state_y) {
        s_last_state_x = s_axis_x.state;
        s_last_state_y = s_axis_y.state;
        app_print_status();
    } else if ((s_axis_x.state == AXIS_STATE_MOVING || s_axis_y.state == AXIS_STATE_MOVING) &&
               (HAL_GetTick() - s_last_progress_tick) >= 500U) {
        s_last_progress_tick = HAL_GetTick();
        app_print_status();
    }

    /* 高频遥测流 */
    if (s_stream_ms > 0 && (HAL_GetTick() - s_stream_last) >= s_stream_ms) {
        s_stream_last = HAL_GetTick();
        app_stream_sample();
    }
}

/* ========================================================================= */
/*  命令接口 (供串口解析层调用)                                               */
/* ========================================================================= */

AxisResult_t App_MovePosCM(float x_cm, float y_cm)
{
    AxisResult_t rx, ry;
    if (x_cm < 0.0f || y_cm < 0.0f) return AXIS_RESULT_INVALID_PARAM;
    rx = AxisController_SetTargetMM(&s_axis_x, x_cm * 10.0f);
    ry = AxisController_SetTargetMM(&s_axis_y, y_cm * 10.0f);
    return (rx != AXIS_RESULT_OK) ? rx : ry;
}

AxisResult_t App_Home(void)
{
    AxisResult_t rx = AxisController_Home(&s_axis_x);
    AxisResult_t ry = AxisController_Home(&s_axis_y);
    return (rx != AXIS_RESULT_OK) ? rx : ry;
}

AxisResult_t App_MoveTurns(float turns)
{
    return AxisController_MoveRelMM(&s_axis_x, turns * AXIS_LEAD_MM_PER_REV);
}

AxisResult_t App_MoveTurnsY(float turns)
{
    return AxisController_MoveRelMM(&s_axis_y, turns * AXIS_LEAD_MM_PER_REV);
}

AxisResult_t App_Stop(void)
{
    AxisResult_t rx = AxisController_Stop(&s_axis_x);
    AxisResult_t ry = AxisController_Stop(&s_axis_y);
    return (rx != AXIS_RESULT_OK) ? rx : ry;
}

AxisResult_t App_Enable(void)
{
    AxisResult_t rx = AxisController_Enable(&s_axis_x);
    AxisResult_t ry = AxisController_Enable(&s_axis_y);
    return (rx != AXIS_RESULT_OK) ? rx : ry;
}

void App_Disable(void)
{
    AxisController_Disable(&s_axis_x);
    AxisController_Disable(&s_axis_y);
}

AxisStatus_t App_GetStatus(void)
{
    return AxisController_GetStatus(&s_axis_x);
}
