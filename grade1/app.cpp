/**
 * *****************************************************************************
 * ファイル名 : app.c
 *
 * 概要 : 2輪倒立振子ライントレースロボットのTOPPERS/HRP2用Cサンプルプログラム
 *
 * 注記 : sample_c4 (sample_c3にBluetooth通信リモートスタート機能を追加)
 *
 * @version task_1.0 : 2017.06.28
 *    +ディレクトリ名をtouchguy に変更する
 *    +コントロールタスク追加し周期ハンドラで動かす
 ******************************************************************************
 */
#define VERSION "task_1.0"

#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/**
 * センサー、モーターの接続を定義します
 */
static const sensor_port_t
    color_sensor    = EV3_PORT_1,
    sonar_sensor    = EV3_PORT_2,
    gyro_sensor     = EV3_PORT_3;

static const motor_port_t
    m_motor         = EV3_PORT_A,
    left_motor      = EV3_PORT_B,
    right_motor     = EV3_PORT_C;

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
#define GYRO_OFFSET  0          /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define LIGHT_WHITE  40         /* 白色の光センサ値 */
#define LIGHT_BLACK  0          /* 黒色の光センサ値 */
#define SONAR_ALERT_DISTANCE 15 /* 超音波センサによる障害物検知距離[cm] */

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

/* メインタスク */
void main_task(intptr_t unused)
{
    char buf[64];

    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    sprintf(buf, "Steel Fight 2017 ver.%s", VERSION );
    ev3_lcd_draw_string(buf, 0, CALIB_FONT_HEIGHT*1);

    /* センサー入力ポートの設定 */
    ev3_sensor_config(color_sensor, COLOR_SENSOR);
    ev3_color_sensor_get_reflect(color_sensor); /* 反射率モード */
    ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
    ev3_sensor_config(gyro_sensor, GYRO_SENSOR);
    /* モーター出力ポートの設定 */
    ev3_motor_config(m_motor, LARGE_MOTOR);
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);

    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    /* スタート待機 */
    while(1) {
        if (ev3_button_is_pressed(ENTER_BUTTON)) {
            break; /* スタート */
        }
        tslp_tsk(10); /* 10msecウェイト */
    }

    /* 走行モーターエンコーダーリセット */
    ev3_motor_reset_counts(m_motor);
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);

    /* ジャイロセンサーリセット */
    ev3_gyro_sensor_reset(gyro_sensor);

    ev3_led_set_color(LED_GREEN); /* スタート通知 */

    ev3_motor_stop(m_motor, false);
    ev3_motor_stop(left_motor, false);
    ev3_motor_stop(right_motor, false);

    ev3_lcd_draw_string("EV3RT Stopped.", 0, CALIB_FONT_HEIGHT*9);
    ext_tsk();
}
