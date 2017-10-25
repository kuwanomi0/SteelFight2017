/**
 ******************************************************************************
 ** ファイル名 : app.cpp
 **
 ** 概要 : 2輪倒立振子ライントレースロボットのTOPPERS/HRP2用C++サンプルプログラム
 **
 ** 注記 : sample_cpp (ライントレース/尻尾モータ/超音波センサ/リモートスタート)
 ******************************************************************************
 **/

#include "ev3api.h"
#include "app.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"

using namespace ev3api;

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/* Bluetooth */
static int32_t   bt_cmd = 0;      /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt = NULL;      /* Bluetoothファイルハンドル */

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
#define CMD_START         '1'    /* リモートスタートコマンド */

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

/* オブジェクトへのポインタ定義 */
SonarSensor*    sonarSensor;
ColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
Motor*          leftMotor;
Motor*          rightMotor;
Motor*          MMotor;
Clock*          clock;

/* メインタスク */
void main_task(intptr_t unused)
{
    /* 各オブジェクトを生成・初期化する */
    colorSensor = new ColorSensor(PORT_1);
    sonarSensor = new SonarSensor(PORT_2);
    gyroSensor  = new GyroSensor(PORT_3);
    rightMotor  = new Motor(PORT_C);
    leftMotor   = new Motor(PORT_B);
    MMotor      = new Motor(PORT_A);
    clock       = new Clock();

    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET sample_cpp", 0, CALIB_FONT_HEIGHT*1);

    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth通信タスクの起動 */
    act_tsk(BT_TASK);

    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    /* スタート待機 */
    while(1)
    {
        if (bt_cmd == 1)
        {
            break; /* リモートスタート */
        }

        clock->sleep(10);
    }

    /* 走行モーターエンコーダーリセット */
    leftMotor->reset();
    rightMotor->reset();
    MMotor->reset();

    /* ジャイロセンサーリセット */
    gyroSensor->reset();

    ev3_led_set_color(LED_GREEN); /* スタート通知 */

    /**
    * Main loop for the self-balance control algorithm
    */
    while(1)
    {
        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        leftMotor->setPWM(10);
        rightMotor->setPWM(10);
        if (clock->now() < 5000) {
            MMotor->setPWM(10);
        }
        if (5000 <= clock->now() && clock->now() < 10000){
            MMotor->setPWM(-10);
        }
        if (10000 <= clock->now()) {
            clock->reset();
        }

        clock->sleep(4); /* 4msec周期起動 */
    }
    leftMotor->reset();
    rightMotor->reset();
    MMotor->reset();

    ter_tsk(BT_TASK);
    fclose(bt);

    ext_tsk();
}

//*****************************************************************************
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
void bt_task(intptr_t unused)
{
    while(1)
    {
        uint8_t c = fgetc(bt); /* 受信 */
        switch(c)
        {
        case '1':
            bt_cmd = 1;
            break;
        default:
            break;
        }
        fputc(c, bt); /* エコーバック */
    }
}
