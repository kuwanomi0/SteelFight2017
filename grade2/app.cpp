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
#define GYRO_OFFSET           0  /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define LIGHT_WHITE          40  /* 白色の光センサ値 */
#define LIGHT_BLACK           0  /* 黒色の光センサ値 */
#define SONAR_ALERT_DISTANCE 30  /* 超音波センサによる障害物検知距離[cm] */
#define CMD_START         '1'    /* リモートスタートコマンド */

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

/* 関数プロトタイプ宣言 */
static int32_t sonar_alert(void);

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
    // int8_t forward;      /* 前後進命令 */
    // int8_t turn;         /* 旋回命令 */
    int8_t pwm_L, pwm_R, pwm_M; /* 左右モータPWM出力 */

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
        int32_t motor_ang_m;
        // int32_t motor_ang_l, motor_ang_r;
        // int32_t gyro, volt;

        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        // if (sonar_alert() == 1) /* 障害物検知 */
        // {
        //     // forward = turn = 0; /* 障害物を検知したら停止 */
        // }
        // else
        // {
        //     // forward = 30; /* 前進命令 */
        //     if (colorSensor->getBrightness() >= (LIGHT_WHITE + LIGHT_BLACK)/2)
        //     {
        //         turn =  20; /* 左旋回命令 */
        //     }
        //     else
        //     {
        //         turn = -20; /* 右旋回命令 */
        //     }
        // }

        /* 倒立振子制御API に渡すパラメータを取得する */
        motor_ang_m = MMotor->getCount();
        // motor_ang_l = leftMotor->getCount();
        // motor_ang_r = rightMotor->getCount();
        // gyro = gyroSensor->getAnglerVelocity();
        // volt = ev3_battery_voltage_mV();


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
// 関数名 : sonar_alert
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
// static int32_t sonar_alert(void)
// {
//     static uint32_t counter = 0;
//     static int32_t alert = 0;

//     int32_t distance;

//     if (++counter == 40/4) /* 約40msec周期毎に障害物検知  */
//     {

//          * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
//          * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
//          * EV3の場合は、要確認

//         distance = sonarSensor->getDistance();
//         if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
//         {
//             alert = 1; /* 障害物を検知 */
//         }
//         else
//         {
//             alert = 0; /* 障害物無し */
//         }
//         counter = 0;
//     }

//     return alert;
// }

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
