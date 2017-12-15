/**
 ******************************************************************************
 ** ファイル名 : app.cpp
 **
 ** 概要 : 2輪倒立振子ライントレースロボットのTOPPERS/HRP2用C++サンプルプログラム
 **
 ** 注記 : sample_cpp (ライントレース/尻尾モータ/超音波センサ/リモートスタート)
 ******************************************************************************
 */
#define VERSION "kuwanomi0_1.0"

#include "ev3api.h"
#include "app.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"
#include "PID.h"
#include "Distance.h"
#include <string.h>

using namespace ev3api;

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

/* Bluetooth */
static int32_t   bt_cmd = 0;      /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt = NULL;      /* Bluetoothファイルハンドル */

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
/* 走行に関するマクロ */
#define RGB_TARGET          631  /*中央の境界線のRGBセンサ合計値 */
#define KLP                 0.6  /* LPF用係数*/
#define COLOR               160

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/* magic number*/)
#define CALIB_FONT_HEIGHT (8/* magic number*/)

/* 関数プロトタイプ宣言 */
static void BTconState();

/* オブジェクトへのポインタ定義 */
ColorSensor*    colorSensor;
SonarSensor*    sonarSensor;
GyroSensor*     gyroSensor;
Motor*          armMotor;
Motor*          leftMotor;
Motor*          rightMotor;
Clock*          clock;
Distance*       distanceWay;
PID*            walkPID; /* 走行用のPIDインスタンス */
PID*            gyroPID;

/* 走行距離 */
static rgb_raw_t rgb_level;  /* カラーセンサーから取得した値を格納する構造体 */
static int8_t pwm_A = 0;     /* アームモータPWM出力 */
static int8_t pwm_L = 0;     /* 左モータPWM出力 */
static int8_t pwm_R = 0;     /* 右モータPWM出力 */
static uint16_t rgb_total = RGB_TARGET;
static uint16_t rgb_before;
static int8_t flag = 0;
static int8_t pid = 0;
static int32_t BGYRO = -185;
static int32_t TGYRO = -85;
static int32_t disBefore = 0;
static int32_t DISTAN = 800;
static int8_t  ends = 0;

/* メインタスク */
void main_task(intptr_t unused)
{
    /* 各オブジェクトを生成・初期化する */
    colorSensor = new ColorSensor(PORT_1);
    sonarSensor = new SonarSensor(PORT_2);
    gyroSensor  = new GyroSensor(PORT_3);
    armMotor    = new Motor(PORT_A);
    leftMotor   = new Motor(PORT_B);
    rightMotor  = new Motor(PORT_C);
    clock       = new Clock();
    distanceWay = new Distance();
    walkPID     = new PID(RGB_TARGET, 0.1800F, 0.0000F, 2.2000F);
    gyroPID     = new PID(-180, 1.2F, 0.0F, 0.0F);


    /* LCD画面表示 */
    char buf[64];
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    sprintf(buf, "Steel Fight 2017 ver.%s", VERSION );
    ev3_lcd_draw_string(buf, 0, CALIB_FONT_HEIGHT*1);

    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth通信タスクの起動 */
    act_tsk(BT_TASK);

    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    /* スタート待機 */
    while(1) {
        if (bt_cmd == 1 || ev3_button_is_pressed(ENTER_BUTTON)) {
            bt_cmd = 1;
            break;
        }

        if (ev3_button_is_pressed(LEFT_BUTTON)) {
            armMotor->setPWM(-20);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON)) {
            armMotor->setPWM(20);
        }
        if (ev3_button_is_pressed(DOWN_BUTTON)) {
            armMotor->setPWM(0);
        }

        BTconState();
        clock->sleep(10); /* 10msecウェイト */
    }

    /* モーターエンコーダーリセット */
    leftMotor->reset();
    rightMotor->reset();
    armMotor->reset();

    /* ジャイロセンサーリセット */
    gyroSensor->reset();

    clock->sleep(500); /* 0.5secウェイト */

    ev3_led_set_color(LED_GREEN); /* スタート通知 */

    ER er = ev3_sta_cyc(CYC_HANDLER);   //周期ハンドラを起動
    sprintf(buf, "main_task: error_code=%d", MERCD(er) );   // APIのエラーコードの表示
    //ev3_lcd_draw_string(buf, 0, CALIB_FONT_HEIGHT*1);     // の仕方です。

    /* 自タスク(メインタスク）を待ち状態にする */
    slp_tsk();

    armMotor->reset();
    leftMotor->reset();
    rightMotor->reset();


    ter_tsk(BT_TASK);
    fclose(bt);

    ext_tsk();
}

//*****************************************************************************
// 関数名 : controller_task
// 引数   : 拡張情報
// 返り値 : なし
// 概要   : コントローラータスク
//*****************************************************************************
void controller_task(intptr_t unused)
{
    int32_t motor_ang_L, motor_ang_R;
    int32_t gyro, volt;

    pwm_A = 0;
    pwm_L = 0;
    pwm_R = 0;

    /* バックボタン */
    if (ev3_button_is_pressed(BACK_BUTTON) || bt_cmd == 0 || flag == 50) {
        ev3_led_set_color(LED_RED);
        wup_tsk(MAIN_TASK);        //メインタスクを起床する
        ev3_stp_cyc(CYC_HANDLER);  //周期ハンドラを停止する
    }

    /* パラメータを取得する */
    motor_ang_L = leftMotor->getCount();
    motor_ang_R = rightMotor->getCount();
    gyro = gyroSensor->getAngle();
    volt = ev3_battery_voltage_mV();

    /* 現在の走行距離を取得 */
    distanceWay->update(motor_ang_L, motor_ang_R);

    /* 色の取得 */
    rgb_before = rgb_total; //LPF用前処理
    colorSensor->getRawColor(rgb_level); /* RGB取得 */
    rgb_total = (rgb_level.r + rgb_level.g + rgb_level.b)  * KLP + rgb_before * (1 - KLP); //LPF

    // ステップ0 スタートからつかむ前まで
    if (distanceWay->getDistance() <= 1600 && flag == 0) {
        pwm_L = 51;
        pwm_R = 60;
        pwm_A = 17;
    }
    else if (sonarSensor->getDistance() >= 30 && flag == 0) {
        pwm_R = 5;
        pwm_L = -5;
    }
    else if (flag == 0) {
        int8_t bSonerDis = sonarSensor->getDistance();
        int8_t nowDis;
        while (sonarSensor->getDistance() >= 6) {
            nowDis = bSonerDis - sonarSensor->getDistance();

            if (nowDis > 0) {
                pwm_L = 10;
                pwm_R = 13;
            }
            else if (nowDis < 0) {
                pwm_L = 13;
                pwm_R = 10;
            }
            else {
                pwm_L = 10;
                pwm_R = 11;
            }

            leftMotor->setPWM(pwm_L);
            rightMotor->setPWM(pwm_R);

            bSonerDis = sonarSensor->getDistance();
        }
        flag = 1;
    }

    // ステップ１ ペットボトルをつかむ
    if (flag == 1) {
        clock->reset();
        clock->sleep(1);
        while (clock->now() <= 2250) {
            pwm_L = 10;
            pwm_R = 10;
            leftMotor->setPWM(pwm_L);
            rightMotor->setPWM(pwm_R);
            armMotor->setPWM(-44);
        }
        flag = 2;
        if (ends == 1) {
            flag = 8;
        }
    }


    // ステップ２ 赤いところまでバック
    if (flag == 2) {
        if (rgb_total >= 400) {
            gyroPID->setTaget(BGYRO);
            gyro = gyroSensor->getAngle();
            pid = -gyroPID->calcControl(gyro);
            pwm_L = -55 - pid;
            pwm_R = -55 + pid;
        }
        else {
            flag = 3;
        }
    }

    // ステップ３ ペットボトルを放す
    if (flag == 3) {
        BGYRO += 5;
        clock->reset();
        clock->sleep(1);
        while (clock->now() <= 350) {
            gyroPID->setTaget(BGYRO);
            gyro = gyroSensor->getAngle();
            pid = -gyroPID->calcControl(gyro);
            pwm_L = -55 - pid;
            pwm_R = -55 + pid;
            leftMotor->setPWM(pwm_L);
            rightMotor->setPWM(pwm_R);
            armMotor->setPWM(0);
        }
        clock->reset();
        clock->sleep(1);
        while (clock->now() <= 1000) {
            leftMotor->setPWM(0);
            rightMotor->setPWM(0);
            armMotor->setPWM(60);
        }
        clock->reset();
        clock->sleep(1);
        while (clock->now() <= 320) {
            gyroPID->setTaget(BGYRO);
            gyro = gyroSensor->getAngle();
            pid = -gyroPID->calcControl(gyro);
            pwm_L = -55 - pid;
            pwm_R = -55 + pid;
            leftMotor->setPWM(pwm_L);
            rightMotor->setPWM(pwm_R);
            armMotor->setPWM(54);
        }
        flag = 4;
        pwm_L = 0;
        pwm_R = 0;
    }

    if (flag == 4) {
        gyro = gyroSensor->getAngle();
        while (gyro <= TGYRO) {
            gyro = gyroSensor->getAngle();
            leftMotor->setPWM(60);
            rightMotor->setPWM(0);
            armMotor->stop();
        }

        flag = 5;
        if (ends == 2) {
            flag = 12;
        }
    }

    if (flag == 5) {
        if (gyro >= 45) {
            DISTAN = 820;
        }
        motor_ang_L = leftMotor->getCount();
        motor_ang_R = rightMotor->getCount();
        distanceWay->update(motor_ang_L, motor_ang_R);
        disBefore = distanceWay->getDistance();
        while (distanceWay->getDistance() - disBefore <= DISTAN) {
            gyroPID->setTaget(TGYRO);
            gyro = gyroSensor->getAngle();
            pid = gyroPID->calcControl(gyro);
            pwm_L = 55 + pid;
            pwm_R = 55 - pid;
            leftMotor->setPWM(pwm_L);
            rightMotor->setPWM(pwm_R);
            armMotor->setPWM(2);
            motor_ang_L = leftMotor->getCount();
            motor_ang_R = rightMotor->getCount();
            distanceWay->update(motor_ang_L, motor_ang_R);
        }
        DISTAN = 330;
        BGYRO += 85;
        TGYRO += 89;
        flag = 0;
        if (gyro >= 45) {
            ends = 1;
        }
    }

    if (flag == 8)
    {
        motor_ang_L = leftMotor->getCount();
        motor_ang_R = rightMotor->getCount();
        distanceWay->update(motor_ang_L, motor_ang_R);
        disBefore = distanceWay->getDistance();
        while (distanceWay->getDistance() - disBefore <= 250) {
            pwm_L = 12;
            pwm_R = 12;
            leftMotor->setPWM(pwm_L);
            rightMotor->setPWM(pwm_R);
            armMotor->stop();
            motor_ang_L = leftMotor->getCount();
            motor_ang_R = rightMotor->getCount();
            distanceWay->update(motor_ang_L, motor_ang_R);
        }
        clock->reset();
        clock->sleep(1);
        while (clock->now() <= 1000) {
            leftMotor->setPWM(0);
            rightMotor->setPWM(0);
            armMotor->setPWM(60);
        }
        motor_ang_L = leftMotor->getCount();
        motor_ang_R = rightMotor->getCount();
        distanceWay->update(motor_ang_L, motor_ang_R);
        disBefore = distanceWay->getDistance();
        while (distanceWay->getDistance() - disBefore >= -200) {
            pwm_L = -30;
            pwm_R = -30;
            leftMotor->setPWM(pwm_L);
            rightMotor->setPWM(pwm_R);
            armMotor->stop();
            motor_ang_L = leftMotor->getCount();
            motor_ang_R = rightMotor->getCount();
            distanceWay->update(motor_ang_L, motor_ang_R);
        }

        flag = 4;
        ends = 2;
    }

    if (flag == 12) {
        if (rgb_total >= 500) {
            gyroPID->setTaget(183);
            gyro = gyroSensor->getAngle();
            pid = gyroPID->calcControl(gyro);
            pwm_L = 55 + pid;
            pwm_R = 55 - pid;
        }
        else {
            flag = 20;
        }
    }
    if (flag == 20) {
        motor_ang_L = leftMotor->getCount();
        motor_ang_R = rightMotor->getCount();
        distanceWay->update(motor_ang_L, motor_ang_R);
        disBefore = distanceWay->getDistance();
        while (distanceWay->getDistance() - disBefore >= 160) {
            gyroPID->setTaget(TGYRO);
            gyro = gyroSensor->getAngle();
            pid = gyroPID->calcControl(gyro);
            pwm_L = 20 + pid;
            pwm_R = 20 - pid;
            leftMotor->setPWM(pwm_L);
            rightMotor->setPWM(pwm_R);
            armMotor->setPWM(2);
            motor_ang_L = leftMotor->getCount();
            motor_ang_R = rightMotor->getCount();
            distanceWay->update(motor_ang_L, motor_ang_R);
        }
    }

    /* EV3ではモーター停止時のブレーキ設定が事前にできないため */
    /* 出力0時に、その都度設定する */
    if (pwm_A == 0) {
        armMotor->stop();
    }
    else {
        armMotor->setPWM(pwm_A);
    }

    if (pwm_L == 0) {
        leftMotor->stop();
    }
    else {
        leftMotor->setPWM(pwm_L);
    }

    if (pwm_R == 0) {
        rightMotor->stop();
    }
    else {
        rightMotor->setPWM(pwm_R);
    }

    /* ログを送信する処理 */
    syslog(LOG_NOTICE, "V:%5d  G:%3d\r", volt, gyro);

    // BTconState();
    char bufg[64];
    sprintf(bufg, "G:%3d", gyro);
    ev3_lcd_draw_string(bufg, 0, CALIB_FONT_HEIGHT*4);

    ext_tsk();
}

//*****************************************************************************
// 関数名 : cyc_handler
// 引数   : 拡張情報
// 返り値 : なし
// 概要   : 周期ハンドラ(4ms)
//*****************************************************************************
void cyc_handler(intptr_t unused)
{
    // コントローラタスクを起動する
    act_tsk(CONTROLLER_TASK);
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
    while(1) {
        uint8_t c = fgetc(bt); /* 受信 */
        switch(c) {
        case '1':
            bt_cmd = 1;
            break;
        case '0':
            bt_cmd = 0;
            break;
        default:
            break;
        }
        // fputc(c, bt); /* エコーバック */
    }
}

//*****************************************************************************
// 関数名 : BTconState
// 引数 : なし
// 返り値 : なし
// 概要 : Bluetooth接続状態表示
//*****************************************************************************
static void BTconState() {
    if (ev3_bluetooth_is_connected()) {
        ev3_lcd_draw_string("BT connection : true ", 0, CALIB_FONT_HEIGHT*3);
    }
    else {
        ev3_lcd_draw_string("BT connection : false", 0, CALIB_FONT_HEIGHT*3);
    }
}
