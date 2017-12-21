/**
 ******************************************************************************
 ** ファイル名 : app.cpp
 **
 ** 概要 : 2輪倒立振子ライントレースロボットのTOPPERS/HRP2用C++サンプルプログラム
 **
 ** 注記 : sample_cpp (ライントレース/尻尾モータ/超音波センサ/リモートスタート)
 ******************************************************************************
 */
#define VERSION "kuwanomi0_2.1"

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
#define PWM_ABS_MAX         100 /* 完全停止用モータ制御PWM絶対最大値 */
#define ARM_OFF            -180 /* 閉じている時のアームの値 */
#define ARM_ON              600 /* 開いている時のアームの値 */

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/* magic number*/)
#define CALIB_FONT_HEIGHT (8/* magic number*/)

/* 関数プロトタイプ宣言 */
static void armControl(int angle);

/* オブジェクトへのポインタ定義 */
ColorSensor*    colorSensor;
SonarSensor*    sonarSensor;
GyroSensor*     gyroSensor;
Motor*          armMotor;
Motor*          leftMotor;
Motor*          rightMotor;
Clock*          clock;
Distance*       distanceWay;
PID*            gyroPID; /* ジャイロトレース用のPIDインスタンス */
PID*            armPID;  /* アームモータ用のPID */

/* 走行距離 */
static rgb_raw_t rgb_level;  /* カラーセンサーから取得した値を格納する構造体 */
static int8_t pwm_L = 0;     /* 左モータPWM出力 */
static int8_t pwm_R = 0;     /* 右モータPWM出力 */
static uint16_t rgb_total = RGB_TARGET;
static uint16_t rgb_before;
static int8_t flag = 0;
static int8_t startFlag = 0;
static int8_t pid = 0;
static int32_t BGYRO = -95;
static int32_t TGYRO =   0;
static int32_t disBefore = 0;
static int32_t DISTAN = 830;
static int8_t  ends = 0;
static int32_t  sCount = 0;

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
    gyroPID     = new PID(-180, 1.0F, 0.0005F, 0.07F);
    armPID      = new PID(0, 3.5F, 0.0F, 1.0F);


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

        char bufg[64];
        sprintf(bufg, "G:%4d", gyroSensor->getAngle());
        ev3_lcd_draw_string(bufg, 0, CALIB_FONT_HEIGHT*4);
        if (ev3_button_is_pressed(UP_BUTTON)) {
            leftMotor->setPWM(20);
            rightMotor->setPWM(20);
        }
        if (ev3_button_is_pressed(LEFT_BUTTON)) {
            armMotor->setPWM(-20);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON)) {
            armMotor->setPWM(20);
        }
        if (ev3_button_is_pressed(DOWN_BUTTON)) {
            armMotor->stop();
            leftMotor->setPWM(0);
            rightMotor->setPWM(0);
        }

        clock->sleep(10); /* 10msecウェイト */
    }

    clock->sleep(1000); /* 0.5secウェイト */

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
    int     gyro, volt;

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
    if (distanceWay->getDistance() <= 500 && flag == 0 && startFlag == 0) { // 1600mm前進するまで
        motor_ang_L = leftMotor->getCount();
        motor_ang_R = rightMotor->getCount();
        distanceWay->update(motor_ang_L, motor_ang_R);
        disBefore = distanceWay->getDistance();
        while (distanceWay->getDistance() - disBefore <= 920) {
            gyroPID->setTaget(0);
            gyro = gyroSensor->getAngle();
            pid = gyroPID->calcControl(gyro);
            pwm_L = 55 + pid;
            pwm_R = 55 - pid;
            leftMotor->setPWM(pwm_L);
            rightMotor->setPWM(pwm_R);
            armControl(ARM_ON);
            motor_ang_L = leftMotor->getCount();
            motor_ang_R = rightMotor->getCount();
            distanceWay->update(motor_ang_L, motor_ang_R);
        }
    }
    else if (sonarSensor->getDistance() >= 30 && flag == 0) { // ソナーセンサーが30cm以上の値を返しているとき
        if (sCount <= 1000/4) {
            pwm_L = -5;
            pwm_R =  5;
            sCount++;
        }
        else if (sCount <= 3000/4) {
            pwm_L =  5;
            pwm_R = -5;
            sCount++;
        }
        else {
            sCount = -1000/4;
        }
        armControl(ARM_ON);
    }
    else if (flag == 0) {
        sCount = 0;
        int8_t bSonerDis = sonarSensor->getDistance();
        int8_t nowDis;
        while (sonarSensor->getDistance() >= 6) { //ソナーセンサーが6cm以上の値を返しているとき
            nowDis = bSonerDis - sonarSensor->getDistance();

            if (nowDis > 0) {
                pwm_L = 10;
                pwm_R = 12;
            }
            else if (nowDis < 0) {
                pwm_L = 12;
                pwm_R = 10;
            }
            else {
                pwm_L = 10;
                pwm_R = 10;
            }

            leftMotor->setPWM(pwm_L);
            rightMotor->setPWM(pwm_R);
            armControl(ARM_ON);
            bSonerDis = sonarSensor->getDistance();
        }
        flag = 1; // 次の処理へ
    }

    // ステップ１ ペットボトルをつかむ
    if (flag == 1) {

        clock->reset();
        clock->sleep(1);
        while (clock->now() <= 1200) { // 1500ms経過するまで
            pwm_L = 10;
            pwm_R = 10;
            leftMotor->setPWM(pwm_L);
            rightMotor->setPWM(pwm_R);
            armControl(ARM_ON);
        }
        if (startFlag == 0) {
            // 90度カーブ
            gyro = gyroSensor->getAngle();
            while (gyro >= -90) {
                gyro = gyroSensor->getAngle();
                leftMotor->setPWM(0);
                rightMotor->setPWM(30);
                armControl(ARM_ON);
            }
            clock->reset();
            clock->sleep(1);
            while (clock->now() <= 200) { // 100ms経過するまで
                pwm_L = 10;
                pwm_R = 10;
                leftMotor->setPWM(pwm_L);
                rightMotor->setPWM(pwm_R);
                armControl(ARM_ON);
            }
            startFlag = 1;
        }
        clock->reset();
        clock->sleep(1);
        while (clock->now() <= 1000) {
            pwm_L = 0;
            pwm_R = 0;
            leftMotor->setPWM(pwm_L);
            rightMotor->setPWM(pwm_R);
            armControl(ARM_OFF);
        }
        flag = 2;
        if (ends == 1) {
            flag = 8;
        }
    }


    // ステップ２ 赤いところまでバック
    if (flag == 2) {
        if (rgb_total >= 400) {  //RGB値が400以上であるとき
            gyroPID->setTaget(BGYRO);
            gyro = gyroSensor->getAngle();
            pid = -gyroPID->calcControl(gyro);
            pwm_L = -55 - pid;
            pwm_R = -55 + pid;
            armControl(ARM_OFF);
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
        while (clock->now() <= 400) {
            gyroPID->setTaget(BGYRO);
            gyro = gyroSensor->getAngle();
            pid = -gyroPID->calcControl(gyro);
            pwm_L = -55 - pid;
            pwm_R = -55 + pid;
            leftMotor->setPWM(pwm_L);
            rightMotor->setPWM(pwm_R);
            armControl(ARM_OFF);
        }
        clock->reset();
        clock->sleep(1);
        while (clock->now() <= 1000) {
            leftMotor->setPWM(0);
            rightMotor->setPWM(0);
            armControl(ARM_ON);
        }
        clock->reset();
        clock->sleep(1);
        while (clock->now() <= 400) {
            gyroPID->setTaget(BGYRO);
            gyro = gyroSensor->getAngle();
            pid = -gyroPID->calcControl(gyro);
            pwm_L = -55 - pid;
            pwm_R = -55 + pid;
            leftMotor->setPWM(pwm_L);
            rightMotor->setPWM(pwm_R);
            armControl(ARM_ON);
        }
        flag = 4;
        pwm_L = 0;
        pwm_R = 0;
    }

    // 90度カーブ
    if (flag == 4) {
        gyro = gyroSensor->getAngle();
        if (ends == 2) {
            TGYRO = 270;
        }
        while (gyro <= TGYRO) {
            gyro = gyroSensor->getAngle();
            leftMotor->setPWM(30);
            rightMotor->setPWM(0);
            armControl(ARM_ON);
        }
        flag = 5;
        if (ends == 2) {
            flag = 12;
        }
    }

    // 次のペットボトルの近くまで近づく
    if (flag == 5) {
        if (gyro >= 135) {
            TGYRO += 3;
            DISTAN = 830;
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
            armControl(ARM_ON);
            motor_ang_L = leftMotor->getCount();
            motor_ang_R = rightMotor->getCount();
            distanceWay->update(motor_ang_L, motor_ang_R);
        }
        DISTAN = 280;
        BGYRO += 85;
        TGYRO += 89;
        flag = 0;
        if (gyro >= 135) {
            ends = 1;
        }
    }

    // ペットボトルを押し出す
    if (flag == 8) {
        motor_ang_L = leftMotor->getCount();
        motor_ang_R = rightMotor->getCount();
        distanceWay->update(motor_ang_L, motor_ang_R);
        disBefore = distanceWay->getDistance();
        while (distanceWay->getDistance() - disBefore <= 250) {
            pwm_L = 30;
            pwm_R = 30;
            leftMotor->setPWM(pwm_L);
            rightMotor->setPWM(pwm_R);
            armControl(ARM_OFF);
            motor_ang_L = leftMotor->getCount();
            motor_ang_R = rightMotor->getCount();
            distanceWay->update(motor_ang_L, motor_ang_R);
        }
        clock->reset();
        clock->sleep(1);
        while (clock->now() <= 1000) {
            leftMotor->setPWM(0);
            rightMotor->setPWM(0);
            armControl(ARM_ON);
        }
        motor_ang_L = leftMotor->getCount();
        motor_ang_R = rightMotor->getCount();
        distanceWay->update(motor_ang_L, motor_ang_R);
        disBefore = distanceWay->getDistance();
        while (distanceWay->getDistance() - disBefore >= -200) {
            pwm_L = -50;
            pwm_R = -50;
            leftMotor->setPWM(pwm_L);
            rightMotor->setPWM(pwm_R);
            armControl(ARM_ON);
            motor_ang_L = leftMotor->getCount();
            motor_ang_R = rightMotor->getCount();
            distanceWay->update(motor_ang_L, motor_ang_R);
        }

        flag = 4;
        ends = 2;
    }

    // ゴールに向かって走る
    if (flag == 12) {
        if (rgb_total >= 600) {
            gyroPID->setTaget(270);
            gyro = gyroSensor->getAngle();
            pid = gyroPID->calcControl(gyro);
            pwm_L = 55 + pid;
            pwm_R = 55 - pid;
            armControl(ARM_ON);
        }
        else {
            flag = 50;
        }
    }

    /* EV3ではモーター停止時のブレーキ設定が事前にできないため */
    /* 出力0時に、その都度設定する */
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
    syslog(LOG_NOTICE, "V:%5d  G:%4d\r", volt, gyro);

    char bufg[64];
    sprintf(bufg, "G:%4d", gyro);
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
// 関数名 : armControl
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : アームの角度制御
//*****************************************************************************
static void armControl(int angle) {
    armPID->setTaget(angle);
    int pwm = (int)armPID->calcControl(armMotor->getCount()); /* PID制御 */
    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX) {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX) {
        pwm = -PWM_ABS_MAX;
    }

    armMotor->setPWM(pwm);
}
