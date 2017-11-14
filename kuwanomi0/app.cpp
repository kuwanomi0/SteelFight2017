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
#include "balancer.h"
#include "BalancerCpp.h"
#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"
#include "PID.h"
#include "Distance.h"
#include "Course.h"
#include <string.h>

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
/* 走行に関するマクロ */
#define GYRO_OFFSET          -1  /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define RGB_WHITE           160  /* 白色のRGBセンサの合計 */
#define RGB_BLACK            10  /* 黒色のRGBセンサの合計 */
#define RGB_TARGET          325  /*240 115*/ /*中央の境界線のRGBセンサ合計値 */
#define RGB_NULL              7  /* 何もないときのセンサの合計 */
#define PIDX               1.00  /* PID倍率 */
#define FORWARD_X          1.00  /* forward倍率 電源出力低下時にここで調整 */
#define KLP                 0.6  /* LPF用係数*/
#define GOOL_DISTANCE     11800  /* 難所の処理を有効にする距離 */

/* 超音波センサーに関するマクロ */
#define SONAR_ALERT_DISTANCE 20  /* 超音波センサによる障害物検知距離[cm] */

/* 尻尾に関するマクロ */
#define TAIL_ANGLE_STAND_UP   97 /* 完全停止時の角度[度] */
#define TAIL_ANGLE_ROKET      99 /* ロケットダッシュ時の角度[度] */
#define TAIL_ANGLE_DRIVE       3 /* バランス走行時の角度[度] */
#define TAIL_ANGLE_STOP       87 /* 停止処理時の角度[度] */
#define KP_TAIL             2.7F /* 尻尾用定数P */
#define KI_TAIL            0.02F /* 尻尾用定数I */
#define KD_TAIL            14.0F /* 尻尾用定数D */
#define PWM_ABS_MAX           60 /* 完全停止用モータ制御PWM絶対最大値 */
#define LOOK_UP_COLOR         28 /* ルックアップゲート用尻尾だしトレース */
#define STAIRS_COLOR          40 /* 階段用尻尾だしトレース */

/* サウンド */
#define NOTE_C4 (261.63)
#define NOTE_B6 (1975.53)
#define MY_SOUND_MANUAL_STOP (100)
#define VOLUME 50
#define TONE NOTE_C4

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/* magic number*/)
#define CALIB_FONT_HEIGHT (8/* magic number*/)

/* 関数プロトタイプ宣言 */
static int32_t sonar_alert(void);
static void tail_control(int32_t angle);
static void run_result(void);
static void balance(int8_t forward, int8_t turn, int32_t gyro, int32_t motor_ang_r, int32_t motor_ang_l, int32_t volt);
static void setCourse(int device_num, Course* gCourseR, Course* gCourseL);
static void BTconState();
static int RoboIdentification();

/* オブジェクトへのポインタ定義 */
TouchSensor*    touchSensor;
SonarSensor*    sonarSensor;
ColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
Motor*          leftMotor;
Motor*          rightMotor;
Motor*          tailMotor;
Clock*          clock;
Clock*          clock_gate;

/* インスタンスの生成 */
Balancer balancer;
PID pid_walk(      0,       0,       0); /* 走行用のPIDインスタンス */
PID pid_tail(KP_TAIL, KI_TAIL, KD_TAIL); /* 尻尾用のPIDインスタンス */
Distance distance_way;

/* Lコース */
static Course gCourseL[] {  // TODO 2: コース関連 だいぶ改善されました これで30.36secでた。
    { 0,     0, 22,  0, 0.0500F, 0.0000F, 1.2000F }, //スタート
    { 1,  2000,112,  0, 0.1500F, 0.0001F, 2.2000F }, //大きく右
    { 2,  3927,115,  0, 0.1300F, 0.0002F, 1.7000F }, //左
    { 3,  4754,121,  0, 0.0700F, 0.0000F, 1.6000F }, //直
    { 4,  5209,115,  0, 0.1150F, 0.0002F, 1.8000F }, //左
    { 5,  6134,122,  0, 0.0800F, 0.0000F, 1.6000F }, //直
    { 6,  6674,115,  0, 0.1300F, 0.0002F, 2.0000F }, //左
    { 7,  7562,110,  0, 0.1800F, 0.0002F, 1.9000F }, //右
    { 8,  8800,122,  0, 0.0450F, 0.0000F, 1.6000F }, //直GOOLまで
    { 9, 10030,122,  0, 0.0000F, 0.0000F, 0.0000F }, //灰
    {10, 10351, 80,  0, 0.1150F, 0.0002F, 1.5000F }, //左
    {11, 11000, 30,  0, 0.1150F, 0.0002F, 1.5000F }, //左
    {12, 11476, 30,  0, 0.0200F, 0.0000F, 0.2000F }, //灰
    {13, 11766, 30,  0, 0.1900F, 0.0000F, 1.4000F }, //階段
    {99, 99999,  1,  0, 0.0000F, 0.0000F, 0.0000F }  //終わりのダミー
};

/* Rコース */
static Course gCourseR[]  {  //TODO :2 コース関連 だいぶ改善されました これで31.25secでた
    { 0,     0, 22,  0, 0.1400F, 0.0000F, 1.7000F }, //スタート
    { 1,  2240,113,  0, 0.1600F, 0.0002F, 1.6900F }, //大きく右
    { 2,  5400,113,  0, 0.1500F, 0.0001F, 1.8000F }, //左やや直進
    { 3,  6350,110,  0, 0.1660F, 0.0002F, 1.4500F }, //強く左
    { 4,  7150,110,  0, 0.1600F, 0.0002F, 1.6000F }, //緩やかに大きく右
    { 5,  8750,122,  0, 0.1500F, 0.0000F, 1.5000F }, //直GOOLまで
    { 6, 10380,110,  2, 0.0000F, 0.0000F, 0.0000F }, //直GOOLまで
    { 7, 10475, 10,  0, 0.0000F, 0.0000F, 0.0000F }, //直GOOLまで
    { 8, 10550, 80,  0, 0.1200F, 0.0002F, 1.5000F }, //左
    { 9, 11900, 20,  0, 0.0000F, 0.0000F, 0.0000F }, //灰
    {10, 12150, 10,  0, 0.1200F, 0.0002F, 0.6000F }, //ルックアップ
    {99, 99999,  1,  0, 0.0000F, 0.0000F, 0.0000F }  //終わりのダミー
};

/* デフォルト */
static Course gCourse[] {
    { 0,     0, 30,  0, 0.1900F, 0.0001F, 1.4000F }, //スタート
    { 1, 99999,  1,  0, 0.0000F, 0.0000F, 0.0000F } //終わりのダミー
};

/* タイム格納用 */
static int time[2][100];
static int lapTime_count = 0;

/* 走行距離 */
static int32_t distance_now; /*現在の走行距離を格納する変数 */

/* メインタスク */
void main_task(intptr_t unused)
{
    int8_t    forward;      /* 前後進命令 */
    int8_t    turn;         /* 旋回命令 */
    rgb_raw_t rgb_level;    /* カラーセンサーから取得した値を格納する構造体 */
    int course_number = 0; //TODO :2 コース関連 だいぶ改善されました
    int count = 0;  //TODO :2 コース関連 だいぶ改善されました
    int roket = 0;  //TODO :3 ロケットスタート用変数 タイマーの役割をしています
    int tail_i = 0; //TODO :4 停止用コマンド
    int forward_course = 50; //TODO :2 コース関連 だいぶ改善されました
    int turn_course = 0; //TODO :2 コース関連 だいぶ改善されました
    uint16_t rgb_total = RGB_TARGET;
    uint16_t rgb_before;
    int gyro_flag = 0;
    int gyro_wait = 0;
    int8_t tail_flags = 0;
    int stairs = 1;
    int8_t garage = 0;
    int garage_cnt = 0;
    int distance_tmp = 0;
    int8_t hard_flag = 0;
    Course* mCourse = NULL;

    /* 各オブジェクトを生成・初期化する */
    touchSensor = new TouchSensor(PORT_1);
    sonarSensor = new SonarSensor(PORT_2);
    colorSensor = new ColorSensor(PORT_3);
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new Motor(PORT_C);
    rightMotor  = new Motor(PORT_B);
    tailMotor   = new Motor(PORT_A);
    clock       = new Clock();
    clock_gate  = new Clock();

    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET 16JZ", 0, CALIB_FONT_HEIGHT*1);
    ev3_lcd_draw_string("             M", 0, CALIB_FONT_HEIGHT*2);

    /* 尻尾モーターのリセット */
    for(int i = 0; i < 300; i++){
        tailMotor->setPWM(-3);
        clock->wait(1);
    }
    tailMotor->reset();

    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth通信タスクの起動 */
    act_tsk(BT_TASK);

    int device_num = RoboIdentification();

    setCourse(2, gCourseR, gCourseL);
    if (1 <= device_num && device_num <= 2) {
        setCourse(device_num, gCourseR, gCourseL);
    }

    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    /* スタート待機 */
    double angle = (double)TAIL_ANGLE_STAND_UP;
    int rotation_flag = 0;
    int run_flag = 0;
    int ctlspeed;
    while(1)
    {
        tail_control(angle); /* 完全停止用角度に制御、調整も可 */

        /* Lコース */
        if (bt_cmd == 1)
        {
            mCourse = gCourseL;
            break; /* リモートスタート */
        }
        /* Rコース */
        if (bt_cmd == 2)
        {
            mCourse = gCourseR;
            break; /* リモートスタート */
        }
        /* デフォルコース */
        if (touchSensor->isPressed() || bt_cmd == 3)
        {
            mCourse = gCourse;
            break; /* タッチセンサが押された */
        }

        /* ラジコン操作 */
        if (bt_cmd == 'd' && rotation_flag == 0) {
            ev3_led_set_color(LED_ORANGE);
            leftMotor->setPWM(18);
            rightMotor->setPWM(-17);
            bt_cmd = 0;
            rotation_flag = 1;
        }
        if (bt_cmd == 'a' && rotation_flag == 0) {
            ev3_led_set_color(LED_ORANGE);
            leftMotor->setPWM(-18);
            rightMotor->setPWM(17);
            bt_cmd = 0;
            rotation_flag = 1;
        }
        if (bt_cmd == 'w' && rotation_flag == 0) {
            ev3_led_set_color(LED_ORANGE);
            leftMotor->setPWM(18);
            rightMotor->setPWM(18);
            bt_cmd = 0;
            rotation_flag = 1;
        }
        if (bt_cmd == 's' && rotation_flag == 0) {
            ev3_led_set_color(LED_ORANGE);
            leftMotor->setPWM(-18);
            rightMotor->setPWM(-18);
            bt_cmd = 0;
            rotation_flag = 1;
        }
        if ((bt_cmd == 'd' ||
             bt_cmd == 'a' ||
             bt_cmd == 'w' ||
             bt_cmd == 's') && rotation_flag == 1) {
            ev3_led_set_color(LED_GREEN);
            leftMotor->setPWM(0);
            rightMotor->setPWM(0);

            bt_cmd = 0;
            rotation_flag = 0;
        }


        // スタート前の尻尾調整
        if (ev3_button_is_pressed(DOWN_BUTTON) || bt_cmd == ']') {
            angle -= 0.1;
            bt_cmd = 0; // コマンドリセット
            syslog(LOG_NOTICE, "DEBUG, angle : %d, RealAngle : %d\r", (int)angle, tailMotor->getCount());
        }
        if (ev3_button_is_pressed(UP_BUTTON) || bt_cmd == '[') {
            angle += 0.1;
            bt_cmd = 0; // コマンドリセット
            syslog(LOG_NOTICE, "DEBUG, angle : %d, RealAngle : %d\r", (int)angle, tailMotor->getCount());
        }

        if (bt_cmd == '@') {
            syslog(LOG_NOTICE, "DEBUG, angle : %d, RealAngle : %d\r", (int)angle, tailMotor->getCount());
        }

        // 回転
        if (bt_cmd == 9 && rotation_flag == 0) {
            syslog(LOG_NOTICE, "DEBUG, 回転\r");
            leftMotor->setPWM(18);
            rightMotor->setPWM(-18);
            bt_cmd = 0;
            rotation_flag = 1;
        }
        if (bt_cmd == 9 && rotation_flag == 1) {
            syslog(LOG_NOTICE, "DEBUG, 回転停止\r");
            leftMotor->setPWM(0);
            rightMotor->setPWM(0);
            bt_cmd = 0;
            rotation_flag = 0;
        }

        // ラジコン操作
        if (bt_cmd == 't' && run_flag == 0) {
            syslog(LOG_NOTICE, "DEBUG, 前進\r");
            leftMotor->setPWM(10);
            rightMotor->setPWM(10);
            bt_cmd = 0;
            run_flag = 1;
        }
        if (bt_cmd == 't' && run_flag == 1) {
            syslog(LOG_NOTICE, "DEBUG, 前進停止\r");
            for (int i = 50; i >= 0; i--) {
                leftMotor->setPWM(i);
                rightMotor->setPWM(i);
            }
            bt_cmd = 0;
            run_flag = 0;
        }
        // ラジコン操作2
        if (bt_cmd == 'w') {    // 前進
            ctlspeed = 10;  // ラジコンのスピード
            while (ctlspeed >= 0) {
                leftMotor->setPWM(ctlspeed);
                rightMotor->setPWM(ctlspeed);
                ctlspeed--;
                // 以下遅延処理
                clock->reset();
                while (clock->now() < 500) {
                    // 遅延
                }
            }
            bt_cmd = 0;
        }
        if (bt_cmd == 's') {    // 後進
            ctlspeed = -10;  // ラジコンのスピード
            while (ctlspeed <= 0) {
                leftMotor->setPWM(ctlspeed);
                rightMotor->setPWM(ctlspeed);
                ctlspeed++;
                // 以下遅延処理
                clock->reset();
                while (clock->now() < 500) {
                    // 遅延
                }
            }
            bt_cmd = 0;
        }
        if (bt_cmd == 'd') {
            turn = 10;
            clock->reset();
            while (clock->now() < 500) {
                // 何もしない
            }
            turn = 0;
            bt_cmd = 0;
        }
        if (bt_cmd == 'a') {
            turn = -10;
            clock->reset();
            while (clock->now() < 500) {
                // 何もしない
            }
            turn = 0;
            bt_cmd = 0;
        }

        BTconState();

        clock->sleep(10); /* 10msecウェイト */
    }

    /* 走行モーターエンコーダーリセット */
    leftMotor->reset();
    rightMotor->reset();

    /* ジャイロセンサーリセット */
    gyroSensor->reset();
    balancer.init(GYRO_OFFSET); /* 倒立振子API初期化 */  // <1>

    ev3_led_set_color(LED_GREEN); /* スタート通知 */

    /**
    * Main loop for the self-balance control algorithm
    */
    int gray = 0;
    int radioCtl_enable = 0;
    int balancer_enable = 1;
    int tail_flag = 0;
    int tailCtl_enable = 0;
    while(1)
    {
        int32_t motor_ang_l, motor_ang_r;
        int32_t gyro, volt;

        /* 参照しているコース配列が切り替わったことをLEDで知らせます */
        if((course_number % 2) == 1) {  /* 奇数配列参照時、赤 */
            ev3_led_set_color(LED_RED);
        }
        else {                          /* 偶数配列参照時、緑 */
            ev3_led_set_color(LED_GREEN);
        }

        /* 尻尾の制御 */
        if (bt_cmd == 6) {  // TODO :4 停止用コマンド
        }
        else if (tailCtl_enable == 0) {
            // if文をすり抜ける
        }
        else if(roket++ < 25) {  //TODO :3 ロケットスタートと呼ぶにはまだ怪しい、改良必須
            tail_control(TAIL_ANGLE_ROKET); /* ロケット走行用角度に制御 */
        }
        else if(tail_flags == 1){
            tail_control(80); /* 階段時に使用する角度 */
        }
        else {
            tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */
        }

        rgb_before = rgb_total; //LPF用前処理
        colorSensor->getRawColor(rgb_level); /* RGB取得 */
        rgb_total = (rgb_level.r + rgb_level.g + rgb_level.b)  * KLP + rgb_before * (1 - KLP); //LPF

        /* バックボタン, 転倒時停止処理 */
        if (ev3_button_is_pressed(BACK_BUTTON) || rgb_total <= RGB_NULL) {
            run_result();
            break;
        }

        /* 現在の走行距離を取得 */
        distance_now = distance_way.distanceAll(leftMotor->getCount(), rightMotor->getCount());

        /* 灰色検知 */
        if ( 110 >= rgb_level.r && rgb_level.g <= 110 &&
         300 < (rgb_level.r + rgb_level.g + rgb_level.b) && hard_flag == 3) {
            gray = 1;
            ev3_speaker_set_volume(VOLUME);
            ev3_speaker_play_tone(TONE, MY_SOUND_MANUAL_STOP);
        }
        else {
            gray = 0;
        }
        /* 灰色検知後のガレージのための処理 */
        if (gray == 1) {
                turn = 0;
                garage = 1;
                distance_tmp = distance_now;
                ev3_led_set_color(LED_ORANGE);
        }

        /* 区間変更を監視、行うプログラム */
        if (distance_now >= mCourse[count].getDis()) { //TODO :2 コース関連 だいぶ改善されました ここがまだ改良できる
            course_number  = mCourse[count].getCourse_num();
            forward_course = mCourse[count].getForward();
            turn_course    = mCourse[count].getTurn();
            pid_walk.setPID(mCourse[count].getP() * PIDX, mCourse[count].getI() * PIDX, mCourse[count].getD() * PIDX);
            count++;
        }
        if (distance_now >= 10000) {
            hard_flag = 1;
        }

/*========================ゲートをくぐる=========================*/
        if (sonar_alert() == 1 && hard_flag >= 1) {/* 障害物検知 */
            forward = turn = 0; /* 障害物を検知したら停止 */
            ev3_speaker_set_volume(VOLUME);
            ev3_speaker_play_tone(TONE, MY_SOUND_MANUAL_STOP);

            for (int angle = 79; angle >= 70; angle--)
            {
                if (ev3_button_is_pressed(BACK_BUTTON)) break;
                if(angle >= 77){
                    leftMotor->setPWM(16);
                    rightMotor->setPWM(16);
                }else{
                    leftMotor->setPWM(0);
                    rightMotor->setPWM(0);
                }
                clock->reset();
                clock->sleep(1);
                while (clock->now() <= 400){
                    tail_control(angle);
                    clock->sleep(4);
                }
            }
            /* 前進一回目のくぐり */
            clock->reset();
            clock->sleep(1);
            while (clock->now() <= 7200) {
                leftMotor->setPWM(4);
                rightMotor->setPWM(4);
                tail_control(68);
            }
            /* 一度停止して尻尾の調整 */
            clock->reset();
            clock->sleep(1);
            while (clock->now() <= 2000) {
                leftMotor->setPWM(0);
                rightMotor->setPWM(0);
                tail_control(68);
            }
            /* バックしてくぐる */
            clock->reset();
            clock->sleep(1);
            while (clock->now() <= 15000) {
                leftMotor->setPWM(-2);
                rightMotor->setPWM(-2);
                tail_control(68);
            }
            /* 前進して2回目のくぐり */
            clock->reset();
            clock->sleep(1);
            while (clock->now() <= 9000) {
                int pwmL, pwmR;
                colorSensor->getRawColor(rgb_level); /* RGB取得 */
                pwmL = 5 + (rgb_level.r - LOOK_UP_COLOR) * 0.4;
                pwmR = 5 + (LOOK_UP_COLOR - rgb_level.r) * 0.4;
                // syslog(LOG_NOTICE, "RED:%3d\r", rgb_level.r);
                leftMotor->setPWM(pwmL);
                rightMotor->setPWM(pwmR);
                tail_control(68);
            }

            /* ここから起き上がり */
            clock->reset();
            clock->sleep(1);
            while (clock->now() <= 200) {
                leftMotor->setPWM(-20);
                rightMotor->setPWM(-20);
                tail_control(73);
            }
            clock->reset();
            clock->sleep(1);
            while (clock->now() <= 1000) {
                leftMotor->setPWM(0);
                rightMotor->setPWM(0);
                tail_control(78);
            }
            clock->reset();
            clock->sleep(1);
            while (clock->now() <= 1000) {
                leftMotor->setPWM(0);
                rightMotor->setPWM(0);
                tail_control(85);
            }
            clock->reset();
            clock->sleep(1);
            while (clock->now() <= 1000) {
                leftMotor->setPWM(0);
                rightMotor->setPWM(0);
                tail_control(90);
            }
            clock->reset();
            clock->sleep(1);
            while (clock->now() <= 1000) {
                leftMotor->setPWM(0);
                rightMotor->setPWM(0);
                tail_control(92);
            }
            clock->reset();
            clock->sleep(1);
            while (clock->now() <= 1000) {
                leftMotor->setPWM(0);
                rightMotor->setPWM(0);
                tail_control(93);
            }
            clock->reset();
            clock->sleep(1);
            while (clock->now() <= 1000) {
                leftMotor->setPWM(0);
                rightMotor->setPWM(0);
                tail_control(95);
            }
            clock->reset();
            clock->sleep(1);
            while (clock->now() <= 1000) {
                leftMotor->setPWM(0);
                rightMotor->setPWM(0);
                tail_control(96);
            }
            while (clock->now() <= 100) {
                leftMotor->setPWM(0);
                rightMotor->setPWM(0);
                tail_control(96);
            }

            /*
                ここから走行情報のリセット
                これをしないと今の段階では倒立制御ができずすぐに倒れる
            */
            /* 走行モーターエンコーダーリセット */
            leftMotor->reset();
            rightMotor->reset();

            /* ジャイロセンサーリセット */
            gyroSensor->reset();
            balancer.init(GYRO_OFFSET); /* 倒立振子API初期化 */
            hard_flag = 3;
        }
        else {
            if (bt_cmd == 6) //TODO 4: おまけコマンド停止処理用
            {
                forward = 0; //TODO 4: おまけコマンド停止処理用
            }
            else {
                forward = forward_course * FORWARD_X; /* 前進命令 */
            }
            /* PID制御 */
            turn = pid_walk.calcControl(RGB_TARGET - rgb_total) + turn_course;
        }

        /* 倒立振子制御API に渡すパラメータを取得する */
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        /* ガレージ処理 */
        // if (garage == 1) { /* テスト用 */
        if (garage == 1) {
            if (distance_now - distance_tmp < 220) {
                syslog(LOG_NOTICE, "--- garage ---\r");
                forward = 10;
            }
            else if (garage_cnt <= 240/4) {
                syslog(LOG_NOTICE, " S T O P\r");
                forward = -2;
                turn = 0;
                garage_cnt++;
            }
            else {
                syslog(LOG_NOTICE, " S T O P  S T O P  S T O P\r");
                forward = -40;
                turn = 0;
                bt_cmd = 6;
            }
        }


        if (gyro_wait >= 3000/4) {
            gyro_wait = 0;
        }
        // TODO :KAIDAN
        // syslog(LOG_NOTICE, "KAIDAN : %d\r", stairs);
        if (gyro_wait == 0 && (gyro >= 70 || gyro_flag >= 1) && roket >= 45 && hard_flag == 1) {
            gyro_flag++;
            if(gyro_flag <= 300/4) {
                forward = 90;
                if (stairs == 2) {
                    forward = 100;
                    turn = 0;
                }
            }
            else if(gyro_flag <= 2000/4) {
                forward = 0;
                if (stairs == 2) {
                    forward = -2;
                    turn = 0;
                }
                tail_flags = 1;
            }
            else if(gyro_flag <= 4000/4) {
                forward = -3;
                turn = 0;
                if (stairs == 2) {
                    forward = -2;
                    turn = 0;
                }
            }
            else if (gyro_flag <= 6000/4) {
                forward = 0;
                if (stairs == 2) {
                    turn = 0;
                }
            }
            else if (gyro_flag > 6000/4) {
                clock->reset();
                clock->sleep(1);
                while (clock->now() <= 100) {
                    leftMotor->setPWM(20);
                    rightMotor->setPWM(20);
                    tail_control(80);
                }
                clock->reset();
                clock->sleep(1);
                while (clock->now() <= 1000) {
                    leftMotor->setPWM(0);
                    rightMotor->setPWM(0);
                    tail_control(80);
                }
                clock->reset();
                clock->sleep(1);
                while (clock->now() <= 250) {
                    leftMotor->setPWM(-15);
                    rightMotor->setPWM(-15);
                    if (stairs == 2) {
                        leftMotor->setPWM(10);
                        rightMotor->setPWM(10);
                    }
                    tail_control(80);
                }
                clock->reset();
                clock->sleep(1);
                while (clock->now() <= 1000) {
                    leftMotor->setPWM(80);
                    rightMotor->setPWM(-80);
                    tail_control(80);
                }
                if (stairs ==2) {
                    clock->reset();
                    clock->sleep(1);
                    while (clock->now() <= 750) {
                        leftMotor->setPWM(80);
                        rightMotor->setPWM(-80);
                        tail_control(80);
                    }
                }
                if (stairs == 1) {
                    clock->reset();
                    clock->sleep(1);
                    while (clock->now() <= 1000) {
                        leftMotor->setPWM(-10);
                        rightMotor->setPWM(-10);
                        tail_control(80);
                    }
                    clock->reset();
                    clock->sleep(1);
                    while (clock->now() <= 2000) {
                        int pwmL, pwmR;
                        colorSensor->getRawColor(rgb_level); /* RGB取得 */
                        pwmL = 5 + (rgb_level.r - STAIRS_COLOR) * 0.7;
                        pwmR = 5 + (STAIRS_COLOR - rgb_level.r) * 0.7;
                        // syslog(LOG_NOTICE, "RED:%3d\r", rgb_level.r);
                        leftMotor->setPWM(pwmL);
                        rightMotor->setPWM(pwmR);
                        tail_control(80);
                    }
                }

                /* ここから起き上がり */
                clock->reset();
                clock->sleep(1);
                while (clock->now() <= 1500) {
                    leftMotor->setPWM(-5);
                    rightMotor->setPWM(-5);
                    tail_control(80);
                }
                clock->reset();
                clock->sleep(1);
                while (clock->now() <= 450) {
                    leftMotor->setPWM(-10);
                    rightMotor->setPWM(-10);
                    if (stairs == 2) {
                        leftMotor->setPWM(-12);
                        rightMotor->setPWM(-12);
                    }
                    tail_control(85);
                }
                clock->reset();
                clock->sleep(1);
                while (clock->now() <= 600) {
                    leftMotor->setPWM(0);
                    rightMotor->setPWM(0);
                    tail_control(90);
                }
                clock->reset();
                clock->sleep(1);
                while (clock->now() <= 2000) {
                    leftMotor->setPWM(0);
                    rightMotor->setPWM(0);
                    tail_control(93);
                }
                clock->reset();
                clock->sleep(1);
                while (clock->now() <= 2000) {
                    leftMotor->setPWM(0);
                    rightMotor->setPWM(0);
                    tail_control(95);
                }
                clock->reset();
                clock->sleep(1);
                while (clock->now() <= 300) {
                    leftMotor->setPWM(-1);
                    rightMotor->setPWM(-1);
                    if (stairs == 1) {
                        tail_control(96);
                    }
                    else {
                        tail_control(96);
                    }
                }
                gyro = gyroSensor->getAnglerVelocity();
                gyro_flag = 0;
                if (stairs == 1) {
                    gyro_wait++;
                    stairs = 2;
                }
                else if (stairs == 2) {
                    stairs = 3;
                    hard_flag = 2;
                    tail_flags = 0;
                }
            }
        }
        else if (gyro_wait != 0) {
            forward = 0;
            gyro_wait++;
        }
        if (stairs >= 3) {
            turn = -turn;
        }

        if(stairs == 3 ) {
            forward = 0;
            turn = -3;
            if (RGB_TARGET >= rgb_total) {
                stairs = 4;
            }
        }
        else if (4 <= stairs && stairs < 5000/4) {
            forward = 0;
            stairs++;
        }
        else if (5000/4 <= stairs && stairs < 7000/4) {
            forward = 70;
            turn = 2;
            stairs++;
        }
        else if(7000/4 <= stairs) {
            mCourse[count].setForward(10);
            hard_flag = 3;
            stairs = 0;
        }

        if (bt_cmd == 'q' && radioCtl_enable == 0) {
            radioCtl_enable = 1;
        }
        else if (bt_cmd == 'q' && radioCtl_enable == 1) {
            radioCtl_enable = 0;
        }
        if (radioCtl_enable == 1) {
            forward = 0;
            turn = 0;
        }
        if (radioCtl_enable == 1) {
            syslog(LOG_NOTICE, "%d,%d,%d,%d,%d,%d,%d\r", rgb_level.r, rgb_level.g, rgb_level.b, rgb_level.r + rgb_level.g + rgb_level.b, distance_now, gyro, volt);
            /* 遅いラジコン */
            if (bt_cmd == 'w') {
                forward = 50;
                turn = 0;
            }
            if (bt_cmd == 's') {
                forward = -50;
                turn = 0;
            }
            if (bt_cmd == 'a') {
                forward = 0;
                turn = -10;
            }
            if (bt_cmd == 'd') {
                forward = 0;
                turn = 10;
            }
            /* 速いラジコン */
            if (bt_cmd == 'W') {
                forward = -150;
                turn = 0;
            }
            if (bt_cmd == 'S') {
                forward = 150;
                turn = 0;
            }
            if (bt_cmd == 'A') {
                forward = 0;
                turn = -30;
            }
            if (bt_cmd == 'D') {
                forward = 0;
                turn = 30;
            }
            if (bt_cmd == ' ') {
                forward = 0;
                turn = 0;
            }
            /* ラジコン尻尾 */
            if (bt_cmd == '[') {
                tail_control(tailMotor->getCount() + 1);
            }
            if (bt_cmd == ']') {
                tail_control(tailMotor->getCount() - 1);
            }
            /* ラジコン尻尾出し入れ */
            if (bt_cmd == 'n' && tail_flag == 0) {
                tail_control(TAIL_ANGLE_STOP);
                tail_flag = 1;
            }
            if (bt_cmd == 'n' && tail_flag == 1) {
                tail_control(TAIL_ANGLE_DRIVE);
                tail_flag = 0;
                tailCtl_enable = 1;
            }
            /* バランサー切り替え */
            if (balancer_enable == 1 && bt_cmd == 'b') {
                balancer_enable = 0;
            }
            if (balancer_enable == 0 && bt_cmd == 'b') {
                balancer_enable = 1;
            }
        }

        if (balancer_enable == 1) {
            /* 倒立振子制御APIを呼び出し、倒立走行するための */
            /* 左右モータ出力値を得る */
            balance(forward, turn, gyro, motor_ang_r, motor_ang_l, volt);
        }


        /* ログを送信する処理 */
        if (bt_cmd == 1 || gray == 1)
        {
            syslog(LOG_NOTICE, "C:%2d, D:%5d, G:%3d, V:%5d, RGB%3d\r", course_number, distance_now, gyro, volt, rgb_total);
            bt_cmd = 0;
        }

        // TODO :4 停止用コマンド
        if (bt_cmd == 6)
        {
            if (tail_i++ < 500/4) {
                tail_control(TAIL_ANGLE_STOP);
            }
            else {
                break;
            }
        }

        BTconState();

        clock->sleep(4); /* 4msec周期起動 */
    }
    leftMotor->reset();
    rightMotor->reset();
    tailMotor->reset();


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
static int32_t sonar_alert(void)
{
    static uint32_t counter = 0;
    int32_t alert = 0;

    int32_t distance;

    if (++counter == 4/4) /* 約40msec周期毎に障害物検知  */
    {
        /*
         * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
         * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
         * EV3の場合は、要確認
         */
        distance = sonarSensor->getDistance();
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
        {
            alert = 1; /* 障害物を検知 */
        }
        else
        {
            alert = 0; /* 障害物無し */
        }
        counter = 0;
    }

    return alert;
}


//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
static void tail_control(int32_t angle)
{
    int pwm = (int)pid_tail.calcControl(angle - tailMotor->getCount()); /* PID制御 */
    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }

    tailMotor->setPWM(pwm);
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
        case 'l':
            bt_cmd = 1;
            break;
        case '2':
        case 'r':
            bt_cmd = 2;
            break;
        case '3':
            bt_cmd = 3;
            break;
        case '6':
            bt_cmd = 6;
            break;
        case '9':
            bt_cmd = 9;
            break;
        case '[':   // 上
            bt_cmd = '[';
            break;
        case ']':   // 下
            bt_cmd = ']';
            break;
        case 't':
            bt_cmd = 't';
            break;
        case 'w':
            bt_cmd = 'w';
            break;
        case 's':
            bt_cmd = 's';
            break;
        case 'd':
            bt_cmd = 'd';
            break;
        case 'a':
            bt_cmd = 'a';
            break;
        case 'W':
            bt_cmd = 'W';
            break;
        case 'S':
            bt_cmd = 'S';
            break;
        case 'D':
            bt_cmd = 'D';
            break;
        case 'A':
            bt_cmd = 'A';
            break;
        case '@':
            bt_cmd = '@';
            break;
        case '\r':
            bt_cmd = '\r';
            break;
        case 'q':
            bt_cmd = 'q';
            break;
        case ' ':
            bt_cmd = ' ';
            break;
        case 'b':
            bt_cmd = 'b';
            break;
        case 'n':
            bt_cmd = 'n';
            break;
        default:
            break;
        }
        if (!(bt_cmd == '[' || bt_cmd == ']')) {    // TODO uとdのときはエコーバックしないようにしたい。未完成
            fputc(c, bt); /* エコーバック */
        }
        if (bt_cmd == 1 || bt_cmd == 2) {
            clock->reset();
        }
        else if (bt_cmd == '\r' && lapTime_count < 100) {
            syslog(LOG_NOTICE, "DEBUG, TIME : //////////////////////////////\r");
            syslog(LOG_NOTICE, "DEBUG, TIME : %d.%03d s , 距離 : %d.%d m\r", clock->now() / 1000, clock->now() % 1000, distance_now / 1000, distance_now % 1000);
            syslog(LOG_NOTICE, "DEBUG, TIME : //////////////////////////////\r");
            time[0][lapTime_count] = clock->now();
            time[1][lapTime_count] = distance_now;
            lapTime_count++;
        }
    }
}

//*****************************************************************************
// 関数名 : run_result
// 引数 : unused
// 返り値 : なし
// 概要 : 走行結果を表示する
//*****************************************************************************
static void run_result() {
    if (lapTime_count > 0) {
        syslog(LOG_NOTICE, "DEBUG, TIME --------------------\r");
        for (int i = 0; i < lapTime_count; i++) {
            syslog(LOG_NOTICE, "TIME(%3d) : %d.%03d s , 距離 : %d.%03d m",i + 1 , time[0][i] / 1000, time[0][i] % 1000, time[1][i] / 1000, time[1][i] % 1000);
            syslog(LOG_NOTICE, "(%2d cm/s)\r",  (time[1][i] / 10) / (time[0][i] / 1000));
        }
        syslog(LOG_NOTICE, "DEBUG, TIME --------------------\r");
    }
}

//*****************************************************************************
// 関数名 : balance
// 引数 : balancer, forward, turn, gyro, motor_ang_r, motor_ang_l, volt
// 返り値 : なし
// 概要 : バランス走行制御を行う
//*****************************************************************************
static void balance(int8_t forward, int8_t turn, int32_t gyro, int32_t motor_ang_r, int32_t motor_ang_l, int32_t volt) {
    int8_t    pwm_L, pwm_R;
    balancer.setCommand(forward, turn);
    balancer.update(gyro, motor_ang_r, motor_ang_l, volt);
    pwm_L = balancer.getPwmRight();
    pwm_R = balancer.getPwmLeft();

    leftMotor->setPWM(pwm_L);
    rightMotor->setPWM(pwm_R);
}

//*****************************************************************************
// 関数名 : setCourse
// 引数 : device_num, gCourseR, gCourseL
// 返り値 : なし
// 概要 : 走行コース配列セット（筐体ごとに変更）
//*****************************************************************************
static void setCourse(int device_num, Course* gCourseR, Course* gCourseL) {
    /* Lコース（No.1） */
    static Course CourseL1[] {
        { 0,     0,  1,  0, 0.0500F, 0.0000F, 1.2000F }, //スタート
        { 1,  2000,112,  0, 0.1500F, 0.0001F, 2.2000F }, //大きく右
        { 2,  3927,115,  0, 0.1300F, 0.0002F, 1.7000F }, //左
        { 3,  4754,121,  0, 0.0700F, 0.0000F, 1.6000F }, //直
        { 4,  5209,115,  0, 0.1150F, 0.0002F, 1.8000F }, //左
        { 5,  6134,122,  0, 0.0800F, 0.0000F, 1.6000F }, //直
        { 6,  6674,115,  0, 0.1300F, 0.0002F, 2.0000F }, //左
        { 7,  7562,110,  0, 0.1800F, 0.0002F, 1.9000F }, //右
        { 8,  8800,122,  0, 0.0450F, 0.0000F, 1.6000F }, //直GOOLまで
        { 9, 10030,122,  0, 0.0000F, 0.0000F, 0.0000F }, //灰
        {10, 10351, 80,  0, 0.1150F, 0.0002F, 1.5000F }, //左
        {10, 11000, 30,  0, 0.1150F, 0.0002F, 1.5000F }, //左
        {11, 11476, 30,  0, 0.0200F, 0.0000F, 0.2000F }, //灰
        {12, 11766, 30,  0, 0.1900F, 0.0000F, 1.4000F }, //階段
        {99, 99999,  1,  0, 0.0000F, 0.0000F, 0.0000F }  //終わりのダミー
    };

    /* Rコース（No.1） */
    static Course CourseR1[]  {
        { 0,     0,  1,  0, 0.1000F, 0.0000F, 1.0000F }, //スタート
        { 1,  2200,106,  0, 0.1200F, 0.0002F, 1.4900F }, //大きく右
        { 2,  4200,106,  0, 0.1000F, 0.0001F, 1.5100F }, //大きく右
        { 3,  5400,108,  0, 0.1200F, 0.0001F, 1.3000F }, //左やや直進
        { 4,  6350,105,  0, 0.1260F, 0.0002F, 1.4500F }, //強く左
        { 5,  7150,106,  0, 0.1100F, 0.0002F, 1.5000F }, //緩やかに大きく右
        { 6,  8750,122,  0, 0.0500F, 0.0000F, 1.0000F }, //直GOOLまで
        { 7, 10380,110,  2, 0.0000F, 0.0000F, 0.0000F }, //直GOOLまで
        { 8, 10475, 10,  0, 0.0000F, 0.0000F, 0.0000F }, //直GOOLまで
        { 9, 10550, 80,  0, 0.1200F, 0.0002F, 1.5000F }, //左
        {10, 11900, 20,  0, 0.0000F, 0.0000F, 0.0000F }, //灰
        {11, 12150, 10,  0, 0.1200F, 0.0002F, 0.6000F }, //ルックアップ
        {99, 99999,  1,  0, 0.0000F, 0.0000F, 0.0000F }  //終わりのダミー
    };

    /* Lコース（No.2） */
    static Course CourseL2[] {
        { 0,     0,  1,  0, 0.0500F, 0.0000F, 1.2000F }, //スタート
        { 1,  2000,112,  0, 0.1500F, 0.0001F, 2.2000F }, //大きく右
        { 2,  3927,115,  0, 0.1300F, 0.0002F, 1.7000F }, //左
        { 3,  4754,121,  0, 0.0700F, 0.0000F, 1.6000F }, //直
        { 4,  5209,115,  0, 0.1150F, 0.0002F, 1.8000F }, //左
        { 5,  6134,122,  0, 0.0800F, 0.0000F, 1.6000F }, //直
        { 6,  6674,115,  0, 0.1300F, 0.0002F, 2.0000F }, //左
        { 7,  7562,110,  0, 0.1800F, 0.0002F, 1.9000F }, //右
        { 8,  8800,122,  0, 0.0450F, 0.0000F, 1.6000F }, //直GOOLまで
        { 9, 10030,122,  0, 0.0000F, 0.0000F, 0.0000F }, //灰
        {10, 10351, 80,  0, 0.1150F, 0.0002F, 1.5000F }, //左
        {10, 11000, 30,  0, 0.1150F, 0.0002F, 1.5000F }, //左
        {11, 11476, 30,  0, 0.0200F, 0.0000F, 0.2000F }, //灰
        {12, 11766, 30,  0, 0.1900F, 0.0000F, 1.4000F }, //階段
        {99, 99999,  1,  0, 0.0000F, 0.0000F, 0.0000F }  //終わりのダミー
    };

    /* Rコース（No.2） */
    static Course CourseR2[]  {
        { 0,     0,  1,  0, 0.1000F, 0.0000F, 1.0000F }, //スタート
        { 1,  2200,106,  0, 0.1200F, 0.0002F, 1.4900F }, //大きく右
        { 2,  4200,106,  0, 0.1000F, 0.0001F, 1.5100F }, //大きく右
        { 3,  5400,108,  0, 0.1200F, 0.0001F, 1.3000F }, //左やや直進
        { 4,  6350,105,  0, 0.1260F, 0.0002F, 1.4500F }, //強く左
        { 5,  7150,106,  0, 0.1100F, 0.0002F, 1.5000F }, //緩やかに大きく右
        { 6,  8750,122,  0, 0.0500F, 0.0000F, 1.0000F }, //直GOOLまで
        { 7, 10380,110,  2, 0.0000F, 0.0000F, 0.0000F }, //直GOOLまで
        { 8, 10475, 10,  0, 0.0000F, 0.0000F, 0.0000F }, //直GOOLまで
        { 9, 10550, 80,  0, 0.1200F, 0.0002F, 1.5000F }, //左
        {10, 11900, 20,  0, 0.0000F, 0.0000F, 0.0000F }, //灰
        {11, 12150, 10,  0, 0.1200F, 0.0002F, 0.6000F }, //ルックアップ
        {99, 99999,  1,  0, 0.0000F, 0.0000F, 0.0000F }  //終わりのダミー
    };

    int size = sizeof gCourseR / sizeof gCourseR[0];

    switch (device_num) {
        case 1:
            for (int i = 0; i < size; i++) {
                gCourseR[i] = CourseR1[i];
                gCourseL[i] = CourseL1[i];
            }
            break;
        case 2:
            for (int i = 0; i < size; i++) {
                gCourseR[i] = CourseR2[i];
                gCourseL[i] = CourseL2[i];
            }
            break;
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


//*****************************************************************************
// 関数名 : RoboIdentification
// 引数 : なし
// 返り値 : device_num（-1 : 不明、0以上 : ロボットの番号）
// 概要 : SDカード内の device.txt ファイルをもとにロボットを識別
//*****************************************************************************
static int RoboIdentification() {
    FILE *fp; // FILE型構造体
	// char fname[] = "ev3rt\etc\rc.conf.ini";
    char fname[] = "device.txt";
    char str[256];
    int device_num;
    char deviceList[][10] = {"ET49", "ET49-2"};
    int deviceListSize = sizeof deviceList / sizeof deviceList[0];

	fp = fopen(fname, "r");
    if (fp != NULL) {
        ev3_lcd_draw_string("OPEN  ", 0, CALIB_FONT_HEIGHT*4);
        while (fgets(str, 256, fp) != NULL) {
            ev3_lcd_draw_string(str, 0, CALIB_FONT_HEIGHT*4);
            int i = 0;
            while (!strcmp(str, deviceList[i]) || i < deviceListSize) {
                device_num = i;
                i++;
            }
            if (strcmp(str, deviceList[i])) {
                ev3_lcd_draw_string("??????", 0, CALIB_FONT_HEIGHT*4);
                device_num = -1;
            }
        }
    }
    else {
        ev3_lcd_draw_string("NoOPEN", 0, CALIB_FONT_HEIGHT*4);
    }

    return device_num;
}
