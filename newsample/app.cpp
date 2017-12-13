/**
 ******************************************************************************
 ** ファイル名 : app.cpp
 **
 ** 概要 : 2輪倒立振子ライントレースロボットのTOPPERS/HRP2用C++サンプルプログラム
 **
 ** 注記 : sample_cpp (ライントレース/尻尾モータ/超音波センサ/リモートスタート)
 * @version task_1.0 : 2017.06.28
 *    +ディレクトリ名をtouchguy に変更する
 *    +コントロールタスク追加し周期ハンドラで動かす
 ******************************************************************************
 */
#define VERSION "kuwanomi0_0.4"

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
#define RGB_WHITE           820/*800  /* 白色のRGBセンサの合計 */
#define RGB_BLACK           310 /*80  /* 黒色のRGBセンサの合計 */
#define RGB_TARGET          565/*436  /*90 570 875*/ /*中央の境界線のRGBセンサ合計値 */
#define KLP                 0.6  /* LPF用係数*/
#define FORWARD             15

/* 超音波センサーに関するマクロ */
#define SONAR_ALERT_DISTANCE 2  /* 超_音波センサによる障害物検知距離[cm] */

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/* magic number*/)
#define CALIB_FONT_HEIGHT (8/* magic number*/)

/* 関数プロトタイプ宣言 */
static int32_t sonar_alert(void);
static void BTconState();
static void armMotor_initialize(void);
static void armMotor_grab(void);
static void setAngle(int32_t angle);
static int16_t controlAngle(void);
static void controlAngle(int32_t angle);

/* オブジェクトへのポインタ定義 */
ColorSensor*    colorSensor;
SonarSensor*    sonarSensor;
GyroSensor*     gyroSensor;
Motor*          armMotor;
Motor*          leftMotor;
Motor*          rightMotor;
Clock*          clock;

/* インスタンスの生成 */
Distance distance_way;
PID pid_walk( 0.04F, 0.0F,   0.0F); /* 走行用のPIDインスタンス */
////////////0.04

/* 走行距離 */
static rgb_raw_t rgb_level;  /* カラーセンサーから取得した値を格納する構造体 */
static int32_t distance_now; /*現在の走行距離を格納する変数 */
static int8_t pwm_A = 0;     /* アームモータPWM出力 */
static int8_t pwm_L = 0;     /* 左モータPWM出力 */
static int8_t pwm_R = 0;     /* 右モータPWM出力 */
static uint16_t rgb_total = RGB_TARGET;
static uint16_t rgb_before;
static int8_t ishave = 0;
static int8_t sonartemp = 0;
//static int8_t forward;

static int32_t disEnabled = 0;  // 距離で動作（0 : 無効, 1 : 有効）

/* スタート（0）からの傾き */
static int16_t thisAngle = 0;

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
            clock->reset();
            armMotor_initialize();
            break;
        }

        if (ev3_button_is_pressed(RIGHT_BUTTON)) {
            // armMotor_initialize();
            armMotor_grab();
        }

        if (ev3_button_is_pressed(UP_BUTTON)) {
            // armMotor->setCount(-100);
            armMotor->setPWM(-10);
            syslog(LOG_NOTICE, "getCount: %d\r", armMotor->getCount());

        }
        if (ev3_button_is_pressed(DOWN_BUTTON)) {
            // armMotor->setCount(100);
            armMotor->setPWM(10);
        }
        if (ev3_button_is_pressed(LEFT_BUTTON)) {
            armMotor->stop();
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
    int32_t motor_ang_l, motor_ang_r;
    int32_t gyro, volt;
    int32_t pid = 0;
    int32_t pid_L = 0;
    int32_t pid_R = 0;

    pwm_A = 0;
    pwm_L = 0;
    pwm_R = 0;

    /* バックボタン */
    if (ev3_button_is_pressed(BACK_BUTTON) || bt_cmd == 0) {
        wup_tsk(MAIN_TASK);        //メインタスクを起床する
        ev3_stp_cyc(CYC_HANDLER);  //周期ハンドラを停止する
    }

    /* パラメータを取得する */
     motor_ang_l = leftMotor->getCount();
     motor_ang_r = rightMotor->getCount();
     gyro = gyroSensor->getAnglerVelocity();
     volt = ev3_battery_voltage_mV();

    /* 現在の走行距離を取得 */
    distance_way.Distance_update(motor_ang_l, motor_ang_r);
    distance_now = distance_way.Distance_getDistance();

    /* 色の取得 */
    rgb_before = rgb_total; //LPF用前処理
    colorSensor->getRawColor(rgb_level); /* RGB取得 */
    // int16_t gyroAngle = gyroSensor->getAngle();
    rgb_total = (rgb_level.r + rgb_level.g + rgb_level.b)  * KLP + rgb_before * (1 - KLP); //LPF

    pid = pid_walk.calcControl(((RGB_BLACK + RGB_WHITE) / 2) - rgb_total);

    if (pid < 0) {
        //黒の時
        pid_L = pid;
    }
    else {
        //pid_L = -pid
        pid_R = -pid;
    }

    pwm_L = /*(rgb_total - RGB_TARGET) *0.1*/ +  FORWARD + pid_L;
    pwm_R = /*(RGB_TARGET - rgb_total) *0.1*/ +  FORWARD + pid_R;

    /*if (pwm_L < 0 && 50 < pwm_L) {
        pwm_L = 10;
    }
    if (pwm_R < 0 && 50 < pwm_L) {
        pwm_R = 10;
    }*/

    //pwm_L = 10;
    //pwm_R = 10;
    //pwm_L = 0;
    //pwm_R = 0;
    //ペットボトル見つけたらいい感じに修正する。
    if(sonarSensor->getDistance() <= 45 && ishave == 0) {
        int8_t sonar_min = 255;
        sonartemp = sonarSensor->getDistance();//ソナーセンサーの値格納
        while (sonartemp >= 10) { //ペットボトルに近づくまで
            if (sonartemp < sonar_min + 1) {
                leftMotor->setPWM(5);//左に曲がる
                rightMotor->setPWM(20);
                ev3_led_set_color(LED_RED);
                tslp_tsk(100);
            }
            else {
                leftMotor->setPWM(20);//右に曲がる
                rightMotor->setPWM(5);
                ev3_led_set_color(LED_GREEN);
                tslp_tsk(100);

            }
            if (sonar_min > sonartemp) {//最短距離更新
                sonar_min = sonartemp;
            }

            sonartemp = sonarSensor->getDistance(); //値更新
        }

        leftMotor->setPWM(40);//直進
        rightMotor->setPWM(40);
        ishave = 1;
        tslp_tsk(500);
        leftMotor->setPWM(0);
        rightMotor->setPWM(0);
        armMotor_grab();
        tslp_tsk(1000);
        leftMotor->setPWM(-40);
        rightMotor->setPWM(40);
        tslp_tsk(500);
        while (rgb_total > 350) {
            colorSensor->getRawColor(rgb_level); /* RGB取得 */

            rgb_total = (rgb_level.r + rgb_level.g + rgb_level.b)  * KLP + rgb_before * (1 - KLP); //LPF
            leftMotor->setPWM(40);
            rightMotor->setPWM(40);
            tslp_tsk(300);
        }
        armMotor_initialize();

    }

    //90度曲がる
    //赤を見つけるまで直進
    //見つけたら少し進んでｐを置く
    //バックして右にに９０度


    // ショボいラジコン操作
    if (bt_cmd == 'a') {
        pwm_L = -10;
        pwm_R = 10;
    }
    if (bt_cmd == 'd') {
        pwm_L = 10;
        pwm_R = -10;
    }
    if (bt_cmd == 'w') {
        pwm_L = 30;
        pwm_R = 30;
    }
    if (bt_cmd == 's') {
        pwm_L = -30;
        pwm_R = -30;
    }
    if (bt_cmd == 'e') {
        pwm_L = 100;
        pwm_R = 100;
    }
    if (bt_cmd == 'A') {
        pwm_L = -30;
        pwm_R = 30;
    }
    if (bt_cmd == 'D') {
        pwm_L = 30;
        pwm_R = -30;
    }
    if (bt_cmd == 'W') {
        pwm_L = 100;
        pwm_R = 100;
    }
    if (bt_cmd == 'S') {
        pwm_L = -100;
        pwm_R = -100;
    }
    if (bt_cmd == 'm') {
        armMotor_initialize();
        bt_cmd = 'l';   // ダミー
    }
    if (bt_cmd == 'n') {
        armMotor_grab();
        bt_cmd = 'l';
    }
    if (bt_cmd == 'b') {
        controlAngle(90);
        bt_cmd = 'l';
    }
    if (bt_cmd == 'c') {
        ev3_speaker_play_tone(200.00, 40);
        bt_cmd = 'l';   // ダミー
    }
    if (bt_cmd == ' ') {
        pwm_L = 0;
        pwm_R = 0;
        pwm_A = 0;
    }
    if (bt_cmd == 4) {
        if (disEnabled == 0) {
            disEnabled = 1;
            ev3_speaker_play_tone(200.00, 40);
        }
        else {
            disEnabled = 0;
            ev3_speaker_play_tone(200.00, 40);
        }
        syslog(LOG_NOTICE, "disEnabled : %d", disEnabled);
        bt_cmd = 'l';   // ダミー
    }
    if (bt_cmd == 5) {
        pwm_A = 70;
        // setAngle(10);
        pwm_L = 0;
        pwm_R = 0;
    }
    if (bt_cmd == 6) {
        pwm_A = -70;
        // setAngle(0);
        pwm_L = 0;
        pwm_R = 0;
    }
    if (bt_cmd == 9) {
        pwm_L = 100;
        pwm_R = -100;
    }
    if (sonar_alert() == 1){ /* 障害物検知 */
        pwm_R = pwm_L = 0; /* 障害物を検知したら停止 */
    }
    if (bt_cmd == 3) {
        leftMotor->reset();
        rightMotor->reset();
        motor_ang_l = 0;
        motor_ang_r = 0;
    }

    // 0cm以下はストップ
    if (disEnabled && 0 > distance_now) {
        pwm_R = 0;
        pwm_L = 0;
        rightMotor->reset();
        leftMotor->reset();
    }
    // 55cmを超えたらストップ
    if (disEnabled && distance_now >= 600 && (pwm_R > 0 || pwm_L > 0)) {
        rightMotor->setPWM(0);
        leftMotor->setPWM(0);
        // pwm_R = 0;
        // pwm_L = 0;
        // armMotor_grab();
    }

    /* EV3ではモーター停止時のブレーキ設定が事前にできないため */
    /* 出力0時に、その都度設定する */
    if (pwm_A == 0) {
        armMotor->stop();
    } else {
        armMotor->setPWM(pwm_A);
    }

    if (pwm_L == 0) {
        leftMotor->stop();
    } else {
        leftMotor->setPWM(pwm_L);
    }

    if (pwm_R == 0) {
        rightMotor->stop();
    } else {
        rightMotor->setPWM(pwm_R);
    }

    /* ログを送信する処理 */
    //syslog(LOG_NOTICE, "V:%5d  G:%3d R%3d G:%3d B:%3d\r", volt, gyro, rgb_level.r, rgb_level.g, rgb_level.b);

    //int16_t gyroAngle = gyroSensor->getAngle();
    // syslog(LOG_NOTICE, "gyro: %d", gyroAngle);

    thisAngle = controlAngle();
    int sonarDistance = sonarSensor->getDistance();
    syslog(LOG_NOTICE, "sonarDistance: %d, distance_now: %d, rgb_total: %d, gyro: %d\r", sonarDistance, distance_now, rgb_total, gyroSensor->getAnglerVelocity());

    BTconState();

    ext_tsk();
}

//*****************************************************************************
// 関数名 : cyc_handler
// 引数   : 拡張情報
// 返り値 : なし
// 概要   : 周期ハンドラ(4ms)
//*****************************************************************************
void cyc_handler(intptr_t unused) {
    // コントローラタスクを起動する
    act_tsk(CONTROLLER_TASK);
}
//*****************************************************************************
// 関数名 : sonar_alert
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
static int32_t sonar_alert(void){
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
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
void bt_task(intptr_t unused){
    while(1) {
        uint8_t c = fgetc(bt); /* 受信 */
        switch(c) {
        case '3':
            bt_cmd = 3;
            break;
        case '1':
            bt_cmd = 1;
            break;
        case '0':
            bt_cmd = 0;
            break;
        case 'w':
            bt_cmd = 'w';
            break;
        case 'a':
            bt_cmd = 'a';
            break;
        case 's':
            bt_cmd = 's';
            break;
        case 'd':
            bt_cmd = 'd';
            break;
        case 'e':
            bt_cmd = 'e';
            break;
        case 'W':
            bt_cmd = 'W';
            break;
        case 'A':
            bt_cmd = 'A';
            break;
        case 'S':
            bt_cmd = 'S';
            break;
        case 'D':
            bt_cmd = 'D';
            break;
        case '4':
            bt_cmd = 4;
            break;
        case '5':
            bt_cmd = 5;
            break;
        case '6':
            bt_cmd = 6;
            break;
        case '9':
            bt_cmd = 9;
            break;
        case 'm':
            bt_cmd = 'm';
            break;
        case 'n':
            bt_cmd = 'n';
            break;
        case 'b':
            bt_cmd = 'b';
            break;
        case ' ':
            bt_cmd = ' ';
            break;
        case 'c':
            bt_cmd = 'c';
            break;
        default:
            break;
        }
        fputc(c, bt); /* エコーバック */
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
// 関数名 : m_motor_initializes
// 引数 : unused
// 返り値 : なし
// 概要 : はさみを最大に開いて初期化する
//*****************************************************************************
// void m_motor_initialize(void) {
//     int32_t angle = ev3_motor_get_counts(m_motor);
//     int32_t last_angle = angle - 999;
//
//     syslog(LOG_NOTICE, "はさみ初期化前 angle: %d, last_angle: %d, if: %d\r", angle, last_angle, angle - last_angle);
//     while (angle - last_angle > 10) {
//         ev3_motor_rotate(m_motor, 50, 100, false);
//         last_angle = angle;
//
//         tslp_tsk(300);
//         angle = ev3_motor_get_counts(m_motor);
//         syslog(LOG_NOTICE, "はさみ初期化中 angle: %d, last_angle: %d, if: %d\r", angle, last_angle, angle - last_angle);
//     }
//     syslog(LOG_NOTICE, "はさみ初期化終了 angle: %d, last_angle: %d, if: %d\r", angle, last_angle, angle - last_angle);
//     ev3_motor_reset_counts(m_motor);
// }
//*****************************************************************************
// 関数名 : m_motor_initializes
// 引数 : unused
// 返り値 : なし
// 概要 : はさみを最大に開いて初期化する
//*****************************************************************************
void armMotor_initialize(void) {
    int32_t nowAngle = armMotor->getCount();
    int32_t lastAngle = nowAngle - 999;
    int32_t currentAngle = 0;
    armMotor->setPWM(30);
    tslp_tsk(300);

    while (nowAngle - lastAngle >= 65) {
        // armMotor->setCount(50);
        armMotor->setPWM(20);
        lastAngle = nowAngle;

        tslp_tsk(300);
        nowAngle = armMotor->getCount();
        syslog(LOG_NOTICE, "armMotor初期化中 now: %d, last: %d, 差: %d\r", nowAngle, lastAngle, nowAngle - lastAngle);
    }
    syslog(LOG_NOTICE, "初期化完了\r");
    armMotor->stop();
    armMotor->reset();
}
void setAngle(int32_t angle) {
    armMotor->setCount(angle);
}
//*****************************************************************************
// 関数名 : armMotor_grab
// 引数 : unused
// 返り値 : なし
// 概要 : はさみを閉じて初期化する
//*****************************************************************************
void armMotor_grab(void) {
    int32_t nowAngle = armMotor->getCount();
    int32_t lastAngle = nowAngle + 999;
    int32_t currentAngle = 0;
    armMotor->setPWM(-30);
    tslp_tsk(300);

    while (nowAngle - lastAngle <= -60) {
        armMotor->setPWM(-20);
        lastAngle = nowAngle;

        tslp_tsk(300);
        nowAngle = armMotor->getCount();
        syslog(LOG_NOTICE, "armMotorOPEN now: %d, last: %d, 差: %d\r", nowAngle, lastAngle, nowAngle - lastAngle);
    }
    syslog(LOG_NOTICE, "armMotorOPEN finished!\r");
    armMotor->stop();
    armMotor->reset();



    // armMotor->setCount(-660);
    // ev3_speaker_play_tone(200.00, 40);
    // clock->sleep(1000);
    //
    // int32_t nowAngle = armMotor->getCount();
    // int32_t lastAngle = nowAngle + 999;
    // int32_t currentAngle = 0;
    // armMotor->setPWM(-30);
    // tslp_tsk(300);
    //
    // while (nowAngle - lastAngle <= -65) {
    //     // armMotor->setCount(50);
    //     armMotor->setPWM(-20);
    //     lastAngle = nowAngle;
    //
    //     tslp_tsk(300);
    //     nowAngle = armMotor->getCount();
    //     syslog(LOG_NOTICE, "armMotorGrab now: %d, last: %d, 差: %d\r", nowAngle, lastAngle, nowAngle - lastAngle);
    // }
    // syslog(LOG_NOTICE, "完了\r");
    // armMotor->stop();
    // armMotor->reset();
}
int16_t controlAngle(void) {
    // thisAngle = ev3_gyro_sensor_get_angle(EV3_PORT_3);
    // thisAngle = gyroSensor->getAngle();
    thisAngle += gyroSensor->getAnglerVelocity();

    // thisAngle = 0;
    // thisAngle /= 25;
    return thisAngle;
}
void controlAngle(int32_t angle) {
    int32_t nowAngle = controlAngle();
    while (angle - 10 < nowAngle && nowAngle < angle + 10) {

        if (angle > 0) {
            rightMotor->setPWM(-10);
            leftMotor->setPWM(10);
        }
        else {
            rightMotor->setPWM(10);
            leftMotor->setPWM(-10);
        }
    }
    syslog(LOG_NOTICE, "controlAngle Finished!\r");
}
//*****************************************************************************
// 関数名 : rajikon
// 引数 : unused
// 返り値 : なし
// 概要 : しょぼいラジコン操作関数
//*****************************************************************************


// if(sonarSensor->getDistance() <= 45 && ishave == 0) {
//     int8_t RUDDER_RIGHT = 1;
//     int8_t RUDDER_LEFT = -1;
//     int8_t rudder = RUDDER_RIGHT;
//     int8_t sonar_min = 255;
//     sonartemp = sonarSensor->getDistance();
//     leftMotor->setPWM(5);//右に曲がる
//     rightMotor->setPWM(1);
//     tslp_tsk(500);
//     while (sonartemp >= 10) { //ペットボトルに近づくまで
//         if (sonar_min + 2 <= sonartemp) {
//             if (rudder == RUDDER_RIGHT) {
//                 leftMotor->setPWM(5);//左に曲がる
//                 rightMotor->setPWM(15);
//                 rudder = RUDDER_LEFT;
//                 ev3_led_set_color(LED_RED);
//             }
//             else {
//                 leftMotor->setPWM(15);//右に曲がる
//                 rightMotor->setPWM(5);
//                 rudder = RUDDER_RIGHT;
//                 ev3_led_set_color(LED_ORANGE);
//             }
//         }
//         else {
//             ev3_led_set_color(LED_GREEN);
//             leftMotor->setPWM(20);//直進
//             rightMotor->setPWM(20);
//         }
//
//         if (sonar_min > sonartemp) {//最短距離更新
//             sonar_min = sonartemp;
//         }
//
//         sonartemp = sonarSensor->getDistance(); //値更新
//     }
