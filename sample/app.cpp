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

static int      bt_cmd = 0;     /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt = NULL;     /* Bluetoothファイルハンドル */

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
#define GYRO_OFFSET  0          /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define LIGHT_WHITE  40         /* 白色の光センサ値 */
#define LIGHT_BLACK  0          /* 黒色の光センサ値 */
#define SONAR_ALERT_DISTANCE 0 /* 超音波センサによる障害物検知距離[cm] */

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

/* 関数プロトタイプ宣言 */
static int sonar_alert(void);

static signed char pwm_L, pwm_R, pwm_M; /* 左右モータPWM出力 */

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

    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth通信タスクの起動 */
    act_tsk(BT_TASK);

    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    /* スタート待機 */
    while(1) {
        if (bt_cmd == 1) {
            break; /* リモートスタート */
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

    ER er = ev3_sta_cyc(CYC_HANDLER);	//周期ハンドラを起動
    sprintf(buf, "main_task: error_code=%d", MERCD(er) );	// APIのエラーコードの表示
    //ev3_lcd_draw_string(buf, 0, CALIB_FONT_HEIGHT*1);		// の仕方です。

    /* 自タスク(メインタスク）を待ち状態にする */
    slp_tsk();

    ev3_motor_stop(m_motor, false);
    ev3_motor_stop(left_motor, false);
    ev3_motor_stop(right_motor, false);

    ter_tsk(BT_TASK);
    fclose(bt);

    ev3_lcd_draw_string("EV3RT Stopped.", 0, CALIB_FONT_HEIGHT*9);
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
    pwm_L = 0;
    pwm_R = 0;
    pwm_M = 0;

    // ショボいラジコン操作
    if (bt_cmd == 'a') {
        pwm_R = 30;
        pwm_L = -30;
    }
    if (bt_cmd == 'd') {
        pwm_R = -30;
        pwm_L = 30;
    }
    if (bt_cmd == 'w') {
        pwm_R = 30;
        pwm_L = 30;
    }
    if (bt_cmd == 's') {
        pwm_R = -30;
        pwm_L = -30;
    }
    if (bt_cmd == 'e') {
        pwm_R = 100;
        pwm_L = 100;
    }
    if (bt_cmd == 5) {
        pwm_L = 0;
        pwm_R = 0;
        pwm_M = 70;
    }
    if (bt_cmd == 6) {
        pwm_L = 0;
        pwm_R = 0;
        pwm_M = -70;
    }
    if (sonar_alert() == 1){ /* 障害物検知 */
        pwm_R = pwm_L = 0; /* 障害物を検知したら停止 */
    }

    /* EV3ではモーター停止時のブレーキ設定が事前にできないため */
    /* 出力0時に、その都度設定する */
    if (pwm_L == 0) {
         ev3_motor_stop(left_motor, true);
    } else {
        ev3_motor_set_power(left_motor, (int)pwm_L);
    }

    if (pwm_R == 0) {
         ev3_motor_stop(right_motor, true);
    } else {
        ev3_motor_set_power(right_motor, (int)pwm_R);
    }

    if (pwm_M == 0) {
        ev3_motor_stop(m_motor, true);
    } else {
        ev3_motor_set_power(m_motor, (int)pwm_M);
    }

    /* タッチセンサが押されたら停止する */
    if (bt_cmd == 0) {
        wup_tsk(MAIN_TASK);        //メインタスクを起床する
        ev3_stp_cyc(CYC_HANDLER);  //周期ハンドラを停止する
    }

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
// 関数名 : sonar_alert
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
static int sonar_alert(void)
{
    static unsigned int counter = 0;
    static int alert = 0;

    signed int distance;

    if (++counter == 40/4){ /* 約40msec周期毎に障害物検知  */
        /*
         * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
         * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
         * EV3の場合は、要確認
         */
        distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0)) {
            alert = 1; /* 障害物を検知 */
        } else {
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
        case '5':
            bt_cmd = 5;
            break;
        case '6':
            bt_cmd = 6;
            break;
        default:
            break;
        }
        fputc(c, bt); /* エコーバック */
    }
}
