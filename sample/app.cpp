/**
 ******************************************************************************
 ** �t�@�C���� : app.cpp
 **
 ** �T�v : 2�֓|���U�q���C���g���[�X���{�b�g��TOPPERS/HRP2�pC++�T���v���v���O����
 **
 ** ���L : sample_cpp (���C���g���[�X/�K�����[�^/�����g�Z���T/�����[�g�X�^�[�g)
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
static int32_t   bt_cmd = 0;      /* Bluetooth�R�}���h 1:�����[�g�X�^�[�g */
static FILE     *bt = NULL;      /* Bluetooth�t�@�C���n���h�� */

/* ���L�̃}�N���͌�/���ɍ��킹�ĕύX����K�v������܂� */
#define CMD_START         '1'    /* �����[�g�X�^�[�g�R�}���h */

/* LCD�t�H���g�T�C�Y */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

/* �I�u�W�F�N�g�ւ̃|�C���^��` */
SonarSensor*    sonarSensor;
ColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
Motor*          leftMotor;
Motor*          rightMotor;
Motor*          MMotor;
Clock*          clock;

/* ���C���^�X�N */
void main_task(intptr_t unused)
{
    /* �e�I�u�W�F�N�g�𐶐��E���������� */
    colorSensor = new ColorSensor(PORT_1);
    sonarSensor = new SonarSensor(PORT_2);
    gyroSensor  = new GyroSensor(PORT_3);
    rightMotor  = new Motor(PORT_C);
    leftMotor   = new Motor(PORT_B);
    MMotor      = new Motor(PORT_A);
    clock       = new Clock();

    /* LCD��ʕ\�� */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET sample_cpp", 0, CALIB_FONT_HEIGHT*1);

    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth�ʐM�^�X�N�̋N�� */
    act_tsk(BT_TASK);

    ev3_led_set_color(LED_ORANGE); /* �����������ʒm */

    /* �X�^�[�g�ҋ@ */
    while(1)
    {
        if (bt_cmd == 1)
        {
            break; /* �����[�g�X�^�[�g */
        }

        clock->sleep(10);
    }

    /* ���s���[�^�[�G���R�[�_�[���Z�b�g */
    leftMotor->reset();
    rightMotor->reset();
    MMotor->reset();

    /* �W���C���Z���T�[���Z�b�g */
    gyroSensor->reset();

    ev3_led_set_color(LED_GREEN); /* �X�^�[�g�ʒm */

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

        clock->sleep(4); /* 4msec�����N�� */
    }
    leftMotor->reset();
    rightMotor->reset();
    MMotor->reset();

    ter_tsk(BT_TASK);
    fclose(bt);

    ext_tsk();
}

//*****************************************************************************
// �֐��� : bt_task
// ���� : unused
// �Ԃ�l : �Ȃ�
// �T�v : Bluetooth�ʐM�ɂ�郊���[�g�X�^�[�g�B Tera Term�Ȃǂ̃^�[�~�i���\�t�g����A
//       ASCII�R�[�h��1�𑗐M����ƁA�����[�g�X�^�[�g����B
//*****************************************************************************
void bt_task(intptr_t unused)
{
    while(1)
    {
        uint8_t c = fgetc(bt); /* ��M */
        switch(c)
        {
        case '1':
            bt_cmd = 1;
            break;
        default:
            break;
        }
        fputc(c, bt); /* �G�R�[�o�b�N */
    }
}
