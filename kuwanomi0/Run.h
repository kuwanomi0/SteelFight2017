/**
 ******************************************************************************
 ** ファイル名 : Run.h
 ** クラス名   : Run
 **
 ** 概要 : 走行するための値を管理するクラス
 ******************************************************************************
 **/
#ifndef Run_H
#define Run_H

#include "ev3api.h"

class Run {
private:
    int mForward;
    int mTurn;
    int8_t mLeftPwm;
    int8_t mRightPwm;

public:
    Run();
    // void update(int gyro, int rwEnc, int lwEnc, int battery);
    void update(int gyro);
    void update();
    void setCommand(int forward, int turn);
    int8_t getPwmLeft();
    int8_t getPwmRight();
};
#endif
