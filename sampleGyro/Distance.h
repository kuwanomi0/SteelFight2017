/**
 ******************************************************************************
 ** ファイル名 : Distance.h
 ** クラス名   : Distance
 **
 ** 概要 : 移動距離を測るクラス
 ******************************************************************************
 **/
#ifndef Distance_H
#define Distance_H

class Distance {
private:
    const float PI = 3.141592653589793F;
    float distance = 0.0;
    float distance4msL = 0.0;
    float distance4msR = 0.0;
    float pre_angleL, pre_angleR;

public:
    void init(int motor_ang_L, int motor_ang_R);
    void update(int motor_ang_L, int motor_ang_R);
    float getDistance();
    float getDistance4msL();
    float getDistance4msR();
};
#endif
