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
    void Distance_init(int motor_ang_L, int motor_ang_R);
    void Distance_update(int motor_ang_L, int motor_ang_R);
    float Distance_getDistance();
    float Distance_getDistance4msL();
    float Distance_getDistance4msR();
};
#endif
