/**
 ******************************************************************************
 ** ファイル名 : Distance.cpp
 ** クラス名   : Distance
 **
 ** 概要 : 移動距離を測るクラス
 ******************************************************************************
 **/
#include "Distance.h"

#define TIRE_DIAMETER 54.9  //タイヤ直径

/* 初期化関数 */
void Distance::Distance_init(int motor_ang_L, int motor_ang_R) {
    //各変数の値の初期化
    distance = 0.0;
    distance4msR = 0.0;
    distance4msL = 0.0;
    //モータ角度の過去値に現在値を代入
    pre_angleL = motor_ang_L;
    pre_angleR = motor_ang_R;
}

/* 距離更新（4ms間の移動距離を毎回加算している） */
void Distance::Distance_update(int motor_ang_L, int motor_ang_R){
    float cur_angleL = motor_ang_L; //左モータ回転角度の現在値
    float cur_angleR = motor_ang_R;//右モータ回転角度の現在値
    float distance4ms = 0.0;        //4msの距離

    // 4ms間の走行距離 = ((円周率 * タイヤの直径) / 360) * (モータ角度過去値　- モータ角度現在値)
    distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms間の左モータ距離
    distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms間の右モータ距離
    distance4ms = (distance4msL + distance4msR) / 2.0; //左右タイヤの走行距離を足して割る
    distance += distance4ms;

    //モータの回転角度の過去値を更新
    pre_angleL = cur_angleL;
    pre_angleR = cur_angleR;
}

/* 走行距離を取得 */
float Distance::Distance_getDistance(){
    return distance;
}

/* 左タイヤの4ms間の距離を取得 */
float Distance::Distance_getDistance4msL(){
    return distance4msL;
}

/* 右タイヤの4ms間の距離を取得 */
float Distance::Distance_getDistance4msR(){
    return distance4msR;
}
