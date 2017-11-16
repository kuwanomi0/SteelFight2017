/**
 ******************************************************************************
 ** ファイル名 : Distance.cpp
 ** クラス名   : Distance
 **
 ** 概要 : 移動距離を測るクラス
 ******************************************************************************
 **/
#include "Distance.h"

int Distance::distanceAll(int left, int right) {
    return (distanceAll(left) + distanceAll(right)) / 2;
}
int Distance::distanceAll(int motor_ang) {
    return 56.0 * PAI * motor_ang / 360;
}
