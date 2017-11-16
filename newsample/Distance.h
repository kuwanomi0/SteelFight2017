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
    const float PAI = 3.141592653589793F;

public:
    int distanceAll(int left, int right);
};
#endif
