/**
 ******************************************************************************
 ** ファイル名 : PID.h
 ** クラス名   : PID
 **
 ** 概要 : PID制御を行うための処理をするクラス
 ******************************************************************************
 **/
#ifndef PID_H
#define PID_H


class PID {

private:
    float kp; /*比例定数*/
    float ki; /*積分定数*/
    float kd; /*微分定数*/
    int diff[2]; /* カラー格納用変数 */
    float integral; /* 積分計算用変数 */

public:
    PID(float p_value,float i_value ,float d_value) {
        setPID(p_value, i_value, d_value);
    }
    int calcControl(int now_value);
    void setPID(float p_value,float i_value ,float d_value) {
        kp = p_value; /*比例定数*/
        ki = i_value; /*積分定数*/
        kd = d_value; /*微分定数*/
        diff[1] = 0;
        integral = 0.0;
    }
};
#endif
