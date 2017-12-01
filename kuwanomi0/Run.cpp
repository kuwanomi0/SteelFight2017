/**
 ******************************************************************************
 ** ファイル名 : Run.cpp
 ** クラス名   : Run
 **
 ** 概要 : 走行するための値を管理するクラス
 ******************************************************************************
 **/
#include "Run.h"

/**
 *  コンストラクタ
 */
Run::Run()
    :   mForward(0),
        mTurn(0),
        mLeftPwm(0),
        mRightPwm(0) {
}

/**
 * バランサの値を更新する
 * @param angle   角速度
 * @param lwEnc   左車輪エンコーダ値
 * @param rwEnc   右車輪エンコーダ値
 * @param battety バッテリ電圧値
 */
// void Run::update(int gyro, int lwEnc, int rwEnc, int battery) {
//     mLeftPwm = mRightPwm = mForward + mTurn + gyro;
// }
void Run::update(int gyro) {
    mLeftPwm = mRightPwm = mForward + mTurn + gyro;
}

void Run::update() {
    mLeftPwm  = mForward + turn;
    mRightPwm = mForward
}
/**
 * 前進値、旋回値を設定する
 * @param forward 前進値
 * @param turn    旋回値
 */
void Run::setCommand(int forward, int turn) {
    mForward = forward;
    mTurn    = turn;
}

/**
 * 左車輪のPWM値を取得する
 * @return 左車輪のPWM値
 */
int8_t Run::getPwmLeft() {
    return mLeftPwm;
}

/**
 * 右車輪のPWM値を取得する
 * @return 右車輪のPWM値
 */
int8_t Run::getPwmRight() {
    return mRightPwm;
}