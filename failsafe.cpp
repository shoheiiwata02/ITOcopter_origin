#include "control.hpp"

/*
    耐故障制御簡易版
    左前と右後を止める場合にだけ対応
    状態フィードバックによるレギュレーター（各状態を0に制御）
    実験が成功したら、全ての場合に対応できる様に改良してください。
    角速度制御ループの中でフェイルセーフモードの時この関数を呼ぶ．
    読んだ後、この関数が決定したDutyにT_refを加えること．

    引数
    fl_duty :右左モータDuty変数のポインタ
    rr_duty :右後ろモータDuty変数のポインタ
    p：バイアスを引いた後のロール角速度
    q：バイアスを引いた後のピッチ角速度
    phi:ロール角
    theta:ピッチ角
*/

// void failsafe(float* fl_duty, float* rr_duty, float p, float q, float phi, float theta)
void failsafe()
{
    float p_d, q_d,     phi_d, theta_d;
    const float sqrt2 = 1.41421356237;
    //状態フィードバックゲイン
    const float f11 =  22.4239;  
    const float f12 = -34.5829;
    const float f13 = -22.3607;
    const float f14 = -22.8487;
    const float f21 = -22.4239;
    const float f22 =  34.5829;
    const float f23 =  22.3607;
    const float f24 =  22.8487;

    //座標変換
    // p_d = ( p + q)/sqrt2;
    // q_d = (-p + q)/sqrt2;
    p_d = (Wp)/sqrt2;
    q_d = (Wq)/sqrt2;
    phi_d = Phi; //ここまだ未確定
    theta_d = Theta;//ここまだ未確定

    //状態フィードバック
    FL_duty = f11*p_d + f12*q_d + f13*phi_d + f14*theta_d;
    RR_duty = f21*p_d + f22*q_d + f23*phi_d + f24*theta_d;

}