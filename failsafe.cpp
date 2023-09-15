
// /*
//     耐故障制御簡易版
//     左前と右後を止める場合にだけ対応
//     状態フィードバックによるレギュレーター（各状態を0に制御）
//     実験が成功したら、全ての場合に対応できる様に改良してください。
//     角速度制御ループの中でフェイルセーフモードの時この関数を呼ぶ．
//     読んだ後、この関数が決定したDutyにT_refを加えること．

//     引数
//     fl_duty :右左モータDuty変数のポインタ
//     rr_duty :右後ろモータDuty変数のポインタ
//     p：バイアスを引いた後のロール角速度
//     q：バイアスを引いた後のピッチ角速度
//     phi:ロール角
//     theta:ピッチ角
// */
#include "control.hpp"
#include<math.h>

void failsafe()
{
    float p_d, q_d, phi_d, theta_d;
    float q0, q1 , q2 , q3;
    const float sqrt2 = 1.41421356237;
    const float costh = 1/sqrt2;
    const float sinth = 1/sqrt2;

    //状態フィードバックゲイン
    const float f11 =  22.4239;  
    const float f12 = -34.5829;
    const float f13 = -22.3607;
    const float f14 = -22.8487;
    const float f21 = -22.4239;
    const float f22 =  34.5829;
    const float f23 =  22.3607;
    const float f24 =  22.8487;

    //方向余弦行列
    float e11 = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    float e12 = 2*(q1*q2 + q0*q3);
    float e13 = 2*(q1*q3 - q0*q2);
    float e21 = 2*(q1*q2 - q0*q3);
    float e22 = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    float e23 = 2*(q2*q3 + q0*q1);
    float e31 = 2*(q1*q3 + q0*q2);
    float e32 = 2*(q2*q3 - q0*q1);
    float e33 = q0*q0 - q1*q1 + q2*q2 - q3*q3;

    //ロドリゲスの回転行列
    float rod11 = e31*e31*(1-costh)+costh;
    float rod12 = e31*e32*(1-costh)-e33*sinth;
    float rod13 = e31*e33*(1-costh)+e32*sinth;
    float rod21 = e31*e32*(1-costh)+e33*sinth;
    float rod22 = e32*e32*(1-costh)+costh;
    float rod23 = e32*e33*(1-costh)-e31*sinth;
    float rod31 = e31*e33*(1-costh)-e32*sinth;
    float rod32 = e32*e33*(1-costh)+e31*sinth;
    float rod33 = e33*e33*(1-costh)+costh;

    //回転
    float e11_ = rod11*e11 + rod12*e12 + rod13*e13;
    float e12_ = rod21*e11 + rod22*e12 + rod23*e13;
    float e13_ = rod31*e11 + rod32*e12 + rod33*e13;

    float e21_ = rod11*e21 + rod12*e22 + rod13*e23;
    float e22_ = rod21*e21 + rod22*e22 + rod23*e23;
    float e23_ = rod31*e21 + rod32*e22 + rod33*e23;

    e11 = e11_;
    e12 = e12_;
    e13 = e13_;
    e21 = e21_;
    e22 = e22_;
    e23 = e23_;

    //座標変換
    p_d = ( Wp + Wq)/sqrt2;
    q_d = (-Wp + Wq)/sqrt2;
    phi_d = atan2f(e23, e33);
    theta_d = atan2f(-e13, sqrt(e23*e23 + e33*e33));

    //状態フィードバック
    FL_duty = T_ref + p_d + f12*q_d + f13*phi_d + f14*theta_d;
    RR_duty = T_ref + p_d + f22*q_d + f23*phi_d + f24*theta_d;
    FR_duty = 0;
    RL_duty = 0;

}
