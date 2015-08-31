#include "LegKinematicsIV.h"
#include <cmath>

using namespace Model;

Leg::Leg()
{
    this->m_legID = LEG_ID_LB;
}

Leg::Leg(int legID)
{
    this->m_legID = legID;
}

void Leg::SetID(int legID)
{
    this->m_legID = legID;
}

Leg::~Leg()
{}

int Leg::InverseSolution(double* tipPosition, double* jointLength, bool requireTransformToHip)
{
    TransformTipPositionToHip(tipPosition, m_tempPosition, requireTransformToHip);

    double ftpos_o[3];
    for (int i = 0; i < 3; i++){
        ftpos_o[i] = m_tempPosition[i] * 1000.0;
    }
    
    double s1, s2, s3; //输出丝杆位置

    double Lg1, Lg2, Lg3, Lgos, Lgc, phi, phiC, Hh;

    double blen, lp, l1, l2, l3, halfangplt, AU1, BU2, CU3, slid_os, Theta1_h, Theta23_h, s_h;
    double Ap[3], Bp[3], Cp[3];
    const double PI = 3.14159265359;

    Lg1 = 250;
    Lg2 = 280;
    Lg3 = 330;
    Lgos = Lg1 - 50;
    phi = 180 * PI / 180;
    Lgc = 110;
    phiC = 25 * PI / 180;
    Hh = 20;//腿部两个R副之间竖直距离

    blen = 65.0/2; //动平台三角形腰长
    lp = 140;
    l1 = 219.34;
    l2 = 189;
    l3 = l2;
    halfangplt = PI / 2;
    Ap[0] = -14; Ap[1] = 0; Ap[2] = 82.5;
    Bp[0] = -224; Bp[1] = 123; Bp[2] = 82.5;
    Cp[0] = -224; Cp[1] = -123; Cp[2] = 82.5;
    AU1 = 230;
    BU2 = 230;
    CU3 = 230;
    slid_os = 115;
    Theta1_h = -7.59*PI / 180; //h下标代表home位置
    Theta23_h = 7.72*PI / 180;
    s_h = 30;

    double theta_s, POX, PO, PK, POK, Lg2_X, Lgc_X, PKB, BKO, Lg1_X;
    double ftpos[3], pos_driv[3], C[3];
    //static double Rm[3][3];

    //以下腿的反解

    theta_s = -atan(ftpos_o[1] / ftpos_o[2]);
    //Rm={{1,0,0},{0,cos(theta_s),-sin(theta_s)},{0,sin(theta_s),cos(theta_s)}};
    ftpos[0] = ftpos_o[0];
    ftpos[1] = cos(-theta_s)*ftpos_o[1] - sin(-theta_s)*ftpos_o[2];
    ftpos[2] = sin(-theta_s)*ftpos_o[1] + cos(-theta_s)*ftpos_o[2] + Hh;
    POX = atan2(ftpos[2], ftpos[0]);	//注意POX在矢状面xz内是负角
    PO = sqrt(ftpos[0] * ftpos[0] + ftpos[2] * ftpos[2]);
    PK = sqrt(Lg3*Lg3 + Lg1*Lg1 - 2 * Lg3*Lg1*cos(phi));
    POK = acos((PO*PO + Lg2*Lg2 - PK*PK) / (2 * PO*Lg2));
    Lg2_X = POX + POK;
    Lgc_X = Lg2_X + phiC;
    PKB = acos((PK*PK + Lg1*Lg1 - Lg3*Lg3) / (2 * Lg1*PK));
    BKO = acos((PK*PK + Lg2*Lg2 - PO*PO) / (2 * PK*Lg2)) - PKB;
    Lg1_X = Lg2_X - (PI - BKO);
    C[0] = Lgos*cos(Lg1_X) + Lgc*cos(Lgc_X);
    C[1] = 0;
    C[2] = Lgos*sin(Lg1_X) + Lgc*sin(Lgc_X);
    pos_driv[0] = C[0];
    pos_driv[1] = cos(theta_s)*C[1] - sin(theta_s)*(C[2] - Hh);
    pos_driv[2] = sin(theta_s)*C[1] + cos(theta_s)*(C[2] - Hh);

    double AP_v[3], S2[3], S3[3], S2RY[3], S3RY[3], S2Y[3], S3Y[3], BS2_v[3], CS3_v[3];
    double PU1, PAX, PA_xz, PU1_xz, PAU1_xz, Theta1, AU1P_xz, Roll1, Yaw1, S2U2, S2BX, S2B_xz, S2U2_xz, S2BU2_xz, Theta2, S3U3, S3CX, S3C_xz, S3U3_xz, S3CU3_xz, Theta3;
    //static double Rm1[3][3];

    //以下驱动反解
    PU1 = l1 + lp;
    AP_v[0] = pos_driv[0] - Ap[0];
    AP_v[1] = pos_driv[1] - Ap[1];
    AP_v[2] = pos_driv[2] - Ap[2];
    PAX = atan2(AP_v[2], AP_v[0]);//在xz平面内和X轴的夹角
    PA_xz = sqrt(AP_v[0] * AP_v[0] + AP_v[2] * AP_v[2]);
    PU1_xz = sqrt(PU1*PU1 - AP_v[1] * AP_v[1]);
    PAU1_xz = acos((PA_xz*PA_xz + AU1*AU1 - PU1_xz*PU1_xz) / (2 * PA_xz*AU1));
    Theta1 = -PAX - PAU1_xz;//AU1杆绕y轴转角，注意方向
    AU1P_xz = acos((PU1_xz*PU1_xz + AU1*AU1 - PA_xz*PA_xz) / (2 * PU1_xz*AU1));
    Roll1 = (PI - AU1P_xz);
    Yaw1 = asin(AP_v[1] / (PU1));
    //Rm1(:,:,iRm)=[cos(Theta1(iRm)) 0 sin(Theta1(iRm)); 0 1 0; -sin(Theta1(iRm)) 0 cos(Theta1(iRm))]*...
    //[cos(Roll1(iRm)) 0 sin(Roll1(iRm)); 0 1 0; -sin(Roll1(iRm)) 0 cos(Roll1(iRm))]*...
    //[cos(Yaw1(iRm)) -sin(Yaw1(iRm)) 0;sin(Yaw1(iRm)) cos(Yaw1(iRm)) 0;0 0 1 ];
    //S2_d=[l1;blen*sin(halfangplt);-blen*cos(halfangplt)];
    //S3_d=[l1;-blen*sin(halfangplt);-blen*cos(halfangplt)];
    //U1=A+AU1*[cos(-Theta1);zeros(1,nC);sin(-Theta1)];
    S2Y[0] = cos(Yaw1)*l1 - sin(Yaw1)*blen*sin(halfangplt);
    S2Y[1] = sin(Yaw1)*l1 + cos(Yaw1)*blen*sin(halfangplt);
    S2Y[2] = -blen*cos(halfangplt);
    S2RY[0] = cos(Roll1)*S2Y[0] + sin(Roll1)*S2Y[2];
    S2RY[1] = S2Y[1];
    S2RY[2] = -sin(Roll1)*S2Y[0] + cos(Roll1)*S2Y[2];
    S2[0] = Ap[0] + AU1*cos(-Theta1) + cos(Theta1)*S2RY[0] + sin(Theta1)*S2RY[2];
    S2[1] = Ap[1] + S2RY[1];
    S2[2] = Ap[2] + AU1*sin(-Theta1) - sin(Theta1)*S2RY[0] + cos(Theta1)*S2RY[2];

    S3Y[0] = cos(Yaw1)*l1 + sin(Yaw1)*blen*sin(halfangplt);
    S3Y[1] = sin(Yaw1)*l1 - cos(Yaw1)*blen*sin(halfangplt);
    S3Y[2] = -blen*cos(halfangplt);
    S3RY[0] = cos(Roll1)*S3Y[0] + sin(Roll1)*S3Y[2];
    S3RY[1] = S3Y[1];
    S3RY[2] = -sin(Roll1)*S3Y[0] + cos(Roll1)*S3Y[2];
    S3[0] = Ap[0] + AU1*cos(-Theta1) + cos(Theta1)*S3RY[0] + sin(Theta1)*S3RY[2];
    S3[1] = Ap[1] + S3RY[1];
    S3[2] = Ap[2] + AU1*sin(-Theta1) - sin(Theta1)*S3RY[0] + cos(Theta1)*S3RY[2];

    S2U2 = l2;
    BS2_v[0] = S2[0] - Bp[0];
    BS2_v[1] = S2[1] - Bp[1];
    BS2_v[2] = S2[2] - Bp[2];
    S2BX = atan2(BS2_v[2], BS2_v[0]);//在xz平面内和X轴的夹角
    S2B_xz = sqrt(BS2_v[0] * BS2_v[0] + BS2_v[2] * BS2_v[2]);
    S2U2_xz = sqrt(S2U2*S2U2 - BS2_v[1] * BS2_v[1]);
    S2BU2_xz = acos((S2B_xz*S2B_xz + BU2*BU2 - S2U2_xz*S2U2_xz) / (2 * S2B_xz*BU2));
    Theta2 = -S2BX - S2BU2_xz;//BU2杆绕y轴转角

    S3U3 = l3;
    CS3_v[0] = S3[0] - Cp[0];
    CS3_v[1] = S3[1] - Cp[1];
    CS3_v[2] = S3[2] - Cp[2];
    S3CX = atan2(CS3_v[2], CS3_v[0]);//在xz平面内和X轴的夹角
    S3C_xz = sqrt(CS3_v[0] * CS3_v[0] + CS3_v[2] * CS3_v[2]);
    S3U3_xz = sqrt(S3U3*S3U3 - CS3_v[1] * CS3_v[1]);
    S3CU3_xz = acos((S3C_xz*S3C_xz + CU3*CU3 - S3U3_xz*S3U3_xz) / (2 * S3C_xz*CU3));
    Theta3 = -S3CX - S3CU3_xz;//CU3杆绕y轴转角

    s2 = slid_os*tan(Theta1 - Theta1_h) + s_h;
    s3 = slid_os*tan(Theta2 - Theta23_h) + s_h;
    s1 = slid_os*tan(Theta3 - Theta23_h) + s_h;

    jointLength[0] = s1 / 1000.0;
    jointLength[1] = s2 / 1000.0;
    jointLength[2] = s3 / 1000.0;

    return 0;
}

// Transform the tip position w.r.t. Body Direction to the Leg Hip Direction
int Leg::TransformTipPositionToHip(double* tipPositionRaw, double* relativeTipPositionWithHip, bool requireTransformToHip)
{
    if ( m_legID == LEG_ID_LB || m_legID == LEG_ID_MB || m_legID == LEG_ID_RB ) 
    {
        for(int i = 0; i < 3; i++)
        {
            relativeTipPositionWithHip[i] = tipPositionRaw[i];
        }
        // Reverse the x value when the leg is the back leg
        if(requireTransformToHip)
            relativeTipPositionWithHip[0] = -relativeTipPositionWithHip[0];
    }

    return 0;
}

int Leg::InverseSolutionPole(double* tipPositionPole, double* jointLength, bool requireTransformToHip)
{
    double tipPositionCart[3];
    TransformFromPoleToCart(tipPositionPole, tipPositionCart);
    return InverseSolution(tipPositionCart, jointLength, requireTransformToHip);
}

int Leg::TransformFromPoleToCart(double* positionPole, double* positionCart)
{
    double l = positionPole[2];
    double ty = positionPole[0];
    double tx = positionPole[1];
    positionCart[0] = l * sin(ty);// X direction
    positionCart[1] = l * sin(tx) * cos(ty); // Y direction
    positionCart[2] = -l * cos(tx) * cos(ty); // Z direction
    return 0;
}
