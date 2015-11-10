#ifndef GEO_DEFINITION_H
#define GEO_DEFINITION_H

namespace Model
{
    const double PI = 3.14159265359;

    const double LG1 = 250;
    const double LG2 = 280;
    const double LG3 = 280;
    const double LGOS = LG1 - 50;
    const double PHI = 180 * PI / 180;
    const double LGC = 110;
    const double PHIC = 25 * PI / 180;
    const double HH = 20;//腿部两个R副之间竖直距离

    const double BLEN = 65.0/2; //动平台三角形腰长
    const double LP = 140;
    const double L1 = 211.34;
    const double L2 = 193;
    const double L3 = L2;
    const double HALFANGPLT = PI / 2;
    const double AP[3] = { -14, 0, 82.5 };
    const double BP[3] = { -224, 123, 82.5};
    const double CP[3] = { -224, -123, 82.5};
    const double AU1 = 230;
    const double BU2 = 230;
    const double CU3 = 230;
    const double SLID_OS = 115;
    const double THETA1_H = -7.59*PI / 180; //h下标代表home位置
    const double THETA23_H = 7.72*PI / 180;
    const double S_H = 30;

}


#endif
