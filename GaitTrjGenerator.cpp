#include "GaitTrjGenerator.h"
#include "Planners.h"

namespace Model
{

double walk_cxb(
        double timeFromStart,
        const RobotHighLevelControl::ParamCXB& param,
        double *legTipPositionPole)
{
    return walk_cxb(
            timeFromStart,
            param.totalPeriodCount,
            param.stepLength,
            param.Lside,
            param.rotationAngle,
            param.duty,
            param.stepHeight,
            param.T,
            param.standHeight,
            param.tdDeltaMidLeg,
            param.tdDeltaSideLeg,
            legTipPositionPole);
}

double walk_cxb(
        double timeFromStart,
        unsigned int totalPeriodCount,
        double stepLength,
        double Lside,
        double rotationAngle,
        double duty,
        double stepHeight , //positive value
        double T,
        double standHeight,
        double tdDeltaMidLeg,
        double tdDeltaSideLeg,
        double *legTipPositionPole)
{
    rotationAngle=rotationAngle*PI/180;

    int accPeriodCount = 2;
    int decPeriodCount = 1;// acceleration and deceleration periods

    double totalTimeSpan = totalPeriodCount * T;
    double accTimeSpan = accPeriodCount * T;
    double decTimeSpan = decPeriodCount * T;

    double Ltemp = stepLength;
    double Lsidetemp = Lside;
    double rottemp = rotationAngle;

    double legTipPoints[18]; // in cart coordinate

    double xComOffset = 100;
    double yComOffset = 0;
    double xComOffsetUse = 100;


    if (timeFromStart < accTimeSpan)
    {
        double ratio = timeFromStart / accTimeSpan;
        stepLength    = Ltemp     * ratio;
        Lside         = Lsidetemp * ratio;
        rotationAngle = rottemp   * ratio;
    }
    else if (timeFromStart > totalTimeSpan - decTimeSpan)
    {
        double ratio = 1 - (timeFromStart - (totalTimeSpan - decTimeSpan)) / decTimeSpan;
        stepLength    = Ltemp     * ratio;
        Lside         = Lsidetemp * ratio;
        rotationAngle = rottemp   * ratio;
    }

    double z0, t_re, phi[6],td_delta,td_deltapi,k_td,k_tdpi, z_rot_body, omega_z, phi_td;
    double con;
    int legid;
    double t_leg[6];


    z_rot_body = rotationAngle; // rad

    td_delta = tdDeltaSideLeg;//4;      // unused
    td_deltapi = tdDeltaMidLeg;//7;		// Mid Leg

    z0 = -standHeight;    // body height at start


    k_td=2*(2*td_delta)/(T*duty/2);
    k_tdpi=2*(2*td_deltapi)/(T*duty/2);


    omega_z = z_rot_body / (T * duty);     // the change rate of rotational angle

    //double xdelta=-64.83980+104.8398;
    double xdelta = 0;
    double accdelt = 0;

    double  rot_ini[3][3]=
    {{ cos(-z_rot_body),  sin(-z_rot_body),  0},
    {-sin(-z_rot_body),  cos(-z_rot_body),  0},
    {                0,                 0,  1}};
    double  rot_end[3][3]=
    {{cos(-z_rot_body), -sin(-z_rot_body),   0},
    {sin(-z_rot_body),  cos(-z_rot_body),   0},
    {               0,                 0,   1}};

    double  Dhip[6][3]=
    {{-396,357,0},{-539,0,0},{-396,-357,0},{396,-357,0},{539,0,0},{396,357,0}
    };

    double  foottip_ini_rotz_hipframe[3]={0,0,z0};

    double  foottip_end_rotz_hipframe[6][3];
    double  foottip_ini[6][3]=
    {{-stepLength/2, -Lside/2, z0},
        {-stepLength/2, -Lside/2, z0},
        {-stepLength/2, -Lside/2, z0},
        {-stepLength/2, -Lside/2, z0},
        {-stepLength/2, -Lside/2, z0},
        {-stepLength/2, -Lside/2, z0}};

    double  footsteppara[6][3]=
    {{-stepLength/2, -Lside/2, 0},
        {-stepLength/2, -Lside/2, 0},
        {-stepLength/2, -Lside/2, 0},
        {-stepLength/2, -Lside/2, 0},
        {-stepLength/2, -Lside/2, 0},
        {-stepLength/2, -Lside/2, 0}};

    double  foottip_end[6][3];
    double  foottip_stand_bodyfr[6][3];

    for(legid=0; legid<6; legid++)
    {


        for(int i=0;i<3;i++)
        {
            foottip_stand_bodyfr[legid][i]=Dhip[legid][i];

            foottip_stand_bodyfr[legid][2]=z0;
        }

        for(int i=0;i<3;i++)
        {
            foottip_end_rotz_hipframe[legid][i] =
                rot_end[i][0]*foottip_stand_bodyfr[legid][0]
                +rot_end[i][1]*foottip_stand_bodyfr[legid][1]
                +rot_end[i][2]*foottip_stand_bodyfr[legid][2]
                -Dhip[legid][i];

            foottip_end[legid][i] =
                foottip_end_rotz_hipframe[legid][i]
                -footsteppara[legid][i];
        }

    }

    t_re = fmod(timeFromStart, T)/T;

    phi_td= T*(1-duty);  // the time ratio in a period when touchdown

    phi[0]=0.5-(duty-0.5)/2;

    phi[2]=0.5-(duty-0.5)/2;

    phi[4]=0.5-(duty-0.5)/2;

    phi[1]=0-(duty-0.5)/2;

    phi[3]=0-(duty-0.5)/2;

    phi[5]=0-(duty-0.5)/2;


    double xtdwn,Tcs,tacs,cosp,sinp,ep,pp,xtdwn_pi,vx_ini,par_bx,par_cx,par_ax;
    double ytdwn_pi,vy_ini,par_by,par_cy,par_ay;
    double kx1,kx2,kz1,kz2,z1,z2,x1,x2,p0,p1,p2,p3,tpara,Bz,Bx,tmc,kbnd,ptop,para1,para2;
    double ky1,ky2,y1,y2;

    pp = T * (duty - 0.5);

    Tcs=sqrt(-z0/980);

    tacs=(T*(1-duty))/Tcs;

    cosp=(exp(tacs)+exp(-tacs))/2;

    sinp=(exp(tacs)-exp(-tacs))/2;

    ep=exp(T*(1-duty)/Tcs);

    xtdwn_pi=(stepLength)/(2-1*980.0*pp*pp/(3*z0)+2*pp*((ep+1)/(Tcs*ep-Tcs)));//stepLength/2;

    ytdwn_pi=(Lside)/(2-1*980*pp*pp/(3*z0)+2*pp*((ep+1)/(Tcs*ep-Tcs)));//stepLength/2;

    vx_ini=(xtdwn_pi/Tcs*sinp)/(1-cosp);
    vy_ini=(ytdwn_pi/Tcs*sinp)/(1-cosp);

    par_bx=980*(xtdwn_pi)/z0;

    par_cx=vx_ini;

    par_ax=-par_bx/(T*(duty-0.5));

    par_by=980*(ytdwn_pi)/z0;

    par_cy=vy_ini;

    par_ay=-par_by/(T*(duty-0.5));


    for ( legid=0;legid<6;legid++)
    {
        if (legid % 2 == 0) //LEG_GROUP_B
        {
            xComOffsetUse = -xComOffset;
        }
        else
        {
            xComOffsetUse = xComOffset;
        }

        con=phi[legid]+duty-int(phi[legid]+duty);

        if ( t_re<=con)
            t_leg[legid]=(1-(con-t_re))*T;

        if ( t_re>con)
            t_leg[legid]=(t_re-con)*T;

        double rot_combination = -z_rot_body+omega_z*(t_leg[legid]-phi_td);
        double rotz_slpframe[3][3]=
        {{cos(rot_combination), -sin(rot_combination), 0},
            {sin(rot_combination),  cos(rot_combination), 0},
            {                   0,                     0, 1}};

        // planning in stance phase
        if (t_leg[legid]>=phi_td)
        {
            // planning the Z trajectory in stance phase

            if ((legid == 1) || (legid == 4))
            {
                kbnd=-k_tdpi;
                ptop=-2*td_deltapi;
            } else
            {
                kbnd=-k_td;
                ptop=-2*td_delta;
            }


            para1=T*duty*kbnd/5;

            para2=16*ptop/5-para1;

            tpara=(t_leg[legid]-phi_td)/(T*(duty));

            Bz=5*pow((1-tpara),4)*tpara*para1
                +10*pow((1-tpara),3)*pow((tpara),2)*para2
                +10*pow((1-tpara),2)*pow((tpara),3)*para2
                +5*(1-tpara)*pow((tpara),4)*para1;

            if ((legid == 1) || (legid == 4))
            {
                legTipPoints[2+legid*3]=Bz+z0+td_deltapi;
            } else
            {
                legTipPoints[2+legid*3]=Bz+z0+td_delta;
            }



            // planning the X, Y trajectory in stance phase
            if (t_leg[legid]<=phi_td+(duty-0.5)*T)
            {
                tmc=(t_leg[legid]-phi_td);
                legTipPoints[0+legid*3]=
                    par_ax*pow(tmc,3)/3
                    +par_bx*pow(tmc,2)/2
                    +par_cx*tmc+stepLength/2 + xComOffsetUse;
                legTipPoints[1+legid*3]=
                    par_ay*pow(tmc,3)/3
                    +par_by*pow(tmc,2)/2
                    +par_cy*tmc+Lside/2 + yComOffset;

            }

            if ((t_leg[legid]<=T-(duty-0.5)*T) && (t_leg[legid]>phi_td+(duty-0.5)*T))
            {
                tmc=(t_leg[legid]-phi_td-(duty-0.5)*T)/Tcs;
                double csh;
                csh=(exp(tmc)+exp(-tmc))/2;
                double snh;
                snh=(exp(tmc)-exp(-tmc))/2;

                legTipPoints[0+legid*3]=xtdwn_pi*csh+Tcs*vx_ini*snh + xComOffsetUse;
                legTipPoints[1+legid*3]=ytdwn_pi*csh+Tcs*vy_ini*snh + yComOffset;
            }

            if (t_leg[legid]>T-(duty-0.5)*T)
            {
                tmc=(t_leg[legid]-(T-T*(duty-0.5)));
                legTipPoints[0+legid*3]=par_ax*pow(tmc,3)/3+par_bx*pow(tmc,2)/2+par_cx*tmc-xtdwn_pi + xComOffsetUse;
                legTipPoints[1+legid*3]=par_ay*pow(tmc,3)/3+par_by*pow(tmc,2)/2+par_cy*tmc-ytdwn_pi + yComOffset;
            }

            // Add rotational shifts to the XY trajectory
            legTipPoints[0+legid*3]=
                legTipPoints[0+legid*3]
                +rotz_slpframe[0][0]*Dhip[legid][0]
                +rotz_slpframe[0][1]*Dhip[legid][1]
                +rotz_slpframe[0][2]*Dhip[legid][2]-Dhip[legid][0];


            legTipPoints[1+legid*3]=
                legTipPoints[1+legid*3]
                +rotz_slpframe[1][0]*Dhip[legid][0]
                +rotz_slpframe[1][1]*Dhip[legid][1]
                +rotz_slpframe[1][2]*Dhip[legid][2]-Dhip[legid][1];

        }

        // Planning swing phase
        if(t_leg[legid] < phi_td)
        {
            // Planning X trajectory in swing phase
            kx1=vx_ini;
            kx2=vx_ini;

            x1=foottip_ini[legid][0];
            x2=foottip_end[legid][0];

            p0=x1;
            p3=x2;
            p1=phi_td*kx1/3+p0;
            p2=p3-phi_td*kx2/3;
            tpara=t_leg[legid]/(phi_td);
            Bx =  pow((1-tpara),3)*p0 +
                3*pow((1-tpara),2)*tpara*p1 +
                3*(1-tpara)*pow((tpara),2)*p2 +
                pow((tpara),3)*p3;

            legTipPoints[0+legid*3]=Bx + xComOffsetUse;

            // Planning Y trajectory in swing phase
            ky1=vy_ini;
            ky2=vy_ini;
            y1=foottip_ini[legid][1];;
            y2=foottip_end[legid][1];

            p0=y1;
            p3=y2;
            p1=phi_td*ky1/3+p0;
            p2=p3-phi_td*ky2/3;
            tpara=t_leg[legid]/(phi_td);
            double By= pow((1-tpara),3)*p0
                +3*pow((1-tpara),2)*tpara*p1
                +3*(1-tpara)*pow((tpara),2)*p2
                +pow((tpara),3)*p3;

            legTipPoints[1+legid*3] = By + yComOffset;

            // Planning Z trajectory in swing phase

            double secpt1=phi_td/2.0;
            double secpt2=phi_td*1/2.0;
            double secpt3=phi_td;

            if (t_leg[legid]<=secpt1)
            {
                double Tpsz=secpt1;
                if ((legid == 1) || (legid == 4))
                {
                    kz1=k_tdpi;
                    z1=z0+td_deltapi;
                } else
                {
                    kz1=k_td;
                    z1=z0+td_delta;
                }
                kz2=0;


                z2=z1+stepHeight;

                p0=z1;
                p3=z2;
                p1=Tpsz*kz1/3+p0;
                p2=p3-Tpsz*kz2/3;
                tpara=t_leg[legid]/(Tpsz);
                Bz=pow((1-tpara),3)*p0
                    +3*pow((1-tpara),2)*tpara*p1
                    +3*(1-tpara)*pow((tpara),2)*p2
                    +pow((tpara),3)*p3;

                legTipPoints[2+legid*3]=Bz;
            }
            if ((t_leg[legid]>secpt1)&(t_leg[legid]<=secpt2))
            {

                //legTipPoints[2+legid*3]=z0+td_delta+stepHeight;
                if ((legid == 1) || (legid == 4))
                {
                    legTipPoints[2+legid*3]=z0+td_deltapi+stepHeight;
                } else
                {
                    legTipPoints[2+legid*3]=z0+td_delta+stepHeight;
                }

            }
            if (t_leg[legid]>secpt2)
            {

                double Tpsz=secpt3-secpt2;
                kz1=0;
                if ((legid == 1) || (legid == 4))
                {
                    kz2=-k_tdpi;
                } else
                {
                    kz2=-k_td;
                }



                if ((legid == 1) || (legid == 4))
                {
                    z1=z0+td_deltapi+stepHeight;
                    z2=z0+td_deltapi;
                } else
                {
                    z1=z0+td_delta+stepHeight;
                    z2=z0+td_delta;
                }


                p0=z1;
                p3=z2;
                p1=Tpsz*kz1/3+p0;
                p2=p3-Tpsz*kz2/3;
                tpara=(t_leg[legid]-secpt2)/(Tpsz);
                Bz=    pow((1-tpara),3)*p0
                    +3*pow((1-tpara),2)*tpara*p1
                    +3*(1-tpara)*pow((tpara),2)*p2
                    +  pow((tpara),3)*p3;
                legTipPoints[2+legid*3]=Bz;
            }
        }
        if ((legid==2) || (legid==1) || (legid == 0))
        {
            //legTipPoints[0+legid*3]=legTipPoints[0+legid*3]-xdelta;
            legTipPoints[0+legid*3]=-legTipPoints[0+legid*3];
            legTipPoints[1+legid*3]=-legTipPoints[1+legid*3];

            legTipPoints[0+legid*3]=legTipPoints[0+legid*3]/1000.0;
            legTipPoints[1+legid*3]=legTipPoints[1+legid*3]/1000.0;
            legTipPoints[2+legid*3]=legTipPoints[2+legid*3]/1000.0;
        }
        else
        {
            legTipPoints[0+legid*3]=legTipPoints[0+legid*3]/1000.0;
            legTipPoints[1+legid*3]=legTipPoints[1+legid*3]/1000.0;
            legTipPoints[2+legid*3]=legTipPoints[2+legid*3]/1000.0;
        }
    }

    for( int i = 0; i < 6; i++)
    {
        double xTip = legTipPoints[i*3+0];
        double yTip = legTipPoints[i*3+1];
        double zTip = legTipPoints[i*3+2];
        double lTip = sqrt(xTip*xTip + yTip*yTip + zTip*zTip);
        legTipPositionPole[i*3 + 0] = -asin(xTip/lTip); // sagittal leg angle
        legTipPositionPole[i*3 + 1] = asin(yTip/lTip); // lateral  leg angle
        legTipPositionPole[i*3 + 2] = lTip; // leg length
    }

    return totalPeriodCount * T - timeFromStart;
}

}
