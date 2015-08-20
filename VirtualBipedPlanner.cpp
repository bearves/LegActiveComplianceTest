#include "VirtualBipedPlanner.h"
#include <iostream>

VirtualBipedPlanner::VirtualBipedPlanner()
{
}

VirtualBipedPlanner::~VirtualBipedPlanner()
{
}

int VirtualBipedPlanner::Initialize()
{
    th        = 0.001;    // s
    mass      = 200;      // kg
    height    = 0.85;     // meter
    gravity   = 9.81;     // m/s^2
    damp      = 800;     // N/(m/s) 
    kspr      = 300;      // N/m

    halfperiod= 1.6;      // second
    alpha     = 0.1;       
    beta      = 3.2;
    stepheight= 0.045;    // meter
    kpsw      = 72;  
    kdsw      = 20;  
    ksat      = 5000;
    satEffectRange = 0.05;

    u1        = 0.05;
    u2        = 0.08;
    w1        = 0.898;
    w2        = 0.9;

    acclimitStance = 1.0; // m/s/s
    acclimitSwing  = 1.6; // m/s/s
    vellimit       = 2;   // m/s
    poslimit       = 0.25; // m

    // Set intial condition for both horizontal dimensions
    for( int i = 0; i < 2; i++)
    {
        lastydot[i] = 0;
        thisydot[i] = 0;

        lasty[i] = 0;
        thisy[i] = 0;

        lastyswdot[i] = 0;
        thisyswdot[i] = 0;

        lastysw[i] = 0;
        thisysw[i] = 0;
    }

    hsw = 0;

    timeRatio = 0;
    stepCount = 0;
    startTime = 0;
    lastTransitionTime = 0;

    gaitState = VGS_READY;

    return 0;
}

int VirtualBipedPlanner::Start( double timeNow )
{
    if ( gaitState == VGS_READY )
    {
        startTime = timeNow;
        lastTransitionTime = startTime;
        gaitState = VGS_STARTED;
    }
    return 0;
}

int VirtualBipedPlanner::RequireStop( double timeNow )
{
    if ( gaitState == VGS_STARTED )
    {
        requireStopTime = timeNow;
        stepLeft = STEP_TO_COMPLETELY_STOP;
        gaitState = VGS_STOPPING;
    }
    return 0;
}

int VirtualBipedPlanner::DoIteration( 
    /*IN*/double timeNow, 
    /*IN*/double * fext, 
    /*OUT*/double *pgrp, 
    /*OUT*/double *pgrpdot )
{
    bool isaStepComplete = false;

    // automatically switch to STOPPED state
    if ( gaitState == VGS_STOPPING)
    {
        fext[0] = 0;
        fext[1] = 0;
    }

    if ( gaitState == VGS_READY )
    {
        for( int i = 0; i < 2; i++)
        {
            thisy[i]      = 0;
            thisydot[i]   = 0;
            thisysw[i]    = 0;
            thisyswdot[i] = 0;
        }

        hsw = 0;
    }
    else if ( gaitState == VGS_STARTED || gaitState == VGS_STOPPING)
    {
        timeRatio = (timeNow - lastTransitionTime) / halfperiod;
        if(timeRatio > 1)
        {
            timeRatio -= 1;
            lastTransitionTime = timeNow;
            isaStepComplete = true;
        }

        for( int i = 0; i < 2; i++)
        {
            // Discrete equation of spring-damped linear inverted pendulum
            yddot[i] = (-damp * lastydot[i] 
                    - kspr * lasty[i]
                    + alpha * mass * gravity / height * lasty[i]
                    + fext[i]) / mass;
                    //+ SaturateSpringDamper(lasty[i], poslimit, satEffectRange);

            yddot[i] = Saturate(yddot[i], acclimitStance);

            thisydot[i] = lastydot[i] + th * yddot[i];
            thisy[i]    = lasty[i]    + th * thisydot[i];
            thisy[i]    = Saturate(thisy[i], poslimit);

            // calculate the capture point
            this->GetSwingFootTarget();

            // PD tracking to move the swing foot to the capture point
            yswddot[i] = -kpsw * (lastysw[i] - ytarget[i]) - kdsw * (lastyswdot[i] - ytargetdot[i]);
            yswddot[i] = Saturate(yswddot[i], acclimitSwing);
            thisyswdot[i] = lastyswdot[i] + th * yswddot[i];
            thisysw[i] = lastysw[i] + th * thisyswdot[i];
            thisysw[i] = Saturate(thisysw[i], poslimit);

        }
        PlanningFootHeight();

        // State transition
        if (isaStepComplete)
        {
            this->StateTransition();
            stepLeft--;
            if ( gaitState == VGS_STOPPING && stepLeft <= 0)
                gaitState = VGS_STOPPED;
            stepCount++;
        }
    }
    else if ( gaitState == VGS_STOPPED )
    {
        // do nothing here, hold where it is
    }

    // Output of this virtual model
    this->AssignStateToCorrespondFoot();
    for (int i = 0; i < 2; i++)
    {
        pgrp[i * 3 + 0] = ygrp[i * 2 + 0];
        pgrp[i * 3 + 1] = hgrp[i];
        pgrp[i * 3 + 2] = ygrp[i * 2 + 1];
        pgrpdot[i * 3 + 0] = ygrpdot[i * 2 + 0];
        pgrpdot[i * 3 + 2] = ygrpdot[i * 2 + 1];
    }

    // State updating
    for (int i = 0; i < 2; i++)
    {
        lastysw[i] = thisysw[i];
        lastyswdot[i] = thisyswdot[i];
        lasty[i] = thisy[i];
        lastydot[i] = thisydot[i];
    }
    return 0;
}

double VirtualBipedPlanner::Saturate( double ainput, double limit )
{
    if (ainput > fabs(limit))
    {
        return fabs(limit);
    } 
    else if(ainput < -fabs(limit))
    {
        return -fabs(limit);
    }
    else
    {
        return ainput;
    }
}
double VirtualBipedPlanner::SaturateSpringDamper(double y, double limit, double effectRange)
{    
    if (y > fabs(limit) - fabs(effectRange))
    {
        double penetrate = y - (fabs(limit) - fabs(effectRange));
        return -ksat * penetrate;
    }
    else if ( y < -(fabs(limit) - fabs(effectRange)) )
    {
        double penetrate = -(fabs(limit) - fabs(effectRange)) - y;
        return ksat * penetrate;
    }
    return 0; 
}

int VirtualBipedPlanner::GetSwingFootTarget()
{
    double p = 0;
    for( int i = 0; i < 2; i++)
    {
        // target due to different situations
        ysw1[i]    = lastysw[i] + lastyswdot[i] * th;
        ysw1dot[i] = lastyswdot[i];
        ysw2[i]    = beta * sqrt(height / gravity) * thisydot[i];
        ysw2dot[i] = beta * sqrt(height / gravity) * yddot[i];
        ysw3[i]    = lastysw[i] - thisydot[i] * th;
        ysw3dot[i] = -thisydot[i];
        // do blending
        if (timeRatio < u1)
        {
            ytarget[i] = ysw1[i];
            ytargetdot[i] = ysw1dot[i];
        }
        else if (timeRatio < u2)
        {
            p = (timeRatio - u1) / (u2 - u1);
            ytarget[i] = ysw1[i] * (1-p) + ysw2[i] * p;
            ytargetdot[i] = ysw1dot[i] * (1-p) + ysw2dot[i] * p;
        }
        else if (timeRatio < w1)
        {
            ytarget[i] = ysw2[i];
            ytargetdot[i] = ysw2dot[i];
        }
        else if (timeRatio < w2)
        {
            p = (timeRatio - w1) / (w2 - w1);
            ytarget[i] = ysw2[i] * (1-p) + ysw3[i] * p;
            ytargetdot[i] = ysw2dot[i] * (1-p) + ysw3dot[i] * p;
        }
        else
        {
            ytarget[i] = ysw3[i];
            ytargetdot[i] = ysw3dot[i];
        }
    }
    return 0;
}

int VirtualBipedPlanner::StateTransition()
{
    for(int i = 0; i < 2; i++)
    {
        double tmp = thisy[i];
        double tmpdot = thisydot[i];
        thisy[i] = -thisysw[i];
        thisydot[i] = -thisyswdot[i];
        thisysw[i] = -tmp;
        thisyswdot[i] = -tmpdot;
    }
    return 0;
}

int VirtualBipedPlanner::PlanningFootHeight()
{
    double tacc = 0.3;
    double theta = 0;
    double pi = 3.14159265359;
    double acc = pi / tacc / (1 - tacc);
    double vlel = acc * tacc;
    if (timeRatio < tacc)
    {
        theta = 0.5 * (timeRatio * timeRatio) * acc;
    }
    else if (timeRatio < 1 - tacc)
    {
        theta = 0.5 * vlel * tacc + vlel * (timeRatio - tacc);
    }
    else
    {
        double tt = timeRatio - 1 + tacc;
        theta = vlel * (1 - 1.5 * tacc) + vlel * tt - 0.5 * tt * tt * acc;
    }
    hsw = stepheight * sin(theta);
    return 0;
}

int VirtualBipedPlanner::AssignStateToCorrespondFoot()
{
    int stanceLeg = 0;
    int swingLeg = 1;
    if (stepCount % 2 == 1)
    {
        stanceLeg = 1;
        swingLeg = 0;
    }
    for(int i = 0; i < 2; i++)
    {
        ygrp[stanceLeg * 2 + i] = -thisy[i];
        ygrpdot[stanceLeg * 2 + i] = -thisydot[i];
        ygrp[swingLeg * 2 + i] = thisysw[i];
        ygrpdot[swingLeg * 2 + i] = thisyswdot[i];
    }
    hgrp[stanceLeg] = 0;
    hgrp[swingLeg] = hsw;
    return 0;
}

