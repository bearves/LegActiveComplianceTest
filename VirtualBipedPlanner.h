#ifndef VIRTUAL_BIPED_PLANNER_H
#define VIRTUAL_BIPED_PLANNER_H

#include <cmath>

class VirtualBipedPlanner
{
public:

    static const int STEP_TO_COMPLETELY_STOP = 10;

    enum VIRTUAL_GAIT_STATE
    {
        VGS_READY    = 1,
        VGS_STARTED  = 2,
        VGS_STOPPING = 3,
        VGS_STOPPED  = 4
    };

    VirtualBipedPlanner();
    ~VirtualBipedPlanner();

    int Initialize();
    int Start(double timeNow);
    int RequireStop(double timeNow);
    // One dimensional at this time
    int DoIteration(/*IN*/double timeNow, /*IN*/double * fext, /*OUT*/double *pgrp, /*OUT*/double *pgrpdot);
    
    VIRTUAL_GAIT_STATE GetState() const { return gaitState;};

private:
    // State variables
    double yddot[2];
    double lastydot[2];
    double thisydot[2];
    
    double lasty[2];
    double thisy[2];
    
    double yswddot[2];
    double lastyswdot[2];
    double thisyswdot[2];

    double lastysw[2];
    double thisysw[2];

    double ytarget[2];
    double ytargetdot[2];

    double hsw;

    double timeRatio;
    int stepCount;
    double startTime;
    double requireStopTime;
    double lastTransitionTime;

    // Temp varaibles for calculate swing target
    double ysw1[2];
    double ysw1dot[2]; 
    double ysw2[2];    
    double ysw2dot[2]; 
    double ysw3[2];    
    double ysw3dot[2]; 

    // output variables
    double ygrp[4];
    double ygrpdot[4];
    double hgrp[2];

    // Physical parameters
    double th;        // time interval
    double mass;      // body mass
    double height;    // hip height
    double gravity;   // gravitational constant
    double damp;      // virtual damping
    double kspr;      // virtual string stiffness
    double halfperiod;// half gait period
    double alpha;     // coefficient of the gravity effect
    double beta;      // coefficient of the capture point
    double stepheight;// step height
    double kpsw;      // kp of swing leg tracking
    double kdsw;      // kd of swing leg tracking
    double ksat;      // virtual spring for limit the position range
    double satEffectRange; // effect range of the position limit spring

    double u1;        // for calculate swing leg target point
    double u2;
    double w1;
    double w2;

    double acclimitStance;
    double acclimitSwing;
    double vellimit;
    double poslimit;

    int stepLeft;

    VIRTUAL_GAIT_STATE gaitState;

    double Saturate(double ainput, double limit);
    double SaturateSpringDamper(double y, double limit, double effectRange);
    int GetSwingFootTarget();
    int StateTransition();
    int PlanningFootHeight();
    int AssignStateToCorrespondFoot();
};

#endif
