#include "Planners.h"
#include <algorithm>

using namespace RobotHighLevelControl;

GoToPointPlanner::GoToPointPlanner()
{
    m_state = UNREADY;
    // PTP moving should be accomplished in m_totalTime seconds
    m_totalTime = 5;
    m_timeAcc = 0.4;
    m_timeDcc = 0.4;
}

GoToPointPlanner::~GoToPointPlanner()
{
}

int GoToPointPlanner::SetStartAndEndPoint(double* currentPoint, double* destPoint)
{
    // the destination should be set when the planner has not gone moving
    if (m_state == READY || m_state == FINISHED || m_state == UNREADY)
    {
        for(int i = 0; i < 18; i++)
        {
            m_beginPointPosition[i] = currentPoint[i];
            m_destPointPosition[i] = destPoint[i];
        }
        m_state = READY;
    }
    return 0;
}

int GoToPointPlanner::Start(double timeNow)
{
    // only when the destination has been set, the online planning can be started
    if (m_state == READY)
    {
        m_state = INMOTION;
        m_timeWhenBeginToGo = timeNow;
    }
    return 0;
}

int GoToPointPlanner::GenerateJointTrajectory(double timeNow, double* currentPoint, double* jointLength)
{
    if (m_state == READY)
    {
        // Stay at the start point
        for(int i = 0; i < AXIS_NUMBER; i++)
        {
            jointLength[i] = m_beginPointPosition[i];
        }
    }
    else if (m_state == INMOTION)
    {
        // Planning to the destination
        double timeRatio = (timeNow - m_timeWhenBeginToGo)/m_totalTime;
        double posRatio = 0;
        double acc = -2 / (m_timeAcc * (m_timeAcc + m_timeDcc - 2));
        double dcc =  2 / (m_timeDcc * (m_timeAcc + m_timeDcc - 2));
        double velConstant = m_timeAcc * acc;
        timeRatio = timeRatio > 0 ? timeRatio : 0;
        if (timeRatio > 1)
        {
            // the PTP planning is finished
            m_state = FINISHED;
            timeRatio = 1;
        }

        if (timeRatio < m_timeAcc)
        {
            posRatio = 0.5 * acc * timeRatio * timeRatio;
        }
        else if (timeRatio < 1 - m_timeDcc)
        {
            posRatio = 0.5 * velConstant * m_timeAcc + velConstant * (timeRatio - m_timeAcc); 
        }
        else 
        {
            double t = (timeRatio - 1 + m_timeDcc);
            posRatio = 0.5 * velConstant * m_timeAcc + velConstant * (1 - m_timeDcc - m_timeAcc); 
            posRatio += velConstant * t + 0.5 * dcc * t * t;
        }
        for (int i = 0; i < AXIS_NUMBER; i++)
        {
            jointLength[i] = posRatio * m_destPointPosition[i] + (1 - posRatio) * m_beginPointPosition[i];
        }
        
    }
    else if (m_state == FINISHED)
    {
        // Stay at the destination 
        for(int i = 0; i < AXIS_NUMBER; i++)
        {
            jointLength[i] = m_destPointPosition[i];
        }
    }
    else if (m_state == UNREADY)
    {
        // Stay at current point
        for(int i = 0; i < AXIS_NUMBER; i++)
        {
            jointLength[i] = currentPoint[i];
        }
    }
    return 0;
}

const double ImpedancePlanner::FOOT_POS_UP_LIMIT[3]  = { Model::PI/9,  Model::PI/36, 0.76};
const double ImpedancePlanner::FOOT_POS_LOW_LIMIT[3] = {-Model::PI/9, -Model::PI/36, 0.5};
const double ImpedancePlanner::FORCE_DEADZONE[3]     = { 2.5, 2.5, 10 };
const int ImpedancePlanner::LEG_INDEX_GROUP_A[3] = {Model::Leg::LEG_ID_MB, Model::Leg::LEG_ID_RF, Model::Leg::LEG_ID_LF};
const int ImpedancePlanner::LEG_INDEX_GROUP_B[3] = {Model::Leg::LEG_ID_LB, Model::Leg::LEG_ID_RB, Model::Leg::LEG_ID_MF};
const double ImpedancePlanner::IMPD_RATIO_A[3] = {1.7, 1, 1};
const double ImpedancePlanner::IMPD_RATIO_B[3] = {1, 1, 1.7};
const char * ImpedancePlanner::SUB_STATE_NAME[8] =
{
    "HOLD_INIT_POS",
    "A_SP_B_LT",
    "A_TH_B_TD",
    "A_LT_B_LD",
    "A_LT_B_SP",
    "A_TD_B_TH",
    "A_LD_B_LT",
    "HOLD_END_POS"
};

ImpedancePlanner::ImpedancePlanner()
{
    for(int i = 0; i < 6; i++)
    {
        m_legList[i].SetID(i);
    }

    ResetInitialFootPos();

    ResetImpedanceParam(A_HARD_B_HARD);

    m_state = UNREADY;
}

ImpedancePlanner::~ImpedancePlanner()
{
}

int ImpedancePlanner::Initialize()
{
    if (m_state == UNREADY || m_state == FINISHED)
    {
        ResetInitialFootPos();
        m_state = READY;
        return 0;
    }
    else
    {
        return -1;
    }
}

int ImpedancePlanner::ResetInitialFootPos()
{
    GenerateReferenceTrj(
            0,
            GAIT_SUB_STATE::HOLD_INIT_POS,
            m_beginFootPos,
            m_beginFootVel
            );

    return 0;
}

int ImpedancePlanner::ResetImpedanceParam(int impedanceMode)
{ 
    double K_SOFT_LANDING[3] = {1e8, 1e8, 500};
    double B_SOFT_LANDING[3] = {1e5, 1e5, 1000};
    double M_SOFT_LANDING[3] = {100, 100, 20};

    double K_MEDIUM_SOFT[3] = {1e8, 1e8, 40000};
    double B_MEDIUM_SOFT[3] = {1e5, 1e5, 6000}; // actual damping ratio is much smaller than the desired
    double M_MEDIUM_SOFT[3] = {100, 100, 20};

    switch (impedanceMode)
    { 
        case A_HARD_B_HARD:
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j)
                { 
                    K_ac[LEG_INDEX_GROUP_A[i]][j] = K_MEDIUM_SOFT[j] * IMPD_RATIO_A[i];
                    B_ac[LEG_INDEX_GROUP_A[i]][j] = B_MEDIUM_SOFT[j] * sqrt(IMPD_RATIO_A[i]);
                    M_ac[LEG_INDEX_GROUP_A[i]][j] = M_MEDIUM_SOFT[j];

                    K_ac[LEG_INDEX_GROUP_B[i]][j] = K_MEDIUM_SOFT[j] * IMPD_RATIO_B[i];
                    B_ac[LEG_INDEX_GROUP_B[i]][j] = B_MEDIUM_SOFT[j] * sqrt(IMPD_RATIO_B[i]);
                    M_ac[LEG_INDEX_GROUP_B[i]][j] = M_MEDIUM_SOFT[j];
                }
            }
            break;
        case A_SOFT_B_SOFT:
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 3; ++j)
                { 
                    K_ac[i][j] = K_SOFT_LANDING[j];
                    B_ac[i][j] = B_SOFT_LANDING[j];
                    M_ac[i][j] = M_SOFT_LANDING[j];
                }
            }
            break;
        case A_HARD_B_SOFT:
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j)
                { 
                    K_ac[LEG_INDEX_GROUP_A[i]][j] = K_MEDIUM_SOFT[j] * IMPD_RATIO_A[i];
                    B_ac[LEG_INDEX_GROUP_A[i]][j] = B_MEDIUM_SOFT[j] * sqrt(IMPD_RATIO_A[i]);
                    M_ac[LEG_INDEX_GROUP_A[i]][j] = M_MEDIUM_SOFT[j];

                    K_ac[LEG_INDEX_GROUP_B[i]][j] = K_SOFT_LANDING[j];
                    B_ac[LEG_INDEX_GROUP_B[i]][j] = B_SOFT_LANDING[j];
                    M_ac[LEG_INDEX_GROUP_B[i]][j] = M_SOFT_LANDING[j];
                }
            }
            break;
        case A_SOFT_B_HARD:
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j)
                { 
                    K_ac[LEG_INDEX_GROUP_B[i]][j] = K_MEDIUM_SOFT[j] * IMPD_RATIO_B[i];
                    B_ac[LEG_INDEX_GROUP_B[i]][j] = B_MEDIUM_SOFT[j] * sqrt(IMPD_RATIO_B[i]);
                    M_ac[LEG_INDEX_GROUP_B[i]][j] = M_MEDIUM_SOFT[j];

                    K_ac[LEG_INDEX_GROUP_A[i]][j] = K_SOFT_LANDING[j];
                    B_ac[LEG_INDEX_GROUP_A[i]][j] = B_SOFT_LANDING[j];
                    M_ac[LEG_INDEX_GROUP_A[i]][j] = M_SOFT_LANDING[j];
                }
            }
            break;
    }
    return 0;
}

int ImpedancePlanner::GetInitialJointLength(double* jointLength)
{
    for(int i = 0; i < 6; i++)
    {
        m_legList[i].InverseSolutionPole(&m_beginFootPos[i*3], &jointLength[i*3], false);
    }
    return 0;
}

int ImpedancePlanner::Start(double timeNow)
{
    if (m_state == READY)
    {
        m_state = INMOTION;
        m_subState = HOLD_INIT_POS;
    }
    return 0;
}

int ImpedancePlanner::Stop()
{
    m_state = FINISHED;
    return 0;
}

int ImpedancePlanner::GenerateJointTrajectory(
        double timeNow, 
        double* currentPoint, 
        Aris::RT_CONTROL::CForceData* forceInput, 
        Aris::RT_CONTROL::CIMUData& imuFdbk,
        double* jointLength,
        char* controlDataForLog)
{
    // When the controller has not been started, stay at the initial place
    if (m_state == UNREADY || m_state == READY)
    {
        GetInitialJointLength(m_currentTargetJointPos);
        for(int i = 0; i < 18; i++)
        {
            m_currentAdjustedJointPos[i] = m_currentTargetJointPos[i];
            m_currentAdjustedFootPos[i]  = m_beginFootPos[i];

            m_currentAdjustedFootVel[i]  = 0;
            m_currentOffset[i]    = 0;
            m_currentOffsetdot[i] = 0;
            m_adjForceBP[i] = 0;

            m_lastOffset[i] = m_currentOffset[i];
            m_lastOffsetdot[i] = m_currentOffsetdot[i];
            jointLength[i] = m_currentAdjustedJointPos[i];
        }

        for(int i = 0; i < 3; i++)
        {
            m_currentIntegralValue[i] = 0;
            m_lastIntegralValue[i] = m_currentIntegralValue[i];
        }
        isOnGround = false;

        m_cmdFlag = GAIT_SUB_COMMAND::GSC_NOCMD;
        m_subState = HOLD_INIT_POS;
        ResetImpedanceParam(A_HARD_B_HARD);

        bodyVelLastTouchdown = 0;
        bodyVelNextLiftUp = 0;
    }
    else if (m_state == INMOTION)
    {
        
        // re-initial desire force
        for( int i = 0; i < 18; i++)
        {
            m_forceDesire[i] = 0;
        }
        // Do the force transformation
        for( int i = 0; i < 6; i++)
        {
            for( int j = 0; j < 3; j++)
            {
                m_forceRaw[i*3 + j] = forceInput[i].forceValues[j]/1000.0;
            }
            ForceTransform(&m_forceRaw[i*3], &m_currentAdjustedFootPos[i*3], &m_forceTransfromed[i*3]);
            DeadZone(&m_forceTransfromed[i*3]);
        }
        
        // State machine of the gait controller
        // and select the desired impedance
        DetermineCurrentState(timeNow, m_cmdFlag, m_subState, imuFdbk);

        // Calculate the pose adjustment force
        // Adjust desire force according to the IMU feedback
        CalculateAdjForceBP(imuFdbk, 
                m_currentAdjustedFootPos,
                m_lastErrValue,
                m_lastIntegralValue,
                m_currentIntegralValue, 
                m_adjForceBP,
                m_subState,
                timeNow - m_lastTouchDownTime);

        // Add saturation to Pose ajustment force
        for (int i = 0; i < 18; ++i) 
        {
            if (m_adjForceBP[i] > 2e3 )
            {
                m_adjForceBP[i] = 2e3;
            }
            else if (m_adjForceBP[i] < -2e3 )
            {
                m_adjForceBP[i] = -2e3;
            }
        }
        for (int i = 0; i < 18; ++i) 
        {
            // adjust the desire force 
            m_forceDesire[i] += m_adjForceBP[i];
        }
        for(int i = 0; i < 3; i++)
        {
            m_lastIntegralValue[i] = m_currentIntegralValue[i];
        }
        
        // Generate the reference trajectory 
        GenerateReferenceTrj(timeNow, m_subState, m_currentTargetFootPos, m_currentTargetFootVel);

        // Do the impedance adjustment
        for( int i = 0; i < 6; i++)
        {
            ImpedanceControl(&m_forceTransfromed[i*3], &m_forceDesire[i*3], 
                             &m_lastOffset[i*3], &m_lastOffsetdot[i*3],
                             &m_currentOffset[i*3], &m_currentOffsetdot[i*3], i);

            // Only test the length direction of the legs 
            for (int j = 0; j < 2; ++j) {
                m_currentOffsetdot[i*3+j] = 0;
                m_currentOffset[i*3+j] = 0;
            }
        }

        // Add the impdedance offset to the reference trj
        for( int i = 0; i < 18; i++)
        { 
            m_currentAdjustedFootPos[i] = m_currentTargetFootPos[i] - m_currentOffset[i];
            m_currentAdjustedFootVel[i] = m_currentTargetFootVel[i] - m_currentOffsetdot[i];
            m_lastOffset[i] = m_currentOffset[i];
            m_lastOffsetdot[i] = m_currentOffsetdot[i];
        }
        
        // Calculate the joint length using Inverse Kinematics
        for(int i = 0; i < 6; i++)
        {
            SaturateProcess(&m_currentAdjustedFootPos[i*3]);
            m_legList[i].InverseSolutionPole(&m_currentAdjustedFootPos[i*3], &m_currentAdjustedJointPos[i*3], false);
        }

        // display debug message and log some key data for analysis
        m_logData.gaitState = m_subState;
        for(int i = 0; i < 6; i++)
        {
            m_logData.targetPos[i] = m_currentTargetFootPos[i*3+2];
            m_logData.adjustedPos[i] = m_currentAdjustedFootPos[i*3+2];
            m_logData.legForceOnZ[i] = m_forceTransfromed[i*3+2];

            m_logData.targetAng[i] = m_currentAdjustedFootPos[i*3+0];
            m_logData.legForceOnX[i] = m_forceTransfromed[i*3+0];
        }
        memcpy(controlDataForLog, (void *)&m_logData, sizeof(ControllerLogData));

        if (fmod(timeNow, 0.5) < 1.1e-3)
        {
            for (int i = 0; i < 6; ++i)
            {
                rt_printf("Force desire: ");
                for (int j = 2; j < 3; ++j)
                {
                    rt_printf("FD: %7.6lf  ", m_forceDesire[i*3+j]);
                }
                for (int j = 2; j < 3; ++j)
                {
                    rt_printf("OF: %7.6lf  ", m_forceTransfromed[i*3+j]);
                }
                rt_printf("\n");

            }
            rt_printf("POSE: %7.6f  %7.6f  %7.6f\n", 
                    imuFdbk.EulerAngle[0], 
                    imuFdbk.EulerAngle[1], 
                    imuFdbk.EulerAngle[2]);
            rt_printf("K: %9f  %9f  %9f\n", 
                    K_ac[0][2], 
                    K_ac[1][2], 
                    K_ac[2][2]);
            rt_printf("B: %9f  %9f  %9f\n", 
                    B_ac[0][2], 
                    B_ac[1][2], 
                    B_ac[2][2]);
            rt_printf("M: %9f  %9f  %9f\n", 
                    M_ac[0][2], 
                    M_ac[1][2], 
                    M_ac[2][2]);
            rt_printf("State: %s\n", SUB_STATE_NAME[m_subState]);
            rt_printf("\n");
        }
        // Output
        for(int i = 0; i < 18; i++)
        {
            jointLength[i] = m_currentAdjustedJointPos[i];
        }
    }
    // When the controller has been stopped, the leg should held its current place
    else if (m_state == FINISHED)
    {
        // since the m_currentAdjustedJointPos will be no longer updated,
        // the leg will be held at that place
        for(int i = 0; i < 18; i++)
        {
            m_currentOffset[i] = 0;
            m_currentOffsetdot[i] = 0;
            m_lastOffset[i] = m_currentOffset[i];
            m_lastOffsetdot[i] = m_currentOffsetdot[i];
            jointLength[i] = m_currentAdjustedJointPos[i];
        }
    }
    return 0;
}

int ImpedancePlanner::ForceTransform(double* forceRaw, double* legPositionEstimated, double* forceTransformed)
{
    using namespace Model;
    // Firstly, the x and z direction on the actual force sensor are opposite to the matlab simulink models
    double fx = -forceRaw[0];
    double fy = forceRaw[1];
    double fz = -forceRaw[2];
    double l = legPositionEstimated[2];
    double a0 = (LG1 + LG3)/1000.0;
    double b0 = LG2 / 1000.0;
    // Secondly, we need to rotate the z direction towards the hip joint
    double tmp = (b0*b0-a0*a0-l*l)/(-2 * a0 * l);
    double alpha = 0;
    if ( fabs(tmp) < 1.0)
        alpha = acos(tmp);
    double sa = std::sin(alpha);
    double ca = std::cos(alpha);
    forceTransformed[2] = ca * fz - sa * fx;
    forceTransformed[0] = -(ca * fx + sa * fz) * l;
    forceTransformed[1] = fy * l;

    return 0;
}

int ImpedancePlanner::ImpedanceControl(double* forceInput, double* forceDesire,
        double* lastOffset, double* lastOffsetdot,
        double* currentOffset, double* currentOffsetdot, int legID)
{
    double K_use[3];
    double B_use[3];
    double M_use[3];
    double deltaF[3]; 

    for (int i = 0; i < 3; i++)
    {
        K_use[i] = K_ac[legID][i];
        B_use[i] = B_ac[legID][i];
        M_use[i] = M_ac[legID][i];
    }
    
    for (int i = 0; i < 3; ++i) 
    {
        deltaF[i] = forceInput[i] - forceDesire[i];
    }
    double th = 0.001;
    double xddot[3];
    for (int i = 0; i < 3; ++i) 
    {
        xddot[i] = (deltaF[i] - K_use[i] * lastOffset[i] - B_use[i] * lastOffsetdot[i]) / M_use[i];
        currentOffsetdot[i] = lastOffsetdot[i] + xddot[i] * th;
        currentOffset[i] = lastOffset[i] + currentOffsetdot[i] * th;
    }

    return 0;
}

int ImpedancePlanner::SaturateProcess(double* adjustedFootPos)
{
    for (int i = 0; i < 3; ++i) {
        if (adjustedFootPos[i] < FOOT_POS_LOW_LIMIT[i])
        {
            adjustedFootPos[i] = FOOT_POS_LOW_LIMIT[i];
        }
        else if (adjustedFootPos[i] > FOOT_POS_UP_LIMIT[i])
        {
            adjustedFootPos[i] = FOOT_POS_UP_LIMIT[i];
        }
    }
    return 0;
}

int ImpedancePlanner::DeadZone(double* force)
{
    for (int i = 0; i < 3; ++i) {
        if(fabs(force[i]) < FORCE_DEADZONE[i])
            force[i] = 0;
    }
    return 0;
}

int ImpedancePlanner::CalculateAdjForceBP(
        const Aris::RT_CONTROL::CIMUData &imuFdbk, 
        double* currentFootPos,
        double* lastErrorValue,
        double* lastIntegralValue,
        double* currentIntegralValue,
        double* adjForceBP,
        GAIT_SUB_STATE gaitState,
        double  tdTimeInterval)
{
                      //Roll, Pitch, Height
    double KP_BP[3] = { 20000,  20000,     0};
    double KI_BP[3] = {  2000,   2000,     0};
    double KD_BP[3] = {  1500,   2000,     0};
    double force[3];
    double th = 0.001;

    double currentErrorValue[3];
    double currentErrorDotValue[3];
    // Firstly calcualte the errors
    double crtHeight, errHeight, errHeightDot;
    crtHeight = CalculateCurrentHeight(currentFootPos, gaitState);

    errHeight = crtHeight - 0.71; // CURRENT STAGE: We fix the body height to 0.71m
    errHeightDot = (errHeight - lastErrorValue[2]) / th;

    currentErrorValue[2] = errHeight;
    currentErrorDotValue[2] = errHeightDot;

    for(int i = 0; i < 2; i++) // Get the error and dot error from IMU data
    {
        currentErrorValue[i] = imuFdbk.EulerAngle[i]; 
        currentErrorDotValue[i] = imuFdbk.AngularVel[i];
    }

    // PID control for body pose balance
    for (int i = 0; i < 3; ++i) 
    {
        // trapezoidal integration
        currentIntegralValue[i] = lastIntegralValue[i] +
            th * (lastErrorValue[i] + currentErrorValue[i]) /2;

        force[i] =   KP_BP[i] * currentErrorValue[i] 
                   + KI_BP[i] * currentIntegralValue[i]
                   + KD_BP[i] * currentErrorDotValue[i];
        lastErrorValue[i] = currentErrorValue[i]; 
    }

    // Gravity Compensation of body height
    if (tdTimeInterval < 0.06 && tdTimeInterval > 0)
    {
        force[2] += -9.81*268*tdTimeInterval;
    }
    else
    {
        force[2] += -9.81 * 268;
    }

    // Force distribution
    using Model::Leg;
    // Clear the last assignment at first
    for (int i = 0; i < 6; i++)
    {
        adjForceBP[i*3 + 0] = 0;
        adjForceBP[i*3 + 1] = 0;
        adjForceBP[i*3 + 2] = 0;
    } 
    // assign adjust force to corresponding legs
    switch (gaitState)
    {
        case GAIT_SUB_STATE::HOLD_INIT_POS:
            break;
        case GAIT_SUB_STATE::A_SP_B_LT:
        case GAIT_SUB_STATE::A_TH_B_TD:
            adjForceBP[Leg::LEG_ID_RF*3 + 2] = -1.401 * force[0] - 0.535 * force[1] - 0.288 * force[2];
            adjForceBP[Leg::LEG_ID_LF*3 + 2] =  1.401 * force[0] - 0.535 * force[1] - 0.288 * force[2];
            adjForceBP[Leg::LEG_ID_MB*3 + 2] =      0 * force[0] + 1.070 * force[1] - 0.426 * force[2];
            break;
        case GAIT_SUB_STATE::A_LT_B_LD:
            break;
        case GAIT_SUB_STATE::A_LT_B_SP:
        case GAIT_SUB_STATE::A_TD_B_TH:
            adjForceBP[Leg::LEG_ID_RB*3 + 2] = -1.401 * force[0] + 0.535 * force[1] - 0.288 * force[2];
            adjForceBP[Leg::LEG_ID_LB*3 + 2] =  1.401 * force[0] + 0.535 * force[1] - 0.288 * force[2];
            adjForceBP[Leg::LEG_ID_MF*3 + 2] =      0 * force[0] - 1.070 * force[1] - 0.426 * force[2];
            break;
        case GAIT_SUB_STATE::A_LD_B_LT:
            break;
        case GAIT_SUB_STATE::HOLD_END_POS:
            break;
    }
    return 0;
}

double ImpedancePlanner::CalculateCurrentHeight(double* currentFootPos, GAIT_SUB_STATE gaitState)
{
    using Model::Leg;
    double height = standingHeight - 0.2;

    switch (gaitState)
    {
        case GAIT_SUB_STATE::HOLD_INIT_POS:
        case GAIT_SUB_STATE::HOLD_END_POS:
            for(int i = 0; i < 6; i++)
            {
                height = std::max(currentFootPos[i * 3 + 2], height);
            }
            break;
        case GAIT_SUB_STATE::A_SP_B_LT:
        case GAIT_SUB_STATE::A_TH_B_TD:
        case GAIT_SUB_STATE::A_LT_B_LD:
            for(int i = 0; i < 3; i++)
            {
                height = std::max(currentFootPos[LEG_INDEX_GROUP_A[i] * 3 + 2], height);
            }
            break;
        case GAIT_SUB_STATE::A_LT_B_SP:
        case GAIT_SUB_STATE::A_TD_B_TH:
        case GAIT_SUB_STATE::A_LD_B_LT:
            for(int i = 0; i < 3; i++)
            {
                height = std::max(currentFootPos[LEG_INDEX_GROUP_B[i] * 3 + 2], height);
            }
            break;
    }
    return height;
}

void ImpedancePlanner::DetermineCurrentState(
        double timeNow, 
        GAIT_SUB_COMMAND& cmdFlag, 
        GAIT_SUB_STATE& currentState,
        const Aris::RT_CONTROL::CIMUData& imuData)
{
    switch (currentState)
    {
        case GAIT_SUB_STATE::HOLD_INIT_POS:
            if (cmdFlag == GAIT_SUB_COMMAND::GSC_START)
            {
                currentState = GAIT_SUB_STATE::A_SP_B_LT;
                m_lastStateShiftTime = timeNow;
                m_lastLiftUpTime     = timeNow;
                m_lastTouchDownTime  = timeNow;
                cmdFlag = GAIT_SUB_COMMAND::GSC_NOCMD; // clear the command flag

                // Reset Impedance param and clear the offsets 
                ResetImpedanceParam(A_HARD_B_HARD);
                ClearImpedanceStates("A");
                // Also clear the pose balance states
                ClearBalancePIDStates();
                
                // Update the ref and act pos and vel at the state shift moment
                UpdateTransitionPosVel();
                UpdateTouchDownLegPosVel(imuData);
                UpdateLiftUpLegPosVel();
                
                // Update the body vel at next lift up for the stance leg
                CalculateBodyVelNextLiftUp("A");

            }
            break;
            
        case GAIT_SUB_STATE::A_SP_B_LT:
            if (timeNow - m_lastStateShiftTime > Tset)
            {
                if (cmdFlag == GAIT_SUB_COMMAND::GSC_STOP)
                {
                    currentState = GAIT_SUB_STATE::HOLD_END_POS;
                    m_lastStateShiftTime = timeNow;
                    cmdFlag = GAIT_SUB_COMMAND::GSC_NOCMD; // clear the command flag
                    
                    // Reset Impedance param and clear the offsets 
                    ResetImpedanceParam(A_HARD_B_HARD);
                    ClearImpedanceStates("A");
                    ClearImpedanceStates("B");
                    ClearBalancePIDStates();
                }
                else
                {
                    ResetImpedanceParam(A_HARD_B_SOFT);
                    currentState = GAIT_SUB_STATE::A_TH_B_TD;
                    m_lastStateShiftTime = timeNow;
                }
                UpdateTransitionPosVel();
            }
            break;

        case GAIT_SUB_STATE::A_TH_B_TD:
            if (timeNow - m_lastStateShiftTime > Tth)
            {
                currentState = GAIT_SUB_STATE::A_LT_B_LD;
                m_lastStateShiftTime = timeNow;
                m_lastLiftUpTime     = timeNow;

                // Reset Impedance param and clear the offsets 
                ResetImpedanceParam(A_HARD_B_SOFT);
                ClearBalancePIDStates();
                UpdateTransitionPosVel();
                // save the shift pos and vel for continous planning of swing legs
                UpdateLiftUpLegPosVel();
            }
            break;

        case GAIT_SUB_STATE::A_LT_B_LD:
            if (AllLegOnGround("B") || timeNow - m_lastStateShiftTime > Tfly)
            {
                currentState = GAIT_SUB_STATE::A_LT_B_SP;
                m_lastStateShiftTime = timeNow;
                m_lastTouchDownTime  = timeNow;
                UpdateTransitionPosVel();
                UpdateTouchDownLegPosVel(imuData);
                ResetImpedanceParam(A_HARD_B_HARD);
                ClearBalancePIDStates();
                ClearImpedanceStates("B");
                CalculateBodyVelNextLiftUp("B");

            }
            break;

        case GAIT_SUB_STATE::A_LT_B_SP:
            if (timeNow - m_lastStateShiftTime > Tset)
            {
                currentState = GAIT_SUB_STATE::A_TD_B_TH;
                m_lastStateShiftTime = timeNow;
                ResetImpedanceParam(A_SOFT_B_HARD);
                UpdateTransitionPosVel();
            }
            break;

        case GAIT_SUB_STATE::A_TD_B_TH:
            if (timeNow - m_lastStateShiftTime > Tth)
            {
                currentState = GAIT_SUB_STATE::A_LD_B_LT;
                m_lastStateShiftTime = timeNow;
                m_lastLiftUpTime     = timeNow;

                // Reset Impedance param and clear the offsets 
                ResetImpedanceParam(A_SOFT_B_HARD);
                ClearBalancePIDStates();
                UpdateTransitionPosVel();
                // save the shift pos and vel for continous planning of swing legs
                UpdateLiftUpLegPosVel();
            }
            break;

        case GAIT_SUB_STATE::A_LD_B_LT:
            if (AllLegOnGround("A") || timeNow - m_lastStateShiftTime > Tfly)
            {
                currentState = GAIT_SUB_STATE::A_SP_B_LT;
                m_lastStateShiftTime = timeNow;
                m_lastTouchDownTime  = timeNow;
                UpdateTransitionPosVel();
                UpdateTouchDownLegPosVel(imuData);
                ResetImpedanceParam(A_HARD_B_HARD);
                ClearBalancePIDStates();
                ClearImpedanceStates("A");
                CalculateBodyVelNextLiftUp("A");
            }
            break;

        case GAIT_SUB_STATE::HOLD_END_POS:
            if (cmdFlag == GAIT_SUB_COMMAND::GSC_START)
            {
                currentState = GAIT_SUB_STATE::A_SP_B_LT;
                m_lastStateShiftTime = timeNow;
                m_lastLiftUpTime     = timeNow;

                cmdFlag = GAIT_SUB_COMMAND::GSC_NOCMD; // clear the command flag
                // Reset Impedance param and clear the offsets 
                ResetImpedanceParam(A_HARD_B_HARD);
                ClearImpedanceStates("A");
                ClearBalancePIDStates();
                UpdateTransitionPosVel();
                UpdateLiftUpLegPosVel();
            }
            break;
    }
}

int ImpedancePlanner::SetGaitParameter(const void* param, int dataLength)
{
    if (param == NULL || dataLength < sizeof(ParamCXB))
    {
        rt_printf("Wrong param data received\n");
        return -1;
    }

    const ParamCXB *p_paramCXB = (const ParamCXB *)param;
    rt_printf("Set Param: %d  %.2lf  %.2lf  %.2lf\n", 
            p_paramCXB->gaitCommand, 
            p_paramCXB->duty, 
            p_paramCXB->stepHeight,
            p_paramCXB->standHeight);

    if (p_paramCXB->gaitCommand == GAIT_SUB_COMMAND::GSC_START)
    {
        rt_printf("Gait START cmd received\n");
        m_cmdFlag = GAIT_SUB_COMMAND::GSC_START;
    }
    else if (p_paramCXB->gaitCommand == GAIT_SUB_COMMAND::GSC_STOP)
    {
        rt_printf("Gait STOP cmd received\n");
        m_cmdFlag = GAIT_SUB_COMMAND::GSC_STOP;
    }
   
    return 0;
}

void ImpedancePlanner::ClearImpedanceStates(const char* legGroupName)
{
    const int* groupList;
    if (*legGroupName == 'A')
    {
        groupList = LEG_INDEX_GROUP_A;
    }
    else if (*legGroupName == 'B')
    {
        groupList = LEG_INDEX_GROUP_B;
    }

    for(int i = 0; i < 3; i++)
    {
        for( int j = 0; j < 3; j ++)
        {
            m_lastOffset[groupList[i]*3 + j] = 0;
            m_lastOffsetdot[groupList[i]*3 + j] *= 0.5;
        }
    }
}

void ImpedancePlanner::ClearBalancePIDStates()
{
    for(int i = 0; i < 3; i++)
    {
        m_lastErrValue[i] = 0;
        m_lastIntegralValue[i] = 0;
        m_currentIntegralValue[i] = 0;
    }
}

void ImpedancePlanner::GenerateReferenceTrj(
        double timeNow,
        GAIT_SUB_STATE gaitState, 
        double* targetFootPos, 
        double* targetFootVel)
{
    switch (gaitState)
    {
        case GAIT_SUB_STATE::HOLD_INIT_POS:
            // Hold at the initial position
            for(int i = 0; i < 3; i++)
            {
                // Positions
                targetFootPos[LEG_INDEX_GROUP_A[i]*3 + 0] = 0;
                targetFootPos[LEG_INDEX_GROUP_A[i]*3 + 1] = 0;
                targetFootPos[LEG_INDEX_GROUP_A[i]*3 + 2] = standingHeight - stepLDHeight;

                targetFootPos[LEG_INDEX_GROUP_B[i]*3 + 0] = 0;
                targetFootPos[LEG_INDEX_GROUP_B[i]*3 + 1] = 0;
                targetFootPos[LEG_INDEX_GROUP_B[i]*3 + 2] = standingHeight - stepLDHeight;

                // Velocities
                targetFootVel[LEG_INDEX_GROUP_A[i]*3 + 0] = 0;
                targetFootVel[LEG_INDEX_GROUP_A[i]*3 + 1] = 0;
                targetFootVel[LEG_INDEX_GROUP_A[i]*3 + 2] = 0;

                targetFootVel[LEG_INDEX_GROUP_B[i]*3 + 0] = 0;
                targetFootVel[LEG_INDEX_GROUP_B[i]*3 + 1] = 0;
                targetFootVel[LEG_INDEX_GROUP_B[i]*3 + 2] = 0;
            }
            
            break;
        case GAIT_SUB_STATE::A_SP_B_LT:
            for(int i = 0; i < 3; i++)
            {
                // B Liftup and swing
                int index = LEG_INDEX_GROUP_B[i];
                SwingReferenceTrj(
                        timeNow,   m_lastLiftUpTime, m_lastTouchDownTime,
                        &m_lastLiftRefPos[index*3], &m_lastLiftRefVel[index*3],
                        &targetFootPos[index*3],    &targetFootVel[index*3],
                        index >= 3);

                // leg angles of side swing legs are not changed
                for(int j = 1; j < 2; j++)
                {
                    targetFootPos[index*3 + j] = m_lastShiftRefPos[index*3 + j];
                    targetFootVel[index*3 + j] = m_lastShiftRefVel[index*3 + j];
                }
                
                // A retract very little from the actual TD place 
                index = LEG_INDEX_GROUP_A[i];

                double tt = (timeNow - m_lastStateShiftTime) / Tset;
                double estimateTouchdownVel = (m_lastTouchDownTime - m_lastLiftUpTime) * -9.81 * 0.1;

                Model::HermitInterpolate(
                        Tset,
                        m_lastShiftActPos[index*3 + 2], 
                        estimateTouchdownVel, 
                        m_lastShiftActPos[index*3 + 2],
                        -estimateTouchdownVel/2, 
                        tt, 
                        targetFootPos[index*3+2], 
                        targetFootVel[index*3+2]);

                // leg angle planning for supporting leg
                StanceAngleReferenceTrj(
                        timeNow,  m_lastTouchDownTime,  m_lastTdActPos[index*3 + 2],
                        m_lastTdActPos[index*3+0],  m_lastTdActVel[index*3+0],
                        bodyVelLastTouchdown,  bodyVelNextLiftUp,
                        targetFootPos[index*3+0], targetFootVel[index*3+0],
                        index >= 3);

                for(int j = 1; j < 2; j++)
                {
                    targetFootPos[index*3 + j] = m_lastShiftActPos[index*3 + j];
                    targetFootVel[index*3 + j] = 0; 
                }
            }
            break;

        case GAIT_SUB_STATE::A_TH_B_TD:
            for(int i = 0; i < 3; i++)
            {
                // B swing
                int index = LEG_INDEX_GROUP_B[i];
                SwingReferenceTrj(
                        timeNow,   m_lastLiftUpTime, m_lastTouchDownTime,
                        &m_lastLiftRefPos[index*3], &m_lastLiftRefVel[index*3],
                        &targetFootPos[index*3],    &targetFootVel[index*3],
                        index >= 3);

                // leg angles of swing leg are not changed
                for(int j = 1; j < 2; j++)
                {
                    targetFootPos[index*3 + j] = m_lastShiftRefPos[index*3 + j];
                    targetFootVel[index*3 + j] = m_lastShiftRefVel[index*3 + j];
                }
                
                // A extend to the original standing length 
                index = LEG_INDEX_GROUP_A[i];

                double tt = (timeNow - m_lastStateShiftTime) / Tth;

                CalculateTHLength(index, m_lastTdActPos, m_lastTdBodyOrient, "A", stepTHHeight);

                Model::HermitInterpolate(
                        Tth,
                        m_lastShiftRefPos[index*3 + 2], 
                        m_lastShiftRefVel[index*3 + 2], 
                        m_lastShiftRefPos[index*3 + 2] + stepTHHeight, 
                        -0, 
                        tt, 
                        targetFootPos[index*3+2], 
                        targetFootVel[index*3+2]);

                // leg angle planning for supporting leg
                StanceAngleReferenceTrj(
                        timeNow,  m_lastTouchDownTime,  m_lastTdActPos[index*3 + 2],
                        m_lastTdActPos[index*3+0],  m_lastTdActVel[index*3+0],
                        bodyVelLastTouchdown,  bodyVelNextLiftUp,
                        targetFootPos[index*3+0], targetFootVel[index*3+0],
                        index >= 3);

                for(int j = 1; j < 2; j++)
                {
                    targetFootPos[index*3 + j] = m_lastShiftRefPos[index*3 + j];
                    targetFootVel[index*3 + j] = m_lastShiftRefVel[index*3 + j];
                }
            }
            break;

        case GAIT_SUB_STATE::A_LT_B_LD:
            for(int i = 0; i < 3; i++)
            {
                // B hold at the pre-landing length
                int index = LEG_INDEX_GROUP_B[i];
                targetFootPos[index*3+2] = standingHeight - stepLDHeight;
                targetFootVel[index*3+2] = 0;

                // meanwhile, B swing back trying to cancel the relative speed with the ground
                // leg angles of swing leg are not changed
                for(int j = 0; j < 2; j++)
                {
                    targetFootPos[index*3 + j] = m_lastShiftRefPos[index*3 + j] +
                                                 m_lastShiftRefVel[index*3 + j] * (timeNow - m_lastStateShiftTime);
                    targetFootVel[index*3 + j] = m_lastShiftRefVel[index*3 + j];
                }
                
                // A liftup and swing
                index = LEG_INDEX_GROUP_A[i];

                SwingReferenceTrj(
                        timeNow,   m_lastLiftUpTime, m_lastTouchDownTime,
                        &m_lastLiftRefPos[index*3], &m_lastLiftRefVel[index*3],
                        &targetFootPos[index*3],    &targetFootVel[index*3],
                        index >= 3);

                for(int j = 1; j < 2; j++)
                {
                    targetFootPos[index*3 + j] = m_lastShiftRefPos[index*3 + j];
                    targetFootVel[index*3 + j] = m_lastShiftRefVel[index*3 + j];
                }
            }

            break;

        case GAIT_SUB_STATE::A_LT_B_SP:
            for(int i = 0; i < 3; i++)
            {
                // A Liftup and swing
                int index = LEG_INDEX_GROUP_A[i];
                SwingReferenceTrj(
                        timeNow,   m_lastLiftUpTime, m_lastTouchDownTime,
                        &m_lastLiftRefPos[index*3], &m_lastLiftRefVel[index*3],
                        &targetFootPos[index*3],    &targetFootVel[index*3],
                        index >= 3);

                // leg angles of swing leg are not changed
                for(int j = 1; j < 2; j++)
                {
                    targetFootPos[index*3 + j] = m_lastShiftRefPos[index*3 + j];
                    targetFootVel[index*3 + j] = m_lastShiftRefVel[index*3 + j];
                }
                
                // B retract very little from the actual TD place 
                index = LEG_INDEX_GROUP_B[i];

                double tt = (timeNow - m_lastStateShiftTime) / Tset;
                double estimateTouchdownVel = (m_lastTouchDownTime - m_lastLiftUpTime) * -9.81 * 0.1;

                Model::HermitInterpolate(
                        Tset,
                        m_lastShiftActPos[index*3 + 2], 
                        estimateTouchdownVel,
                        m_lastShiftActPos[index*3 + 2],
                        -estimateTouchdownVel/2, 
                        tt, 
                        targetFootPos[index*3+2], 
                        targetFootVel[index*3+2]);
                 
                // leg angle planning for supporting leg
                StanceAngleReferenceTrj(
                        timeNow,  m_lastTouchDownTime,  m_lastTdActPos[index*3 + 2],
                        m_lastTdActPos[index*3+0],  m_lastTdActVel[index*3+0],
                        bodyVelLastTouchdown,  bodyVelNextLiftUp,
                        targetFootPos[index*3+0], targetFootVel[index*3+0],
                        index >= 3);

                for(int j = 1; j < 2; j++)
                {
                    targetFootPos[index*3 + j] = m_lastShiftActPos[index*3 + j];
                    targetFootVel[index*3 + j] = 0; 
                }
            }
            break;

        case GAIT_SUB_STATE::A_TD_B_TH:
            for(int i = 0; i < 3; i++)
            {
                // A swing
                int index = LEG_INDEX_GROUP_A[i];
                SwingReferenceTrj(
                        timeNow,   m_lastLiftUpTime, m_lastTouchDownTime,
                        &m_lastLiftRefPos[index*3], &m_lastLiftRefVel[index*3],
                        &targetFootPos[index*3],    &targetFootVel[index*3],
                        index >= 3);

                // leg angles of swing leg are not changed
                for(int j = 1; j < 2; j++)
                {
                    targetFootPos[index*3 + j] = m_lastShiftRefPos[index*3 + j];
                    targetFootVel[index*3 + j] = m_lastShiftRefVel[index*3 + j];
                }
                
                // B extend to the original standing length 
                index = LEG_INDEX_GROUP_B[i];

                double tt = (timeNow - m_lastStateShiftTime) / Tth;

                // leg angle planning for supporting leg
                StanceAngleReferenceTrj(
                        timeNow,  m_lastTouchDownTime,  m_lastTdActPos[index*3 + 2],
                        m_lastTdActPos[index*3+0],  m_lastTdActVel[index*3+0],
                        bodyVelLastTouchdown,  bodyVelNextLiftUp,
                        targetFootPos[index*3+0], targetFootVel[index*3+0],
                        index >= 3);

                CalculateTHLength(index, m_lastTdActPos, m_lastTdBodyOrient, "B", stepTHHeight);

                Model::HermitInterpolate(
                        Tth,
                        m_lastShiftRefPos[index*3 + 2], 
                        m_lastShiftRefVel[index*3 + 2], 
                        m_lastShiftRefPos[index*3 + 2] + stepTHHeight, 
                        0, 
                        tt, 
                        targetFootPos[index*3+2], 
                        targetFootVel[index*3+2]);

                for(int j = 1; j < 2; j++)
                {
                    targetFootPos[index*3 + j] = m_lastShiftRefPos[index*3 + j];
                    targetFootVel[index*3 + j] = m_lastShiftRefVel[index*3 + j];
                }
            }
            break;

        case GAIT_SUB_STATE::A_LD_B_LT:
            for(int i = 0; i < 3; i++)
            {
                // A hold at the pre-landing length
                int index = LEG_INDEX_GROUP_A[i];
                targetFootPos[index*3+2] = standingHeight - stepLDHeight;
                targetFootVel[index*3+2] = 0;

                for(int j = 0; j < 2; j++)
                {
                    targetFootPos[index*3 + j] = m_lastShiftRefPos[index*3 + j] +
                                                 m_lastShiftRefVel[index*3 + j] * (timeNow - m_lastStateShiftTime);
                    targetFootVel[index*3 + j] = m_lastShiftRefVel[index*3 + j];
                }
                
                // B liftup and swing
                index = LEG_INDEX_GROUP_B[i];

                SwingReferenceTrj(
                        timeNow,   m_lastLiftUpTime, m_lastTouchDownTime,
                        &m_lastLiftRefPos[index*3], &m_lastLiftRefVel[index*3],
                        &targetFootPos[index*3],    &targetFootVel[index*3], 
                        index >= 3);

                for(int j = 1; j < 2; j++)
                {
                    targetFootPos[index*3 + j] = m_lastShiftRefPos[index*3 + j];
                    targetFootVel[index*3 + j] = m_lastShiftRefVel[index*3 + j];
                }
            }
            break;

        case GAIT_SUB_STATE::HOLD_END_POS:
            for(int i = 0; i < 3; i++)
            {
                // A hold at the last transition actual place
                int index = LEG_INDEX_GROUP_A[i];
                for(int j = 0; j < 3; j++)
                {
                    targetFootPos[index*3 + j] = m_lastShiftActPos[index*3 + j];
                    targetFootVel[index*3 + j] = m_lastShiftActVel[index*3 + j];
                }
                
                // B hold at the last transition actual place
                index = LEG_INDEX_GROUP_B[i];
                for(int j = 0; j < 3; j++)
                {
                    targetFootPos[index*3 + j] = m_lastShiftActPos[index*3 + j];
                    targetFootVel[index*3 + j] = m_lastShiftActVel[index*3 + j];
                }
            }
            break;
    }
}

void ImpedancePlanner::StanceAngleReferenceTrj(
        double timeNow, double  lastTDTime, double lenAtTd,
        double angAtTd, double angVelAtTd,
        double bodyVelAtTd, double bodyVelNextLt,
        double& angRef, double& angVelRef, bool isFront)
{
    double angVelBegin = -bodyVelAtTd * cos (angAtTd) / lenAtTd;
    double angVelEnd   = -bodyVelNextLt / lenAtTd;
    double Tstance     = Tset + Tth;

    if (!isFront)
    {
        angVelBegin = -angVelBegin;
        angVelEnd = -angVelEnd;
    }

    double angEnd = angAtTd + (angVelBegin + angVelEnd) / 2.0 * Tstance;

    double tr = (timeNow - lastTDTime) / (Tset + Tth);
    Model::HermitInterpolate(
            Tstance,
            angAtTd,
            angVelBegin,
            angEnd,
            angVelEnd,
            tr,
            angRef, 
            angVelRef);
}

void ImpedancePlanner::SwingReferenceTrj(
        double timeNow, double lastLiftTime, double lastTDTime,
        double* posAtLift, double* velAtLift,
        double* posRef,    double* velRef, bool isFront)
{
    double tr = (timeNow - lastLiftTime) / Trt;

    double Text = (Tset + Tth - Trt + (lastTDTime - lastLiftTime) - 0.0);
    double tk = (timeNow - lastLiftTime - Trt) / Text;

    double Tbackward = Tfly;
    double Tforward  = (Tset + Tth - Tbackward + (lastTDTime - lastLiftTime) - 0.02);
    double tb = (timeNow - lastLiftTime) / Tbackward;
    double tf = (timeNow - Tbackward - lastLiftTime) / Tforward;
    double tm = (timeNow - Tbackward - Tforward - lastLiftTime) / Tforward;
    double tdAngle, tdAngVel;
    

    // Planning leg length
    if (tr < 1)  // retracting phase
    {
        Model::HermitInterpolate(
                Trt,
                posAtLift[2], velAtLift[2], 
                standingHeight - stepHeight, 0, 
                tr, 
                posRef[2], velRef[2]);
    }
    else if (tk < 1) // extending phase
    {
        Model::Spline2SegInterpolate(
                Text,
                standingHeight - stepHeight, 0, 
                standingHeight - stepLDHeight, 0, 
                standingHeight - (stepHeight + stepLDHeight)/2, 0.7, // t1 is normalized 
                tk, 
                posRef[2], velRef[2]);
    }
    else
    {
        posRef[2] = standingHeight - stepLDHeight;
        velRef[2] = 0;
    }

    // Planning leg angle
    if (tb < 1) // Keep swing backward
    {
        Model::HermitInterpolate(
                Tbackward, 
                posAtLift[0], velAtLift[0], 
                posAtLift[0]+velAtLift[0]*Tbackward, velAtLift[0], 
                tb, 
                posRef[0], velRef[0]);
        posRef[0] = posAtLift[0] + velAtLift[0]*tb*Tbackward;
        velRef[0] = velAtLift[0];
    }
    else if (tf < 1) // Swing forward and try to match the touchdown speed
    {
        tdAngle = -posAtLift[0] * Tset / Tth;
        tdAngVel = 0;
        EstimateTDState(tdAngle, tdAngVel, isFront);
        Model::HermitInterpolate(
                Tforward,
                posAtLift[0] + velAtLift[0]*Tbackward, velAtLift[0],
                tdAngle, tdAngVel,
                tf,
                posRef[0], velRef[0]
                );
    }
    else // Swing back at a constant speed, which is estimated touchdown speed
    {
        EstimateTDState(tdAngle, tdAngVel, isFront);
        posRef[0] = tdAngle + tdAngVel*tm*Tforward;
        velRef[0] = tdAngVel;
    }
}

void ImpedancePlanner::UpdateTransitionPosVel()
{
    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            m_lastShiftActPos[i*3 + j] = m_currentAdjustedFootPos[i*3 + j];
            m_lastShiftActVel[i*3 + j] = m_currentAdjustedFootVel[i*3 + j]; 
            m_lastShiftRefPos[i*3 + j] = m_currentTargetFootPos[i*3 + j];
            m_lastShiftRefVel[i*3 + j] = m_currentTargetFootVel[i*3 + j]; 
        }
    }
}

void ImpedancePlanner::UpdateLiftUpLegPosVel()
{
    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            m_lastLiftActPos[i*3 + j] = m_currentAdjustedFootPos[i*3 + j];
            m_lastLiftActVel[i*3 + j] = m_currentAdjustedFootVel[i*3 + j]; 
            m_lastLiftRefPos[i*3 + j] = m_currentTargetFootPos[i*3 + j];
            m_lastLiftRefVel[i*3 + j] = m_currentTargetFootVel[i*3 + j]; 
        }
    }
}

void ImpedancePlanner::UpdateTouchDownLegPosVel(const Aris::RT_CONTROL::CIMUData& imuData)
{
    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            m_lastTdActPos[i*3 + j] = m_currentAdjustedFootPos[i*3 + j];
            m_lastTdActVel[i*3 + j] = m_currentAdjustedFootVel[i*3 + j]; 
            m_lastTdRefPos[i*3 + j] = m_currentTargetFootPos[i*3 + j];
            m_lastTdRefVel[i*3 + j] = m_currentTargetFootVel[i*3 + j]; 
        }
    }

    for(int i = 0; i < 3; i++)
    {
        m_lastTdBodyOrient[i] = imuData.EulerAngle[i];
    }
}

void ImpedancePlanner::CalculateBodyVelNextLiftUp(const char* legGroupName)
{

    const int* groupList;
    if (*legGroupName == 'A')
    {
        groupList = LEG_INDEX_GROUP_A;
    }
    else if (*legGroupName == 'B')
    {
        groupList = LEG_INDEX_GROUP_B;
    }

    // simply assume that the body velocity at td is equal to that at lift up (We planned)
    bodyVelLastTouchdown = bodyVelNextLiftUp;
    // Calculate the planned lift up body vel in the new step
    double velErr = -bodyVelLastTouchdown + bodyVelDesire;
    
    if (fabs(velErr) < 0.05)
    {
        bodyVelNextLiftUp = bodyVelDesire;
    }
    else
    {
        bodyVelNextLiftUp = bodyVelLastTouchdown + velErr / fabs(velErr) * 0.05;
    }
                   
}

bool ImpedancePlanner::AllLegOnGround(const char* legGroupName)
{

    const int* groupList;
    if (*legGroupName == 'A')
    {
        groupList = LEG_INDEX_GROUP_A;
    }
    else if (*legGroupName == 'B')
    {
        groupList = LEG_INDEX_GROUP_B;
    }

    bool result =  
        ( m_forceTransfromed[groupList[0]*3 + 2] > 100 ) &&
        ( m_forceTransfromed[groupList[1]*3 + 2] > 100 ) &&
        ( m_forceTransfromed[groupList[2]*3 + 2] > 100 );
                   
    return result;
}

void ImpedancePlanner::EstimateTDState(
        double& tdAngle, double& tdAngVel, bool isFront)
{
    double mu = 1.16;
    double Tstance = Tset + Tth;
    tdAngle = atan(mu * bodyVelNextLiftUp * Tstance / 2 / standingHeight);
    tdAngVel = -bodyVelNextLiftUp / standingHeight;

    if (!isFront)
    {
        tdAngle = -tdAngle;
        tdAngVel = -tdAngVel;
    }
}

void ImpedancePlanner::CalculateTHLength(
        int legIndex,
        double* posLastTd, 
        double* bodyOrientLastTd,
        const char* legGroupName, 
        double& stepTHLength)
{
    // Get the max height of current step
    double height = standingHeight - 0.2;
    const int* groupList;

    if (*legGroupName == 'A')
    {
        groupList = LEG_INDEX_GROUP_A;
    }
    else if (*legGroupName == 'B')
    {
        groupList = LEG_INDEX_GROUP_B;
    }

    for(int i = 0; i < 3; i++)
    {
        height = std::max(posLastTd[groupList[i] * 3 + 2], height);
    }

    double tdAngle, tdAngVel;
    // Get the angle planned to lift up at current step
    EstimateTDState(tdAngle, tdAngVel, false);

    // Compensate the pitch error
    double pitchError = bodyOrientLastTd[1] * (0.792+0.143);
    double pitchCompensation = 0;

    using Model::Leg;
    if (pitchError < 0 && ( legIndex == Leg::LEG_ID_LF || legIndex == Leg::LEG_ID_RF || legIndex == Leg::LEG_ID_MF))
    {
        pitchCompensation = std::min(-pitchError, 0.05);
    }
    else if (pitchError > 0 && ( legIndex == Leg::LEG_ID_LB || legIndex == Leg::LEG_ID_RB || legIndex == Leg::LEG_ID_MB))
    {
        pitchCompensation = std::min(pitchError, 0.05);
    }

    stepTHLength = standingHeight * (2 - cos(tdAngle)) - height + 0.005 + pitchCompensation;
}
