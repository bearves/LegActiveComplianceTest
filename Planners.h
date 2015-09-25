#ifndef PLANNERS_H
#define PLANNERS_H

#include <rtdk.h>
#include "Aris_ControlData.h"
#include "LegKinematicsIV.h"
#include "GaitTrjGenerator.h"
#include <cmath>

namespace RobotHighLevelControl
{
    class GoToPointPlanner
    {
        public:
            enum GOTO_POINT_PLANNER_STATE
            {
                UNREADY  = -1,
                READY    = 0,
                INMOTION = 1,
                FINISHED = 2
            };
                
            GoToPointPlanner();
            ~GoToPointPlanner();

            int SetStartAndEndPoint(double* currentPoint, double* destPoint);
            int Start(double timeNow);
            int GenerateJointTrajectory(double timeNow, double* currentPoint, double* jointLength);
        private:
            GOTO_POINT_PLANNER_STATE m_state;
            // the point when the gait start command is received
            double m_beginPointPosition[18];
            double m_timeWhenBeginToGo;
            double m_totalTime;
            double m_timeAcc;
            double m_timeDcc;
            // the destination point when the gait stopped. Actually it should be the 
            double m_destPointPosition[18];
    };

    enum class GAIT_SUB_COMMAND : int
    {
        GSC_NOCMD  =  0, 
        GSC_START  =  1, // start move
        GSC_STOP   =  2, // not implemented
        GSC_CHANGE =  3, // not implemented
        GSC_BEHARD =  4, // used to ask the robot to switch from flexible leg to hard leg
        GSC_BESOFT =  5  // used to ask the robot to switch from hard leg to flexible leg
    };

    class ParamCXB
    {
    public:
        GAIT_SUB_COMMAND gaitCommand;
        unsigned int totalPeriodCount;
        double stepLength;
        double Lside;
        double rotationAngle;
        double duty;
        double stepHeight; //positive value
        double T;
        double standHeight;
        double tdDeltaMidLeg;
        double tdDeltaSideLeg;

        ParamCXB()
        {
            // default values
            gaitCommand      = GAIT_SUB_COMMAND::GSC_NOCMD;
            totalPeriodCount = 10;
            stepLength       = 0;
            Lside            = 0;
            rotationAngle    = 0;
            duty             = 0.52;
            stepHeight       = 60; 
            T                = 1.2;
            standHeight      = 710;
            tdDeltaMidLeg    = 0;
            tdDeltaSideLeg   = 0;
        }
    };

    class ImpedancePlanner
    {
        public:
            enum IMPD_PLANNER_STATE
            {
                UNREADY  = -1,
                READY    = 0,
                INMOTION = 1,
                FINISHED = 2
            };

            enum GAIT_SUB_STATE // for trj generator
            {
                HOLD_INIT_POS = 0,
                WALKING       = 1,
                HOLD_END_POS  = 2
            };

            enum IMPD_MODE
            {
                IM_SOFT_LANDING = 0,  // Used for landing from high place
                IM_MEDIUM_SOFT  = 1,  // Medium deformation
                IM_SUPER_HARD   = 2   // Nearly no deformation
            };
                
            ImpedancePlanner();
            ~ImpedancePlanner();

            int Initialize();
            int GetInitialJointLength(double* jointLength);
            int Start(double timeNow);
            int Stop();
            int SetGaitParameter(const void* param, int dataLength);
            int GenerateJointTrajectory(
                    double timeNow,
                    double* currentPoint, 
                    Aris::RT_CONTROL::CForceData* forceInput, 
                    Aris::RT_CONTROL::CIMUData& imuFdbk,
                    double* jointLength,
                    char* controlDataForLog);

        private:
            static const double FOOT_POS_UP_LIMIT[3];
            static const double FOOT_POS_LOW_LIMIT[3];
            static const double FORCE_DEADZONE[3];
            static const int LEG_INDEX_GROUP_A[3];
            static const int LEG_INDEX_GROUP_B[3];

            // following M_ac, B_ac and K_ac is used for ImpedancePlanner 
            double M_ac[3];
            double B_ac[3];
            double K_ac[3];
            
            IMPD_PLANNER_STATE m_state;
            GAIT_SUB_STATE m_subState;

            double m_walkStartTime;
            double m_walkStopTime;
            ParamCXB m_trjGeneratorParam;

            Model::Leg m_legList[6];

            // the point when the gait should start with
            double m_beginFootPos[18];

            // the position and velocity target generated by the trajectory planner
            double m_currentTargetFootPos[18];
            double m_currentTargetFootVel[18];
            double m_currentTargetJointPos[18];
            double m_currentTargetJointVel[18];

            // the adjusted position after impedance controller's adjustment
            // When the controller is stopped, this output will no longer be updated,
            // thus it can be used as the standstill position in FINISHED state
            double m_currentAdjustedFootPos[18];
            double m_currentAdjustedFootVel[18];
            double m_currentAdjustedJointPos[18];
            double m_currentAdjustedJointVel[18];

            double m_timeWhenBeginToGo;

            // The following are states of the body pose balancer
            double m_lastIntegralValue[2]; // for Roll and Pitch adjustment
            double m_currentIntegralValue[2];
            double m_lastFdbkValue[2];
            double m_adjForceBP[18];

            // The following are states of impedance controller, defined for the foot tip state
            double m_lastOffset[18];
            double m_lastOffsetdot[18];
            double m_currentOffset[18];
            double m_currentOffsetdot[18];

            double m_forceRaw[18];
            double m_forceTransfromed[18];
            double m_forceDesire[18];

            int ResetInitialFootPos();
            int ResetImpedanceParam(int impedanceMode);
            int ForceTransform(double* forceRaw, double* legPositionEstimated, double* forceTransformed);
            int SaturateProcess(double* adjustedFootPos);
            int DeadZone(double* force);
            int ImpedanceControl(double* forceInput, double* forceDesire,
                                 double* lastOffset, double* lastOffsetdot,
                                 double* currentOffset, double* currentOffsetdot, int legID);

            bool bodyPoseBalanceCondition(double* forceInput, int& activeGroup);
            int CalculateAdjForceBP(
                    const Aris::RT_CONTROL::CIMUData &imuFdbk, 
                    double* lastFdbkValue,
                    double* lastIntegralValue,
                    double* currentIntegralValue,
                    double* adjForceBP,
                    int activeGroup);
    };
}


#endif
