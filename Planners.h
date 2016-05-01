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
        GSC_NOCMD = 0,
        GSC_START = 1, // start move
        GSC_STOP  = 2,
        GSC_SPEED = 3,
        GSC_STEPH = 4,
        GSC_SIDE  = 5,
        GSC_TURN  = 6,
        GSC_CLEAR = 7
    };

    class ParamCXB
    {
    public:
        GAIT_SUB_COMMAND gaitCommand;
        unsigned int totalPeriodCount;
        double desireVelocity;
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
            desireVelocity   = 0;
            Lside            = 0;
            rotationAngle    = 0;
            duty             = 0.52;
            stepHeight       = 0.1; 
            T                = 1.2;
            standHeight      = 0.66;
            tdDeltaMidLeg    = 0;
            tdDeltaSideLeg   = 0;
        }
    };

    class ControllerLogData
    {
        public:
            int gaitState;
            float targetPos[6]; // use float to save space
            float adjustedPos[6];
            float legForceOnZ[6];
            float targetAng[6]; 
            float legForceOnX[6];
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
                A_SP_B_LT     = 1,
                A_TH_B_TD     = 2,
                A_LT_B_LD     = 3,
                A_LT_B_SP     = 4,
                A_TD_B_TH     = 5,
                A_LD_B_LT     = 6,
                RECOVERING    = 7,
                HOLD_END_POS  = 8
            };

            static const char *SUB_STATE_NAME[9];

            enum IMPD_MODE
            {
                A_HARD_B_HARD = 1,
                A_HARD_B_SOFT = 2,
                A_SOFT_B_HARD = 3,
                A_SOFT_B_SOFT = 4
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
                    double m_lastHeartbeatTime,
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
            static const double IMPD_RATIO_A[3];
            static const double IMPD_RATIO_B[3];
            static const double BASE_ORIENT[2];
            static const double SAFETY_RETURN_TIMEOUT;

            ControllerLogData m_logData;
            bool m_isOnGround;
            bool m_isSafetyReturnStarted;
            double m_safetyReturnStartTime;

            // following M_ac, B_ac and K_ac is used for ImpedancePlanner 
            double M_ac[6][3];
            double B_ac[6][3];
            double K_ac[6][3];

            // Timing for gait state machine and trj generation
            double Trt;                  
            double Tset;                 
            double Tth;                  
            double Tfly;                  // the maximum flying time
            double Trec;                 
            double stepHeight;           
            double stepLDHeight;
            double stepLDLenVel;         // the vel of length of leg when td
            double stepTHHeight; 
            double standingHeight;       
            double bodyVelDesire;        
            double rotateAngle;          
            double bodyVelLastTouchdown;
            double bodyVelNextLiftUp;    
            
            IMPD_PLANNER_STATE m_state;
            GAIT_SUB_STATE m_subState;
            GAIT_SUB_COMMAND m_cmdFlag;
            
            // discrete states for SM-based planning
            double m_lastStateShiftTime; 
            double m_lastLiftUpTime;
            double m_lastTouchDownTime;

            double m_lastShiftRefPos[18]; // Ref pos generated by Trj planner at state transition moment
            double m_lastShiftRefVel[18];
            double m_lastShiftActPos[18]; // Act pos after impedance adjustments at state transition moment
            double m_lastShiftActVel[18];

            double m_lastTdRefPos[18]; // Ref pos generated by Trj planner at td moment
            double m_lastTdRefVel[18];
            double m_lastTdActPos[18]; // Act pos after impedance adjustments at td moment
            double m_lastTdActVel[18];
            double m_lastTdBodyOrient[3]; // The body orientation at last TD moment

            double m_lastLiftRefPos[18]; // Ref pos generated by Trj planner at lift moment
            double m_lastLiftRefVel[18];
            double m_lastLiftActPos[18]; // Act pos after impedance adjustments at lift moment
            double m_lastLiftActVel[18];

            ParamCXB m_trjGeneratorParam;

            Model::Leg m_legList[6];

            // the point when the gait should start with
            double m_beginFootPos[18];
            double m_beginFootVel[18];

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

            // The following are states of the body pose balancer
            double m_lastIntegralValue[3]; // for Roll and Pitch and Height adjustment
            double m_currentIntegralValue[3];
            double m_lastErrValue[3];
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
            int ResetBasicGaitParameter();
            int ResetImpedanceParam(int impedanceMode);
            int ForceTransform(double* forceRaw, double* legPositionEstimated, double* forceTransformed);
            int SaturateProcess(double* adjustedFootPos);
            int DeadZone(double* force);
            int ImpedanceControl(double* forceInput, double* forceDesire,
                                 double* lastOffset, double* lastOffsetdot,
                                 double* currentOffset, double* currentOffsetdot, int legID);


            void DetermineCurrentState(
                    double timeNow, 
                    GAIT_SUB_COMMAND& cmdFlag, 
                    ParamCXB& gaitParamCmd,
                    GAIT_SUB_STATE& currentState,
                    const Aris::RT_CONTROL::CIMUData& imuData);

            int CalculateAdjForceBP(
                    const Aris::RT_CONTROL::CIMUData &imuFdbk, 
                    double* currentFootPos,
                    double* lastErrorValue,
                    double* lastIntegralValue,
                    double* currentIntegralValue,
                    double* adjForceBP,
                    GAIT_SUB_STATE gaitState,
                    double tdTimeInterval);

            double CalculateCurrentHeight(double* currentFootPos, GAIT_SUB_STATE gaitState);
            void ClearBalancePIDStates();
            void ClearImpedanceStates(const char* legGroupName);
            bool AllLegOnGround(const char* legGroupName);
            void CalculateBodyVelNextLiftUp(const char* legGroupName);
            void UpdateTransitionPosVel();
            void UpdateLiftUpLegPosVel();
            void UpdateTouchDownLegPosVel(const Aris::RT_CONTROL::CIMUData& imuData);
            void GenerateReferenceTrj(
                    double timeNow,
                    GAIT_SUB_STATE gaitState, 
                    double* targetFootPos, 
                    double* targetFootVel);

            void RecoverTrj(double timeNow, double* targetFootPos, double* targetFootVel);
            void SwingReferenceTrj(
                    double timeNow, double lastLiftTime, double lastTDTime,
                    double* posAtLift, double* velAtLift,
                    double* posRef,    double* velRef, bool isFront);

            void StanceAngleReferenceTrj(
                    double timeNow, double  lastTDTime, double lenAtTd,
                    double angAtTd, double angVelAtTd,
                    double bodyVelAtTd, double bodyVelNextLt,
                    double& angRef, double& angVelRef, bool isFront);
            void EstimateTDState( double& tdAngle, double& tdAngVel, bool isFront);
            void CalculateTHLength(
                    int legIndex,
                    double* posLastTd, 
                    double* bodyOrientLastTd,
                    const char* legGroupName, 
                    double& stepTHLength);
            void RotationAdjustment(
                    double timeFromTd, 
                    double rotateAngle, 
                    const char* legGroupName,
                    double* targetFootPos, 
                    double* targetFootVel);

            void CheckHeartbeat(double timeNow, double lastHeartbeatTime);
    };

}


#endif
