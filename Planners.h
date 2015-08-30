#ifndef PLANNERS_H
#define PLANNERS_H

#include "Aris_ControlData.h"
#include "LegKinematicsIV.h"

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
                
            ImpedancePlanner();
            ~ImpedancePlanner();

            int Initialize();
            int GetInitialJointLength(double* jointLength);
            int Start(double timeNow);
            int Stop();
            int GenerateJointTrajectory(double timeNow, double* currentPoint, Aris::RT_CONTROL::CForceData* forceInput, double* jointLength);
        private:
            IMPD_PLANNER_STATE m_state;
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

            // The following are states of impedance controller, defined for the foot tip state
            double m_lastOffset[18];
            double m_lastOffsetdot[18];
            double m_currentOffset[18];
            double m_currentOffsetdot[18];

            double m_forceRaw[18];
            double m_forceTransfromed[18];
            double m_forceDesire[18];

            int ForceTransform(double* forceRaw, double* legPositionEstimated, double* forceTransfromed);
            int ImpedanceControl(double* forceInput, double* forceDesire,
                                 double* lastOffset, double* lastOffsetdot,
                                 double* currentOffset, double* currentOffsetdot);
    };
}


#endif
