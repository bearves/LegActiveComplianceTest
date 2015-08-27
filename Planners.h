#ifndef PLANNERS_H
#define PLANNERS_H

#include "Aris_ControlData.h"

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

}


#endif
