/*
 * Gait.cpp
 *
 *  Created on: Nov 28, 2014
 *      Author: hex
 */
#include"Gait.h"
#include <fstream>
#include <iostream>

namespace RobotHighLevelControl{

OnlinePlanner CGait::onlinePlanner;
EGAIT CGait::m_currentGait[AXIS_NUMBER];
long long int CGait::m_gaitStartTime[AXIS_NUMBER];
int CGait::m_gaitCurrentIndex[AXIS_NUMBER];
EGaitState CGait::m_gaitState[AXIS_NUMBER];
Aris::RT_CONTROL::CMotorData CGait::m_standStillData[AXIS_NUMBER];
Aris::RT_CONTROL::CMotorData CGait::m_commandDataMapped[AXIS_NUMBER];
Aris::RT_CONTROL::CMotorData CGait::m_feedbackDataMapped[AXIS_NUMBER];
int CGait::Gait_iter[AXIS_NUMBER];
int CGait::Gait_iter_count[AXIS_NUMBER];
bool CGait::IsConsFinished[AXIS_NUMBER];
bool CGait::IsHomeStarted[AXIS_NUMBER];
double CGait::m_screwLength[AXIS_NUMBER];
int CGait::m_commandMotorCounts[AXIS_NUMBER];

int GaitHome2Start[GAIT_HOME2START_LEN][GAIT_WIDTH];

void CGait::MapFeedbackDataIn(Aris::RT_CONTROL::CMachineData& p_data )
{
    for(int i=0;i<MOTOR_NUM;i++)
    {

        m_feedbackDataMapped[i].Position=p_data.feedbackData[MapAbsToPhy[i]].Position;
        m_feedbackDataMapped[i].Velocity=p_data.feedbackData[MapAbsToPhy[i]].Velocity;
        m_feedbackDataMapped[i].Torque=p_data.feedbackData[MapAbsToPhy[i]].Torque;
    }
};
void CGait::MapCommandDataOut(Aris::RT_CONTROL::CMachineData& p_data )
{
    for(int i=0;i<MOTOR_NUM;i++)
    {
        p_data.commandData[i].Position=m_commandDataMapped[MapPhyToAbs[i]].Position;
        p_data.commandData[i].Velocity=m_commandDataMapped[MapPhyToAbs[i]].Velocity;
        p_data.commandData[i].Torque=m_commandDataMapped[MapPhyToAbs[i]].Torque;
    }
};

CGait::CGait()
{
    for(int i=1;i<AXIS_NUMBER;i++)
    {
        CGait::m_currentGait[i]=EGAIT::GAIT_NULL;
        CGait::m_gaitState[i]=EGaitState::NONE;
        CGait::IsHomeStarted[i]=false;
        CGait::IsConsFinished[i]=false;
        CGait::Gait_iter[i]=1;
        CGait::Gait_iter_count[i]=0;

    }

};
CGait::~CGait()
{
};

bool CGait::IsGaitFinished()
{
    for(int i=0;i<AXIS_NUMBER;i++)
    {
        if(m_gaitState[i]!=EGaitState::GAIT_STOP)
            return false;
    }
    return true;
};

static std::ifstream fin;
//read file
int CGait::InitGait(Aris::RT_CONTROL::CSysInitParameters& param)
{
    int Line_Num;
    int ret;

    double temp;
    std::cout << "Start Reading File" << std::endl;
    std::cout<<"reading file data..."<<std::endl;

    fin.open("../../resource/gait/start.txt");
    if(fin.fail())
        goto OPEN_FILE_FAIL;

    for(int i=0;i<GAIT_HOME2START_LEN;i++)
    {
        fin>>Line_Num;


        for(int j=0;j<GAIT_WIDTH;j++)
        {
            fin>>temp;
            GaitHome2Start[i][j]=-(int)temp;
            //	cout<<"gait start "<<GaitHome2Start[i][j]<<endl;
        }
    }

    fin.close();
    if (onlinePlanner.LoadData() != 0)
        goto OPEN_FILE_FAIL;

    cout << "All files sucessfully read" << endl;
    return 0;

OPEN_FILE_FAIL:
    cerr << "Open file error: " << strerror(errno) << endl;
    return errno;
};


int CGait::RunGait(double timeNow, EGAIT* p_gait,Aris::RT_CONTROL::CMachineData& p_data)
{
    //rt_printf("operation mode %d\n",p_data.motorsModes[0]);
    MapFeedbackDataIn(p_data);
    for(int i=0;i<AXIS_NUMBER;i++)
    {
        int motorID =MapPhyToAbs[i];
        switch(p_gait[i])
        {
            case GAIT_ONLINE:
                // it should dealt with in a specific place
                break;

            case GAIT_NULL:
                CGait::m_gaitState[i]=EGaitState::GAIT_STOP;

                m_currentGait[i] = p_gait[i];
                m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
                m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
                m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

                m_commandDataMapped[motorID].Position=m_standStillData[motorID].Position;
                m_commandDataMapped[motorID].Velocity=m_standStillData[motorID].Velocity;
                m_commandDataMapped[motorID].Torque=m_standStillData[motorID].Torque;

                //rt_printf("driver %d: GAIT_NULL...\n",i);

                break;
            case GAIT_HOME:

                m_currentGait[i] = p_gait[i];
                m_commandDataMapped[motorID].Position=m_feedbackDataMapped[motorID].Position;
                m_commandDataMapped[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
                m_commandDataMapped[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

                if(p_data.isMotorHomed[i]==true)
                {
                    m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
                    m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
                    m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

                    p_gait[i]=GAIT_STANDSTILL;
                    rt_printf("driver %d: HOMED\n",i);

                }

                break;

            case GAIT_STANDSTILL:
                if(p_gait[i]!=m_currentGait[i])
                {
                    rt_printf("driver %d:   GAIT_STANDSTILL begins\n",i);
                    m_currentGait[i]=p_gait[i];
                    m_gaitStartTime[i]=p_data.time;

                    m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
                    m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
                    m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

                    m_commandDataMapped[motorID].Position=m_standStillData[motorID].Position;
                    m_commandDataMapped[motorID].Velocity=m_standStillData[motorID].Velocity;
                    m_commandDataMapped[motorID].Torque=m_standStillData[motorID].Torque;
                }
                else
                {
                    m_commandDataMapped[motorID].Position=m_standStillData[motorID].Position;
                    m_commandDataMapped[motorID].Velocity=m_standStillData[motorID].Velocity;
                    m_commandDataMapped[motorID].Torque=m_standStillData[motorID].Torque;
                }
                break;
            case GAIT_HOME2START:

                if(p_gait[i]!=m_currentGait[i])
                {
                    rt_printf("driver %d: GAIT_HOME2START begin\n",i);
                    m_gaitState[i]=EGaitState::GAIT_RUN;
                    m_currentGait[i]=p_gait[i];
                    m_gaitStartTime[i]=p_data.time;
                    m_commandDataMapped[motorID].Position=GaitHome2Start[0][motorID];

                }
                else
                {
                    m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);

                    m_commandDataMapped[motorID].Position=GaitHome2Start[m_gaitCurrentIndex[i]][motorID];


                    if(m_gaitCurrentIndex[i]==GAIT_HOME2START_LEN-1)
                    {
                        rt_printf("driver %d:GAIT_HOME2START will transfer to GAIT_STANDSTILL...\n",i);
                        p_gait[i]=GAIT_STANDSTILL;

                        m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
                        m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
                        m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;
                        m_gaitState[i]=EGaitState::GAIT_STOP;
                    }

                }
                break;
        }
    }

    if (onlinePlanner.GetCurrentState() == OnlinePlanner::OGS_ONLINE_WALK)
    {
        onlinePlanner.GenerateJointTrajectory( timeNow, p_data, m_screwLength);
        CalculateActualMotorCounts(m_screwLength, m_commandMotorCounts);
        for ( int i = 0; i < AXIS_NUMBER; i++)
        {
            m_commandDataMapped[i].Position = m_commandMotorCounts[i];
        }
        if (fabs(fmod(timeNow, 1.0)) < 2e-3)
        {
            rt_printf("OL cmd: %d\n", m_commandDataMapped[0].Position);
        }
    }
    MapCommandDataOut(p_data);
    //rt_printf("command data pos%d\n",p_data.commandData[0].Position);

    return 0;
};

void CGait::CalculateActualMotorCounts(double *screwLength, int *motorCounts)
{
    int countPerMeter = 350 * 65536; // the counts per meter
    for (int i = 0; i < AXIS_NUMBER; i++)
    {
        motorCounts[i] = countPerMeter * screwLength[i];
    }
}

}
