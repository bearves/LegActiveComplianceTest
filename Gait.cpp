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
double CGait::m_jointStateInput[AXIS_NUMBER];
double CGait::m_jointStateOutput[AXIS_NUMBER];
Aris::RT_CONTROL::CForceData CGait::m_forceData[FSR_NUM]; // must be 6 (legs) 
Aris::RT_CONTROL::CIMUData CGait::m_imuData; // we assume one imu is connected by default

int GaitHome2Start[GAIT_HOME2START_LEN][GAIT_WIDTH];

void CGait::MapFeedbackDataIn(Aris::RT_CONTROL::CMachineData& data )
{
    for(int i=0;i<MOTOR_NUM;i++)
    {
        m_feedbackDataMapped[i] = data.feedbackData[MapAbsToPhy[i]];
    }
};
void CGait::MapCommandDataOut(Aris::RT_CONTROL::CMachineData& data )
{
    for(int i=0;i<MOTOR_NUM;i++)
    {
        data.commandData[i] = m_commandDataMapped[MapPhyToAbs[i]];
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

    fin.open("../resource/gait/start.txt");
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


int CGait::RunGait(double timeNow, EGAIT* p_gait,Aris::RT_CONTROL::CMachineData& data)
{
    // data.commandData -> m_feedbackDataMapped
    MapFeedbackDataIn(data);
    //CURRENT STAGE: the scale of the first one FSR is 1000 times of the others, so we firstly adjust them to the same scale
    for( int i = 0; i < 1; i++)
    {
        for ( int j = 0; j < 6; j++)
            data.forceData[MapAbsToPhyForceSensor[i]].forceValues[j] /= 1000.0;
    }

    for(int i=0;i<AXIS_NUMBER;i++)
    {
        int motorID =MapPhyToAbs[i];
        switch(p_gait[i])
        {
            case GAIT_ONLINE:
                // it should dealt with in a specific place
                m_currentGait[i] = p_gait[i];
                break;

            case GAIT_NULL:
                CGait::m_gaitState[i]=EGaitState::GAIT_STOP;

                m_currentGait[i] = p_gait[i];
                m_standStillData[motorID] = m_feedbackDataMapped[motorID];
                m_commandDataMapped[motorID] = m_standStillData[motorID];

                break;
            case GAIT_HOME:
                m_currentGait[i] = p_gait[i];
                m_commandDataMapped[motorID] = m_feedbackDataMapped[motorID];

                // BUG detected here, which prevents the robot from going home for multiple times
                if(data.isMotorHomed[i]==true)
                {
                    m_standStillData[motorID] = m_feedbackDataMapped[motorID];
                    p_gait[i]=GAIT_STANDSTILL;
                    rt_printf("driver %d: HOMED\n",i);
                }

                break;

            case GAIT_STANDSTILL:
                if(p_gait[i]!=m_currentGait[i])
                {
                    rt_printf("driver %d: feedback pos: %d  GAIT_STANDSTILL begins\n",i, m_feedbackDataMapped[motorID]);
                    m_currentGait[i]=p_gait[i];
                    m_gaitStartTime[i]=data.time;
                    m_standStillData[motorID] = m_feedbackDataMapped[motorID];
                    m_commandDataMapped[motorID] = m_standStillData[motorID];
                }
                else
                {
                    m_commandDataMapped[motorID] = m_standStillData[motorID];
                }
                break;
            case GAIT_HOME2START:

                if(p_gait[i]!=m_currentGait[i])
                {
                    rt_printf("driver %d: GAIT_HOME2START begin\n",i);
                    m_gaitState[i]=EGaitState::GAIT_RUN;
                    m_currentGait[i]=p_gait[i];
                    m_gaitStartTime[i]=data.time;
                    m_commandDataMapped[motorID].Position=GaitHome2Start[0][motorID];
                }
                else
                {
                    m_gaitCurrentIndex[i]=(int)(data.time-m_gaitStartTime[i]);
                    m_commandDataMapped[motorID].Position=GaitHome2Start[m_gaitCurrentIndex[i]][motorID];

                    if(m_gaitCurrentIndex[i]==GAIT_HOME2START_LEN-1)
                    {
                        rt_printf("driver %d:GAIT_HOME2START will transfer to GAIT_STANDSTILL...\n",i);
                        p_gait[i]=GAIT_STANDSTILL;
                        m_standStillData[motorID] = m_feedbackDataMapped[motorID];
                        m_gaitState[i]=EGaitState::GAIT_STOP;
                    }

                }
                break;
        }
    }


    if (fabs(fmod(timeNow, 0.5)) < 1.1e-3)
    {
        rt_printf("driver %d: standstill pos: %d \n",0, m_standStillData[0]);
        for( int i = 0; i < 3; i++)
        {
            rt_printf("%9.1f\t", data.forceData[MapAbsToPhyForceSensor[i]].forceValues[0] / 1000.0);
        }
        rt_printf("\n");
        for( int i = 3; i < 6; i++)
        {
            rt_printf("%9.1f\t", data.forceData[MapAbsToPhyForceSensor[i]].forceValues[0] / 1000.0);
        }
        
        rt_printf("\n");
        rt_printf("Current Robot State: %d\n", m_currentGait[0]);
        rt_printf("current substate: %d\n", m_gaitState[MapAbsToPhy[0]]);
    }

    if (onlinePlanner.GetCurrentState() == OnlinePlanner::OGS_ONLINE_WALK || 
        onlinePlanner.GetCurrentState() == OnlinePlanner::OGS_ONLINE_GOTO_START_POINT)
    {
        CalculateModelInputs(data, m_jointStateInput, m_forceData, m_imuData);
        onlinePlanner.GenerateJointTrajectory( 
                timeNow, 
                m_jointStateInput, 
                m_forceData, 
                m_imuData,
                m_jointStateOutput,
                data.controlData);
        CalculateActualMotorCounts(m_jointStateOutput, m_commandMotorCounts);

        for ( int i = 0; i < AXIS_NUMBER; i++)
        {
            m_commandDataMapped[i].Position = m_commandMotorCounts[i];
        }
    }
    // m_commandDataMapped -> data.commandData
    MapCommandDataOut(data);
    //rt_printf("command data pos%d\n",data.commandData[0].Position);

    return 0;
};

void CGait::CalculateModelInputs(
        Aris::RT_CONTROL::CMachineData& machineData, 
        double* jointStateInput, 
        Aris::RT_CONTROL::CForceData* forceData,
        Aris::RT_CONTROL::CIMUData& imuData)
{
    for(int i = 0; i < AXIS_NUMBER; i++)
    {
        jointStateInput[i] = m_feedbackDataMapped[i].Position / (double)COUNT_PER_ROT / RATIO_REDUCTION * PITCH_PER_ROT;
    }
    for(int i = 0; i < 6; i++)
    {
        forceData[i] = machineData.forceData[MapAbsToPhyForceSensor[i]];
    }
    imuData = machineData.imuData; // simply copy
}

void CGait::CalculateActualMotorCounts( double* screwLength, int* motorCounts)
{
    for(int i = 0; i < AXIS_NUMBER; i++)
    {
        motorCounts[i] = screwLength[i] * COUNT_PER_ROT / PITCH_PER_ROT * RATIO_REDUCTION;
    }
}
}
