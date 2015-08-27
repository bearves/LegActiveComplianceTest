#include <iostream>
#include "Aris_Control.h"
#include "Aris_Message.h"
#include "Aris_Thread.h"
#include "Aris_Socket.h"
#include "Server.h"

using namespace std;
using namespace Aris::RT_CONTROL;
using namespace RobotHighLevelControl;

static CGait gait;
static EGAIT gaitcmd[AXIS_NUMBER];
static EGAIT gaitcmdtemp[AXIS_NUMBER];

Aris::RT_CONTROL::ACTUATION controlSystem;
Aris::RT_CONTROL::CSysInitParameters initParam;

static const char *GS_STRING[] = 
{
    "GAIT_NULL          ",
    "GAIT_STANDSTILL    ",
    "GAIT_HOME2START    ",
    "GAIT_HOME          ",
    "GAIT_ONLINE        "
};

static const int INVALID_MSG_ID = -100;
static const int LEG_GROUP_A [] = {0, 1, 2, 6, 7, 8, 12, 13, 14};
static const int LEG_GROUP_B [] = {3, 4 ,5, 9, 10, 11, 15, 16, 17};

enum MACHINE_CMD
{
    NOCMD         = 1000,
    POWEROFF      = 1001,
    STOP          = 1002,
    ENABLE        = 1003,
    RUNNING       = 1004,
    GOHOME_1      = 1005,
    GOHOME_2      = 1006,
    HOME2START_1  = 1007,
    HOME2START_2  = 1008,
    ONLINEGAIT    = 1016,
    ONLINEBEGIN   = 1017,
    ONLINEEND     = 1018,
    CLEAR_FORCE   = 1034
};

enum REPLY_MSG_ID
{
    DATA_REPORT   = 1050
};

int msgLoopCounter;
int rtCycleCounter = 0;

// Message loop daemon
void* MessageLoopDaemon(void *)
{
    cout<<"running msgloop"<<endl;
    Aris::Core::RunMsgLoop();
    return NULL;
};


int initFun(Aris::RT_CONTROL::CSysInitParameters& param)
{
    for ( int i = 0; i < AXIS_NUMBER; i++)
    {
        gaitcmd[i] = GAIT_NULL;
        gaitcmdtemp[i] = GAIT_NULL;
    }
    gait.InitGait(param);
    return 0;
};

int tg(Aris::RT_CONTROL::CMachineData& machineData,
       Aris::Core::RT_MSG& msgRecv, 
       Aris::Core::RT_MSG& msgSend)
{
    int CommandID;

    rtCycleCounter++;

    double timeNow = rtCycleCounter * 0.001;

    if (rtCycleCounter % 50 == 0)
    {
        for( int i = 0; i < 1; i++ )
        {
            rt_printf("No. %d GS. %d MS. %d POS. %d \n",
                 i, gaitcmd[i], machineData.motorsStates[i], machineData.feedbackData[i].Position);
        }
        msgSend.SetMsgID(DATA_REPORT);
        msgSend.SetLength(sizeof(machineData));
        msgSend.Copy((const char *)&machineData, sizeof(machineData));
        
        controlSystem.RT_PostMsg(msgSend);
    }

    CommandID=msgRecv.GetMsgID();
    switch(CommandID)
    {
        case INVALID_MSG_ID: // when no message actually is received, a INVALID_MSG_ID will be got here
            break;
        case NOCMD:
            for(int i=0;i<18;i++)
            {
                machineData.motorsCommands[i]=EMCMD_NONE;
                gaitcmd[i] = GAIT_NULL;
            }
            rt_printf("NONE Command Get in NRT\n" );
            break;

        case ENABLE:
            gait.onlinePlanner.Offline();
            for(int i=0;i<18;i++)
            {
                machineData.motorsCommands[i]=EMCMD_ENABLE;
                gaitcmd[i] = GAIT_NULL;
            }
            rt_printf("ENABLE Command Get in NRT\n" );

            break;
        case POWEROFF:
            gait.onlinePlanner.Offline();
            for(int i=0;i<18;i++)
            {
                machineData.motorsCommands[i]=EMCMD_POWEROFF;
                gaitcmd[i] = GAIT_NULL;
            }
            rt_printf("POWEROFF Command Get in NRT\n" );

            break;
        case STOP:
            gait.onlinePlanner.Offline();
            for(int i=0;i<18;i++)
            {
                machineData.motorsCommands[i]=EMCMD_STOP;
                gaitcmd[i] = GAIT_NULL;
            }
            rt_printf("STOP Command Get in NRT\n" );

            break;
        case RUNNING:
            gait.onlinePlanner.Offline();
            for(int i=0;i<18;i++)
            {
                machineData.motorsCommands[i]=EMCMD_RUNNING;
                gaitcmd[i] = GAIT_STANDSTILL;
            }
            rt_printf("RUNNING Command Get in NRT\n" );
            break;

        case GOHOME_1:

            for(auto index : LEG_GROUP_A){
                machineData.motorsCommands[MapAbsToPhy[index]] = EMCMD_GOHOME;
                gaitcmd[MapAbsToPhy[index]] = EGAIT::GAIT_HOME;
            }

            rt_printf("GOHOME_1 Command Get in NRT\n" );

            break;

        case GOHOME_2:

            for(auto index : LEG_GROUP_B){
                machineData.motorsCommands[MapAbsToPhy[index]] = EMCMD_GOHOME;
                gaitcmd[MapAbsToPhy[index]] = EGAIT::GAIT_HOME;
            }
            rt_printf("GOHOME_2 Command Get in NRT\n" );

            break;

        case HOME2START_1:

            if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
            {
                for(int i=0;i<18;i++)
                {
                    machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                }

                for(auto index : LEG_GROUP_A){
                    gaitcmd[MapAbsToPhy[index]] = EGAIT::GAIT_HOME2START;
                }

                rt_printf("HOME2START_1 Command Get in NRT\n" );

            }
            break;

        case HOME2START_2:

            if(gait.m_gaitState[MapAbsToPhy[3]]==GAIT_STOP)
            {
                for(int i=0;i<18;i++)
                {
                    machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                }

                for (auto index : LEG_GROUP_B){
                    gaitcmd[MapAbsToPhy[index]] = EGAIT::GAIT_HOME2START;
                }
                rt_printf("HOME2START_2 Command Get in NRT\n" );
            }
            break;

        case ONLINEGAIT:
            // TODO: add online trj code here

            gait.onlinePlanner.Initialize(1);
            if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
            {
                for(int i=0;i<18;i++)
                {
                    machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                    gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_ONLINE;
                    machineData.motorsCommands[i]=EMCMD_RUNNING;
                }
            }
            break;

        case ONLINEBEGIN:
            gait.onlinePlanner.Start(timeNow);
            break;

        case ONLINEEND:
            gait.onlinePlanner.Stop(timeNow);
            break;
        case CLEAR_FORCE:
            for (int i = 0; i < FORCE_SENSOR_NUMBER; i++){
                machineData.forceData[i].isZeroingRequest = 1;
            }
            break; 

        default:
            //DO NOTHING, CMD AND TRAJ WILL KEEP STILL
            break;
    }

    gait.RunGait(timeNow, gaitcmd, machineData);

    return 0;
};

//offsets driver order
static int HEXBOT_HOME_OFFSETS_RESOLVER[18] =
{
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
};


int OnGetControlCommand(Aris::Core::MSG &msg)
{
    int CommandID;
    msg.Paste(&CommandID,sizeof(int));
    Aris::Core::MSG data;

    switch(CommandID)
    {
        case POWEROFF:
            data.SetMsgID(POWEROFF);
            controlSystem.NRT_PostMsg(data);
            break;
        case STOP:
            data.SetMsgID(STOP);
            controlSystem.NRT_PostMsg(data);
            break;
        case ENABLE:
            data.SetMsgID(ENABLE);
            controlSystem.NRT_PostMsg(data);
            break;
        case RUNNING:
            data.SetMsgID(RUNNING);
            controlSystem.NRT_PostMsg(data);
            break;
        case GOHOME_1:
            data.SetMsgID(GOHOME_1);
            controlSystem.NRT_PostMsg(data);
            break;
        case GOHOME_2:
            data.SetMsgID(GOHOME_2);
            controlSystem.NRT_PostMsg(data);
            break;
        case HOME2START_1:
            data.SetMsgID(HOME2START_1);
            controlSystem.NRT_PostMsg(data);
            break;
        case HOME2START_2:
            data.SetMsgID(HOME2START_2);
            controlSystem.NRT_PostMsg(data);
            break;
        case ONLINEGAIT:
            data.SetMsgID(ONLINEGAIT);
            controlSystem.NRT_PostMsg(data);
            break;
        case ONLINEBEGIN:
            data.SetMsgID(ONLINEBEGIN);
            controlSystem.NRT_PostMsg(data);
            break;
        case ONLINEEND:
            data.SetMsgID(ONLINEEND);
            controlSystem.NRT_PostMsg(data);
            break;
        case CLEAR_FORCE:
            data.SetMsgID(CLEAR_FORCE);
            controlSystem.NRT_PostMsg(data);
            break;

        default:
            printf("Hi! I didn't get validate cmd\n");
            break;
    }
    return CommandID;

};

int OnDataReportReceived(Aris::Core::MSG &msg)
{
    CMachineData msgData;
    if ( ControlSystem.IsConnected() ){
        msg.Paste((void *)&msgData, msg.GetLength());
        ControlSystem.SendData(msg);
    }
    return 0;
}

int main(int argc, char** argv)
{	
    Aris::Core::RegisterMsgCallback(CS_Connected,On_CS_Connected);
    Aris::Core::RegisterMsgCallback(CS_CMD_Received,On_CS_CMD_Received);
    Aris::Core::RegisterMsgCallback(CS_Lost,On_CS_Lost);
    Aris::Core::RegisterMsgCallback(GetControlCommand,OnGetControlCommand);
    Aris::Core::RegisterMsgCallback(DATA_REPORT, OnDataReportReceived);

    //   CONN call back
    /*设置所有CONN类型的回调函数*/
    ControlSystem.SetCallBackOnReceivedConnection(On_CS_ConnectionReceived);
    ControlSystem.SetCallBackOnReceivedData(On_CS_DataReceived);
    ControlSystem.SetCallBackOnLoseConnection(On_CS_ConnectionLost);

    ControlSystem.StartServer("5690");
    Aris::Core::THREAD threadMessageLoop;
    threadMessageLoop.SetFunction(MessageLoopDaemon);
    threadMessageLoop.Start(0);

    controlSystem.SetSysInitializer(initFun);

    controlSystem.SetTrajectoryGenerator(tg);

    //controlSystem.SetModeCycVel();

    initParam.motorNum      = 18;
    initParam.homeHighSpeed = 280000;
    initParam.homeLowSpeed  = 40000;
    initParam.homeMode      = -1;
    initParam.homeTorqueLimit = 950;

    ////necessary steps
    initParam.homeOffsets=HEXBOT_HOME_OFFSETS_RESOLVER;
    controlSystem.SysInit(initParam);

    controlSystem.SysInitCommunication();

    controlSystem.SysStart();

    printf("Will start\n");
    while(!controlSystem.IsSysStopped())
    {
        msgLoopCounter++;
        sleep(1);
    }

    return 0;
};


