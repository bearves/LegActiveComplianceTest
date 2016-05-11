#include <iostream>
#include "Aris_Control.h"
#include "Aris_Message.h"
#include "Aris_Thread.h"
#include "Aris_Socket.h"
#include "Aris_XSensIMU.h"
#include "Server.h"

using namespace std;
using namespace Aris::RT_CONTROL;
using namespace RobotHighLevelControl;

static CGait gait;
static EGAIT gaitcmd[AXIS_NUMBER];
static EGAIT gaitcmdtemp[AXIS_NUMBER];

Aris::RT_CONTROL::ACTUATION controlSystem;
Aris::RT_CONTROL::CSysInitParameters initParam;
Aris::RT_CONTROL::CIMUDevice imuDevice;

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
    GOTO_START    = 1016, // Online home to start
    ONLINEBEGIN   = 1017,
    ONLINEEND     = 1018,
    ONLINEGAIT_2  = 1019, // Online impedance control
    GOTO_SIT      = 1020, // Online goto to sit
    GOTO_STAND    = 1021, // Online goto to stand
    CLEAR_FORCE   = 1034,
    SET_PARA_CXB  = 1035,
    HEARTBEAT     = 2000
};

enum MACHINE_DATA
{
    DATA_REPORT   = 1050,
    IMU_DATA_TO_RT= 1051
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

// Message loop daemon
void* IMUDaemon(void *)
{
    Aris::Core::MSG imuDataMsg;
    Aris::RT_CONTROL::CIMUData imuData;

    imuDataMsg.SetMsgID(IMU_DATA_TO_RT);
    cout<<"running IMU loop"<<endl;
    imuDevice.Initialize();

    while(!controlSystem.IsSysStopped())
    {
        imuDevice.UpdateData(imuData);
        imuDataMsg.SetLength(sizeof(imuData));
        imuDataMsg.Copy((const char *)&imuData, sizeof(imuData));
        controlSystem.NRT_PostMsg(imuDataMsg);
        imuDevice.Sleep(6);
    }

    imuDevice.Close();
    cout << "IMU thread exited" << endl;
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
            for(int i=0;i<AXIS_NUMBER;i++)
            {
                machineData.motorsCommands[i]=EMCMD_NONE;
                gaitcmd[i] = GAIT_NULL;
            }
            rt_printf("NONE Command Get in NRT\n" );
            break;

        case ENABLE:
            gait.onlinePlanner.Offline();
            for(int i=0;i<AXIS_NUMBER;i++)
            {
                machineData.motorsCommands[i]=EMCMD_ENABLE;
                gaitcmd[i] = GAIT_NULL;
            }
            rt_printf("ENABLE Command Get in NRT ---------------------\n" );

            break;
        case POWEROFF:
            gait.onlinePlanner.Offline();
            for(int i=0;i<AXIS_NUMBER;i++)
            {
                machineData.motorsCommands[i]=EMCMD_POWEROFF;
                gaitcmd[i] = GAIT_NULL;
            }
            rt_printf("POWEROFF Command Get in NRT\n" );

            break;
        case STOP:
            gait.onlinePlanner.Offline();
            for(int i=0;i<AXIS_NUMBER;i++)
            {
                machineData.motorsCommands[i]=EMCMD_STOP;
                gaitcmd[i] = GAIT_NULL;
            }
            rt_printf("STOP Command Get in NRT\n" );

            break;
        case RUNNING:
            gait.onlinePlanner.Offline();
            for(int i=0;i<AXIS_NUMBER;i++)
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
                for(int i=0;i<AXIS_NUMBER;i++)
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
                for(int i=0;i<AXIS_NUMBER;i++)
                {
                    machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                }

                for (auto index : LEG_GROUP_B){
                    gaitcmd[MapAbsToPhy[index]] = EGAIT::GAIT_HOME2START;
                }
                rt_printf("HOME2START_2 Command Get in NRT\n" );
            }
            break;

        case GOTO_START: 
            gait.onlinePlanner.Initialize(2); // online home to start
            if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
            {
                for(int i=0;i<AXIS_NUMBER;i++)
                {
                    machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                    gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_ONLINE;
                    machineData.motorsCommands[i]=EMCMD_RUNNING;
                }
            }
            break;

        case GOTO_SIT: 
            gait.onlinePlanner.Initialize(3); // online goto sit point
            if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
            {
                for(int i=0;i<AXIS_NUMBER;i++)
                {
                    machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                    gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_ONLINE;
                    machineData.motorsCommands[i]=EMCMD_RUNNING;
                }
            }
            break;

        case GOTO_STAND: 
            gait.onlinePlanner.Initialize(4); // online goto stand point
            if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
            {
                for(int i=0;i<AXIS_NUMBER;i++)
                {
                    machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                    gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_ONLINE;
                    machineData.motorsCommands[i]=EMCMD_RUNNING;
                }
            }
            break;

        case ONLINEGAIT_2:

            gait.onlinePlanner.Initialize(1); // online impedance control

            if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
            {
                for(int i=0;i<AXIS_NUMBER;i++)
                {
                    machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                    gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_ONLINE;
                    machineData.motorsCommands[i]=EMCMD_RUNNING;
                    rt_printf("Online Gait2 ready to move\n" );
                }
            }
            rt_printf("Online Gait2 ready to move out \n" );
            break;
            
        case ONLINEBEGIN:
            gait.onlinePlanner.Start(timeNow);
            break;

        case ONLINEEND:
            gait.onlinePlanner.Stop(timeNow);
            for(int i = 0; i < AXIS_NUMBER; i++)
            {
                gait.m_gaitState[MapAbsToPhy[i]] = GAIT_STOP;
            }
            rt_printf("Online Gait require to stop\n\
                    ---------------------- \n\n" );
            break;

        case CLEAR_FORCE:
            for (int i = 0; i < FORCE_SENSOR_NUMBER; i++){
                machineData.forceData[i].isZeroingRequest = 1;
            }
            break; 

        case SET_PARA_CXB:
            rt_printf("Got param setting msg from the remote:\n sz = %d\n",
                    msgRecv.GetLength());
            gait.onlinePlanner.SetGaitParameter(msgRecv.GetDataAddress(), msgRecv.GetLength(), 1);
            
            break;

        case IMU_DATA_TO_RT:
            msgRecv.Paste((void *)&machineData.imuData, sizeof(CIMUData));
            break;

        case HEARTBEAT:
            gait.onlinePlanner.UpdateHearbeatTimer(timeNow);
            break;

        default:
            //DO NOTHING, CMD AND TRAJ WILL KEEP STILL
            break;
    }

    gait.RunGait(timeNow, gaitcmd, machineData);

    return 0;
};

//offsets driver order
static int HEXBOT_HOME_OFFSETS_RESOLVER_CUSTOM[18] =
{
    103621+27646+192438, 86453+9806-58606+115079, 75142-2440+192438,  
    113136-16690-25559+192438+25536,125428+22891-115015+135079+36000,82508+130986-86507+152438,
    -43577+13853+90439+192438, -6881+135079, 16263+68812+192438,
    16257+58989+51118+182438, 10630+68989-36372+135079, 6257+8995+66677+192438,
    -16168+31328+192438+22800, -1864+58043-50135+135079+36000, 46195+79912-1966-25000+192438,
    +8342+91000+15898+192438+16000, 60000-40304+135079, -8618+3000+92575+192438-16000 
};

int OnGetControlCommand(Aris::Core::MSG &msg)
{
    int CommandID;
    msg.Paste(&CommandID,sizeof(int));
    Aris::Core::MSG commandMsg;

    switch(CommandID)
    {
        case POWEROFF:
            commandMsg.SetMsgID(POWEROFF);
            controlSystem.NRT_PostMsg(commandMsg);
            break;
        case STOP:
            commandMsg.SetMsgID(STOP);
            controlSystem.NRT_PostMsg(commandMsg);
            break;
        case ENABLE:
            commandMsg.SetMsgID(ENABLE);
            controlSystem.NRT_PostMsg(commandMsg);
            break;
        case RUNNING:
            commandMsg.SetMsgID(RUNNING);
            controlSystem.NRT_PostMsg(commandMsg);
            break;
        case GOHOME_1:
            commandMsg.SetMsgID(GOHOME_1);
            controlSystem.NRT_PostMsg(commandMsg);
            break;
        case GOHOME_2:
            commandMsg.SetMsgID(GOHOME_2);
            controlSystem.NRT_PostMsg(commandMsg);
            break;
        case HOME2START_1:
            commandMsg.SetMsgID(HOME2START_1);
            controlSystem.NRT_PostMsg(commandMsg);
            break;
        case HOME2START_2:
            commandMsg.SetMsgID(HOME2START_2);
            controlSystem.NRT_PostMsg(commandMsg);
            break;
        case GOTO_START:
            commandMsg.SetMsgID(GOTO_START);
            controlSystem.NRT_PostMsg(commandMsg);
            break;
        case GOTO_SIT:
            commandMsg.SetMsgID(GOTO_SIT);
            controlSystem.NRT_PostMsg(commandMsg);
            break;
        case GOTO_STAND:
            commandMsg.SetMsgID(GOTO_STAND);
            controlSystem.NRT_PostMsg(commandMsg);
            break;
        case ONLINEGAIT_2:
            commandMsg.SetMsgID(ONLINEGAIT_2);
            controlSystem.NRT_PostMsg(commandMsg);
            break;
        case ONLINEBEGIN:
            commandMsg.SetMsgID(ONLINEBEGIN);
            controlSystem.NRT_PostMsg(commandMsg);
            break;
        case ONLINEEND:
            commandMsg.SetMsgID(ONLINEEND);
            controlSystem.NRT_PostMsg(commandMsg);
            break;
        case CLEAR_FORCE:
            commandMsg.SetMsgID(CLEAR_FORCE);
            controlSystem.NRT_PostMsg(commandMsg);
            break;

        case HEARTBEAT:
            commandMsg.SetMsgID(HEARTBEAT);
            controlSystem.NRT_PostMsg(commandMsg);
            break;

        case SET_PARA_CXB:
            commandMsg.SetMsgID(SET_PARA_CXB);
            commandMsg.SetLength(msg.GetLength() - sizeof(int));
            commandMsg.Copy(msg.GetDataAddress() + sizeof(int), msg.GetLength() - sizeof(int)); // repost the true data to RT side
            controlSystem.NRT_PostMsg(commandMsg);
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

    Aris::Core::THREAD threadIMU;
    threadIMU.SetFunction(IMUDaemon);
    threadIMU.Start(0);

    controlSystem.SetSysInitializer(initFun);

    controlSystem.SetTrajectoryGenerator(tg);

    //controlSystem.SetModeCycVel();

    initParam.motorNum      = 18;
    initParam.homeHighSpeed = 280000;
    initParam.homeLowSpeed  = 40000;
    initParam.homeMode      = -1;
    initParam.homeTorqueLimit = 950;

    ////necessary steps
    initParam.homeOffsets=HEXBOT_HOME_OFFSETS_RESOLVER_CUSTOM;
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


