#include "Server.h"
#include "iostream"
#include <cstring>

using namespace std;

using namespace Aris::Core;


CONN ControlSystem, VisualSystem;

// CONN call back functions
int On_CS_ConnectionReceived(Aris::Core::CONN *pConn, const char* addr,int port)
{
    Aris::Core::MSG msg;

    msg.SetMsgID(CS_Connected);
    msg.SetLength(sizeof(port));
    msg.Copy(&port,sizeof(port));
    msg.CopyMore(addr,strlen(addr));
    PostMsg(msg);
    return 0;
}

int On_CS_DataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data)
{
    int cmd_id=data.GetMsgID();
    Aris::Core::MSG CMD;
    CMD.SetMsgID(CS_CMD_Received);
    CMD.SetLength(sizeof(int));
    //add control matrix
    bool IsCMDExecutable=true;

    CMD.Copy(&cmd_id,sizeof(int));

    cout<<"received CMD is"<<cmd_id<<endl;

    PostMsg(CMD);
    return 0;
}

int On_CS_ConnectionLost(Aris::Core::CONN *pConn)
{
    PostMsg(Aris::Core::MSG(CS_Lost));
    return 0;
}

//MSG call back functions
int On_CS_Connected(Aris::Core::MSG &msg)
{
    cout<<"Received Connection from Control System:"<<endl;
    cout<<"   Remote IP is: "<<msg.GetDataAddress()+sizeof(int)<<endl;
    cout<<"   Port is     : "<<*((int*)msg.GetDataAddress()) << endl << endl;

    Aris::Core::MSG data(0,0);
    ControlSystem.SendData(data);
    return 0;
}

int On_CS_CMD_Received(Aris::Core::MSG &msg)
{
    MSG Command(msg);
    Command.SetMsgID(GetControlCommand);
    PostMsg(Command);

    MSG data(0,0);
    ControlSystem.SendData(data);
    return 0;
}

int On_CS_Lost(Aris::Core::MSG &msg)
{
    cout << "Control system connection lost" << endl;
    ControlSystem.StartServer("5690");
    return 0;
}


