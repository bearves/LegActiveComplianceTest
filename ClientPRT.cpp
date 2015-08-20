#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>

#include "Platform.h"
#include "Client.h"

using namespace std;
using namespace Aris::Core;

#if ARIS_PLATFORM_ == _PLATFORM_LINUX_
#include <unistd.h>
#endif

int main()
{
#if ARIS_PLATFORM_==_PLATFORM_LINUX_  
    char RemoteIp[] = "127.0.0.1";
#endif
    /*设置完毕*/

    Aris::Core::RegisterMsgCallback(ControlCommandNeeded,OnControlCommandNeeded);
    Aris::Core::RegisterMsgCallback(SystemLost, OnSystemLost);


    //ControlSysClient.SetCallBackOnReceivedConnection(OnConnectionReceived);
    ControlSysClient.SetCallBackOnReceivedData(OnConnDataReceived);
    ControlSysClient.SetCallBackOnLoseConnection(OnConnectLost);

    /*连接服务器*/

    ControlSysClient.Connect(RemoteIp, "5690");

    /*开始消息循环*/
    Aris::Core::RunMsgLoop();
}
