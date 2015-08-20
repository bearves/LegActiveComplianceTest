
#include "Aris_Message.h"
#include "Aris_Socket.h"


extern Aris::Core::CONN ControlSysClient;

enum ClienMsg
{
    ControlCommandNeeded,
    SystemLost,
};

//MSG call back

int OnControlCommandNeeded(Aris::Core::MSG &msg);
int OnSystemLost(Aris::Core::MSG &msg);

//CONN call back

int OnConnDataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data);
int OnConnectLost(Aris::Core::CONN *pConn);







