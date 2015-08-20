#include "Client.h"
#include <iostream>
#include <cstring>

using namespace std;

using namespace Aris::Core;


CONN ControlSysClient;

//CONN callback functions
int OnConnDataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data)
{
    Aris::Core::PostMsg(Aris::Core::MSG(ControlCommandNeeded));
    return 0;
}

int OnConnectLost(Aris::Core::CONN *pConn)
{
    PostMsg(Aris::Core::MSG(SystemLost));
    return 0;
}


//MSG callback functions
int OnControlCommandNeeded(Aris::Core::MSG &msg)
{

    int cmd;

    cout << "Commands:"      << endl;
    cout << "1.PowerOff"     << endl << "2.Stop"           << endl << "3.Enable"         << endl;
    cout << "4.Running"      << endl << "5.Gohome_1"       << endl << "6.Gohome_2"       << endl;
    cout << "7.HOme2start_1" << endl << "8.Home2start_2"   << endl << "9.Forward"        << endl;
    cout << "10.Backward"    << endl << "11.Fast_Forward"  << endl << "12.Fast_Backward" << endl;
    cout << "13.Legup"       << endl << "14.Turnleft"      << endl << "15.TurnRight"     << endl;
    cout << "16.Online Gait" << endl << "17.Online start"  << endl << "18.Online end"    << endl;
    cout << "19.No force"    << endl << "20.Force 1"       << endl << "21.Force 2"       << endl;
    cout << "22.Force 3"     << endl << "23.Force 4"       << endl << "24.Force 5"       << endl;
    cout << "25.Force 6"     << endl << "26.Force 7"       << endl << "27.Force 8"       << endl;
    cout << "28.Clear force" << endl << "29.Online Gait2"  << endl;

    cout<<"Please enter your command: ";
    cin>>cmd;
    while( cmd < 1 || cmd > 29 )
    {
        cout<<"Not valid command ID,enter again : ";
        cin>>cmd;
    }

    MSG data;
    data.SetMsgID(cmd);
    ControlSysClient.SendData(data);
    return 0;
}

int OnSystemLost(Aris::Core::MSG &msg)
{
    cout<<"Control system lost"<<endl;

    return 0;
}


