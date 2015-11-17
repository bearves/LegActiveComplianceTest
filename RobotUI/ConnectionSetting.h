#ifndef CONNECTIONSETTING_H
#define CONNECTIONSETTING_H

#include "Aris_Message.h"

#define REMOTE_PORT            5690
#define REMOTE_IP_STRING    "127.0.0.1"

#define CONNECTION_FREQ        20

#define RMID_MESSAGE_ACK       0
#define RMID_MESSAGE_DATA_REPORT 1050
#define RMID_NOCMD        (1000)
#define RMID_POWEROFF     (1001)
#define RMID_STOP         (1002)
#define RMID_ENABLE       (1003)
#define RMID_RUNNING      (1004)
#define RMID_GOHOME_1     (1005)
#define RMID_GOHOME_2     (1006)
#define RMID_HOME2START_1 (1007)
#define RMID_HOME2START_2 (1008)
#define RMID_GOTO_START   (1016)
#define RMID_GOTO_SIT     (1020)
#define RMID_GOTO_STAND   (1021)
#define RMID_ONLINEGAIT_2 (1019)
#define RMID_ONLINEBEGIN  (1017)
#define RMID_ONLINEEND    (1018)
#define RMID_CLEAR_FORCE  (1034)
#define RMID_SET_PARA_CXB (1035)

typedef union HEAD
{
    struct
    {
        unsigned int dataLength;
        int msgID;
        long long type;
    };
    char header[MSG_HEADER_LENGTH];
} head_t;

// motor state for display
static const char *MOTOR_STATE_DISPLAY_STRING[20] = 
{
    "NONE",
    "POWOFF",
    "STOP",
    "ENABLE",
    "STSTL",
    "HOMING",
    "ERROR",
    "INVAL",
    "EMERG"
};
#endif // CONNECTIONSETTING_H
