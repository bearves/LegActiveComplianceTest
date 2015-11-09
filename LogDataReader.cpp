#include "Aris_ControlData.h"
#include "Planners.h"
#include <fstream>
#include <iostream>
#include <errno.h>
#include <string.h>

using namespace std;
using namespace Aris::RT_CONTROL;

int main(int argc, char** argv)
{
    bool isLegacy = false;
    CMachineData data;
    int fileNameIndex = 1;
    int dataSize = 0;
    RobotHighLevelControl::ControllerLogData conLogData;

    if (argc < 2)
    {
        cout << "Wrong parameter" << endl;
        cout << "Usage: LogDataRead [-l] <PATH_TO_LOGFILE>" << endl;
        return -200;
    }
    
    if (strcmp(argv[1], "-l") == 0)
    {
        fileNameIndex = 2;
        // we use legacyData
        dataSize = sizeof(CMachineDataLegacy);
        isLegacy = true;
    }
    else
    {
        fileNameIndex = 1;
        dataSize = sizeof(CMachineData);
    }

    cout << "Opening log file:" << argv[fileNameIndex] << endl;
    cout << "Data point size: " << dataSize << endl;
    ifstream fin(argv[fileNameIndex], ios::in | ios::binary);
    ofstream fout("ParsedFile.txt");

    if (fin.fail())
    {
        cout << "Open file error: " << strerror(errno) << endl;
        return errno;
    }
    while( fin.read((char *)&data, dataSize))
    {
        fout << data.time << "  ";

        // col 1 -> 36 ForceData
        for(int j = 0; j < 6; j++)
        {
            for(int i = 0; i < 6; i++)
            {
                fout << data.forceData[j].forceValues[i] << "  ";
            }
        }

        // col 37 -> 45 ForceData
        for (int i = 0; i < 3; ++i) {
            fout << data.imuData.EulerAngle[i] << "  ";
        }

        for (int i = 0; i < 3; ++i) {
            fout << data.imuData.AngularVel[i] << "  ";
        }

        for (int i = 0; i < 3; ++i) {
            fout << data.imuData.LinearAcc[i] << "  ";
        }

        // col 46 -> 63 fdbk motor position
        for(int i = 0; i < 18; i++)
        {
            fout << data.feedbackData[i].Position << "  ";
        }

        // col 64 -> 81 command motor position
        for(int i = 0; i < 17; i++)
        {
            fout << data.commandData[i].Position << "  ";
        }

        if (!isLegacy)
        {
            fout << data.commandData[17].Position << "  ";

            memcpy((void *)&conLogData, data.controlData, 
                    sizeof(RobotHighLevelControl::ControllerLogData));

            fout << conLogData.gaitState << "  ";
            for(int i = 0; i < 5; i++){
                fout << conLogData.targetPos[i] 
                     << "  " << conLogData.adjustedPos[i]
                     << "  " << conLogData.legForceOnZ[i] 
                     << "  " << conLogData.targetAng[i]
                     << "  " << conLogData.legForceOnX[i] << "  ";

            }
            fout << conLogData.targetPos[5] 
                << "  " << conLogData.adjustedPos[5]
                << "  " << conLogData.legForceOnZ[5]
                << "  " << conLogData.targetAng[5]
                << "  " << conLogData.legForceOnX[5] << endl;
            
        }
        else
        {
            fout << data.commandData[17].Position << endl;
        }
    }

    fin.close();
    fout.flush();
    fout.close();

    cout << "Log file reading finished" << endl;

    return 0;
}
