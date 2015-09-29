#include "Aris_ControlData.h"
#include <fstream>
#include <iostream>
#include <errno.h>
#include <string.h>

using namespace std;
using namespace Aris::RT_CONTROL;

int main(int argc, char** argv)
{
    CMachineData data;
    int fileNameIndex = 1;
    int dataSize = 0;

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
    }
    else
    {
        fileNameIndex = 1;
        dataSize = sizeof(CMachineData);
    }

    cout << "Opening log file:" << argv[fileNameIndex] << endl;
    cout << "Data point size: " << dataSize << endl;

    ifstream fin(argv[fileNameIndex]);
    ofstream fout("ParsedFile.txt");

    if (fin.fail())
    {
        cout << "Open file error: " << strerror(errno) << endl;
        return errno;
    }
    while( fin.read((char *)&data, dataSize))
    {
        fout << data.time << "  ";
        for(int j = 0; j < 6; j++)
        {
            for(int i = 0; i < 6; i++)
            {
                fout << data.forceData[j].forceValues[i] << "  ";
            }
        }

        for (int i = 0; i < 3; ++i) {
            fout << data.imuData.EulerAngle[i] << "  ";
        }

        for (int i = 0; i < 3; ++i) {
            fout << data.imuData.AngularVel[i] << "  ";
        }

        for (int i = 0; i < 3; ++i) {
            fout << data.imuData.LinearAcc[i] << "  ";
        }

        for(int i = 0; i < 18; i++)
        {
            fout << data.feedbackData[i].Position << "  ";
        }

        for(int i = 0; i < 17; i++)
        {
            fout << data.commandData[i].Position << "  ";
        }
        fout << data.commandData[17].Position << endl;
    }

    fin.close();
    fout.flush();
    fout.close();

    cout << "Log file reading finished" << endl;

    return 0;
}
