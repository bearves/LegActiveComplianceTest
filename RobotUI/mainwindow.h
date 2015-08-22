#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ConnectionSetting.h"
#include <QMainWindow>
#include <QtNetwork>
#include <QTimer>
#include "Aris_Message.h"
#include "Aris_ControlData.h"
#include "GlobalConfiguration.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    static const int DATAGRAM_BUFFER_SIZE = 4096;
    Ui::MainWindow *ui;
    QTimer *m_timer;
    QTcpSocket *m_tcpSocket;
    QHostAddress m_remoteAddr;

    Aris::Core::MSG m_robotMsgReceive;
    Aris::Core::MSG m_robotMsgToSend;
    Aris::RT_CONTROL::CMachineData m_machineData;
    int m_remotePort;
    int m_isConnected;
    int m_sendCount;
    int m_recvCount;
    int m_tickCount;

    char m_datagram[DATAGRAM_BUFFER_SIZE];

    void DisplayDeviceData(Aris::RT_CONTROL::CMachineData& machineData);
    void ProcessCommand(QString cmd);

private slots:
    void OnPushbuttonConnectClicked();
    void OnDatagramReceived();
    void OnCommandLineReturned();
    void OnTimerTick();
};

#endif // MAINWINDOW_H
