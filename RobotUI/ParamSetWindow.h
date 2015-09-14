#ifndef PARAM_SET_WINDOW_H
#define PARAM_SET_WINDOW_H

#include "ConnectionSetting.h"
#include <QMainWindow>
#include <QMessageBox>
#include <QtNetwork>
#include <QTimer>
#include <iostream>
#include "Planners.h"

namespace Ui {
class ParamSetWindow;
}

class ParamSetWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ParamSetWindow(QWidget *parent = 0);
    ~ParamSetWindow();
    void SetParamData(RobotHighLevelControl::ParamCXB);
    RobotHighLevelControl::ParamCXB GetParamData();
    void show();

private:
    Ui::ParamSetWindow *ui;
    RobotHighLevelControl::ParamCXB m_param;
    void ShowMessageBox(QString msg);

private slots:
    void OnPushbuttonOKClicked();
    void OnPushbuttonCancelClicked();
};

#endif // MAINWINDOW_H
