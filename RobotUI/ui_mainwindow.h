/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.3.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout_4;
    QVBoxLayout *verticalLayout;
    QTextBrowser *textBrowserDevStatus;
    QHBoxLayout *horizontalLayout;
    QPlainTextEdit *plainTextEditCmdLog;
    QPlainTextEdit *plainTextEditMsgLog;
    QHBoxLayout *horizontalLayout_2;
    QLabel *labelCmd;
    QLineEdit *lineEditCmd;
    QHBoxLayout *horizontalLayout_3;
    QLabel *labelIP;
    QLineEdit *lineEditIP;
    QLabel *labelPort;
    QLineEdit *lineEditPort;
    QPushButton *pushButtonConnect;
    QVBoxLayout *verticalLayout_2;
    QCustomPlot *widget_3;
    QCustomPlot *widget_2;
    QCustomPlot *widget;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1122, 808);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        sizePolicy.setHeightForWidth(centralWidget->sizePolicy().hasHeightForWidth());
        centralWidget->setSizePolicy(sizePolicy);
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        textBrowserDevStatus = new QTextBrowser(centralWidget);
        textBrowserDevStatus->setObjectName(QStringLiteral("textBrowserDevStatus"));
        QFont font;
        font.setFamily(QStringLiteral("Consolas"));
        font.setPointSize(10);
        textBrowserDevStatus->setFont(font);

        verticalLayout->addWidget(textBrowserDevStatus);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        plainTextEditCmdLog = new QPlainTextEdit(centralWidget);
        plainTextEditCmdLog->setObjectName(QStringLiteral("plainTextEditCmdLog"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Maximum);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(plainTextEditCmdLog->sizePolicy().hasHeightForWidth());
        plainTextEditCmdLog->setSizePolicy(sizePolicy1);
        plainTextEditCmdLog->setMaximumSize(QSize(16777215, 160));
        plainTextEditCmdLog->setFont(font);
        plainTextEditCmdLog->setReadOnly(true);
        plainTextEditCmdLog->setMaximumBlockCount(10000);

        horizontalLayout->addWidget(plainTextEditCmdLog);

        plainTextEditMsgLog = new QPlainTextEdit(centralWidget);
        plainTextEditMsgLog->setObjectName(QStringLiteral("plainTextEditMsgLog"));
        sizePolicy1.setHeightForWidth(plainTextEditMsgLog->sizePolicy().hasHeightForWidth());
        plainTextEditMsgLog->setSizePolicy(sizePolicy1);
        plainTextEditMsgLog->setMaximumSize(QSize(16777215, 160));
        plainTextEditMsgLog->setFont(font);
        plainTextEditMsgLog->setReadOnly(true);
        plainTextEditMsgLog->setMaximumBlockCount(10000);

        horizontalLayout->addWidget(plainTextEditMsgLog);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        labelCmd = new QLabel(centralWidget);
        labelCmd->setObjectName(QStringLiteral("labelCmd"));
        QFont font1;
        font1.setPointSize(12);
        labelCmd->setFont(font1);

        horizontalLayout_2->addWidget(labelCmd);

        lineEditCmd = new QLineEdit(centralWidget);
        lineEditCmd->setObjectName(QStringLiteral("lineEditCmd"));
        lineEditCmd->setFont(font);

        horizontalLayout_2->addWidget(lineEditCmd);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        labelIP = new QLabel(centralWidget);
        labelIP->setObjectName(QStringLiteral("labelIP"));
        labelIP->setFont(font1);

        horizontalLayout_3->addWidget(labelIP);

        lineEditIP = new QLineEdit(centralWidget);
        lineEditIP->setObjectName(QStringLiteral("lineEditIP"));
        lineEditIP->setFont(font);

        horizontalLayout_3->addWidget(lineEditIP);

        labelPort = new QLabel(centralWidget);
        labelPort->setObjectName(QStringLiteral("labelPort"));
        labelPort->setFont(font1);

        horizontalLayout_3->addWidget(labelPort);

        lineEditPort = new QLineEdit(centralWidget);
        lineEditPort->setObjectName(QStringLiteral("lineEditPort"));
        lineEditPort->setFont(font);

        horizontalLayout_3->addWidget(lineEditPort);

        pushButtonConnect = new QPushButton(centralWidget);
        pushButtonConnect->setObjectName(QStringLiteral("pushButtonConnect"));
        pushButtonConnect->setFont(font1);

        horizontalLayout_3->addWidget(pushButtonConnect);


        verticalLayout->addLayout(horizontalLayout_3);


        horizontalLayout_4->addLayout(verticalLayout);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        widget_3 = new QCustomPlot(centralWidget);
        widget_3->setObjectName(QStringLiteral("widget_3"));

        verticalLayout_2->addWidget(widget_3);

        widget_2 = new QCustomPlot(centralWidget);
        widget_2->setObjectName(QStringLiteral("widget_2"));
        QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(widget_2->sizePolicy().hasHeightForWidth());
        widget_2->setSizePolicy(sizePolicy2);
        widget_2->setMinimumSize(QSize(350, 0));

        verticalLayout_2->addWidget(widget_2);

        widget = new QCustomPlot(centralWidget);
        widget->setObjectName(QStringLiteral("widget"));
        sizePolicy2.setHeightForWidth(widget->sizePolicy().hasHeightForWidth());
        widget->setSizePolicy(sizePolicy2);
        widget->setMinimumSize(QSize(350, 0));

        verticalLayout_2->addWidget(widget);


        horizontalLayout_4->addLayout(verticalLayout_2);


        gridLayout->addLayout(horizontalLayout_4, 0, 0, 1, 1);

        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "RobotMonitor", 0));
        labelCmd->setText(QApplication::translate("MainWindow", ">>>", 0));
        labelIP->setText(QApplication::translate("MainWindow", "Remote IP:", 0));
        labelPort->setText(QApplication::translate("MainWindow", "Remote Port:", 0));
        pushButtonConnect->setText(QApplication::translate("MainWindow", "Connect", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
