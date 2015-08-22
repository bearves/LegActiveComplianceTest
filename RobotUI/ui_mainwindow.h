/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.2.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
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

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QWidget *verticalLayoutWidget;
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

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(640, 629);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayoutWidget = new QWidget(centralWidget);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(9, 9, 621, 611));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        textBrowserDevStatus = new QTextBrowser(verticalLayoutWidget);
        textBrowserDevStatus->setObjectName(QStringLiteral("textBrowserDevStatus"));
        QFont font;
        font.setFamily(QStringLiteral("Droid Sans Mono"));
        font.setPointSize(12);
        textBrowserDevStatus->setFont(font);

        verticalLayout->addWidget(textBrowserDevStatus);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        plainTextEditCmdLog = new QPlainTextEdit(verticalLayoutWidget);
        plainTextEditCmdLog->setObjectName(QStringLiteral("plainTextEditCmdLog"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(plainTextEditCmdLog->sizePolicy().hasHeightForWidth());
        plainTextEditCmdLog->setSizePolicy(sizePolicy);
        plainTextEditCmdLog->setMaximumSize(QSize(16777215, 160));
        QFont font1;
        font1.setPointSize(12);
        plainTextEditCmdLog->setFont(font1);
        plainTextEditCmdLog->setReadOnly(true);
        plainTextEditCmdLog->setMaximumBlockCount(10000);

        horizontalLayout->addWidget(plainTextEditCmdLog);

        plainTextEditMsgLog = new QPlainTextEdit(verticalLayoutWidget);
        plainTextEditMsgLog->setObjectName(QStringLiteral("plainTextEditMsgLog"));
        sizePolicy.setHeightForWidth(plainTextEditMsgLog->sizePolicy().hasHeightForWidth());
        plainTextEditMsgLog->setSizePolicy(sizePolicy);
        plainTextEditMsgLog->setMaximumSize(QSize(16777215, 160));
        plainTextEditMsgLog->setFont(font1);
        plainTextEditMsgLog->setReadOnly(true);
        plainTextEditMsgLog->setMaximumBlockCount(10000);

        horizontalLayout->addWidget(plainTextEditMsgLog);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        labelCmd = new QLabel(verticalLayoutWidget);
        labelCmd->setObjectName(QStringLiteral("labelCmd"));
        labelCmd->setFont(font1);

        horizontalLayout_2->addWidget(labelCmd);

        lineEditCmd = new QLineEdit(verticalLayoutWidget);
        lineEditCmd->setObjectName(QStringLiteral("lineEditCmd"));
        lineEditCmd->setFont(font1);

        horizontalLayout_2->addWidget(lineEditCmd);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        labelIP = new QLabel(verticalLayoutWidget);
        labelIP->setObjectName(QStringLiteral("labelIP"));
        labelIP->setFont(font1);

        horizontalLayout_3->addWidget(labelIP);

        lineEditIP = new QLineEdit(verticalLayoutWidget);
        lineEditIP->setObjectName(QStringLiteral("lineEditIP"));
        lineEditIP->setFont(font1);

        horizontalLayout_3->addWidget(lineEditIP);

        labelPort = new QLabel(verticalLayoutWidget);
        labelPort->setObjectName(QStringLiteral("labelPort"));
        labelPort->setFont(font1);

        horizontalLayout_3->addWidget(labelPort);

        lineEditPort = new QLineEdit(verticalLayoutWidget);
        lineEditPort->setObjectName(QStringLiteral("lineEditPort"));
        lineEditPort->setFont(font1);

        horizontalLayout_3->addWidget(lineEditPort);

        pushButtonConnect = new QPushButton(verticalLayoutWidget);
        pushButtonConnect->setObjectName(QStringLiteral("pushButtonConnect"));
        pushButtonConnect->setFont(font1);

        horizontalLayout_3->addWidget(pushButtonConnect);


        verticalLayout->addLayout(horizontalLayout_3);

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
