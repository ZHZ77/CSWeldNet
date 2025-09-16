#include "logmessage.h"
#include <QMutex>
#include <iostream>
#include <QDateTime>
#include <QCoreApplication>
#include <QFile>
#include <QDir>
#include <QTextStream>

//接收调试信息的函数
void outputMessage(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    static QMutex mutex;
    QMutexLocker lock(&mutex);

    QString text;
    QString MessageColor = "red";
    switch(type)
    {
    //如果是debug信息，那么直接打印至应用程序输出，然后退出本函数
//    case QtDebugMsg:
//        std::cout << msg.toStdString() << std::endl;
//        return ;
    //debug信息
    case QtDebugMsg:
        text = QString("Debug...............................");
        MessageColor = "green";
        break;

    //如果是警告，则继续执行后面的数据处理
    case QtWarningMsg:
        text = QString("Warning...............................");
        MessageColor = "yellow";
        break;

    case QtCriticalMsg:
        text = QString("Error..............................");
        MessageColor = "red";
        break;

    case QtFatalMsg:
        text = QString("Fatal.................................");
        MessageColor = "red";
        break;

    default:
        text = QString("Default...............................");
        MessageColor = "red";
        break;
    }
    //获取单例
    LogMessage *instance = LogMessage::Instance();
    //消息打印时间
    QString current_date_time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString current_date = QString("Time: %1").arg(current_date_time);
    //调试信息
    QString message = QString("%1\r\n%2\r\n%3").arg(text).arg(current_date).arg("MSG : "+msg);
    //将调试信息格式化成 html 格式，
    QString msgHtml = msg;
    msgHtml.replace("  ", "&nbsp;");
    msgHtml.remove("\r");
    msgHtml.replace("\n", "<br>");
    msgHtml = QString("<font color=%1>" + msgHtml + "</font>").arg(MessageColor);

    //格式化后的调试信息
    QString messageHtml = QString("%1<br>%2<br>%3")
            .arg(text)
            .arg(current_date)
            .arg("MSG : "+msgHtml);

    //将调试信息写入文件
    QFile file(instance->logPath() + instance->logName());
    file.open(QIODevice::WriteOnly | QIODevice::Append);
    QTextStream text_stream(&file);
    text_stream << message << "\r\n\r\n\r\n";
    file.flush();
    file.close();
    //将处理好的调试信息发送出去
    instance->sigDebugStrData(message);
    //将处理成 html 的调试信息发送出去
    instance->sigDebugHtmlData(messageHtml);
    //检查文件是否达到了指定大小
    if(file.size() < 1024*1024) {
        return ;
    }
    //log达到了限制值则将名字更改，防止文件越来越大
    for(int loop = 1; loop < 100; ++loop)
    {
        QString fileName = QString("%1/log_%2.txt").arg(instance->logPath()).arg(loop);
        QFile file_1(fileName);
        if(file_1.size() < 4)
        {
            file.rename(fileName);
            return ;
        }
    }
}

//MyDebug单例
LogMessage* LogMessage::self = nullptr;
LogMessage* LogMessage::Instance()
{
    if(!self) {
        QMutex muter;
        QMutexLocker clocker(&muter);

        if(!self) {
            self = new LogMessage();
        }
    }
    return self;
}
//安装消息器
void LogMessage::installMessageHandler()
{
    qInstallMessageHandler(outputMessage);
}
//卸载消息器
void LogMessage::uninstallMessageHandler()
{
    qInstallMessageHandler(0);
}
//在程序的运行目录下建立日志文件路径
QString LogMessage::logPath()
{
    QString current_date_file_name = QDateTime::currentDateTime().toString("yyyy-MM-dd");
    QDir dir(QString("log/%1").arg(current_date_file_name));
    if(!dir.exists()) {
        dir.mkpath("./");
    }
    return dir.path() + "/" ;
}

QString LogMessage::logName()
{
    return "log.txt";
}

LogMessage::LogMessage(QObject *parent) : QObject(parent)
{
    static LogMessage::GC gc;
}

LogMessage::~LogMessage()
{
    std::cout << "~LogMessage" << std::endl;
}

//垃圾自动回收
LogMessage::GC::~GC()
{
    if (self != nullptr) {
        delete self;
        self = nullptr;
    }
}
