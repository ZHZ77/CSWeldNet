#ifndef LOGMESSAGE_H
#define LOGMESSAGE_H

#include <QObject>
#include <iostream>

/****************************************************************
 * LogMessage类
 * 负责这个界面的调试日志信息打印
****************************************************************/
class LogMessage : public QObject
{
    Q_OBJECT
private:
    explicit LogMessage(QObject *parent = nullptr);
    ~LogMessage();
    static LogMessage* self;

public:
    static  LogMessage* Instance();
    void installMessageHandler();
    void uninstallMessageHandler();
    QString logPath();
    QString logName();

signals:
    void sigDebugStrData(const QString &);
    void sigDebugHtmlData(const QString &);

private:
    class GC
    {
    public:
        ~GC();
    };
};

#endif // LOGMESSAGE_H
