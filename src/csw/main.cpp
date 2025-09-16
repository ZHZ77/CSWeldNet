#include "mainwindow.h"
#include <QApplication>

#include "logmessage.h"
#include <QIcon>
#include <vtkOutputWindow.h>



int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    vtkOutputWindow::SetGlobalWarningDisplay(0);  // 禁用警告窗口显示

    //安装消息器
    LogMessage::Instance()->installMessageHandler();

    MainWindow w;
    w.show();
    w.setWindowIcon(QIcon(":/image/cswnet.ico"));
    w.setMinimumSize(892,663);
    w.show();
    return app.exec();
}
