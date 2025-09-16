#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVTKOpenGLWidget.h>
#include <QVTKOpenGLStereoWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/types.h>
#include <pcl/point_cloud.h>

#include <vtkSmartPointer.h>
#include <vtkGenericOpenGLRenderWindow.h>

//  -----------------数据库--------------
#include <QSqlDatabase>
#include <QSqlError>
#include <QSqlQuery>
//  -----------------数据库--------------

#include "algo.h"

#include <QListWidget>
#include <QThread>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE
class Algo;
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void InitData();
    void InitUIdesign();
    void InitSqlite();
    bool get_algorithm_selected();
    static void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* args);

public:     // 槽函数
    void open_file_slot();
    void onceRunningSlot(bool checked_);
    void help_slot();
    void data_list_change_slot(QListWidgetItem *item);
    void onListWidgetContextMenu(const QPoint &pos);
    void onHomeAction_slot();


signals:  // 添加这个signals部分
    void startSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int sample_num);
    void startMesurement(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int star_idx);

private:
    Ui::MainWindow *ui;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampler_data;

    QSqlDatabase db;
    // Algo* algo;  // 创建算法类对象
    QThread* sub;
    Algo* algo;
    int algorithm_select = 0;//选择的算法

    pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d;
    static int starPointIdx;

};
#endif // MAINWINDOW_H
