#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "logmessage.h"

#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>

// ------------pcl------------------
#include <pcl/io/pcd_io.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>

using namespace boost::placeholders;
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    ,cloud(new pcl::PointCloud<pcl::PointXYZ>)
    ,sampler_data(new pcl::PointCloud<pcl::PointXYZ>)
{
    ui->setupUi(this);
    InitSqlite();
    InitData();
    InitUIdesign();
    // 算法类
    algo = new Algo();
    sub = new QThread;
    algo->moveToThread(sub);
    sub->start();  // 启动线程
    qDebug() << "Log Init Success!";
    // 创建一个vtkRenderer渲染器对象 智能指针
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    // 创建一个 vtkGenericOpenGLRenderWindow 渲染窗口对象。智能指针
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    // 将上面创建的 renderer 添加到渲染窗口中。一个 renderWindow 可以包含多个 renderer，实现多视口渲染，这里只添加一个。

    // 创建一个 PCLVisualizer 对象（PCL 的可视化类）。
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->interactor(), ui->qvtkWidget->renderWindow());
    viewer->setBackgroundColor(0.5,0.5,0.5);
    clicked_points_3d.reset(new pcl::PointCloud<pcl::PointXYZ>);
    // 设置交互器（Interactor），让用户可以用鼠标/键盘操作 3D 场景（旋转、缩放、平移等）
    viewer->registerPointPickingCallback(&MainWindow::pointPickingCallback, static_cast<void*>(this));

}


MainWindow::~MainWindow()
{
    if (sub && sub->isRunning()) {
        sub->quit();
        sub->wait();
    }
    delete algo;
    delete sub;

    if (db.isOpen()) {
        db.close();
        qDebug() << "close sqlite database";
    }
    delete ui;
}

// 初始化页面数据
void MainWindow::InitData()
{
    ui->data_listWidget->clear(); // 先清空 listWidget 内容
    if (!db.isOpen()) {
        qDebug() << "sqlite database open filed";
        return;
    }
    // 获取所有用户表（排除系统表）
    QStringList tables = db.tables(QSql::Tables);

    // 过滤掉可能的系统表（如 sqlite_sequence），只保留用户创建的表
    for (const QString& table : tables) {
        if (table != "sqlite_sequence") { // 排除系统表
            ui->data_listWidget->addItem(table);
        }
    }

}

// 连接信号槽
void MainWindow::InitUIdesign()
{
    // 从资源文件加载图片
    // 注意资源路径格式：":/前缀/文件路径"
    ui->label->setPixmap(QPixmap(":/image/shiyi.png"));
    ui->label->setScaledContents(true);

    // 在构造函数中
    ui->data_listWidget->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui->data_listWidget, &QListWidget::customContextMenuRequested,
            this, &MainWindow::onListWidgetContextMenu);

    //绑定信号，将调试信息输出值ui
    connect(LogMessage::Instance(), &LogMessage::sigDebugHtmlData, ui->LogMessage_textBrowser, &QTextBrowser::append);

    // 打开文件的action
    connect(ui->file_action, &QAction::triggered,this, &MainWindow::open_file_slot);

    //单次运行
    connect(this->ui->run_action,&QAction::triggered,this,&MainWindow::onceRunningSlot);

    // 帮助页面
    connect(ui->help_action,&QAction::triggered,this,&MainWindow::help_slot);

    // 切换文件后切换显示
    connect(ui->data_listWidget,&QListWidget::itemPressed,this,&MainWindow::data_list_change_slot);

    // Home
    connect(ui->home_action, &QAction::triggered, this,&MainWindow::onHomeAction_slot);

}

// 打开文件槽函数
void MainWindow::open_file_slot()
{
    // 先清理之前的显示内容
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    QString filename = QFileDialog::getOpenFileName(
        this,
        "打开点云",
        "../../../../../",
        "点云文件 (*.pcd *.txt);;PCD文件 (*.pcd);;TXT文件 (*.txt)"
        );
    if (filename.isEmpty())
    {
        return;
    }

    std::string file = filename.toStdString();
    std::string ext = file.substr(file.find_last_of('.') + 1);

    if (ext == "pcd" || ext == "PCD") {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, *cloud) == -1) {
            qDebug() << "无法读取PCD文件";
            return;
        }
    }
    else if (ext == "txt" || ext == "TXT") {
        std::ifstream in(file);
        if (!in.is_open()) {
            std::cerr << "无法打开TXT文件" << std::endl;
            return;
        }
        pcl::PointXYZ point;
        while (in >> point.x >> point.y >> point.z) {
            cloud->points.push_back(point);
        }
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;
        in.close();
    }
    else {
        std::cerr << "不支持的文件格式" << std::endl;
        return;
    }
    qDebug() << "加载成功" << filename;
    // ----------------添加到db中----------------
    QFileInfo fileInfo(filename);
    QString tableName = fileInfo.baseName(); // 获取文件名（不含路径和扩展名）
    // 可选：防止表名非法，替换特殊字符或加引号
    tableName.replace("'", "''"); // 转义单引号
    if (tableName.isEmpty()) {
        tableName = "pointcloud"; // 默认名
    }
    // 确保表存在
    QSqlQuery createQuery(db);
    QString createTableSQL = QString(R"(
    CREATE TABLE IF NOT EXISTS "%1" (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        x REAL NOT NULL,
        y REAL NOT NULL,
        z REAL NOT NULL
    )
    )").arg(tableName);

    if (!createQuery.exec(createTableSQL)) {
        qDebug() << "failed create tabel" << createQuery.lastError().text();
        return;
    }

    // 插入点数据
    QSqlQuery insertQuery(db);
    insertQuery.prepare(QString("INSERT INTO \"%1\" (x, y, z) VALUES (?, ?, ?)").arg(tableName));
    db.transaction(); // 使用事务提高大量插入性能
    for (const auto& point : cloud->points) {
        insertQuery.addBindValue(point.x);
        insertQuery.addBindValue(point.y);
        insertQuery.addBindValue(point.z);
        if (!insertQuery.exec()) {
            qWarning() << "failed insert" << insertQuery.lastError().text();
        }
    }

    if (!db.commit()) {
        db.rollback();
        qDebug() << "transaction failed" << db.lastError().text();
    } else {
        qDebug() << "success" << cloud->size() << "nums point save to:" << tableName;
    }
    // 显示到PCLVisualizer
    viewer->addPointCloud(cloud, "cloud");
    viewer->resetCamera();
    ui->qvtkWidget->GetRenderWindow()->Render();
    InitData();

}

// 算法执行
void MainWindow::onceRunningSlot(bool checked_)
{
    Q_UNUSED(checked_);
    if(get_algorithm_selected()==false)
    {
        // QMessageBox::information(this,"imformation",QString::fromLocal8Bit("请选择要运行的算法"));
        return;
    }
    if(algorithm_select==0)
    {

    }
    if(algorithm_select==1)
    {
        if (cloud->size()==0)
        {
            QMessageBox::information(this,"information", "Please input cloud");
            return;
        }
        // 连接信号槽    多线程传递数据用信号槽机制，另外，传递的格式不常见的要先声明
        qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr>("pcl::PointCloud<pcl::PointXYZ>::Ptr");
        connect(this, &MainWindow::startSampling, algo, &Algo::doSampling);
        connect(algo, &Algo::samplingFinished, this, [this](pcl::PointCloud<pcl::PointXYZ>::Ptr result){
            // 这些操作现在会在主线程中执行
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
            viewer->addPointCloud(result, "cloud");
            viewer->resetCamera();
            ui->qvtkWidget->GetRenderWindow()->Render();
            sampler_data = result;
        }, Qt::QueuedConnection);
        // 发送信号而不是直接调用
        emit startSampling(cloud, 16384);
    }
    if(algorithm_select==2)
    {
        if (sampler_data->size()==0)
        {
            QMessageBox::information(this,"information", "Please input cloud");
            return;
        }
        qRegisterMetaType<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>>("std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>");
        connect(this, &MainWindow::startMesurement, algo, &Algo::domesurement_height);
        connect(algo, &Algo::mesurementFinished, this, [this](std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filter){
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
            // 显示原始点云（默认颜色）
            viewer->addPointCloud(sampler_data, "cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                     0.0, 1.0, 0.0,  // 红色 RGB
                                                     "cloud");
            // 显示每个过滤后的点云，并设置为红色
            for (size_t i = 0; i < filter.size(); ++i) {
                std::string cloud_id = "cloud_" + std::to_string(i);
                viewer->addPointCloud<pcl::PointXYZ>(filter[i], cloud_id);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                         1.0, 0.0, 0.0,  // 红色 RGB
                                                         cloud_id);
            }
            viewer->resetCamera();
            ui->qvtkWidget->GetRenderWindow()->Render();
        }, Qt::QueuedConnection);
        // 发送信号而不是直接调用
        emit startMesurement(sampler_data, starPointIdx);
    }
}

//获取所选择的算法
bool MainWindow::get_algorithm_selected()
{
    if(ui->listWidget->currentRow()==0)
    {
        algorithm_select = 0;
        return true;
    }
    else if(ui->listWidget->currentRow()==1)
    {
        algorithm_select = 1;
        return true;
    }
    else if(ui->listWidget->currentRow()==2)
    {
        algorithm_select = 2;
        return true;
    }
    qDebug() << algorithm_select;
    return false;
}

// 帮助页面
void MainWindow::help_slot()
{
    QMessageBox::about(this, tr("Help"), tr("CSWNet application Help"));
}

// 数据库初始化
void MainWindow::InitSqlite()
{
    db = QSqlDatabase::addDatabase("QSQLITE");
    // 注意：这里 new 的是 QSqlDatabase 对象，传入的是 addDatabase 返回的对象
    db.setDatabaseName("data.db");
    if(db.open())
    {
        qDebug() << tr("connect sqlite database success!");
    }
    else
    {
        qDebug() << tr("Faild connect sqlite database:") << db.lastError().text();
    }
}

// DataList 切换选择
void MainWindow::data_list_change_slot(QListWidgetItem *item)
{
    // 先清空之前的点云
    cloud->clear();

    // 构建查询语句（使用双引号防止表名含特殊字符）
    QString queryString = QString(R"(SELECT x, y, z FROM "%1")").arg(item->text());

    QSqlQuery query(db);
    if (!query.exec(queryString)) {
        qDebug() << query.lastError().text();
        return;
    }
    // 遍历查询结果
    int count = 0;
    while (query.next()) {
        double x = query.value("x").toDouble();
        double y = query.value("y").toDouble();
        double z = query.value("z").toDouble();

        pcl::PointXYZ point;
        point.x = static_cast<float>(x);
        point.y = static_cast<float>(y);
        point.z = static_cast<float>(z);
        cloud->push_back(point);
        ++count;
    }

    // 设置点云属性
    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;  // 假设没有无效点
    // 先清理之前的显示内容
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->addPointCloud(cloud, "cloud");
    viewer->resetCamera();
    ui->qvtkWidget->GetRenderWindow()->Render();
}


void MainWindow::onListWidgetContextMenu(const QPoint &pos)
{
    QListWidgetItem *item = ui->data_listWidget->itemAt(pos);
    if (!item) return;

    QMenu menu(this);
    QAction *deleteTableAction = menu.addAction("删除该表");

    QAction *selectedAction = menu.exec(ui->data_listWidget->mapToGlobal(pos));
    if (selectedAction == deleteTableAction) {
        QString tableName = item->text();

        if (!db.isOpen()) {
            qDebug() << "sqlite database open failed";
            return;
        }
        // 二次确认
        if (QMessageBox::question(this, "confirm",
                                  QString("confirm delete file '%1' ?").arg(tableName),
                                  QMessageBox::Yes | QMessageBox::No) == QMessageBox::No) {
            return;
        }
        QSqlQuery query(db);
        QString sql = QString("DROP TABLE IF EXISTS \"%1\"").arg(tableName);
        if (query.exec(sql)) {
            qDebug() << "delete table success" << tableName;
            delete item; // 从列表中移除
        } else {
            qDebug() << "filed delete list" << query.lastError().text();
        }
    }
    InitData(); // 重新刷新列表
}

void MainWindow::onHomeAction_slot()
{
    // 先清理之前的显示内容
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    ui->LogMessage_textBrowser->clear();
}


int MainWindow::starPointIdx = -1;  // 或者你想要的初始值

void MainWindow::pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* args)
{
    if (event.getPointIndex() == -1)
        return;
    MainWindow* self = static_cast<MainWindow*>(args);
    pcl::PointXYZ current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    int idx = event.getPointIndex();

    self->clicked_points_3d->clear();
    self->clicked_points_3d->points.push_back(current_point);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(self->clicked_points_3d, 255, 0, 0);
    self->viewer->removePointCloud("clicked_points");
    self->viewer->addPointCloud(self->clicked_points_3d, red, "clicked_points");
    self->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");

    qDebug() << QString("select: x=%1 y=%2 z=%3")
                    .arg(current_point.x)
                    .arg(current_point.y)
                    .arg(current_point.z);

    starPointIdx = idx;
    qDebug() << starPointIdx;
}


