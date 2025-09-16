#ifndef ALGO_H
#define ALGO_H


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <QObject>




#include <Eigen/Dense>
#include <map>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>   // std::this_thread::sleep_for
#include <chrono>   // std::chrono::milliseconds

class Algo:public QObject
{
    Q_OBJECT  // 必须添加这个宏
public:
    explicit Algo(QObject *parent = nullptr);  // 建议加上 explicit 和 parent 参数
public:
    // using CloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
    using CloudConstPtr = pcl::PointCloud<pcl::PointXYZ>::ConstPtr;

    // 加载点云（支持PCD/PLY）
    pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloud(const std::string& filename);

    // 最远点采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr farthestPointSampling(const CloudConstPtr& cloud, int sample_num);
    // 在Algo类中添加信号
signals:
    void samplingFinished(pcl::PointCloud<pcl::PointXYZ>::Ptr result);
    void mesurementFinished(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filter);

    // 在Algo类中添加槽函数
public slots:
    void doSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int sample_num);
    void domesurement_height(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int star_idx);// 测量

public:// 计算用到的函数
    pcl::PointCloud<pcl::PointXYZ>::Ptr project_to_xy_plane(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
    double angle_with_x_axis(const pcl::PointXYZ& point);
    std::pair<std::vector<int>, std::vector<int>> filter_points_by_angle(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
        double min_angle, double max_angle);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_fine(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
        double A, double B, double C, double D);
    Eigen::Matrix3d rotate_to_yoz(const Eigen::Vector3d& plane_normal);
    double calculate_distance(const Eigen::MatrixXd& rotated_points);
    // void showProjectedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& projected_pcd);



private:
    std::vector<int> fpsIndices(const CloudConstPtr& cloud, int sample_num);

};

#endif // ALGO_H
