#include "algo.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <limits>
#include <cmath>
#include <pcl/io/pcd_io.h>  // loadPCDFile
#include <pcl/io/ply_io.h>  // loadPLYFile
#include <QDebug>


// algo.cpp
Algo::Algo(QObject *parent) : QObject(parent)  // ✅ 正确初始化基类
{
    // 构造函数体
    // qRegisterMetaType<Algo>("Algo");
}

// 加载点云
pcl::PointCloud<pcl::PointXYZ>::Ptr Algo::loadPointCloud(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    std::string ext = filename.substr(filename.find_last_of('.') + 1);
    int ret = -1;

    if (ext == "pcd" || ext == "PCD") {
        ret = pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud);
    } else if (ext == "ply" || ext == "PLY") {
        ret = pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud);
    } else {
        std::cerr << "[Error] Unsupported file format: " << ext << std::endl;
        return nullptr;
    }

    if (ret == -1) {
        std::cerr << "[Error] Failed to load point cloud: " << filename << std::endl;
        return nullptr;
    }

    std::cout << "[Info] Loaded point cloud: " << filename
              << " (" << cloud->size() << " points)" << std::endl;
    return cloud;
}


// FPS 核心索引计算
std::vector<int> Algo::fpsIndices(const CloudConstPtr& cloud, int sample_num)
{
    std::vector<int> sampled_indices;
    if (!cloud || cloud->empty() || sample_num <= 0) return sampled_indices;

    sampled_indices.reserve(sample_num);
    std::vector<float> min_distances(cloud->size(), std::numeric_limits<float>::max());

    int first_index = 0;
    sampled_indices.push_back(first_index);

    for (int i = 1; i < sample_num; ++i) {
        int farthest_index = -1;
        float max_distance = -1.0f;
        const auto& last_pt = cloud->points[sampled_indices.back()];

        for (size_t j = 0; j < cloud->size(); ++j) {
            const auto& pt = cloud->points[j];
            float dist = (pt.x - last_pt.x) * (pt.x - last_pt.x) +
                         (pt.y - last_pt.y) * (pt.y - last_pt.y) +
                         (pt.z - last_pt.z) * (pt.z - last_pt.z);

            if (dist < min_distances[j]) min_distances[j] = dist;
            if (min_distances[j] > max_distance) {
                max_distance = min_distances[j];
                farthest_index = static_cast<int>(j);
            }
        }
        sampled_indices.push_back(farthest_index);
    }
    return sampled_indices;
}


// FPS 采样
pcl::PointCloud<pcl::PointXYZ>::Ptr Algo::farthestPointSampling(const CloudConstPtr& cloud, int sample_num)
{
    auto indices = fpsIndices(cloud, sample_num);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampled(new pcl::PointCloud<pcl::PointXYZ>());
    sampled->reserve(indices.size());
    for (auto idx : indices) sampled->push_back(cloud->points[idx]);
    return sampled;
}

void Algo::doSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int sample_num)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr res = farthestPointSampling(cloud, sample_num);
    emit samplingFinished(res);
}



// 槽函数
void Algo::domesurement_height(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int star_idx)
{
    qDebug()<< "domesurement height Algo slot";
    qDebug()<< star_idx;

    std::map<int, double> angle_distance_dict;

    if (!cloud || cloud->empty() || cloud->size() <= 10||star_idx==-1)
    {
        qWarning() << "cloud is empty or too small";
        return;
    }

    int init_P_index =  star_idx; // star_idx; // 对应 Python 的 init_P = 10
    const pcl::PointXYZ& init_point = cloud->points[init_P_index];

    // 1) 投影到 XY 平面
    auto projected_pcd = project_to_xy_plane(cloud);


    // 角度设置
    std::vector<int> angle_list = {0, 45, 90, 135, 180, 225, 270, -45};
    double add_sub = 5.0;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered_points_list;
    // 预备：把原始点云复制为 Eigen 矩阵时会用到
    for (int ang : angle_list)
    {
        // 2) 计算当前角度窗口
        double init_angle = angle_with_x_axis(init_point);
        double total_angle = init_angle + static_cast<double>(ang);

        // 3) 在投影点云上按角度筛选
        auto result = filter_points_by_angle(projected_pcd, total_angle - add_sub, total_angle + add_sub);
        // ---------------------------------------------
        // const std::vector<int>& filter_id = result.first;
        // qDebug() << filter_id;
        // // 创建一个新的点云，存放筛选结果
        // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // filtered_cloud->reserve(filter_id.size());
        // for (int idx : filter_id)
        // {
        //     filtered_cloud->push_back(projected_pcd->points[idx]);
        // }
        // filtered_cloud->width = filtered_cloud->size();
        // filtered_cloud->height = 1;
        // filtered_cloud->is_dense = true;
        // // 现在可以显示了
        // showProjectedCloud(filtered_cloud);
        // qDebug() << ang;
        // ---------------------------------------------
        // ============xianshi====================================================================
        // const std::vector<int>& filter_id = result.first;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // filtered_cloud->reserve(filter_id.size());
        // for (int idx : filter_id)
        // {
        //     filtered_cloud->push_back(projected_pcd->points[idx]);
        // }
        // filtered_cloud->width = filtered_cloud->size();
        // filtered_cloud->height = 1;
        // filtered_cloud->is_dense = true;
        // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Compare Viewer"));
        // viewer->setBackgroundColor(0, 0, 0);

        // // 原始投影点云（灰色）
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gray(projected_pcd, 200, 200, 200);
        // viewer->addPointCloud(projected_pcd, gray, "projected");

        // // 筛选点云（红色）
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(filtered_cloud, 255, 0, 0);
        // viewer->addPointCloud(filtered_cloud, red, "filtered");

        // // 可选：设置点大小
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "projected");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "filtered");

        // while (!viewer->wasStopped())
        // {
        //     viewer->spinOnce(10);
        // }
// =============================================================================================

        const std::vector<int>& filter_idx  = result.first;
        const std::vector<int>& middle_idx  = result.second;    // 都不为空



        if (filter_idx.empty() || middle_idx.empty()) {
            // 没有点，标记为 NaN 或跳过
            angle_distance_dict[ang] = std::numeric_limits<double>::quiet_NaN();
            qDebug() << "have empty filter middle";
            continue;
        }

        // 4) 根据索引提取对应的3D点（注意：角度是在 XY 投影上算的，但后续使用原始 3D 坐标）
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points(new pcl::PointCloud<pcl::PointXYZ>);
        filtered_points->reserve(filter_idx.size());
        for (int idx : filter_idx) {
            // idx 来自 projected_pcd 的索引，和 cloud 是一一对应的（因为只是 Z->0 的投影）
            filtered_points->push_back(cloud->points[idx]);
        }
        filtered_points->width = filtered_points->size();
        filtered_points->height = 1;
        filtered_points->is_dense = true;
        filtered_points_list.push_back(filtered_points);

        // ============xianshi====================================================================
        // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Compare Viewer"));
        // viewer->setBackgroundColor(0, 0, 0);
        // // 原始投影点云（灰色）
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gray(projected_pcd, 200, 200, 200);
        // viewer->addPointCloud(cloud, gray, "origin");
        // // 筛选点云（红色）
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(filtered_points, 255, 0, 0);
        // viewer->addPointCloud(filtered_points, red, "filtered");
        // // 可选：设置点大小
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "projected");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "filtered");
        // while (!viewer->wasStopped())
        // {
        //     viewer->spinOnce(10);
        // }
        // =============================================================================================

        // 中间角度附近的点，取第一个作为 P3
        const int mid_idx = middle_idx.front();
        const pcl::PointXYZ& P3 = cloud->points[mid_idx];

        // 5) 通过三点确定法向量：P1=(0,0,0), P2=(0,0,1), P3=middle_points[0]
        Eigen::Vector3d P1(0.0, 0.0, 0.0);
        Eigen::Vector3d P2(0.0, 0.0, 1.0);
        Eigen::Vector3d P3e(P3.x, P3.y, P3.z);
        Eigen::Vector3d normal = (P2 - P1).cross(P3e - P1);
        if (normal.norm() < 1e-12) {
            // 退化情况，无法定义平面
            qDebug() << "plane not generate";
            angle_distance_dict[ang] = std::numeric_limits<double>::quiet_NaN();
            continue;
        }
        normal.normalize();
        double a = normal.x(), b = normal.y(), c = normal.z();

        // 6) 将筛选出的点云投影到该平面（Ax+By+Cz+D=0，这里 D=0）
        auto plane_cloud = plane_fine(filtered_points, a, b, c, 0.0);
        // ============xianshi====================================================================
        // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Compare Viewer"));
        // viewer->setBackgroundColor(0, 0, 0);

        // // 筛选点云（红色）
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(plane_cloud, 255, 0, 0);
        // viewer->addPointCloud(plane_cloud, red, "filtered");
        // // 可选：设置点大小
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "filtered");
        // while (!viewer->wasStopped())
        // {
        //     viewer->spinOnce(10);
        // }
        // =============================================================================================




        if (plane_cloud->empty()) {
            qDebug() << "touyingshibai";
            angle_distance_dict[ang] = std::numeric_limits<double>::quiet_NaN();
            continue;
        }

        // 7) 计算把法向量旋转到 YOZ 平面的旋转矩阵
        Eigen::Matrix3d R = rotate_to_yoz(normal);

        // 8) 把 plane_cloud 的点乘以 R^T，得到旋转后的点
        Eigen::MatrixXd rotated_points(plane_cloud->size(), 3);
        for (size_t i = 0; i < plane_cloud->size(); ++i) {
            Eigen::Vector3d v(plane_cloud->points[i].x,
                              plane_cloud->points[i].y,
                              plane_cloud->points[i].z);
            Eigen::Vector3d vr = R.transpose() * v;
            rotated_points(i, 0) = vr.x();
            rotated_points(i, 1) = vr.y();
            rotated_points(i, 2) = vr.z();
        }

        // ========转换后的显示
        // pcl::PointCloud<pcl::PointXYZ>::Ptr xoy_line(new pcl::PointCloud<pcl::PointXYZ>);
        // xoy_line->reserve(rotated_points.rows());
        // for (int i = 0; i < rotated_points.rows(); ++i) {
        //     pcl::PointXYZ p;
        //     p.x = static_cast<float>(rotated_points(i, 0));
        //     p.y = static_cast<float>(rotated_points(i, 1));
        //     p.z = static_cast<float>(rotated_points(i, 2));
        //     xoy_line->push_back(p);
        // }
        // xoy_line->width = xoy_line->size();
        // xoy_line->height = 1;
        // xoy_line->is_dense = true;

        // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Compare Viewer"));
        // viewer->setBackgroundColor(0, 0, 0);

        // // 筛选点云（红色）
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(xoy_line, 255, 0, 0);
        // viewer->addPointCloud(xoy_line, red, "filtered");
        // // 可选：设置点大小
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "filtered");
        // while (!viewer->wasStopped())
        // {
        //     viewer->spinOnce(10);
        // }
        // =================

    //     // 9) 计算距离（与你的 Python 版本一致）
        double distance = calculate_distance(rotated_points);
        qDebug() << distance;

    // 存入结果
        angle_distance_dict[ang] = distance;
    }

    // 可选：打印结果
    for (const auto& kv : angle_distance_dict) {
        qDebug() << "angle =" << kv.first << ", distance =" << kv.second;
    }


    emit mesurementFinished(filtered_points_list);
}



pcl::PointCloud<pcl::PointXYZ>::Ptr Algo::project_to_xy_plane(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    auto projected = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    projected->reserve(cloud->size());

    for (const auto& p : cloud->points)
    {
        pcl::PointXYZ pt = p;
        pt.z = 0.0f; // 投影到 XY 平面
        projected->push_back(pt);
    }
    projected->width = projected->size();
    projected->height = 1;
    projected->is_dense = true;
    return projected;
}

// 计算与 X 轴夹角（度）
double Algo::angle_with_x_axis(const pcl::PointXYZ& point)
{
    return std::atan2(point.y, point.x) * 180.0 / M_PI;
}

// 按角度筛选点
std::pair<std::vector<int>, std::vector<int>> Algo::filter_points_by_angle(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    double min_angle, double max_angle)
{
    std::vector<int> filtered_indices;
    std::vector<int> middle_indices;
    double mid_angle = (min_angle + max_angle) / 2.0;

    for (size_t i = 0; i < cloud->size(); ++i) {
        double ang = angle_with_x_axis(cloud->points[i]);
        if (ang >= min_angle && ang <= max_angle)
            filtered_indices.push_back(static_cast<int>(i));
        if (std::fabs(ang - mid_angle) <= 0.2)
            middle_indices.push_back(static_cast<int>(i));
    }
    return {filtered_indices, middle_indices};
}

// 将点云投影到平面 Ax+By+Cz+D=0
pcl::PointCloud<pcl::PointXYZ>::Ptr Algo::plane_fine(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    double A, double B, double C, double D)
{
    auto projected = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    projected->reserve(cloud->size());
    double denom = A*A + B*B + C*C;
    for (auto& p : cloud->points) {
        double t = -(A*p.x + B*p.y + C*p.z + D) / denom;
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(A * t + p.x);
        pt.y = static_cast<float>(B * t + p.y);
        pt.z = static_cast<float>(C * t + p.z);
        projected->push_back(pt);
    }
    projected->width = projected->size();
    projected->height = 1;
    projected->is_dense = true;
    return projected;
}

// 旋转到 YOZ 平面
Eigen::Matrix3d Algo::rotate_to_yoz(const Eigen::Vector3d& plane_normal)
{
    Eigen::Vector3d target_normal(1, 0, 0);
    Eigen::Vector3d n = plane_normal.normalized();
    Eigen::Vector3d axis = n.cross(target_normal);
    double axis_norm = axis.norm();

    if (axis_norm < 1e-6) {
        if (n.dot(target_normal) > 0)
            return Eigen::Matrix3d::Identity();
        else
            return -Eigen::Matrix3d::Identity();
    }

    axis.normalize();
    double cos_theta = n.dot(target_normal);
    double theta = std::acos(cos_theta);

    double a = std::cos(theta / 2.0);
    double b = -axis.x() * std::sin(theta / 2.0);
    double c = -axis.y() * std::sin(theta / 2.0);
    double d = -axis.z() * std::sin(theta / 2.0);

    double aa = a*a, bb = b*b, cc = c*c, dd = d*d;
    double bc = b*c, ad = a*d, ac = a*c, ab = a*b, bd = b*d, cd = c*d;

    Eigen::Matrix3d R;
    R << aa + bb - cc - dd, 2*(bc + ad),     2*(bd - ac),
        2*(bc - ad),       aa + cc - bb - dd, 2*(cd + ab),
        2*(bd + ac),       2*(cd - ab),     aa + dd - bb - cc;
    return R;
}

// 计算距离
double Algo::calculate_distance(const Eigen::MatrixXd& rotated_points)
{
    // 1. 输入检查
    if (rotated_points.rows() == 0 || rotated_points.cols() < 3) {
        qWarning() << "calculate_distance: rotated_points is empty or invalid";
        return std::numeric_limits<double>::quiet_NaN();
    }

    // 2. 提取 y 和 x（这里的 x 是第3列）
    Eigen::VectorXd y = rotated_points.col(1);
    Eigen::VectorXd x = rotated_points.col(2);

    // 3. 按 x 排序
    std::vector<int> idx(x.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&](int i, int j) { return x(i) < x(j); });

    // 4. 重新排列
    Eigen::VectorXd xs(idx.size()), ys(idx.size());
    for (int i = 0; i < idx.size(); ++i) {
        xs(i) = x(idx[i]);
        ys(i) = y(idx[i]);
    }

    // 5. 限制最高点位置（方案 4）
    if (xs.size() < 3) {
        qWarning() << "calculate_distance: not enough points to find peak";
        return std::numeric_limits<double>::quiet_NaN();
    }
    double min_ratio = 0.2; // 左边至少 20%
    double max_ratio = 0.8; // 右边至少 20%
    int start_idx = static_cast<int>(xs.size() * min_ratio);
    int end_idx   = static_cast<int>(xs.size() * max_ratio);
    start_idx = std::max(0, start_idx);
    end_idx   = std::min(static_cast<int>(xs.size()) - 1, end_idx);

    if (end_idx <= start_idx) {
        qWarning() << "calculate_distance: invalid peak search range";
        return std::numeric_limits<double>::quiet_NaN();
    }

    int max_idx = start_idx;
    double max_val = ys(start_idx);
    for (int i = start_idx + 1; i <= end_idx; ++i) {
        if (ys(i) > max_val) {
            max_val = ys(i);
            max_idx = i;
        }
    }
    double max_x = xs(max_idx);

    // 6. 分左右
    std::vector<double> left_x, left_y, right_x, right_y;
    for (int i = 0; i < xs.size(); ++i) {
        if (xs(i) < max_x) { left_x.push_back(xs(i)); left_y.push_back(ys(i)); }
        else if (xs(i) > max_x) { right_x.push_back(xs(i)); right_y.push_back(ys(i)); }
    }

    // 7. 检查左右点数
    if (left_x.size() < 2 || right_x.size() < 2) {
        qWarning() << "calculate_distance: not enough points on one side"
                   << " left:" << left_x.size() << " right:" << right_x.size();
        return std::numeric_limits<double>::quiet_NaN();
    }

    // 8. 取左右各 2/3，限制 n 不超过实际数量
    int n_left  = std::max(1, static_cast<int>(left_x.size()  * 2 / 3));
    int n_right = std::max(1, static_cast<int>(right_x.size() * 2 / 3));
    n_left  = std::min(n_left,  static_cast<int>(left_x.size()));
    n_right = std::min(n_right, static_cast<int>(right_x.size()));

    // 9. 组合拟合点
    std::vector<double> combined_x(left_x.begin(), left_x.begin() + n_left);
    combined_x.insert(combined_x.end(), right_x.end() - n_right, right_x.end());

    std::vector<double> combined_y(left_y.begin(), left_y.begin() + n_left);
    combined_y.insert(combined_y.end(), right_y.end() - n_right, right_y.end());

    // 10. 检查拟合点数
    if (combined_x.size() < 2) {
        qWarning() << "calculate_distance: not enough points for line fitting";
        return std::numeric_limits<double>::quiet_NaN();
    }

    // 11. 最小二乘拟合直线
    Eigen::VectorXd X(combined_x.size()), Y(combined_y.size());
    for (int i = 0; i < combined_x.size(); ++i) {
        X(i) = combined_x[i];
        Y(i) = combined_y[i];
    }
    Eigen::MatrixXd A(X.size(), 2);
    A.col(0) = X;
    A.col(1) = Eigen::VectorXd::Ones(X.size());
    Eigen::Vector2d sol = A.colPivHouseholderQr().solve(Y);
    double m = sol(0), c = sol(1);

    // 12. 计算最高点到直线的垂直距离
    auto point_to_line_distance = [&](double x0, double y0) {
        return std::abs(m * x0 - y0 + c) / std::sqrt(m*m + 1);
    };

    return point_to_line_distance(xs(max_idx), ys(max_idx));
}

// void Algo::showProjectedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& projected_pcd)
// {
//     // 创建一个独立的可视化窗口
//     pcl::visualization::PCLVisualizer::Ptr viewer11(new pcl::visualization::PCLVisualizer("Projected Cloud Viewer"));
//     viewer11->setBackgroundColor(0, 0, 0); // 黑色背景

//     // 给投影点云设置颜色（绿色）
//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(projected_pcd, 0, 255, 0);
//     viewer11->addPointCloud<pcl::PointXYZ>(projected_pcd, green, "projected_cloud");

//     // 设置点大小
//     viewer11->setPointCloudRenderingProperties(
//         pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "projected_cloud");

//     viewer11->addCoordinateSystem(50.0); // 可选：添加坐标系
//     viewer11->initCameraParameters();

//     // 循环显示，直到手动关闭窗口
//     while (!viewer11->wasStopped())
//     {
//         viewer11->spinOnce(10);
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     }
// }




