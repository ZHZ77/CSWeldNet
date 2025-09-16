#ifndef PCLALGORITHMS_H
#define PCLALGORITHMS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/farthest_point_sampling.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <memory>
#include <vector>
#include <string>

/**
 * @brief PCL算法封装类
 * 封装了常用的PCL算法，包括采样、滤波、分割、表面重建等
 */
class PCLAlgorithms
{
public:
    // 点云类型定义
    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;
    using PointCloudPtr = PointCloudT::Ptr;
    using PointCloudConstPtr = PointCloudT::ConstPtr;
    
    using PointNormalT = pcl::PointNormal;
    using PointNormalCloudT = pcl::PointCloud<PointNormalT>;
    using PointNormalCloudPtr = PointNormalCloudT::Ptr;

    /**
     * @brief 构造函数
     */
    PCLAlgorithms();
    
    /**
     * @brief 析构函数
     */
    ~PCLAlgorithms();

    // ==================== 采样算法 ====================
    
    /**
     * @brief 体素采样（降采样）
     * @param input 输入点云
     * @param output 输出点云
     * @param leaf_size 体素大小
     * @return 是否成功
     */
    bool voxelSampling(const PointCloudPtr& input, PointCloudPtr& output, float leaf_size = 0.01f);
    
    /**
     * @brief 最远点采样
     * @param input 输入点云
     * @param output 输出点云
     * @param num_samples 采样点数
     * @return 是否成功
     */
    bool farthestPointSampling(const PointCloudPtr& input, PointCloudPtr& output, int num_samples = 1000);
    
    /**
     * @brief 随机采样
     * @param input 输入点云
     * @param output 输出点云
     * @param num_samples 采样点数
     * @return 是否成功
     */
    bool randomSampling(const PointCloudPtr& input, PointCloudPtr& output, int num_samples = 1000);
    
    /**
     * @brief 均匀采样
     * @param input 输入点云
     * @param output 输出点云
     * @param radius 采样半径
     * @return 是否成功
     */
    bool uniformSampling(const PointCloudPtr& input, PointCloudPtr& output, float radius = 0.01f);

    // ==================== 滤波算法 ====================
    
    /**
     * @brief 统计离群点移除
     * @param input 输入点云
     * @param output 输出点云
     * @param mean_k 邻域点数
     * @param std_dev_mul_thresh 标准差倍数阈值
     * @return 是否成功
     */
    bool statisticalOutlierRemoval(const PointCloudPtr& input, PointCloudPtr& output, 
                                   int mean_k = 20, double std_dev_mul_thresh = 1.0);
    
    /**
     * @brief 半径离群点移除
     * @param input 输入点云
     * @param output 输出点云
     * @param radius 搜索半径
     * @param min_neighbors 最小邻居数
     * @return 是否成功
     */
    bool radiusOutlierRemoval(const PointCloudPtr& input, PointCloudPtr& output,
                              double radius = 0.05, int min_neighbors = 2);
    
    /**
     * @brief 直通滤波
     * @param input 输入点云
     * @param output 输出点云
     * @param field_name 字段名
     * @param min_limit 最小值
     * @param max_limit 最大值
     * @return 是否成功
     */
    bool passThroughFilter(const PointCloudPtr& input, PointCloudPtr& output,
                           const std::string& field_name = "z", 
                           double min_limit = 0.0, double max_limit = 1.0);
    
    /**
     * @brief 包围盒滤波
     * @param input 输入点云
     * @param output 输出点云
     * @param min_pt 最小点
     * @param max_pt 最大点
     * @return 是否成功
     */
    bool cropBoxFilter(const PointCloudPtr& input, PointCloudPtr& output,
                       const Eigen::Vector4f& min_pt, const Eigen::Vector4f& max_pt);

    // ==================== 分割算法 ====================
    
    /**
     * @brief RANSAC平面分割
     * @param input 输入点云
     * @param inliers 内点索引
     * @param coefficients 平面系数
     * @param distance_threshold 距离阈值
     * @return 是否成功
     */
    bool ransacPlaneSegmentation(const PointCloudPtr& input, 
                                 pcl::PointIndices::Ptr& inliers,
                                 pcl::ModelCoefficients::Ptr& coefficients,
                                 double distance_threshold = 0.01);
    
    /**
     * @brief 欧几里得聚类分割
     * @param input 输入点云
     * @param cluster_indices 聚类索引
     * @param cluster_tolerance 聚类容差
     * @param min_cluster_size 最小聚类大小
     * @param max_cluster_size 最大聚类大小
     * @return 是否成功
     */
    bool euclideanClustering(const PointCloudPtr& input,
                             std::vector<pcl::PointIndices>& cluster_indices,
                             double cluster_tolerance = 0.02,
                             int min_cluster_size = 100,
                             int max_cluster_size = 25000);

    // ==================== 表面重建算法 ====================
    
    /**
     * @brief 贪婪投影三角化
     * @param input 输入点云（带法向量）
     * @param output 输出网格
     * @param search_radius 搜索半径
     * @param mu 最大邻域距离
     * @param max_nearest_neighbors 最大最近邻数
     * @param max_surface_angle 最大表面角度
     * @param min_angle 最小角度
     * @param max_angle 最大角度
     * @param normal_consistency 法向量一致性
     * @return 是否成功
     */
    bool greedyProjectionTriangulation(const PointNormalCloudPtr& input,
                                       pcl::PolygonMesh& output,
                                       double search_radius = 0.025,
                                       double mu = 2.5,
                                       int max_nearest_neighbors = 100,
                                       double max_surface_angle = M_PI/4,
                                       double min_angle = M_PI/18,
                                       double max_angle = 2*M_PI/3,
                                       bool normal_consistency = false);
    
    /**
     * @brief Poisson重建
     * @param input 输入点云（带法向量）
     * @param output 输出网格
     * @param depth 八叉树深度
     * @param solver_divide 求解器分割
     * @param iso_divide 等值面分割
     * @param point_weight 点权重
     * @param scale 缩放
     * @param linear_fit 线性拟合
     * @return 是否成功
     */
    bool poissonReconstruction(const PointNormalCloudPtr& input,
                              pcl::PolygonMesh& output,
                              int depth = 8,
                              int solver_divide = 8,
                              int iso_divide = 8,
                              float point_weight = 4.0f,
                              float scale = 1.1f,
                              bool linear_fit = false);

    // ==================== 法向量计算 ====================
    
    /**
     * @brief 计算法向量
     * @param input 输入点云
     * @param output 输出带法向量的点云
     * @param radius 搜索半径
     * @return 是否成功
     */
    bool computeNormals(const PointCloudPtr& input, PointNormalCloudPtr& output, double radius = 0.03);

    // ==================== 工具函数 ====================
    
    /**
     * @brief 保存点云到文件
     * @param cloud 点云
     * @param filename 文件名
     * @return 是否成功
     */
    bool savePointCloud(const PointCloudPtr& cloud, const std::string& filename);
    
    /**
     * @brief 从文件加载点云
     * @param cloud 点云
     * @param filename 文件名
     * @return 是否成功
     */
    bool loadPointCloud(PointCloudPtr& cloud, const std::string& filename);
    
    /**
     * @brief 获取点云信息
     * @param cloud 点云
     * @return 点云信息字符串
     */
    std::string getPointCloudInfo(const PointCloudPtr& cloud);

private:
    // 私有成员变量
    pcl::KdTreeFLANN<PointT>::Ptr kdtree_;  // KD树用于快速搜索
    
    // 私有辅助函数
    void initializeKdTree(const PointCloudPtr& cloud);
};

#endif // PCLALGORITHMS_H

