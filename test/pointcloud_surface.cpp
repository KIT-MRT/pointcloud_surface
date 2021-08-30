#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

TEST(point_cloud, matrix) {
    pcl::PointCloud<pcl::PointXYZI> cl;
    constexpr size_t num_points{20};
    for (size_t c = 0; c < num_points; c++) {
        cl.push_back(pcl::PointXYZI());
    }
    const Eigen::MatrixXf matrix{cl.getMatrixXfMap()};

    EXPECT_EQ(matrix.rows(), sizeof(pcl::PointXYZI) / sizeof(float));
    EXPECT_EQ(matrix.cols(), num_points);
}
